// The Anycubic ACE Pro / ACE 2 Pro as an AFC unit (port of the cosmoace_afc
// plugin, AFC_ACE.py: afcACE, AceSetup, the dryer heater and humidity
// sensor). Stepperless: the ACE firmware feeds and retracts, the unit drives
// it over ace_transport and homes its feeds on the filament sensors.
use super::ace_transport::{self as tp, monotonic, truthy, RpcResult, Transport, ACE2_ASSIST_ERROR, ACE2_FORBIDDEN_WAIT_S, ACE2_MOTION_METHODS, ACE2_PRELOAD_MAX_MM, ACTIVE_STATES, HEARTBEAT_S, STOP_METHODS};
use super::codes::*;
use super::extruder::AfcExtruder;
use super::lane::{AfcLane, LaneState};
use super::unit::{AfcUnit, UnitKind, UnitSettings};
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::extras::filament_switch_sensor::{GcodeSlot, RunoutHelper, SwitchSensor};
use kaliru::extras::heaters::{HeaterSlot, PrinterHeaters};
use kaliru::gcode::GCodeCommand;
use kaliru::printer::{PObj, PrinterObject};
use kaliru::pyval::PyVal;
use kaliru::reactor::{Timer, NEVER};
use kaliru::IndexMap;
use serde_json::{json, Map, Value};
use std::cell::{Cell, RefCell};
use std::collections::HashMap;
use std::rc::{Rc, Weak};

/// Unit type string: the config default, and the name AFC's lane-type gates key on
pub const UNIT_TYPE: &str = "ACE";
// _feed_until(extend=False) ran its length without reaching the sensor
const NOT_REACHED: &str = "sensor not reached";
// Hub -> toolhead length assumed when a lane has no hub object
const DEFAULT_BOWDEN_MM: f64 = 500.0;
const DRYER_MAX_C: i64 = 65;
const SPIN_UP_S: f64 = 0.6;
const RPC_REPLY_S: f64 = 5.0;
// [AFC_ACE] ace: -> unit protocol
const ACE_MODELS: [(&str, &str); 2] = [("pro", "ace1"), ("2pro", "ace2")];

/// A sensor reader: ("hub" | "tool_start", reads filament present)
type SensorFn = Rc<dyn Fn() -> KResult<bool>>;
type PathSensor = (&'static str, SensorFn);

/// AceDryerHeater: the firmware dryer as a heater UIs can set
pub struct AceDryer {
    unit: RefCell<Weak<AfcUnit>>,
    pub temperature: Cell<f64>,
    pub target: Cell<f64>,
}

impl PrinterObject for AceDryer {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        Some(PyVal::Dict(kaliru::pydict! {
            "temperature" => PyVal::Float(kaliru::pyfmt::py_round(self.temperature.get(), 1)),
            "target" => PyVal::Float(self.target.get()),
        }))
    }
    fn get_temp(&self, _eventtime: f64) -> Option<(f64, f64)> {
        Some((self.temperature.get(), self.target.get()))
    }
    fn call_method(&self, name: &str, args: &[PyVal]) -> Option<KResult<PyVal>> {
        match name {
            // a drying run is a timed firmware cycle: the TURN_OFF_HEATERS
            // sweep reaches it as set_temp(0), so set_temp only starts it
            "set_temp" => {
                let degrees = args.first().map(|a| super::utils::py_float_of(a).unwrap_or(0.0)).unwrap_or(0.0);
                if degrees > 0.0 {
                    if let Some(u) = self.unit.borrow().upgrade() {
                        return Some(set_dryer(&u, degrees as i64, None, None).map(|_| PyVal::None));
                    }
                }
                Some(Ok(PyVal::None))
            }
            "alter_target" => {
                self.target.set(args.first().map(|a| super::utils::py_float_of(a).unwrap_or(0.0)).unwrap_or(0.0));
                Some(Ok(PyVal::None))
            }
            "check_busy" => Some(Ok(PyVal::Bool(false))),
            _ => None,
        }
    }
}

/// AceHumiditySensor: the enclosure's humidity and temperature
#[derive(Default)]
pub struct AceHumidity {
    pub humidity: Cell<f64>,
    pub temperature: Cell<f64>,
}

impl PrinterObject for AceHumidity {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        Some(PyVal::Dict(kaliru::pydict! {
            "humidity" => PyVal::Float(kaliru::pyfmt::py_round(self.humidity.get(), 1)),
            "temperature" => PyVal::Float(kaliru::pyfmt::py_round(self.temperature.get(), 1)),
        }))
    }
    // TEMPERATURE_WAIT SENSOR= reads any listed sensor through get_temp
    fn get_temp(&self, _eventtime: f64) -> Option<(f64, f64)> {
        Some((self.temperature.get(), 0.0))
    }
}

/// The ACE unit's part (afcACE)
pub struct AceUnit {
    pub protocol: String,
    serial_port: String,
    ace_unit_index: i64,
    monitor_only: bool,
    dist_hub: Option<f64>,
    optional: bool,
    feed_speed: i64,
    retract_speed: i64,
    retract_mode: i64,
    bowden_overshoot: f64,
    hub_clear_mm: f64,
    sensor_approach_mm: f64,
    sensor_approach_speed: i64,
    sync_load_mm: Option<f64>,
    tail_purge_speed: f64,
    purge_park_cmd: Option<String>,
    dryer_temp: i64,
    dryer_duration: i64,
    dryer_fan_speed: i64,
    hub_sensor_name: Option<String>,
    toolhead_sensor_name: Option<String>,
    bypass_runout_distance: f64,
    pub dryer_heater: Rc<AceDryer>,
    pub humidity_sensor: Rc<AceHumidity>,

    pub transport: RefCell<Option<Transport>>,
    slot_map: RefCell<IndexMap<String, i64>>,
    last_prep: RefCell<HashMap<i64, bool>>,
    last_rfid: RefCell<HashMap<i64, (Value, String)>>,
    operation_active: Cell<bool>,
    early_feed: RefCell<Option<(String, bool)>>,
    poll_timer: RefCell<Option<Timer>>,
    log_throttle: RefCell<HashMap<String, f64>>,
    tail_pending: RefCell<Vec<String>>,
    /// Lane whose runout paused the print (no runout lane): a new spool in
    /// it is loaded and purged, and the user only presses resume
    reload_lane: RefCell<Option<String>>,
    assist_slot: Cell<Option<i64>>,
    assist_gen: Cell<Option<u64>>,
    gears_engaged: Cell<bool>,
    /// ace_tail_in_bowden when a lane has no hub (getattr(lane.hub_obj or self))
    tail_in_bowden: Cell<bool>,
}

fn ace(unit: &AfcUnit) -> &AceUnit {
    match &unit.kind {
        UnitKind::Ace(a) => a,
        _ => unreachable!("not an ACE unit"),
    }
}

/// {"index": slot, ...}: an rpc's params in python's dict order
fn params(items: &[(&str, Value)]) -> Option<Map<String, Value>> {
    Some(items.iter().map(|(k, v)| (k.to_string(), v.clone())).collect())
}

fn lane_state_is(lane: &AfcLane, states: &[LaneState]) -> bool {
    states.contains(&lane.status())
}

/// _inherit_setup_defaults: a unit section gets every key it does not set
/// itself from [AFC_ACE]
fn inherit_setup_defaults(config: &ConfigWrapper) -> KResult<()> {
    let section = config.get_name().to_string();
    let mut fc = config.fileconfig.borrow_mut();
    if !fc.has_section("AFC_ACE") {
        return Ok(());
    }
    if !fc.has_option(&section, "protocol") && fc.has_option("AFC_ACE", "ace") {
        let model = fc.get("AFC_ACE", "ace")?.trim().to_lowercase();
        if let Some((_, p)) = ACE_MODELS.iter().find(|(m, _)| *m == model) {
            fc.set(&section, "protocol", p)?;
        }
    }
    let protocol = if fc.has_option(&section, "protocol") { fc.get(&section, "protocol")?.trim().to_string() } else { "ace1".into() };
    let mut per_unit = vec!["ace", "units", "unit_index", "optional"];
    if protocol == "ace1" {
        per_unit.push("serial");
    }
    for option in fc.options("AFC_ACE")? {
        if !per_unit.contains(&option.as_str()) && !fc.has_option(&section, &option) {
            let v = fc.get("AFC_ACE", &option)?;
            fc.set(&section, &option, &v)?;
        }
    }
    Ok(())
}

/// try: register ... except error: pass (the command already exists)
fn ignore_registered<T>(r: KResult<T>) -> KResult<()> {
    match r {
        Err(e) if matches!(e.base(), KError::Config(_)) => Ok(()),
        r => r.map(|_| ()),
    }
}

/// [AFC_ACE]: one unit per ACE, `units: auto` counts them at startup
pub struct AceSetup;

impl PrinterObject for AceSetup {}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let printer = config.get_printer();
    let choices: Vec<(&str, String)> = ACE_MODELS.iter().map(|(k, v)| (*k, v.to_string())).collect();
    let protocol: String = config.getchoice("ace", &choices, "pro")?;
    let units = config.get("units", "auto")?.trim().to_lowercase();
    let serial_port = config.get("serial", "auto")?;
    let count = if units == "auto" {
        let found = tp::count_ace_units(&protocol, &serial_port);
        kaliru::logging::info(&format!("AFC_ACE: {} ACE unit(s) found at startup", found));
        found.max(1)
    } else {
        let count: i64 = units.parse().map_err(|_| KError::config(format!("[AFC_ACE] units must be auto or a number, not {}", kaliru::pyfmt::py_str_repr(&units))))?;
        if count < 1 {
            return Err(KError::config("[AFC_ACE] units must be at least 1"));
        }
        count as usize
    };
    // Every key here is read by the units; read them once here too, or the
    // option check rejects them as unused in [AFC_ACE]
    let options = config.fileconfig.borrow().options(config.get_name())?;
    for option in options {
        config.get(&option, kaliru::configfile::Dflt::Required)?;
    }
    for number in 1..=count {
        let section = format!("AFC_ACE ACE_{}", number);
        {
            let mut fc = config.fileconfig.borrow_mut();
            if !fc.has_section(&section) {
                fc.add_section(&section)?;
            }
            let mut defaults = vec![("unit_index".to_string(), (number - 1).to_string())];
            // A chained unit unplugged at startup is reported, not a failure
            if number > 1 {
                defaults.push(("optional".into(), "True".into()));
            }
            if number == 1 && protocol == "ace1" && fc.has_option("AFC_ACE", "serial") {
                defaults.push(("serial".into(), fc.get("AFC_ACE", "serial")?));
            }
            for (key, value) in defaults {
                if !fc.has_option(&section, &key) {
                    fc.set(&section, &key, &value)?;
                }
            }
        }
        printer.load_object(config, &section, true)?;
    }
    Ok(Rc::new(AceSetup))
}

/// [AFC_ACE <name>]: the unit, then its lanes
pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    inherit_setup_defaults(config)?;
    let printer = config.get_printer();
    let unit = AfcUnit::new(config, |config, settings: &UnitSettings| {
        let _ = settings;
        let protocol: String = config.getchoice("protocol", &[("ace1", "ace1".to_string()), ("ace2", "ace2".to_string())], "ace1")?;
        let serial_port = config.get("serial", "auto")?;
        let ace_unit_index = config.getint("unit_index", 0, Lim::new().min(0))?;
        let monitor_only = config.getboolean("monitor_only", false)?;
        let dist_hub = config.getfloat_opt("dist_hub", Lim::new().min(0.0))?;
        let optional = config.getboolean("optional", false)?;
        let feed_speed = config.getint("feed_speed", 50, Lim::new().min(10))?;
        let retract_speed = config.getint("retract_speed", 75, Lim::new().min(10))?;
        let retract_mode = config.getint("retract_mode", 0, Lim::new())?;
        let bowden_overshoot = config.getfloat("bowden_overshoot", 300.0, Lim::new().min(0.0))?;
        let hub_clear_mm = config.getfloat("hub_clear_mm", 100.0, Lim::new().min(0.0))?;
        let sensor_approach_mm = config.getfloat("sensor_approach_mm", 30.0, Lim::new().min(0.0))?;
        let sensor_approach_speed = config.getint("sensor_approach_speed", 15, Lim::new().min(5))?;
        let sync_load_mm = config.getfloat_opt("sync_load_mm", Lim::new().min(0.0))?;
        let tail_purge_speed = config.getfloat("tail_purge_speed", 10.0, Lim::new().min(1.0))?;
        let purge_park_cmd = config.get_opt("purge_park_cmd")?;
        let dryer_temp = config.getint("dryer_temp", 45, Lim::new())?;
        let dryer_duration = config.getint("dryer_duration", 240, Lim::new())?;
        let dryer_fan_speed = config.getint("dryer_fan_speed", 7000, Lim::new())?;
        let hub_sensor_name = config.get_opt("hub_sensor_name")?;
        let toolhead_sensor_name = config.get_opt("toolhead_sensor_name")?;
        let bypass_runout_distance = config.getfloat("bypass_runout_distance", 770.0, Lim::new().min(0.0))?;
        let kind = AceUnit {
            protocol,
            serial_port,
            ace_unit_index,
            monitor_only,
            dist_hub,
            optional,
            feed_speed,
            retract_speed,
            retract_mode,
            bowden_overshoot,
            hub_clear_mm,
            sensor_approach_mm,
            sensor_approach_speed,
            sync_load_mm,
            tail_purge_speed,
            purge_park_cmd,
            dryer_temp,
            dryer_duration,
            dryer_fan_speed,
            hub_sensor_name,
            toolhead_sensor_name,
            bypass_runout_distance,
            dryer_heater: Rc::new(AceDryer { unit: RefCell::new(Weak::new()), temperature: Cell::new(0.0), target: Cell::new(0.0) }),
            humidity_sensor: Rc::new(AceHumidity::default()),
            transport: RefCell::new(None),
            slot_map: RefCell::new(IndexMap::default()),
            last_prep: RefCell::new(HashMap::new()),
            last_rfid: RefCell::new(HashMap::new()),
            operation_active: Cell::new(false),
            early_feed: RefCell::new(None),
            poll_timer: RefCell::new(None),
            log_throttle: RefCell::new(HashMap::new()),
            tail_pending: RefCell::new(vec![]),
            reload_lane: RefCell::new(None),
            assist_slot: Cell::new(None),
            assist_gen: Cell::new(None),
            gears_engaged: Cell::new(false),
            tail_in_bowden: Cell::new(false),
        };
        Ok((UnitKind::Ace(Box::new(kind)), UNIT_TYPE.to_string()))
    })?;
    let a = ace(&unit);
    unit.stepperless_drive.set(true);
    *a.dryer_heater.unit.borrow_mut() = Rc::downgrade(&unit);

    // The firmware dryer as a heater (+ humidity sensor) for the UIs
    let dryer_name = format!("ace_dryer_{}", unit.name);
    let hum_name = format!("ace_humidity_{}", unit.name);
    let pheaters = printer.lookup_or_load::<PrinterHeaters>(config, "heaters")?;
    let dryer: PObj = a.dryer_heater.clone();
    pheaters.heaters.borrow_mut().insert(dryer_name.clone(), HeaterSlot::Plugin(dryer.clone()));
    pheaters.name_list("available_heaters").unwrap().borrow_mut().push(dryer_name.clone());
    pheaters.name_list("available_sensors").unwrap().borrow_mut().push(hum_name.clone());
    printer.add_object(&dryer_name, dryer)?;
    printer.add_object(&hum_name, a.humidity_sensor.clone())?;
    // Kalico registers SET_HEATER_TEMPERATURE per heater: register ours
    let gcode = unit.afc.gcode.clone();
    let u = unit.clone();
    ignore_registered(gcode.register_mux_command(
        "SET_HEATER_TEMPERATURE",
        "HEATER",
        Some(&dryer_name),
        Rc::new(move |g| set_dryer(&u, g.get_float("TARGET", Some(0.0), Lim::new())? as i64, None, None)),
        Some(&format!("Set {} ACE dryer temperature", unit.name)),
    ))?;
    let n = unit.name.clone();
    let cmds: [(&str, fn(&Rc<AfcUnit>, &GCodeCommand) -> KResult<()>, String); 5] = [
        ("AFC_ACE_DRYER_START", cmd_dryer_start, format!("Start the ACE dryer: AFC_ACE_DRYER_START UNIT={} [TEMP=] [DURATION=] [FAN_SPEED=]", n)),
        ("AFC_ACE_DRYER_STOP", cmd_dryer_stop, "Stop the ACE dryer".into()),
        ("AFC_ACE_STATUS", cmd_status, "Report raw ACE status".into()),
        ("AFC_ACE_STOP", cmd_stop, format!("Stop all ACE motion for a lane: AFC_ACE_STOP UNIT={} LANE=", n)),
        ("AFC_ACE_RESET", cmd_reset, format!("Retract a lane clear of the sensors and park it behind the splitter: AFC_ACE_RESET UNIT={} LANE=", n)),
    ];
    for (cmd, f, help) in cmds {
        let u = unit.clone();
        gcode.register_mux_command(cmd, "UNIT", Some(&n), Rc::new(move |g| f(&u, g)), Some(&help))?;
    }
    // One command each for all units: the first unit registers them
    let u = unit.clone();
    ignore_registered(gcode.register_command("AFC_ACE_BYPASS", Some(Rc::new(move |g| cmd_bypass(&u, g))), false, Some("Feed a spool directly, bypassing the ACE and AFC: AFC_ACE_BYPASS [ENABLE=0|1] (no ENABLE toggles)")))?;
    let u = unit.clone();
    ignore_registered(gcode.register_command("AFC_ACE_PANEL", Some(Rc::new(move |g| cmd_panel(&u, g))), false, Some("Install or remove the optional ACE Pro Mainsail panel: AFC_ACE_PANEL ACTION=INSTALL|REMOVE")))?;
    // Registered before its lanes are built: a lane loads its unit object in
    // its own constructor, and an unregistered unit would be built twice
    let obj: PObj = unit.clone();
    printer.add_object(config.get_name(), obj)?;
    create_lanes(&unit, config)?;
    Ok(unit)
}

/// _create_lanes: this unit's four lanes, numbered by unit_index
fn create_lanes(unit: &Rc<AfcUnit>, config: &ConfigWrapper) -> KResult<()> {
    let a = ace(unit);
    let taken: Vec<String> = {
        let fc = config.fileconfig.borrow();
        fc.sections()
            .into_iter()
            .filter(|s| s.starts_with("AFC_lane ") && fc.has_option(s, "unit"))
            .map(|s| fc.get(&s, "unit").map(|v| v.trim().to_string()))
            .collect::<Result<_, _>>()?
    };
    for slot in 1..=4 {
        let unit_ref = format!("{}:{}", unit.name, slot);
        if taken.contains(&unit_ref) {
            continue;
        }
        let number = a.ace_unit_index * 4 + slot;
        let section = format!("AFC_lane lane{}", number);
        let existed = {
            let mut fc = config.fileconfig.borrow_mut();
            let existed = fc.has_section(&section);
            if !existed {
                fc.add_section(&section)?;
            } else if fc.has_option(&section, "unit") {
                return Err(KError::config(format!(
                    "[{}] belongs to unit {}, but {} (unit_index {}) numbers slot {} {}. Remove its unit line or change unit_index.",
                    section,
                    fc.get(&section, "unit")?,
                    unit.name,
                    a.ace_unit_index,
                    slot,
                    section
                )));
            }
            let mut keys = vec![("unit".to_string(), unit_ref.clone()), ("map".into(), format!("T{}", number - 1)), ("load_to_hub".into(), "False".into())];
            if let Some(d) = a.dist_hub {
                keys.push(("dist_hub".into(), kaliru::pyfmt::py_float_repr(d)));
            }
            for (key, value) in keys {
                if !fc.has_option(&section, &key) {
                    fc.set(&section, &key, &value)?;
                }
            }
            existed
        };
        if !existed {
            unit.printer.load_object(config, &section, true)?;
        }
    }
    Ok(())
}

// ── lifecycle ───────────────────────────────────────────────────────

pub fn unit_handle_connect(unit: &AfcUnit) {
    let model = if ace(unit).protocol == "ace2" { "ACE 2 PRO" } else { "ACE PRO" };
    *unit.logo.borrow_mut() = ams_box_logo(model, 4, &unit.name);
    *unit.logo_error.borrow_mut() = ams_box_logo_error(model, 4, &unit.name);
}

/// AFC_OpenAMS._ams_box_logo: a titled box with one spool bay per slot
pub fn ams_box_logo(title: &str, n_slots: usize, name: &str) -> String {
    let n = n_slots.max(1);
    let mut bay_w = 3;
    while n * bay_w + (n - 1) < title.chars().count() {
        bay_w += 1;
    }
    let inner = n * bay_w + (n - 1);
    let bar = "-".repeat(bay_w);
    let spool = super::utils::py_center("O", bay_w);
    let rows = [
        format!("+{}+", "-".repeat(inner)),
        format!("|{}|", super::utils::py_center(title, inner)),
        format!("+{}+", vec![bar.clone(); n].join("+")),
        format!("|{}|", vec![spool; n].join("|")),
        format!("+{}+", vec![bar; n].join("+")),
    ];
    let body: Vec<String> = "READY".chars().zip(rows.iter()).map(|(l, r)| format!("{}  {}", l, r)).collect();
    format!("<span class=success--text>{}</span>\n   {}\n", body.join("\n"), name)
}

pub fn ams_box_logo_error(title: &str, n_slots: usize, name: &str) -> String {
    let n = n_slots.max(1);
    let mut bay_w = 3;
    while n * bay_w + (n - 1) < title.chars().count() {
        bay_w += 1;
    }
    let inner = (n * bay_w + (n - 1)).max("X ERROR".len());
    let rows = [
        format!("+{}+", "-".repeat(inner)),
        format!("|{}|", super::utils::py_center(title, inner)),
        format!("+{}+", "-".repeat(inner)),
        format!("|{}|", super::utils::py_center("X ERROR", inner)),
        format!("+{}+", "-".repeat(inner)),
    ];
    let body: Vec<String> = "ERROR".chars().zip(rows.iter()).map(|(l, r)| format!("{}  {}", l, r)).collect();
    format!("<span class=error--text>{}</span>\n   {}\n", body.join("\n"), name)
}

/// handle_ready (after the base unit's lane sort)
pub fn unit_handle_ready(unit: &AfcUnit) -> KResult<()> {
    let c = &AFC_ACE_AFCACE_HANDLE_READY;
    let a = ace(unit);
    let me = unit.rc();
    let lanes: Vec<Rc<AfcLane>> = unit.lanes.borrow().values().cloned().collect();
    // AFC_spool registers lane macros in its own connect handler, after the
    // lanes fired their event: register them directly
    for lane in &lanes {
        match unit.afc.spool.register_lane_macros(lane) {
            Err(e) if matches!(e.base(), KError::Config(_)) => {}
            r => r?,
        }
    }
    for lane in &lanes {
        let obj: PObj = lane.clone();
        let _ = unit.printer.add_object(&format!("AFC_stepper {}", lane.name), obj);
        let slot = lane.index - 1;
        if !(0..=3).contains(&slot) {
            unit.afc.logger.error(
                c,
                &format!("AFC_ACE {}: lane {} has unit index {}, expected 1..4 (e.g. 'unit: {}:1') — lane disabled", unit.name, lane.name, lane.index, unit.name),
            );
            continue;
        }
        if hub_sensor_fn(unit, lane).is_some() && toolhead_sensor_fn(unit, lane).is_some() {
            unit.afc.logger.error(c, &format!("AFC_ACE {}: lane {} has both a hub (frame) sensor and a toolhead sensor; use one — lane disabled", unit.name, lane.name));
            continue;
        }
        a.slot_map.borrow_mut().insert(lane.name.clone(), slot);
    }
    let transport = if a.protocol == "ace2" {
        Transport::Two(tp::Ace2Unit::start(&a.serial_port, a.ace_unit_index))
    } else {
        Transport::Pro(tp::AceTransport::start(&a.serial_port, a.ace_unit_index))
    };
    *a.transport.borrow_mut() = Some(transport);
    let u = me.clone();
    unit.printer.register_event_handler("klippy:disconnect", Rc::new(move |_| {
        if let Some(t) = ace(&u).transport.borrow().as_ref() {
            t.stop();
        }
        Ok(())
    }));
    let u = me.clone();
    unit.printer.register_event_handler("klippy:shutdown", Rc::new(move |_| handle_shutdown(&u)));
    let w = Rc::downgrade(&me);
    let timer = unit.afc.reactor.register_timer(Box::new(move |et| w.upgrade().map_or(NEVER, |u| poll_status(&u, et))), unit.afc.reactor.monotonic() + 2.0);
    *a.poll_timer.borrow_mut() = Some(timer);
    Ok(())
}

/// On M112 / an MCU shutdown: halt all ACE motion and stop the dryer
fn handle_shutdown(unit: &AfcUnit) -> KResult<()> {
    let a = ace(unit);
    let t = a.transport.borrow();
    let Some(t) = t.as_ref() else { return Ok(()) };
    let mut slots: Vec<i64> = vec![];
    for s in a.slot_map.borrow().values() {
        if !slots.contains(s) {
            slots.push(*s);
        }
    }
    for slot in slots {
        for method in ["stop_feed_filament", "stop_unwind_filament", "stop_feed_assist"] {
            t.submit(method, params(&[("index", json!(slot))]));
        }
    }
    t.submit("drying_stop", None);
    a.dryer_heater.target.set(0.0);
    Ok(())
}

// ── rpc plumbing (reactor side) ─────────────────────────────────────

fn rpc(unit: &AfcUnit, method: &str, p: Option<Map<String, Value>>, timeout: f64) -> KResult<RpcResult> {
    let a = ace(unit);
    let pending = {
        let t = a.transport.borrow();
        let Some(t) = t.as_ref() else { return Ok(RpcResult::err("ACE transport not started")) };
        // After M112 / an MCU shutdown only stops and status get out
        if unit.printer.is_shutdown() && !STOP_METHODS.contains(&method) && method != "get_status" {
            return Ok(RpcResult::err("printer is shut down"));
        }
        let mut timeout = timeout;
        if t.is_ace2() && ACE2_MOTION_METHODS.contains(&method) {
            timeout = timeout.max(ACE2_FORBIDDEN_WAIT_S + RPC_REPLY_S + 2.0);
        }
        (t.submit(method, p), timeout)
    };
    let (pending, timeout) = pending;
    let reactor = &unit.afc.reactor;
    let deadline = reactor.monotonic() + timeout;
    while !pending.done.load(std::sync::atomic::Ordering::SeqCst) {
        if reactor.monotonic() > deadline {
            // Abandon it so a not-yet-sent motion command cannot fire later
            pending.cancelled.store(true, std::sync::atomic::Ordering::SeqCst);
            let sent = pending.result.lock().unwrap().sent;
            return Ok(RpcResult { sent, ..RpcResult::err(format!("timeout waiting for ACE rpc {}", method)) });
        }
        reactor.pause(reactor.monotonic() + 0.05)?;
    }
    let r = pending.result.lock().unwrap().clone();
    Ok(r)
}

fn cached_status(unit: &AfcUnit) -> (Option<Value>, f64) {
    match ace(unit).transport.borrow().as_ref() {
        Some(t) => t.state().cached_status(),
        None => (None, 0.0),
    }
}

/// _get_status: from the cache when fresh enough, else via rpc
fn get_status(unit: &AfcUnit, max_age: f64, newer_than: f64, strict: bool) -> KResult<Option<Value>> {
    let (status, when) = cached_status(unit);
    if status.is_some() && max_age > 0.0 && when > newer_than && monotonic() - when <= max_age {
        return Ok(status);
    }
    let res = rpc(unit, "get_status", None, 8.0)?;
    if res.ok {
        if let Some(Value::Object(m)) = res.result() {
            return Ok(Some(Value::Object(m.clone())));
        }
    }
    if strict || (status.is_some() && when <= newer_than) {
        return Ok(None);
    }
    Ok(status)
}

fn slot_info(slot: i64, status: Option<&Value>) -> Map<String, Value> {
    match status.and_then(|s| s.get("slots")) {
        Some(Value::Array(slots)) if slot >= 0 && (slot as usize) < slots.len() => match &slots[slot as usize] {
            Value::Object(m) => m.clone(),
            _ => Map::new(),
        },
        _ => Map::new(),
    }
}

fn slot_present(slot: i64, status: Option<&Value>) -> bool {
    let s = slot_info(slot, status).get("status").map(tp::py_str).unwrap_or_default().to_lowercase();
    !s.is_empty() && s != "empty"
}

fn motion_active(slot: i64, status: &Value) -> bool {
    let info = slot_info(slot, Some(status));
    [status.get("status"), status.get("action"), info.get("status")]
        .iter()
        .any(|v| v.filter(|v| truthy(v)).map(|v| tp::py_str(v).to_lowercase()).is_some_and(|s| ACTIVE_STATES.contains(&s.as_str())))
}

// ── ACE motion primitives ───────────────────────────────────────────

fn motion_blocked(unit: &AfcUnit, what: &str) -> bool {
    if ace(unit).monitor_only {
        unit.afc.logger.error(
            &AFC_ACE_AFCACE__MOTION_BLOCKED,
            &format!("{} is monitor_only — refusing {}. Remove monitor_only from its config once its filament path feeds this printer.", unit.name, what),
        );
        return true;
    }
    false
}

/// _motion_rpc: a motion command, tolerating a lost reply frame
fn motion_rpc(unit: &AfcUnit, slot: i64, method: &str, p: Map<String, Value>, done_fn: Option<&SensorFn>) -> KResult<RpcResult> {
    let issued = unit.afc.reactor.monotonic();
    let res = rpc(unit, method, Some(p.clone()), 8.0)?;
    if res.ok || !res.sent {
        return Ok(res);
    }
    unit.afc.logger.info(&AFC_ACE_AFCACE__MOTION_RPC, &format!("{} reply lost/garbled ({}); verifying via status", method, res.error_str()));
    let Some(status) = get_status(unit, 0.0, 0.0, true)? else {
        return Ok(RpcResult::err(format!("{} reply lost and motion state indeterminate", method)));
    };
    let done = match done_fn {
        Some(f) => f()?,
        None => false,
    };
    if motion_active(slot, &status) || done {
        return Ok(RpcResult { ok: true, assumed_started: true, ..Default::default() });
    }
    let length = p.get("length").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let speed = p.get("speed").and_then(|v| v.as_f64()).unwrap_or(1.0);
    let move_s = length / speed.max(1.0);
    if done_fn.is_some() || unit.afc.reactor.monotonic() - issued < move_s {
        return rpc(unit, method, Some(p), 8.0);
    }
    Ok(RpcResult::err(format!("{} reply lost and the move may already have run", method)))
}

fn stop_motion(unit: &AfcUnit, slot: i64, method: &str) -> KResult<RpcResult> {
    let res = rpc(unit, method, params(&[("index", json!(slot))]), 8.0)?;
    // The ACE occasionally eats the reply to a stop; the stop still lands
    if !res.ok && res.sent && res.error_str().to_lowercase().contains("timeout") {
        unit.afc.logger.debug(&AFC_ACE_AFCACE__STOP_MOTION, &format!("{} reply timed out; assuming success", method));
        return Ok(RpcResult { ok: true, assumed_success: true, ..Default::default() });
    }
    Ok(res)
}

/// _wait_motion_idle: two consecutive fresh idle reads
fn wait_motion_idle(unit: &AfcUnit, slot: i64, timeout: f64, settle: f64) -> KResult<bool> {
    let reactor = unit.afc.reactor.clone();
    reactor.pause(reactor.monotonic() + settle)?;
    let deadline = reactor.monotonic() + timeout;
    let mut idle_reads = 0;
    while reactor.monotonic() < deadline {
        match get_status(unit, 0.0, 0.0, true)? {
            Some(status) if !motion_active(slot, &status) => {
                idle_reads += 1;
                if idle_reads >= 2 {
                    return Ok(true);
                }
            }
            _ => idle_reads = 0,
        }
        reactor.pause(reactor.monotonic() + 0.1)?;
    }
    Ok(false)
}

/// _halt: stop a move and confirm the ACE actually went idle
fn halt(unit: &AfcUnit, slot: i64, method: &str) -> KResult<(bool, String)> {
    for _ in 0..2 {
        let stop = stop_motion(unit, slot, method)?;
        if stop.ok && wait_motion_idle(unit, slot, 3.0, 0.1)? {
            return Ok((true, String::new()));
        }
    }
    Ok((false, format!("{}: the ACE did not stop", method)))
}

fn feed_params(slot: i64, length: f64, speed: f64) -> Map<String, Value> {
    params(&[("index", json!(slot)), ("length", json!(length as i64)), ("speed", json!(speed as i64))]).unwrap()
}

/// _feed_until: an ACE feed stopped when the sensor goes True
fn feed_until(unit: &AfcUnit, slot: i64, sensor_fn: Option<&SensorFn>, length: f64, speed: f64, extend: bool) -> KResult<(bool, String)> {
    let reactor = unit.afc.reactor.clone();
    let res = motion_rpc(unit, slot, "feed_filament", feed_params(slot, length, speed), sensor_fn)?;
    if !res.ok {
        return Ok((false, format!("feed_filament failed: {}", res.error_str())));
    }
    let Some(sensor_fn) = sensor_fn else {
        if !wait_motion_idle(unit, slot, length / speed.max(1.0) + 30.0, SPIN_UP_S)? {
            stop_motion(unit, slot, "stop_feed_filament")?;
            return Ok((false, "timeout waiting for blind feed to finish".into()));
        }
        return Ok((true, String::new()));
    };
    let budget = length * 2.0 + 500.0;
    let mut fed = length;
    // Only a status read taken after spin-up can say the feed ran out
    let mut settled = monotonic() + SPIN_UP_S;
    let deadline = reactor.monotonic() + budget / speed.max(1.0) + 60.0;
    while reactor.monotonic() < deadline {
        if sensor_fn()? {
            let (ok, msg) = halt(unit, slot, "stop_feed_filament")?;
            if !ok {
                return Ok((false, msg));
            }
            // Confirm the sensor held through the stop wind-down
            reactor.pause(reactor.monotonic() + 0.2)?;
            if !sensor_fn()? {
                return Ok((false, "sensor lost after feed stop".into()));
            }
            return Ok((true, String::new()));
        }
        // Commanded length exhausted without the sensor: extend the feed
        if monotonic() >= settled {
            let status = get_status(unit, HEARTBEAT_S * 2.0, settled, false)?;
            if let Some(status) = status.filter(|s| !motion_active(slot, s)) {
                let _ = status;
                if !extend {
                    return Ok((false, NOT_REACHED.into()));
                }
                if fed >= budget {
                    break;
                }
                let step = (budget - fed).min(200.0);
                let res = motion_rpc(unit, slot, "feed_filament", feed_params(slot, step, speed), Some(sensor_fn))?;
                if !res.ok {
                    return Ok((false, format!("feed extension failed: {}", res.error_str())));
                }
                fed += step;
                settled = monotonic() + SPIN_UP_S;
            }
        }
        reactor.pause(reactor.monotonic() + 0.05)?;
    }
    stop_motion(unit, slot, "stop_feed_filament")?;
    Ok((false, format!("sensor did not trigger within {:.0}mm (expected ~{:.0}mm) — check the filament path", fed, length)))
}

/// _approach_sensor: slowly for the last sensor_approach_mm before the gears
fn approach_sensor(unit: &AfcUnit, slot: i64, sensor_fn: &SensorFn, near: f64, far: f64) -> KResult<(bool, String)> {
    let a = ace(unit);
    if near > 0.0 {
        let (ok, msg) = feed_until(unit, slot, Some(sensor_fn), near, a.sensor_approach_speed as f64, false)?;
        if msg != NOT_REACHED {
            return Ok((ok, msg));
        }
    }
    let mut fast = far - near - a.sensor_approach_mm;
    if a.protocol == "ace2" {
        // A fresh spool's tip may be up to a preload past the slot
        fast -= ACE2_PRELOAD_MAX_MM;
    }
    if fast > 0.0 && !sensor_fn()? {
        let (ok, msg) = feed_until(unit, slot, Some(sensor_fn), fast, a.feed_speed as f64, false)?;
        if ok {
            unit.afc.logger.warning(
                &AFC_ACE_AFCACE__APPROACH_SENSOR,
                &format!("{}: reached the toolhead sensor at full speed: the tip was further along than the park, or dist_hub is longer than the real path", unit.name),
            );
        }
        if msg != NOT_REACHED {
            return Ok((ok, msg));
        }
    }
    if sensor_fn()? {
        return Ok((true, String::new()));
    }
    let rest = (far - near - fast.max(0.0)).max(a.sensor_approach_mm);
    feed_until(unit, slot, Some(sensor_fn), rest, a.sensor_approach_speed as f64, true)
}

/// _retract_until_clear: one unwind stopped when the sensor clears, then a park
fn retract_until_clear(unit: &AfcUnit, slot: i64, sensor_name: &str, sensor_fn: &SensorFn, cap: f64, park: f64) -> KResult<(bool, String)> {
    let a = ace(unit);
    let reactor = unit.afc.reactor.clone();
    let f = sensor_fn.clone();
    let cleared: SensorFn = Rc::new(move || Ok(!f()?));
    let p = params(&[("index", json!(slot)), ("length", json!(cap as i64)), ("speed", json!(a.retract_speed)), ("mode", json!(a.retract_mode))]).unwrap();
    let res = motion_rpc(unit, slot, "unwind_filament", p, Some(&cleared))?;
    if !res.ok {
        return Ok((false, format!("unwind_filament failed: {}", res.error_str())));
    }
    if res.assumed_started && !sensor_fn()? {
        // The reply was lost and the sensor cleared unwatched: position unknown
        let (ok, msg) = halt(unit, slot, "stop_unwind_filament")?;
        if !ok {
            return Ok((false, msg));
        }
        return Ok((false, format!("{} sensor cleared while the unwind reply was lost: filament position unknown, check the merger before the next load", sensor_name)));
    }
    let deadline = reactor.monotonic() + cap / (a.retract_speed as f64).max(1.0) + 30.0;
    let mut cleared_ok = false;
    while reactor.monotonic() < deadline {
        if !sensor_fn()? {
            cleared_ok = true;
            break;
        }
        reactor.pause(reactor.monotonic() + 0.05)?;
    }
    if !cleared_ok {
        stop_motion(unit, slot, "stop_unwind_filament")?;
        return Ok((false, format!("{} sensor did not clear during retract", sensor_name)));
    }
    let (ok, msg) = halt(unit, slot, "stop_unwind_filament")?;
    if !ok {
        return Ok((false, msg));
    }
    // The tip is at the sensor: everything past here is measured from it
    if park > 0.0 {
        return retract(unit, slot, park, a.retract_speed as f64, true);
    }
    Ok((true, String::new()))
}

fn retract(unit: &AfcUnit, slot: i64, length: f64, speed: f64, wait: bool) -> KResult<(bool, String)> {
    let a = ace(unit);
    let p = params(&[("index", json!(slot)), ("length", json!(length as i64)), ("speed", json!(speed as i64)), ("mode", json!(a.retract_mode))]).unwrap();
    let res = motion_rpc(unit, slot, "unwind_filament", p, None)?;
    if !res.ok {
        return Ok((false, format!("unwind_filament failed: {}", res.error_str())));
    }
    if wait && !wait_motion_idle(unit, slot, length / speed.max(1.0) + 30.0, SPIN_UP_S)? {
        stop_motion(unit, slot, "stop_unwind_filament")?;
        return Ok((false, "timeout waiting for retract to finish".into()));
    }
    Ok((true, String::new()))
}

/// _sync_load: ACE feed chunks matched to extruder moves into the gears
fn sync_load(unit: &AfcUnit, slot: i64, cur_extruder: &AfcExtruder, length: f64, chunk: f64) -> KResult<(bool, String)> {
    let a = ace(unit);
    let reactor = unit.afc.reactor.clone();
    a.gears_engaged.set(true);
    let mut fed = 0.0;
    let speed = cur_extruder.tool_load_speed.get().max(1.0);
    while fed < length {
        let step = chunk.min(length - fed);
        // The ACE takes whole mm; a sub-mm remainder is the extruder's alone
        let ace_mm = kaliru::pyfmt::py_round(step, 0) as i64;
        if ace_mm > 0 {
            let res = motion_rpc(unit, slot, "feed_filament", params(&[("index", json!(slot)), ("length", json!(ace_mm)), ("speed", json!(speed as i64))]).unwrap(), None)?;
            if !res.ok {
                return Ok((false, format!("sync load feed failed: {}", res.error_str())));
            }
        }
        unit.afc.move_e_pos(step, speed, "sync load", true)?;
        if ace_mm > 0 && !wait_motion_idle(unit, slot, step / speed + 5.0, SPIN_UP_S)? {
            stop_motion(unit, slot, "stop_feed_filament")?;
            return Ok((false, "timeout waiting for sync load chunk to finish".into()));
        }
        reactor.pause(reactor.monotonic() + 0.25)?;
        fed += step;
    }
    Ok((true, String::new()))
}

/// _set_assist: print-time feed assist on/off (re-armed after USB resets)
fn set_assist(unit: &AfcUnit, slot: i64, enable: bool) -> KResult<()> {
    let a = ace(unit);
    if enable {
        a.assist_slot.set(Some(slot));
        arm_assist(unit)
    } else {
        a.assist_slot.set(None);
        stop_motion(unit, slot, "stop_feed_assist").map(|_| ())
    }
}

fn assist_after_load(unit: &AfcUnit, slot: i64) -> KResult<()> {
    let a = ace(unit);
    if a.protocol == "ace2" {
        a.assist_slot.set(Some(slot));
        a.assist_gen.set(None);
        Ok(())
    } else {
        set_assist(unit, slot, true)
    }
}

fn transport_generation(unit: &AfcUnit) -> Option<u64> {
    ace(unit).transport.borrow().as_ref().map(|t| t.state().generation())
}

fn assist_needs_arming(unit: &AfcUnit, status: &Value) -> bool {
    let a = ace(unit);
    let Some(slot) = a.assist_slot.get() else { return false };
    if a.monitor_only {
        return false;
    }
    if a.protocol != "ace2" {
        return a.assist_gen.get() != transport_generation(unit);
    }
    let dropped = slot_info(slot, Some(status)).get("ace2_state").and_then(|v| v.as_u64()) == Some(ACE2_ASSIST_ERROR);
    (dropped || a.assist_gen.get() != transport_generation(unit)) && extruding(unit)
}

fn extruding(unit: &AfcUnit) -> bool {
    let Some(report) = unit.printer.lookup_object_opt("motion_report") else { return false };
    match report.get_status(unit.afc.reactor.monotonic()) {
        Some(PyVal::Dict(d)) => d.get("live_extruder_velocity").and_then(|v| super::utils::py_float_of(v).ok()).is_some_and(|v| v > 0.0),
        _ => false,
    }
}

fn arm_assist(unit: &AfcUnit) -> KResult<()> {
    let a = ace(unit);
    let generation = transport_generation(unit);
    let slot = a.assist_slot.get();
    let res = rpc(unit, "start_feed_assist", params(&[("index", slot.map_or(Value::Null, |s| json!(s)))]), 8.0)?;
    if res.ok {
        a.assist_gen.set(generation);
    } else {
        log_once(
            unit,
            "assist",
            &format!(
                "{}: feed assist did not start ({}) — the extruder is pulling the spool unaided, expect under-extrusion. Retrying on each status poll.",
                unit.name,
                res.error_str()
            ),
            60.0,
        );
    }
    Ok(())
}

// ── sensors ─────────────────────────────────────────────────────────

fn named_sensor_fn(unit: &AfcUnit, sensor_name: &str) -> Option<SensorFn> {
    let sensor = unit.printer.lookup_object_opt(&format!("filament_switch_sensor {}", sensor_name)).and_then(kaliru::printer::downcast_rc::<SwitchSensor>);
    match sensor {
        None => {
            unit.afc.logger.error(&AFC_ACE_AFCACE__NAMED_SENSOR_FN, &format!("{}: filament_switch_sensor '{}' not found in config", unit.name, sensor_name));
            None
        }
        Some(s) => Some(Rc::new(move || Ok(s.runout_helper.filament_present.get()))),
    }
}

fn path_sensor(unit: &AfcUnit, lane: &AfcLane) -> Option<PathSensor> {
    if let Some(f) = hub_sensor_fn(unit, lane) {
        return Some(("hub", f));
    }
    toolhead_sensor_fn(unit, lane).map(|f| ("tool_start", f))
}

fn hub_sensor_fn(unit: &AfcUnit, lane: &AfcLane) -> Option<SensorFn> {
    if let Some(hub) = lane.hub_obj_opt().filter(|h| !h.is_virtual_pin()) {
        return Some(Rc::new(move || Ok(hub.state())));
    }
    ace(unit).hub_sensor_name.as_deref().and_then(|n| named_sensor_fn(unit, n))
}

fn afc_watches(lane: &AfcLane, sensor: &str) -> bool {
    if sensor == "tool_start" {
        return lane.extruder_obj_opt().is_some_and(|e| e.enable_runout.get() && e.tool_start.as_deref().is_some_and(|t| !t.is_empty()));
    }
    lane.hub_obj_opt().is_some_and(|h| !h.is_virtual_pin() && h.enable_runout)
}

fn toolhead_sensor_fn(unit: &AfcUnit, lane: &AfcLane) -> Option<SensorFn> {
    if let Some(ext) = lane.extruder_obj_opt() {
        if ext.tool_start.as_deref().is_some_and(|t| !t.is_empty() && t != "buffer") {
            let l = lane.rc();
            return Some(Rc::new(move || l.get_toolhead_pre_sensor_state()));
        }
    }
    ace(unit).toolhead_sensor_name.as_deref().and_then(|n| named_sensor_fn(unit, n))
}

// ── bypass ──────────────────────────────────────────────────────────

fn bypass_filament_present(unit: &AfcUnit) -> KResult<bool> {
    let names: Vec<String> = ace(unit).slot_map.borrow().keys().cloned().collect();
    for name in names {
        let Some(lane) = unit.lanes.borrow().get(&name).cloned() else { continue };
        if let Some((_, f)) = path_sensor(unit, &lane) {
            return f();
        }
    }
    Ok(false)
}

thread_local! {
    /// helper._afc_ace_saved: what the bypass replaced on a frame sensor's
    /// runout helper, (runout_gcode, runout_distance), by helper
    static BYPASS_SAVED: RefCell<HashMap<usize, (Option<GcodeSlot>, f64)>> = RefCell::new(HashMap::new());
}

/// _sync_bypass: follow AFC's bypass switch on the path sensor's runout
fn sync_bypass(unit: &AfcUnit) -> KResult<()> {
    let c = &AFC_ACE_AFCACE__SYNC_BYPASS;
    let a = ace(unit);
    let bypass = unit.afc.get_bypass_state();
    if let Some(name) = &a.hub_sensor_name {
        let Some(sensor) = unit.printer.lookup_object_opt(&format!("filament_switch_sensor {}", name)).and_then(kaliru::printer::downcast_rc::<SwitchSensor>) else {
            return Ok(());
        };
        let helper: Rc<RunoutHelper> = sensor.runout_helper.clone();
        let key = Rc::as_ptr(&helper) as usize;
        let saved = BYPASS_SAVED.with(|b| b.borrow().contains_key(&key));
        if bypass && !saved {
            let old = helper.runout_gcode.borrow_mut().take();
            BYPASS_SAVED.with(|b| b.borrow_mut().insert(key, (old, helper.runout_distance.get())));
            let h = Rc::downgrade(&helper);
            *helper.runout_gcode.borrow_mut() = Some(GcodeSlot::Render(Rc::new(move || {
                // Re-checked when the runout distance is used up
                if h.upgrade().is_some_and(|h| h.filament_present.get()) {
                    return Ok(String::new());
                }
                Ok("RESPOND MSG=\"Bypass spool ran out: load a new one, then RESUME\"\nPAUSE".into())
            })));
            helper.runout_distance.set(a.bypass_runout_distance);
            helper.sensor_enabled.set(true);
            unit.afc.logger.info(c, "Bypass on: frame sensor runout pause restored");
        } else if !bypass && saved {
            let (g, d) = BYPASS_SAVED.with(|b| b.borrow_mut().remove(&key)).unwrap();
            *helper.runout_gcode.borrow_mut() = g;
            helper.runout_distance.set(d);
            // muted again, as _afc_ace_mute_sensor_gates leaves it
            helper.sensor_enabled.set(false);
        }
        return Ok(());
    }
    let names: Vec<String> = a.slot_map.borrow().keys().cloned().collect();
    for name in names {
        let Some(lane) = unit.lanes.borrow().get(&name).cloned() else { continue };
        let Some(ext) = lane.extruder_obj_opt() else { continue };
        let Some(fila) = ext.fila_tool_start.borrow().clone() else { continue };
        let saved = ext.afc_ace_saved.get();
        if bypass && saved.is_none() {
            ext.afc_ace_saved.set(Some((ext.enable_runout.get(), ext.enable_runout_in_bypass.get(), fila.runout_helper.sensor_enabled.get())));
            ext.enable_runout.set(true);
            ext.enable_runout_in_bypass.set(true);
            fila.runout_helper.sensor_enabled.set(true);
            unit.afc.logger.info(c, "Bypass on: toolhead sensor runout pause on");
        } else if let (false, Some((r, rb, se))) = (bypass, saved) {
            ext.enable_runout.set(r);
            ext.enable_runout_in_bypass.set(rb);
            fila.runout_helper.sensor_enabled.set(se);
            ext.afc_ace_saved.set(None);
        }
    }
    Ok(())
}

fn cmd_bypass(unit: &Rc<AfcUnit>, gcmd: &GCodeCommand) -> KResult<()> {
    let afc = &unit.afc;
    let bypass = afc.get_bypass_state();
    let enable = gcmd.get_int("ENABLE", Some(!bypass as i64), Lim::new().min(0).max(1))?;
    let bypass_name = afc.bypass.borrow().as_ref().map(|b| b.name.clone()).unwrap_or_default();
    if !bypass_name.contains("virtual") {
        return Err(gcmd.error("A hardware [filament_switch_sensor bypass] is configured: it decides bypass, not this command"));
    }
    if (enable != 0) == bypass {
        gcmd.respond_info(&format!("Bypass already {}", if bypass { "on" } else { "off" }), true);
        return Ok(());
    }
    if afc.function.in_print()? {
        return Err(gcmd.error("Not during a print"));
    }
    if enable != 0 {
        if let Some(cur) = afc.current() {
            // AFC refuses bypass while a lane is loaded
            gcmd.respond_info(&format!("Unloading {} first", cur), true);
            afc.gcode.run_script_from_command("TOOL_UNLOAD")?;
            if afc.current().is_some() {
                return Err(gcmd.error("Unload failed, bypass stays off"));
            }
        }
    } else if bypass_filament_present(unit)? {
        return Err(gcmd.error("The bypass spool is still in the path sensor: UNLOAD_FILAMENT, pull the filament out, then turn bypass off"));
    }
    afc.gcode.run_script_from_command(&format!("SET_FILAMENT_SENSOR SENSOR=virtual_bypass ENABLE={}", enable))?;
    sync_bypass(unit)?;
    if enable != 0 {
        gcmd.respond_info(
            "Bypass on: AFC will not load or change lanes. Feed the spool through the path sensor to the extruder, heat the nozzle and extrude until it flows. To go back: UNLOAD_FILAMENT, pull the filament out, AFC_ACE_BYPASS ENABLE=0.",
            true,
        );
    } else {
        gcmd.respond_info("Bypass off: the ACE lanes are back in use", true);
    }
    Ok(())
}

// ── status polling / lane sync ──────────────────────────────────────

/// _log_once: the first occurrence, then at most one per `every` seconds
fn log_once(unit: &AfcUnit, key: &str, message: &str, every: f64) {
    let a = ace(unit);
    let now = monotonic();
    if now - a.log_throttle.borrow().get(key).copied().unwrap_or(0.0) < every {
        return;
    }
    a.log_throttle.borrow_mut().insert(key.to_string(), now);
    unit.afc.logger.error(&AFC_ACE_AFCACE__LOG_ONCE, message);
}

/// _poll_status: the timer; nothing escapes
fn poll_status(unit: &AfcUnit, eventtime: f64) -> f64 {
    match poll_status_inner(unit, eventtime) {
        Ok(t) => t,
        Err(e) => {
            log_once(unit, "poll", &format!("AFC_ACE {}: status poll failed, continuing:\n{}\n", unit.name, kaliru::logging::format_traceback(&e)), 60.0);
            eventtime + 2.0
        }
    }
}

fn poll_status_inner(unit: &AfcUnit, eventtime: f64) -> KResult<f64> {
    let a = ace(unit);
    let notes = a.transport.borrow().as_ref().map(|t| t.state().drain_notes());
    // the serial thread cannot log for itself
    for note in notes.unwrap_or_default() {
        unit.afc.logger.info(&AFC_ACE_AFCACE__POLL_STATUS_INNER, &note);
    }
    sync_bypass(unit)?;
    if a.operation_active.get() || a.transport.borrow().is_none() {
        return Ok(eventtime + 2.0);
    }
    if a.reload_lane.borrow().is_some() && !unit.afc.function.is_paused()? {
        // Resumed (or cancelled) without a new spool in that lane
        *a.reload_lane.borrow_mut() = None;
    }
    // Hub-path occupancy is AFC's own state: derive it from tool_loaded
    for (name, lane) in unit.lanes.borrow().iter() {
        if a.slot_map.borrow().contains_key(name) {
            lane.load_state_raw.set(lane.tool_loaded.get());
        }
    }
    let (mut status, mut when) = cached_status(unit);
    if status.is_none() || monotonic() - when > 10.0 {
        // No word from this ACE: its lanes are not loadable
        mark_lanes_unloadable(unit);
        return Ok(eventtime + 2.0);
    }
    if assist_needs_arming(unit, status.as_ref().unwrap()) {
        arm_assist(unit)?;
        // The rpc paused this timer: an operation may have started
        if a.operation_active.get() {
            return Ok(eventtime + 2.0);
        }
        (status, when) = cached_status(unit);
        let _ = when;
        if status.is_none() {
            return Ok(eventtime + 2.0);
        }
    }
    let status = status.unwrap();
    // The dryer heater + humidity objects for the UIs
    let num = |v: Option<&Value>| -> Option<f64> {
        match v {
            Some(Value::Number(n)) => n.as_f64(),
            Some(Value::String(s)) => s.trim().parse().ok(),
            Some(Value::Bool(b)) => Some(*b as i64 as f64),
            _ => None,
        }
    };
    let temp_v = status.get("temp").filter(|v| truthy(v));
    if let Some(t) = temp_v.map_or(Some(0.0), |v| num(Some(v))) {
        a.dryer_heater.temperature.set(t);
        a.humidity_sensor.temperature.set(t);
        let dryer = status.get("dryer_status").filter(|v| truthy(v)).or_else(|| status.get("dryer").filter(|v| truthy(v)));
        let dryer = match dryer {
            Some(Value::Object(m)) => m.clone(),
            _ => Map::new(),
        };
        let ds = dryer.get("status").map(tp::py_str).unwrap_or_default().to_lowercase();
        let mut ok = true;
        if ["stop", "stopped", ""].contains(&ds.as_str()) {
            a.dryer_heater.target.set(0.0);
        } else {
            match dryer.get("target_temp").filter(|v| truthy(v)).map_or(Some(0.0), |v| num(Some(v))) {
                Some(t) => a.dryer_heater.target.set(t),
                None => ok = false,
            }
        }
        if ok {
            if let Some(h) = status.get("humidity").filter(|v| !v.is_null()) {
                if let Some(h) = num(Some(h)) {
                    a.humidity_sensor.humidity.set(h);
                }
            }
        }
    }
    let lanes: Vec<(String, Rc<AfcLane>)> = unit.lanes.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect();
    for (name, lane) in lanes {
        // A lane's runout handling can pause this timer in an rpc
        if a.operation_active.get() {
            break;
        }
        let Some(slot) = a.slot_map.borrow().get(&name).copied() else { continue };
        if let Err(e) = sync_lane(unit, &name, &lane, slot, &status, eventtime) {
            log_once(unit, &format!("sync:{}", name), &format!("AFC_ACE {}: syncing {} failed, continuing:\n{}\n", unit.name, name, kaliru::logging::format_traceback(&e)), 60.0);
        }
    }
    Ok(eventtime + 1.5)
}

fn mark_lanes_unloadable(unit: &AfcUnit) {
    let a = ace(unit);
    for (name, lane) in unit.lanes.borrow().iter() {
        if a.slot_map.borrow().contains_key(name) && lane.loaded_to_hub.get() {
            lane.loaded_to_hub.set(false);
        }
    }
}

fn tail_pending_has(unit: &AfcUnit, name: &str) -> bool {
    ace(unit).tail_pending.borrow().iter().any(|n| n == name)
}

fn tail_pending_discard(unit: &AfcUnit, name: &str) {
    ace(unit).tail_pending.borrow_mut().retain(|n| n != name);
}

fn sync_lane(unit: &AfcUnit, lane_name: &str, lane: &Rc<AfcLane>, slot: i64, status: &Value, eventtime: f64) -> KResult<()> {
    let c = &AFC_ACE_AFCACE__SYNC_LANE;
    let a = ace(unit);
    if !slot_info(slot, Some(status)).contains_key("status") {
        // No data for this slot: unknown, not empty
        return Ok(());
    }
    let present = slot_present(slot, Some(status));
    lane.prep_state.set(present);
    lane.loaded_to_hub.set(present && !a.monitor_only);
    let prev = a.last_prep.borrow().get(&slot).copied();
    if let Some(prev) = prev.filter(|p| *p != present) {
        let _ = prev;
        if present {
            tail_pending_discard(unit, lane_name);
            lane.handle_load_runout(eventtime, true)?;
            seed_spool_weight(unit, lane);
            apply_rfid(lane, &slot_info(slot, Some(status)), true)?;
            publish_lane(unit, lane);
            if a.reload_lane.borrow().as_deref() == Some(lane_name) {
                *a.reload_lane.borrow_mut() = None;
                let loaded = lane.extruder_obj()?.lane_loaded.borrow().clone();
                if unit.afc.function.is_paused()? && loaded.as_deref().is_none_or(|l| l == lane_name) {
                    if loaded.is_some() {
                        // The pause left the spent spool booked in
                        // (unload_on_runout off): book it out as an unload
                        // of a spent spool does
                        tail_unload(unit, lane, slot)?;
                    }
                    reload(unit, lane_name);
                }
            }
        } else if lane.tool_loaded.get() {
            // Spool ran out while loaded: print out the tail first
            unit.afc.logger.info(c, &format!("{}: {} spool ran out — printing out the tail, endless-spool swap fires when the hub sensor clears", unit.name, lane_name));
            set_assist(unit, slot, false)?;
            if !tail_pending_has(unit, lane_name) {
                a.tail_pending.borrow_mut().push(lane_name.to_string());
            }
        } else {
            // Spool removed from an idle slot
            lane.handle_load_runout(eventtime, false)?;
        }
    }
    a.last_prep.borrow_mut().insert(slot, present);

    // A tag can be read after the spool went in: apply it whenever it
    // changes; the first sighting only records
    let info = slot_info(slot, Some(status));
    let tag = (info.get("type").cloned().unwrap_or(Value::Null), tp::py_str(info.get("color").unwrap_or(&Value::Null)));
    let seen = a.last_rfid.borrow().get(&slot).cloned();
    a.last_rfid.borrow_mut().insert(slot, tag.clone());
    if present && seen.as_ref().is_some_and(|s| *s != tag) && info.get("type").is_some_and(truthy) {
        apply_rfid(lane, &info, true)?;
        publish_lane(unit, lane);
    }

    // Broken filament: spool still in its slot, first path sensor empty
    let sensor = path_sensor(unit, lane);
    if present && lane.tool_loaded.get() && !tail_pending_has(unit, lane_name) {
        if let Some((kind, f)) = &sensor {
            if !f()? && !afc_watches(lane, kind) {
                lane.handle_toolhead_runout(kind)?;
            }
        }
    }

    // Endless-spool stage 2: the tail end passed the path sensor
    if tail_pending_has(unit, lane_name) {
        let passed = match &sensor {
            None => true,
            Some((_, f)) => !f()?,
        };
        if passed {
            tail_pending_discard(unit, lane_name);
            if sensor.as_ref().is_some_and(|(k, _)| *k == "hub") {
                set_tail_in_bowden(unit, lane, true);
            }
            unit.afc.logger.info(c, &format!("{}: {} tail passed the {} — starting swap", unit.name, lane_name, sensor.as_ref().map_or("path", |(k, _)| *k)));
            lane.handle_load_runout(eventtime, false)?;
            // A runout that paused (no runout lane): a new spool in this
            // lane is loaded (see reload) while the pause lasts
            if lane.runout_lane.borrow().is_none() && unit.afc.function.is_paused()? {
                *a.reload_lane.borrow_mut() = Some(lane_name.to_string());
            }
        }
    }
    Ok(())
}

/// _reload: load and purge a spool put into the lane whose runout paused
/// the print, off the status timer (TOOL_LOAD runs for minutes and polls the
/// ACE itself); the head goes back to the saved print position on resume
fn reload(unit: &AfcUnit, lane_name: &str) {
    unit.afc.logger.info(&AFC_ACE_AFCACE__RELOAD, &format!("{}: new spool in {} — loading it; press resume when the purge is done", unit.name, lane_name));
    let (afc, name, lane_name) = (unit.afc.clone(), unit.name.clone(), lane_name.to_string());
    unit.afc.reactor.register_callback(
        move |_| {
            if let Err(e) = afc.gcode.run_script(&format!("TOOL_LOAD LANE={}", lane_name)) {
                afc.logger.error(&AFC_ACE_AFCACE__RELOAD_RUN, &format!("{}: loading {} failed: {}. Load it by hand, then resume.", name, lane_name, e.message()));
            }
        },
        kaliru::reactor::NOW,
    );
}

/// _apply_rfid: a slot's tag (material, color) onto its lane
fn apply_rfid(lane: &AfcLane, info: &Map<String, Value>, force: bool) -> KResult<()> {
    let material = info.get("type").filter(|v| truthy(v)).map(tp::py_str).unwrap_or_default();
    let material = kaliru::pyfmt::py_strip(&material).to_string();
    if !material.is_empty() && (force || !lane.material().truthy()) {
        lane.set_material(PyVal::Str(material))?;
    }
    if let Some(Value::Array(rgb)) = info.get("color") {
        if rgb.len() >= 3 && rgb.iter().any(truthy) && (force || !lane.color.borrow().truthy()) {
            let ch = |v: &Value| (v.as_f64().unwrap_or(0.0) as i64) & 0xFF;
            *lane.color.borrow_mut() = PyVal::Str(format!("#{:02X}{:02X}{:02X}", ch(&rgb[0]), ch(&rgb[1]), ch(&rgb[2])));
        }
    }
    Ok(())
}

fn publish_lane(unit: &AfcUnit, lane: &AfcLane) {
    if let Err(e) = lane.send_lane_data() {
        unit.afc.logger.debug(&AFC_ACE_AFCACE__PUBLISH_LANE, &format!("{}: publishing {} lane data failed: {}", unit.name, lane.name, e.message()));
    }
}

// ── shared-bowden tail state ────────────────────────────────────────

fn tail_in_bowden(unit: &AfcUnit, lane: &AfcLane) -> bool {
    match lane.hub_obj_opt() {
        Some(h) => h.ace_tail_in_bowden.get(),
        None => ace(unit).tail_in_bowden.get(),
    }
}

fn set_tail_in_bowden(unit: &AfcUnit, lane: &AfcLane, value: bool) {
    match lane.hub_obj_opt() {
        Some(h) => h.ace_tail_in_bowden.set(value),
        None => ace(unit).tail_in_bowden.set(value),
    }
}

/// (hub -> toolhead bowden, slot -> hub sensor)
fn path_lengths(lane: &AfcLane) -> (f64, f64) {
    let bowden = lane.hub_obj_opt().map_or(DEFAULT_BOWDEN_MM, |h| h.afc_bowden_length.get());
    (bowden, lane.dist_hub.get())
}

/// _seed_spool_weight: a fresh spool gets the unit's full_weight
fn seed_spool_weight(unit: &AfcUnit, lane: &AfcLane) {
    if !lane.weight.borrow().truthy() {
        *lane.weight.borrow_mut() = PyVal::Float(unit.settings.full_weight);
    }
}

// ── AFC unit hooks ──────────────────────────────────────────────────

/// lane_move: LANE_MOVE, firmware-driven
pub fn lane_move(unit: &AfcUnit, cur_lane: &AfcLane, distance: f64) -> KResult<()> {
    let c = &AFC_ACE_AFCACE_LANE_MOVE;
    let a = ace(unit);
    if motion_blocked(unit, "LANE_MOVE") {
        return Ok(());
    }
    let Some(slot) = a.slot_map.borrow().get(&cur_lane.name).copied() else { return Ok(()) };
    let tool_fn = toolhead_sensor_fn(unit, cur_lane);
    let (ok, msg);
    if let (true, Some(f)) = (distance >= 0.0, tool_fn) {
        if f()? {
            unit.afc.logger.error(c, &format!("LANE_MOVE refused: the toolhead sensor already shows filament, {} would feed into the gears", cur_lane.name));
            return Ok(());
        }
        let (o, m) = feed_until(unit, slot, Some(&f), distance, a.sensor_approach_speed as f64, false)?;
        if m == NOT_REACHED {
            (ok, msg) = (true, String::new());
        } else {
            if o {
                unit.afc.logger.info(c, &format!("LANE_MOVE {} stopped at the toolhead sensor", cur_lane.name));
            }
            (ok, msg) = (o, m);
        }
    } else if distance >= 0.0 {
        (ok, msg) = feed_until(unit, slot, None, distance, a.feed_speed as f64, true)?;
    } else {
        // An unwind past the slot runs the filament out of the ACE
        let cap = retract_cap(unit, cur_lane);
        if -distance > cap {
            unit.afc.logger.error(c, &format!("LANE_MOVE refused: {:.0}mm retract is over the {:.0}mm unload cap for {}", -distance, cap, cur_lane.name));
            return Ok(());
        }
        (ok, msg) = retract(unit, slot, -distance, a.retract_speed as f64, true)?;
    }
    if !ok {
        unit.afc.logger.error(c, &format!("ACE lane_move failed for {}: {}", cur_lane.name, msg));
    }
    Ok(())
}

/// reset_lane: retract until the path sensor clears, park behind the merger
fn reset_lane(unit: &AfcUnit, cur_lane: &Rc<AfcLane>) -> KResult<bool> {
    let c = &AFC_ACE_AFCACE_RESET_LANE;
    let a = ace(unit);
    if motion_blocked(unit, "lane reset") {
        return Ok(false);
    }
    let Some(slot) = a.slot_map.borrow().get(&cur_lane.name).copied() else { return Ok(false) };
    if cur_lane.tool_loaded.get() {
        unit.afc.logger.error(c, &format!("{} is loaded in the toolhead — run TOOL_UNLOAD instead", cur_lane.name));
        return Ok(false);
    }
    a.operation_active.set(true);
    *a.early_feed.borrow_mut() = None;
    let r = (|| -> KResult<bool> {
        // Filament at the path sensor may reach into the toolhead: clear it first
        if let (Some((_, f)), Some(ext)) = (path_sensor(unit, cur_lane), cur_lane.extruder_obj_opt()) {
            if f()? {
                let temp_state = unit.afc.capture_toolhead_temp(None, false)?;
                let res = (|| {
                    unit.afc.check_extruder_temp(cur_lane, false)?;
                    clear_toolhead(unit, cur_lane, &ext)
                })();
                unit.afc.restore_toolhead_temp(temp_state.as_ref(), false)?;
                let (ok, msg) = res?;
                if !ok {
                    unit.afc.logger.error(c, &format!("ACE reset failed for {}: {}", cur_lane.name, msg));
                    return Ok(false);
                }
            }
        }
        let (ok, msg) = retract_clear_of_sensors(unit, cur_lane, slot)?;
        if !ok {
            unit.afc.logger.error(c, &format!("ACE reset failed for {}: {}", cur_lane.name, msg));
            return Ok(false);
        }
        cur_lane.loaded_to_hub.set(false);
        cur_lane.load_state_raw.set(false);
        Ok(true)
    })();
    a.operation_active.set(false);
    r
}

pub fn get_lane_reset_command(unit: &AfcUnit, lane: &AfcLane) -> String {
    format!("AFC_ACE_RESET UNIT={} LANE={}", unit.name, lane.name)
}

/// check_runout: the active lane, while printing, not ejecting/calibrating
pub fn check_runout(unit: &AfcUnit, cur_lane: &AfcLane) -> KResult<bool> {
    let f = &unit.afc.function;
    Ok(f.get_current_lane().as_deref() == Some(cur_lane.name.as_str()) && f.is_printing(false)? && !lane_state_is(cur_lane, &[LaneState::Ejecting, LaneState::Calibrating]))
}

pub fn eject_lane(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<()> {
    let slot = ace(unit).slot_map.borrow().get(&lane.name).copied();
    let status = match slot {
        Some(_) => get_status(unit, 0.0, 0.0, false)?,
        None => None,
    };
    if let (Some(slot), Some(status)) = (slot, status.as_ref()) {
        if slot_info(slot, Some(status)).contains_key("status") && !slot_present(slot, Some(status)) {
            // A spent spool (a runout with unload_on_runout ejects it): the
            // ACE no longer grips its end, so there is nothing to retract,
            // as OpenAMS skips the hardware unload of an empty spool
            lane.loaded_to_hub.set(false);
            lane.load_state_raw.set(false);
            unit.afc.logger.info(&AFC_ACE_AFCACE_EJECT_LANE, &format!("Lane {} is empty: nothing to retract. Remove the empty spool from {}.", lane.name, unit.name));
            return Ok(());
        }
    }
    if reset_lane(unit, lane)? {
        unit.afc.logger.info(
            &AFC_ACE_AFCACE_EJECT_LANE,
            &format!("Lane {} retracted clear of the merger. Press the slot lever on {} to release and remove the spool.", lane.name, unit.name),
        );
    }
    Ok(())
}

/// lane_loading: the base LED, then (TOOL_LOAD) the cold ACE-only feed
pub fn lane_loading(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<()> {
    let a = ace(unit);
    unit.base_lane_loading(lane)?;
    *a.early_feed.borrow_mut() = None;
    if lane.status() == LaneState::ToolLoading
        && unit.afc.current_loading.borrow().as_deref() == Some(lane.name.as_str())
        && !a.operation_active.get()
        && !tail_in_bowden(unit, lane)
    {
        if let Some(ext) = lane.extruder_obj_opt() {
            park(unit, &ext)?;
            let ok = unit_load_lane(unit, lane, &ext, true)?;
            *a.early_feed.borrow_mut() = Some((lane.name.clone(), ok));
        }
    }
    Ok(())
}

/// unit_load_lane: the full spool -> nozzle load
pub fn unit_load_lane(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, cur_extruder: &AfcExtruder, to_sensor: bool) -> KResult<bool> {
    let c = &AFC_ACE_AFCACE_UNIT_LOAD_LANE;
    let a = ace(unit);
    let early = a.early_feed.borrow_mut().take();
    let mut resuming = early.as_ref().is_some_and(|e| e.0 == cur_lane.name) && !to_sensor;
    if resuming && !early.as_ref().unwrap().1 {
        return Ok(false); // the early feed failed and has been reported
    }
    if resuming {
        // Only a tip still at the path sensor is the early feed's
        resuming = match path_sensor(unit, cur_lane) {
            Some((_, f)) => f()?,
            None => false,
        };
    }
    if motion_blocked(unit, "TOOL_LOAD") {
        return Ok(false);
    }
    // A lit path sensor at load start means untracked filament in the bowden
    if !resuming && !tail_in_bowden(unit, cur_lane) {
        if let Some((kind, f)) = path_sensor(unit, cur_lane) {
            if f()? {
                unit.afc.error.handle_lane_failure(
                    c,
                    cur_lane,
                    &format!(
                        "Refusing to load {}: {} sensor already shows filament but no lane is tool-loaded. Clear the path, or if filament is legitimately loaded run SET_LANE_LOADED LANE=<lane>.",
                        cur_lane.name, kind
                    ),
                    unit.afc.function.in_print()?,
                )?;
                return Ok(false);
            }
        }
    }
    a.operation_active.set(true);
    *a.early_feed.borrow_mut() = None;
    a.gears_engaged.set(false);
    let r = (|| -> KResult<bool> {
        if !load_inner(unit, cur_lane, cur_extruder, to_sensor, resuming)? {
            // Best-effort rescue: pull whatever was fed back clear of the path
            if let Some(slot) = a.slot_map.borrow().get(&cur_lane.name).copied() {
                if a.assist_slot.get() == Some(slot) {
                    set_assist(unit, slot, false)?;
                }
                stop_motion(unit, slot, "stop_feed_filament")?;
                let (mut ok, mut msg) = (true, String::new());
                if a.gears_engaged.get() {
                    (ok, msg) = clear_toolhead(unit, cur_lane, cur_extruder)?;
                }
                if ok {
                    (ok, msg) = retract_clear_of_sensors(unit, cur_lane, slot)?;
                }
                if ok {
                    cur_lane.loaded_to_hub.set(false);
                    cur_lane.load_state_raw.set(false);
                } else {
                    unit.afc.logger.error(c, &format!("ACE post-failure retract: {}", msg));
                }
            }
            unit.afc.error.handle_lane_failure(c, cur_lane, &format!("ACE load failed for {}", cur_lane.name), unit.afc.function.in_print()?)?;
            return Ok(false);
        }
        Ok(true)
    })();
    a.operation_active.set(false);
    r
}

fn load_inner(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, cur_extruder: &AfcExtruder, to_sensor: bool, fed: bool) -> KResult<bool> {
    let c = &AFC_ACE_AFCACE__LOAD_INNER;
    let a = ace(unit);
    let afc = &unit.afc;
    let Some(slot) = a.slot_map.borrow().get(&cur_lane.name).copied() else {
        afc.logger.error(c, &format!("No ACE slot mapped for {}", cur_lane.name));
        return Ok(false);
    };
    let mut status = get_status(unit, 0.0, 0.0, false)?;
    if status.as_ref().is_some_and(|s| motion_active(slot, s)) {
        // The ACE's own insert preload: wait it out
        wait_motion_idle(unit, slot, 60.0, 0.0)?;
        status = get_status(unit, 0.0, 0.0, false)?;
    }
    let state = slot_info(slot, status.as_ref()).get("status").map(tp::py_str).unwrap_or_default().to_lowercase();
    if state != "ready" {
        afc.logger.error(c, &format!("ACE slot {} on {} is not ready (status={})", slot + 1, unit.name, if state.is_empty() { "unknown" } else { &state }));
        return Ok(false);
    }
    let (bowden, dist_hub) = path_lengths(cur_lane);
    let sensor = path_sensor(unit, cur_lane);
    let (kind, sensor_fn) = match &sensor {
        Some((k, f)) => (Some(*k), Some(f.clone())),
        None => (None, None),
    };
    cur_lane.set_status(LaneState::ToolLoading);
    lane_loading(unit, cur_lane)?;

    if tail_in_bowden(unit, cur_lane) {
        // A spent spool's tail occupies sensor -> nozzle: push it through
        if kind == Some("hub") && !sensor_fn.as_ref().unwrap()()? {
            let (ok, msg) = feed_until(unit, slot, sensor_fn.as_ref(), dist_hub + a.bowden_overshoot, a.feed_speed as f64, true)?;
            if !ok {
                afc.logger.error(c, &format!("ACE feed to hub sensor failed for {}: {}", cur_lane.name, msg));
                return Ok(false);
            }
        }
        cur_lane.sync_to_extruder(true)?;
        if !tail_push(unit, slot, cur_extruder, bowden + cur_extruder.tool_stn.get())? {
            return Ok(false);
        }
        set_tail_in_bowden(unit, cur_lane, false);
        cur_lane.loaded_to_hub.set(true);
        cur_lane.load_state_raw.set(true);
        afc.delta_time.log_with_time(c, "Tail pushed through, new filament at nozzle", true);
        assist_after_load(unit, slot)?;
        cur_lane.set_status(LaneState::ToolLoaded);
        afc.save_vars()?;
        return Ok(true);
    }

    if fed {
        // the early feed already did the ACE-only part
    } else if kind == Some("hub") {
        let f = sensor_fn.as_ref().unwrap();
        if !f()? {
            let (ok, msg) = feed_until(unit, slot, Some(f), dist_hub + a.bowden_overshoot, a.feed_speed as f64, true)?;
            if !ok {
                afc.logger.error(c, &format!("ACE feed to hub sensor failed for {}: {}", cur_lane.name, msg));
                return Ok(false);
            }
            afc.delta_time.log_with_time(c, "ACE fed to hub sensor", true);
        }
        let (ok, msg) = feed_until(unit, slot, None, bowden, a.feed_speed as f64, true)?;
        if !ok {
            afc.logger.error(c, &format!("ACE bowden push failed for {}: {}", cur_lane.name, msg));
            return Ok(false);
        }
    } else if kind == Some("tool_start") {
        let f = sensor_fn.as_ref().unwrap();
        if !f()? {
            let (ok, msg) = approach_sensor(unit, slot, f, a.hub_clear_mm + a.sensor_approach_mm, bowden + dist_hub)?;
            if !ok {
                afc.logger.error(c, &format!("ACE feed to tool_start sensor failed for {}: {}", cur_lane.name, msg));
                return Ok(false);
            }
            afc.delta_time.log_with_time(c, "ACE fed to tool_start sensor", true);
        }
    } else {
        let blind = bowden + dist_hub;
        afc.logger.info(c, &format!("{}: no path sensor — blind feeding {:.0}mm", unit.name, blind));
        let (ok, msg) = feed_until(unit, slot, None, blind, a.feed_speed as f64, true)?;
        if !ok {
            afc.logger.error(c, &format!("ACE blind feed failed: {}", msg));
            return Ok(false);
        }
    }
    cur_lane.loaded_to_hub.set(true);
    cur_lane.load_state_raw.set(true);
    if to_sensor {
        return Ok(true);
    }

    // Toolhead engagement: sync-load into the gears, then assist
    if !fed {
        park(unit, cur_extruder)?;
    }
    cur_lane.sync_to_extruder(true)?;
    let mut sync_mm = cur_extruder.tool_stn.get();
    if let Some(s) = a.sync_load_mm {
        sync_mm = s.min(sync_mm);
    }
    let (ok, msg) = sync_load(unit, slot, cur_extruder, sync_mm, 5.0)?;
    if !ok {
        afc.logger.error(c, &format!("ACE sync load failed for {}: {}", cur_lane.name, msg));
        return Ok(false);
    }
    if cur_extruder.tool_stn.get() > sync_mm {
        // The extruder pulls the rest alone, at once: assist on first
        set_assist(unit, slot, true)?;
        afc.move_e_pos(cur_extruder.tool_stn.get() - sync_mm, cur_extruder.tool_load_speed.get(), "tool stn", true)?;
    } else {
        assist_after_load(unit, slot)?;
    }
    if cur_extruder.tool_end.as_deref().is_some_and(|t| !t.is_empty()) {
        let mut attempts = 0;
        while !cur_extruder.tool_end_state.get() {
            attempts += 1;
            if attempts > 20 {
                afc.logger.error(c, &format!("Filament failed to trigger post-gear sensor for {}", cur_lane.name));
                return Ok(false);
            }
            afc.move_e_pos(cur_lane.short_move_dis(), cur_extruder.tool_load_speed.get(), "Tool end", true)?;
        }
    }
    afc.delta_time.log_with_time(c, "Filament loaded to nozzle", true);
    cur_lane.set_status(LaneState::ToolLoaded);
    afc.save_vars()?;
    Ok(true)
}

// ── unload ──────────────────────────────────────────────────────────

/// _clear_toolhead: cut, then the extruder pulls it out of the gears
fn clear_toolhead(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, cur_extruder: &AfcExtruder) -> KResult<(bool, String)> {
    let afc = &unit.afc;
    afc.move_e_pos(-2.0, cur_extruder.tool_unload_speed.get(), "Quick Pull", false)?;
    cur_lane.disable_buffer()?;
    cur_lane.sync_to_extruder(true)?;
    cur_lane.select_lane()?;
    afc.do_tool_cut_tip_form(cur_lane, cur_extruder)?;
    if cur_extruder.tool_stn_unload.get() > 0.0 {
        afc.move_e_pos(-cur_extruder.tool_stn_unload.get(), cur_extruder.tool_unload_speed.get(), "tool stn unload", true)?;
    }
    let mut attempts = 0;
    while cur_extruder.tool_end_state.get() {
        attempts += 1;
        if attempts > cur_lane.tool_max_unload_attempts() {
            cur_lane.unsync_to_extruder(true)?;
            return Ok((false, format!("Failed to clear toolhead sensors unloading {}; filament may be stuck in the toolhead", cur_lane.name)));
        }
        afc.move_e_pos(-cur_lane.short_move_dis(), cur_extruder.tool_unload_speed.get(), "clear sensors", true)?;
    }
    if cur_extruder.tool_sensor_after_extruder.get() > 0.0 {
        afc.move_e_pos(-cur_extruder.tool_sensor_after_extruder.get(), cur_extruder.tool_unload_speed.get(), "After extruder", true)?;
    }
    cur_lane.unsync_to_extruder(true)?;
    Ok((true, String::new()))
}

/// unit_unload_lane: the full nozzle -> slot unload
pub fn unit_unload_lane(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, cur_extruder: &AfcExtruder) -> KResult<bool> {
    let c = &AFC_ACE_AFCACE_UNIT_UNLOAD_LANE;
    let a = ace(unit);
    let afc = &unit.afc;
    if motion_blocked(unit, "TOOL_UNLOAD") {
        return Ok(false);
    }
    a.operation_active.set(true);
    *a.early_feed.borrow_mut() = None;
    let r = (|| -> KResult<bool> {
        let Some(slot) = a.slot_map.borrow().get(&cur_lane.name).copied() else { return Ok(false) };
        let Some(status) = get_status(unit, 0.0, 0.0, true)? else {
            afc.error.handle_lane_failure(c, cur_lane, &format!("Cannot unload {}: no status from {}", cur_lane.name, unit.name), afc.function.in_print()?)?;
            return Ok(false);
        };
        // Spent spool: only an ungripped tail remains in the path
        if !slot_present(slot, Some(&status)) {
            return tail_unload(unit, cur_lane, slot);
        }
        // Assist off FIRST: it fights every backward move that follows
        set_assist(unit, slot, false)?;
        cur_lane.set_status(LaneState::ToolUnloading);
        let (ok, msg) = clear_toolhead(unit, cur_lane, cur_extruder)?;
        if !ok {
            afc.error.handle_lane_failure(c, cur_lane, &msg, afc.function.in_print()?)?;
            return Ok(false);
        }
        afc.delta_time.log_with_time(c, "Unloaded from toolhead", true);
        let (ok, msg) = retract_clear_of_sensors(unit, cur_lane, slot)?;
        if !ok {
            afc.error.handle_lane_failure(c, cur_lane, &format!("ACE retract failed for {}: {}", cur_lane.name, msg), afc.function.in_print()?)?;
            return Ok(false);
        }
        // The spool is still in its slot and the tip parked behind the
        // merger: staged and loadable, as the status poll keeps it
        cur_lane.loaded_to_hub.set(true);
        cur_lane.load_state_raw.set(false);
        afc.delta_time.log_with_time(c, "ACE retract complete", true);
        if let Some(m) = &afc.post_unload_macro {
            afc.gcode.run_script_from_command(m)?;
        }
        cur_lane.set_tool_unloaded(true)?;
        cur_lane.set_status(LaneState::None);
        unit.lane_tool_unloaded(cur_lane)?;
        afc.save_vars()?;
        Ok(true)
    })();
    a.operation_active.set(false);
    r
}

/// _tail_unload: book-keeping unload for a spent spool
fn tail_unload(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, slot: i64) -> KResult<bool> {
    let afc = &unit.afc;
    cur_lane.set_status(LaneState::ToolUnloading);
    cur_lane.disable_buffer()?;
    set_assist(unit, slot, false)?;
    cur_lane.unsync_to_extruder(true)?;
    tail_pending_discard(unit, &cur_lane.name);
    if path_sensor(unit, cur_lane).is_some_and(|(k, _)| k == "hub") {
        set_tail_in_bowden(unit, cur_lane, true);
    }
    cur_lane.loaded_to_hub.set(false);
    cur_lane.load_state_raw.set(false);
    if let Some(m) = &afc.post_unload_macro {
        afc.gcode.run_script_from_command(m)?;
    }
    cur_lane.set_tool_unloaded(true)?;
    cur_lane.set_status(LaneState::None);
    unit.lane_tool_unloaded(cur_lane)?;
    afc.save_vars()?;
    afc.logger.info(&AFC_ACE_AFCACE__TAIL_UNLOAD, &format!("{} spent-spool unload: tail left in bowden, next load pushes it through the nozzle", cur_lane.name));
    Ok(true)
}

/// _tail_push: drive a spent tail through the melt zone in chunks
fn tail_push(unit: &AfcUnit, slot: i64, cur_extruder: &AfcExtruder, total: f64) -> KResult<bool> {
    let c = &AFC_ACE_AFCACE__TAIL_PUSH;
    let a = ace(unit);
    let afc = &unit.afc;
    park(unit, cur_extruder)?;
    a.gears_engaged.set(true);
    let speed = a.tail_purge_speed;
    let mut pushed = 0.0;
    while pushed < total {
        let step = (total - pushed).min(100.0);
        let ace_mm = kaliru::pyfmt::py_round(step, 0) as i64;
        if ace_mm > 0 {
            let res = motion_rpc(unit, slot, "feed_filament", params(&[("index", json!(slot)), ("length", json!(ace_mm)), ("speed", json!(speed as i64))]).unwrap(), None)?;
            if !res.ok {
                afc.logger.error(c, &format!("Tail push feed failed: {}", res.error_str()));
                return Ok(false);
            }
        }
        afc.move_e_pos(step, speed, "tail push", true)?;
        if ace_mm > 0 && !wait_motion_idle(unit, slot, step / speed + 5.0, SPIN_UP_S)? {
            stop_motion(unit, slot, "stop_feed_filament")?;
            afc.logger.error(c, "Tail push: timeout waiting for ACE feed to finish");
            return Ok(false);
        }
        pushed += step;
        if afc.kick.get() {
            if let Some(k) = &afc.kick_cmd {
                afc.gcode.run_script_from_command(k)?;
            }
        }
    }
    Ok(true)
}

/// _park: where a load may extrude (purge_park_cmd, else AFC's park_cmd)
fn park(unit: &AfcUnit, cur_extruder: &AfcExtruder) -> KResult<()> {
    let afc = &unit.afc;
    let cmd = ace(unit).purge_park_cmd.clone().or_else(|| afc.park_cmd.clone());
    if afc.park.get() {
        if let Some(cmd) = cmd.filter(|c| !c.is_empty()) {
            afc.gcode.run_script_from_command(&format!("{} EXTRUDER={}", cmd, cur_extruder.name))?;
        }
    }
    Ok(())
}

/// _retract_cap: the longest unwind an unload or recovery may make
fn retract_cap(unit: &AfcUnit, cur_lane: &AfcLane) -> f64 {
    let bowden = match cur_lane.hub_obj_opt() {
        Some(h) => {
            let u = h.afc_unload_bowden_length.get();
            if u != 0.0 {
                u
            } else {
                h.afc_bowden_length.get()
            }
        }
        None => DEFAULT_BOWDEN_MM,
    };
    let tool_stn = cur_lane.extruder_obj_opt().map_or(0.0, |e| e.tool_stn.get());
    bowden + tool_stn + ace(unit).hub_clear_mm
}

fn retract_clear_of_sensors(unit: &AfcUnit, cur_lane: &AfcLane, slot: i64) -> KResult<(bool, String)> {
    let a = ace(unit);
    let cap = retract_cap(unit, cur_lane);
    let Some((name, f)) = path_sensor(unit, cur_lane) else {
        // Nothing to home against: the configured path, blind
        return retract(unit, slot, cap, a.retract_speed as f64, true);
    };
    let (ok, msg) = retract_until_clear(unit, slot, name, &f, cap, a.hub_clear_mm)?;
    if !ok {
        return Ok((ok, msg));
    }
    if f()? {
        return Ok((false, format!("{} sensor re-triggered after retract", name)));
    }
    Ok((true, String::new()))
}

// ── PREP / system test ──────────────────────────────────────────────

pub fn system_test(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, assign_tcmd: bool) -> KResult<bool> {
    let a = ace(unit);
    let afc = &unit.afc;
    let mut msg = String::new();
    let mut succeeded = true;
    let status = get_status(unit, 0.0, 0.0, false)?;
    let slot = a.slot_map.borrow().get(&cur_lane.name).copied().unwrap_or(-1);
    match &status {
        None => {
            if a.optional {
                msg = "ACE NOT CONNECTED (optional unit, lanes inactive)".into();
            } else {
                msg = "<span class=error--text>ACE NOT CONNECTED</span>".into();
                if let Some(e) = a.transport.borrow().as_ref().and_then(|t| t.state().last_error()).filter(|e| !e.is_empty()) {
                    msg += &format!(" ({})", e);
                }
                succeeded = false;
            }
        }
        Some(status) => {
            let present = slot_present(slot, Some(status));
            cur_lane.prep_state.set(present);
            cur_lane.load_state_raw.set(cur_lane.tool_loaded.get());
            cur_lane.loaded_to_hub.set(present && !a.monitor_only);
            a.last_prep.borrow_mut().insert(slot, present);
            if present {
                seed_spool_weight(unit, cur_lane);
                // Fill blanks from RFID; AFC.var-persisted user edits win
                apply_rfid(cur_lane, &slot_info(slot, Some(status)), false)?;
                publish_lane(unit, cur_lane);
                unit.lane_loaded(cur_lane, false)?;
                cur_lane.set_status(LaneState::Loaded);
                msg += "<span class=success--text>SPOOL READY</span>";
                if cur_lane.tool_loaded.get() && cur_lane.extruder_obj()?.lane_loaded.borrow().as_deref() == Some(cur_lane.name.as_str()) {
                    cur_lane.sync_to_extruder(true)?;
                    if afc.current().as_deref() == Some(cur_lane.name.as_str()) {
                        afc.spool.set_active_spool(&cur_lane.spool_id.borrow().clone())?;
                        unit.lane_tool_loaded(cur_lane)?;
                        cur_lane.set_status(LaneState::Tooled);
                        cur_lane.enable_buffer(false)?;
                        // Print-time feed assist: armed by the status poll
                        a.assist_slot.set(Some(slot));
                        a.assist_gen.set(None);
                    } else {
                        unit.lane_tool_loaded_idle(cur_lane)?;
                    }
                    unit.printer.send_event("afc:tool_loaded", &[cur_lane])?;
                }
            } else {
                if cur_lane.tool_loaded.get() {
                    // Booted mid-tail: spool spent but its filament still loaded
                    if path_sensor(unit, cur_lane).is_some_and(|(k, _)| k == "hub") {
                        set_tail_in_bowden(unit, cur_lane, true);
                    }
                    msg += "<span class=warning--text>SPENT SPOOL (tail loaded) </span>";
                }
                if !cur_lane.remember_spool() {
                    afc.spool.clear_values(cur_lane)?;
                }
                unit.lane_not_ready(cur_lane)?;
                msg += "EMPTY READY FOR SPOOL";
            }
        }
    }
    if assign_tcmd {
        afc.function.tcmd_assign(cur_lane)?;
    }
    cur_lane.do_enable(false)?;
    afc.logger.info(&AFC_ACE_AFCACE_SYSTEM_TEST, &format!("{} tool cmd: {:<3} {}", cur_lane.name, cur_lane.map_to_string(), msg));
    cur_lane.set_afc_prep_done();
    Ok(succeeded)
}

// ── calibration ─────────────────────────────────────────────────────

pub const CALIBRATION_LANE_MESSAGE: &str =
    "ACE lanes are not calibrated by AFC: loads home on the path sensor. With the frame runout sensor, tune afc_bowden_length by hand (see the cosmoace_afc README).";

// ── gcode commands ──────────────────────────────────────────────────

/// set_dryer: start (temp > 0) or stop the ACE dryer
fn set_dryer(unit: &AfcUnit, temp: i64, duration: Option<i64>, fan_speed: Option<i64>) -> KResult<()> {
    let c = &AFC_ACE_AFCACE_SET_DRYER;
    let a = ace(unit);
    if temp > 0 {
        let temp = temp.min(DRYER_MAX_C);
        let duration = duration.filter(|d| *d != 0).unwrap_or(a.dryer_duration);
        let fan = fan_speed.filter(|f| *f != 0).unwrap_or(a.dryer_fan_speed);
        let res = rpc(unit, "drying", params(&[("temp", json!(temp)), ("fan_speed", json!(fan)), ("duration", json!(duration))]), 8.0)?;
        if res.ok {
            a.dryer_heater.target.set(temp as f64);
            unit.afc.logger.info(c, &format!("{} dryer set to {}C for {} minutes", unit.name, temp, duration));
        } else {
            unit.afc.logger.error(c, &format!("{} dryer start failed: {}", unit.name, res.error_str()));
        }
    } else {
        // A unit that is not plugged in has no dryer running
        let connected = a.transport.borrow().as_ref().is_some_and(|t| t.state().is_connected());
        if !connected {
            a.dryer_heater.target.set(0.0);
            unit.afc.logger.debug(c, &format!("{} dryer stop skipped, unit not connected", unit.name));
            return Ok(());
        }
        let res = rpc(unit, "drying_stop", None, 8.0)?;
        if res.ok {
            a.dryer_heater.target.set(0.0);
            unit.afc.logger.info(c, &format!("{} dryer stopped", unit.name));
        } else {
            unit.afc.logger.warning(c, &format!("{} dryer stop failed: {}", unit.name, res.error_str()));
        }
    }
    Ok(())
}

fn cmd_dryer_start(unit: &Rc<AfcUnit>, gcmd: &GCodeCommand) -> KResult<()> {
    let a = ace(unit);
    let temp = gcmd.get_int("TEMP", Some(a.dryer_temp), Lim::new())?;
    let duration = gcmd.get_int("DURATION", Some(a.dryer_duration), Lim::new())?;
    let fan = gcmd.get_int("FAN_SPEED", Some(a.dryer_fan_speed), Lim::new())?;
    set_dryer(unit, temp, Some(duration), Some(fan))
}

fn cmd_dryer_stop(unit: &Rc<AfcUnit>, _gcmd: &GCodeCommand) -> KResult<()> {
    set_dryer(unit, 0, None, None)
}

fn cmd_status(unit: &Rc<AfcUnit>, _gcmd: &GCodeCommand) -> KResult<()> {
    let c = &AFC_ACE_AFCACE_CMD_AFC_ACE_STATUS;
    match get_status(unit, 0.0, 0.0, false)? {
        None => {
            let err = match ace(unit).transport.borrow().as_ref() {
                Some(t) => t.state().last_error().unwrap_or_else(|| "None".into()),
                None => "transport not started".into(),
            };
            unit.afc.logger.info(c, &format!("{}: not connected ({})", unit.name, err));
        }
        Some(status) => unit.afc.logger.info(c, &format!("{}: {}", unit.name, PyVal::from(&status).json_indent(1))),
    }
    Ok(())
}

fn cmd_stop(unit: &Rc<AfcUnit>, gcmd: &GCodeCommand) -> KResult<()> {
    let lane_name = gcmd.get("LANE")?;
    let Some(slot) = ace(unit).slot_map.borrow().get(&lane_name).copied() else {
        unit.afc.logger.error(&AFC_ACE_AFCACE_CMD_AFC_ACE_STOP, &format!("Unknown lane {}", lane_name));
        return Ok(());
    };
    stop_motion(unit, slot, "stop_feed_filament")?;
    stop_motion(unit, slot, "stop_unwind_filament")?;
    if ace(unit).assist_slot.get() == Some(slot) {
        set_assist(unit, slot, false)
    } else {
        // Another lane's print-time assist stays wanted
        stop_motion(unit, slot, "stop_feed_assist").map(|_| ())
    }
}

fn cmd_reset(unit: &Rc<AfcUnit>, gcmd: &GCodeCommand) -> KResult<()> {
    let c = &AFC_ACE_AFCACE_CMD_AFC_ACE_RESET;
    let name = gcmd.get("LANE")?;
    let lane = unit.lanes.borrow().get(&name).cloned();
    let Some(lane) = lane.filter(|l| ace(unit).slot_map.borrow().contains_key(&l.name)) else {
        unit.afc.logger.error(c, &format!("Unknown lane {} on {}", gcmd.get("LANE")?, unit.name));
        return Ok(());
    };
    if reset_lane(unit, &lane)? {
        unit.afc.logger.info(c, &format!("{} reset: parked behind the merger", lane.name));
    }
    Ok(())
}

fn cmd_panel(unit: &Rc<AfcUnit>, gcmd: &GCodeCommand) -> KResult<()> {
    let action = gcmd.get("ACTION")?.to_lowercase();
    if action != "install" && action != "remove" {
        return Err(gcmd.error("ACTION must be INSTALL or REMOVE"));
    }
    // the script ships with the plugin: <klippy>/plugins/AFC_ACE/webui
    let script = kaliru::plugins::klippy_dir().join("plugins/AFC_ACE/webui/setup-panel.sh");
    let child = std::process::Command::new("sh")
        .arg(&script)
        .arg(&action)
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .map_err(|e| KError::os(&e, "sh"))?;
    gcmd.respond_info(&format!("AFC_ACE_PANEL: {} started", action), true);
    let child = RefCell::new(Some(child));
    let u = unit.clone();
    let timer: Rc<RefCell<Option<Timer>>> = Rc::new(RefCell::new(None));
    let t2 = timer.clone();
    let t = unit.afc.reactor.register_timer(
        Box::new(move |et| {
            let c = &AFC_ACE_AFCACE_CMD_AFC_ACE_PANEL_CHECK;
            let mut ch = child.borrow_mut();
            let Some(proc_) = ch.as_mut() else { return NEVER };
            let Ok(Some(st)) = proc_.try_wait() else { return et + 0.5 };
            let mut out = String::new();
            if let Some(mut so) = proc_.stdout.take() {
                let _ = std::io::Read::read_to_string(&mut so, &mut out);
            }
            if let Some(mut se) = proc_.stderr.take() {
                let _ = std::io::Read::read_to_string(&mut se, &mut out);
            }
            let out = out.trim().to_string();
            let code = st.code().unwrap_or(-1);
            let msg = format!("AFC_ACE_PANEL {}: {}", action, if out.is_empty() { format!("exit {}", code) } else { out });
            if code != 0 {
                u.afc.logger.error(c, &msg);
            } else {
                u.afc.logger.info(c, &msg);
            }
            *ch = None;
            if let Some(t) = t2.borrow_mut().take() {
                u.afc.reactor.unregister_timer(&t);
            }
            NEVER
        }),
        unit.afc.reactor.monotonic() + 0.5,
    );
    *timer.borrow_mut() = Some(t);
    Ok(())
}

// ── status for the UI ───────────────────────────────────────────────

pub fn status_extra(unit: &AfcUnit, r: &mut kaliru::pyval::PyDict) {
    let a = ace(unit);
    let (status, _) = cached_status(unit);
    r.insert("connected".into(), PyVal::Bool(a.transport.borrow().as_ref().is_some_and(|t| t.state().is_connected())));
    if let Some(status) = status {
        let dryer = status.get("dryer_status").filter(|v| truthy(v)).or_else(|| status.get("dryer")).cloned().unwrap_or(Value::Null);
        let get = |k: &str| status.get(k).cloned().unwrap_or(Value::Null);
        r.insert(
            "ace".into(),
            PyVal::from(&json!({"temp": get("temp"), "humidity": get("humidity"), "dryer": dryer, "slots": get("slots")})),
        );
    }
}

