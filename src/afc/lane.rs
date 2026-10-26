// An AFC lane: one spool's path from its unit to the toolhead (port of
// AFC_lane.py; the Canvas lane's overrides are in canvas.rs)
//
// Copyright (C) 2024-2026 Armored Turtle
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::assist::Espooler;
use super::buffer::AfcBuffer;
use super::canvas::CanvasLane;
use super::codes::*;
use super::extruder::AfcExtruder;
use super::hub::AfcHub;
use super::stats::StatVar;
use super::unit::AfcUnit;
use super::utils::{self, add_filament_switch, DebounceButton, SwitchOptions};
use super::{no_attr, py_bool, py_opt_str, Afc};
use kaliru::configfile::{ConfigWrapper, Dflt, Lim};
use kaliru::error::{KError, KResult};
use kaliru::extras::buttons::PrinterButtons;
use kaliru::extras::query_endstops::QueryEndstops;
use kaliru::gcode::GCodeCommand;
use kaliru::mcu::EndstopRef;
use kaliru::pins::PinsObject;
use kaliru::printer::{Printer, PrinterObject};
use kaliru::pyfmt::py_float_repr;
use kaliru::pyval::{PyDict, PyVal};
use kaliru::reactor::{Reactor, ReactorMutex, Timer, NEVER};
use kaliru::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

/// "afc_stepper:register_macros" (lane)
pub const REGISTER_MACROS_EVENT: &str = "afc_stepper:register_macros";

/// Unit types that only have a load switch
/// (AFC_ACE appends "ACE" to both lists when it is imported)
const ONLY_LOAD_TYPES: [&str; 4] = ["HTLF", "Claymore", "OpenAMS", "ACE"];
const EXCLUDE_TYPES: [&str; 5] = ["HTLF", "Claymore", "OpenAMS", "ViViD", "ACE"];
/// Names skipped when searching the config for a lane's unit section
const INVALID_UNIT_NAMES: [&str; 8] = ["AFC_buffer", "AFC_button", "AFC_extruder", "AFC_hub", "AFC_lane", "AFC_led", "AFC_prep", "AFC_stepper"];
pub const VALID_DIRECT_HUB: [&str; 2] = ["direct", "direct_load"];
const UPDATE_WEIGHT_DELAY: f64 = 10.0;
const PREP_GUARD_WINDOW: f64 = 2.0;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum AssistActive {
    Yes,
    No,
    Dynamic,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum SpeedMode {
    None,
    Long,
    Short,
    Hub,
    Night,
    Calibration,
    DistHub,
    /// a value that is no SpeedMode (python passes AssistActive.YES as the
    /// speed mode in AFC_LANE_RESET): get_speed_accel's last branch
    Other,
}

impl SpeedMode {
    /// AFC_LANE_RESET passes AssistActive.YES where the speed mode goes
    pub fn from_assist(_a: AssistActive) -> SpeedMode {
        SpeedMode::Other
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum MoveDirection {
    Pos,
    Neg,
}

impl MoveDirection {
    pub fn value(self) -> f64 {
        match self {
            MoveDirection::Pos => 1.0,
            MoveDirection::Neg => -1.0,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum MoveWarning {
    None,
    Warn,
    Error,
}

/// AFCLaneState (a str enum: str() is the value)
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum LaneState {
    None,
    Error,
    Loaded,
    Tooled,
    ToolLoaded,
    ToolLoading,
    ToolUnloading,
    HubLoading,
    Ejecting,
    Calibrating,
    InfiniteRunout,
}

impl LaneState {
    pub fn as_str(self) -> &'static str {
        match self {
            LaneState::None => "None",
            LaneState::Error => "Error",
            LaneState::Loaded => "Loaded",
            LaneState::Tooled => "Tooled",
            LaneState::ToolLoaded => "Tool Loaded",
            LaneState::ToolLoading => "Tool Loading",
            LaneState::ToolUnloading => "Tool Unloading",
            LaneState::HubLoading => "HUB Loading",
            LaneState::Ejecting => "Ejecting",
            LaneState::Calibrating => "Calibrating",
            LaneState::InfiniteRunout => "Infinite Runout",
        }
    }
}

/// An endstop to home to: an AFCHomingPoints value or a lane's endstop name
#[derive(Clone, PartialEq, Debug)]
pub enum Endstop {
    None,
    Name(String),
}

impl Endstop {
    pub fn is(&self, name: &str) -> bool {
        matches!(self, Endstop::Name(n) if n == name)
    }
}

pub const HOMING_HUB: &str = "hub";
pub const HOMING_LOAD: &str = "load";
pub const HOMING_TOOL: &str = "tool";
pub const HOMING_TOOL_START: &str = "tool_start";
pub const HOMING_BUFFER: &str = "buffer";
pub const HOMING_BUFFER_TRAIL: &str = "buffer_trailing";
pub const HOMING_SELECTOR: &str = "selector";
pub const HOMING_PREP: &str = "prep";

/// A lane's homing endstop (endstops[name])
pub struct LaneEndstop {
    pub endstop: EndstopRef,
    pub endstop_name: String,
}

/// Per-lane values that fall back to the unit's (None until handle_unit_connect)
#[derive(Default)]
pub struct LaneSettings {
    pub led_fault: Option<String>,
    pub led_ready: Option<String>,
    pub led_not_ready: Option<String>,
    pub led_loading: Option<String>,
    pub led_prep_loaded: Option<String>,
    pub led_unloading: Option<String>,
    pub led_tool_loaded: Option<String>,
    pub led_tool_loaded_idle: Option<String>,
    pub led_tool_unloaded: Option<String>,
    pub led_spool_illum: Option<String>,
    pub led_use_filament_color: Option<bool>,
    pub long_moves_speed: Option<f64>,
    pub long_moves_accel: Option<f64>,
    pub short_moves_speed: Option<f64>,
    pub short_moves_accel: Option<f64>,
    pub short_move_dis: Option<f64>,
    pub max_move_dis: Option<f64>,
    pub n20_break_delay_time: Option<f64>,
    pub homing_overshoot: Option<f64>,
    pub homing_delta: Option<f64>,
    pub load_then_home_var: Option<bool>,
    pub load_undershoot: Option<f64>,
    pub extruder_clear_dis: Option<f64>,
    pub tool_max_unload_attempts: Option<i64>,
    pub rev_long_moves_speed_factor: Option<f64>,
    pub assisted_unload: Option<bool>,
    pub td1_when_loaded: Option<bool>,
    pub td1_bowden_length: Option<f64>,
    pub post_prep_macro: Option<String>,
    pub dist_hub_move_speed: f64,
    pub dist_hub_move_accel: f64,
    /// which of the move speeds are python ints (inherited int defaults)
    pub move_ints: super::MoveInts,
}

/// [AFC_lane] / [AFC_canvas_lane]
pub struct AfcLane {
    me: Weak<AfcLane>,
    config: ConfigWrapper,
    pub printer: Rc<Printer>,
    pub afc: Rc<Afc>,
    reactor: Rc<Reactor>,
    mutex: Rc<ReactorMutex>,
    cb_update_weight: Timer,

    unit_obj: RefCell<Option<Rc<AfcUnit>>>,
    hub_obj: RefCell<Option<Rc<AfcHub>>>,
    /// is_direct_hub(): python's stand-in hub (state False, move_dis 75,
    /// hub_clear_move_dis 65)
    direct_hub: Cell<bool>,
    extruder_obj: RefCell<Option<Rc<AfcExtruder>>>,
    pub endstops: RefCell<IndexMap<String, LaneEndstop>>,

    pub fullname: String,
    pub name: String,

    pub tool_loaded: Cell<bool>,
    pub loaded_to_hub: Cell<bool>,
    pub spool_id: RefCell<PyVal>,
    pub color: RefCell<PyVal>,
    pub multi_color: RefCell<PyVal>,
    pub spool_vendor: RefCell<PyVal>,
    pub filament_name: RefCell<PyVal>,
    pub weight: RefCell<PyVal>,
    pub auto_switch_triggered: Cell<bool>,
    material: RefCell<PyVal>,
    pub extruder_temp: RefCell<PyVal>,
    pub bed_temp: RefCell<PyVal>,
    pub td1_data: RefCell<PyVal>,
    pub runout_lane: RefCell<Option<String>>,
    status: Cell<LaneState>,
    pub need_purge: Cell<bool>,
    afc_staged_spool_id: RefCell<PyVal>,
    load_suppressed: Cell<bool>,

    pub multi_hubs_found: Cell<bool>,
    pub unit: String,
    pub hub: RefCell<Option<String>>,
    pub buffer_name: RefCell<Option<String>>,
    buffer_obj: RefCell<Option<Rc<AfcBuffer>>>,
    pub index: i64,
    pub afc_extruder_name: RefCell<Option<String>>,
    pub standalone_lane: bool,
    pub remember_spool: RefCell<Option<bool>>,
    pub map: RefCell<Vec<String>>,
    /// self._map: the map from the config (AFC_RESET_MAPPING restores it)
    pub config_map: Vec<String>,
    pub current_map: RefCell<String>,
    sent_lane_data_keys: RefCell<Vec<String>>,

    pub led_index: Option<String>,
    pub led_spool_index: Option<String>,
    pub current_led_state: RefCell<String>,
    pub settings: RefCell<LaneSettings>,

    pub custom_load_cmd: RefCell<Option<String>>,
    pub custom_unload_cmd: RefCell<Option<String>>,

    pub dist_hub: Cell<f64>,
    pub config_dist_hub: f64,
    pub load_to_hub: bool,
    pub enable_sensors_in_gui: bool,
    debounce_delay: f64,
    pub td1_device_id: RefCell<Option<String>>,
    pub calibrated_lane: Cell<bool>,
    pub only_lane: Cell<bool>,

    pub prep: Option<String>,
    pub prep_state: Cell<bool>,
    pub load: Option<String>,
    pub load_state_raw: Cell<bool>,
    pub selector: Option<String>,
    pub selector_state: Cell<Option<bool>>,
    pub selector_cal_dis: f64,
    pub fila_selector: RefCell<Option<Rc<kaliru::extras::filament_switch_sensor::SwitchSensor>>>,
    /// the unit's drive stepper (an AFC_lane of a ViViD / Claymore)
    pub drive_stepper: RefCell<Option<Rc<AfcLane>>>,

    pub espooler: Rc<Espooler>,
    lane_load_count: RefCell<Option<Rc<StatVar>>>,

    pub filament_diameter: RefCell<PyVal>,
    pub filament_density: RefCell<PyVal>,
    pub inner_diameter: f64,
    pub outer_diameter: f64,
    pub empty_spool_weight: RefCell<PyVal>,
    pub max_motor_rpm: PyVal,
    pub rwd_speed_multi: Cell<f64>,
    pub fwd_speed_multi: Cell<f64>,
    past_extruder_position: Cell<f64>,
    save_counter: Cell<i64>,

    pub afc_prep_done: Cell<bool>,
    fila_prep: RefCell<Option<Rc<kaliru::extras::filament_switch_sensor::SwitchSensor>>>,
    prep_debounce_button: RefCell<Option<Rc<DebounceButton>>>,
    fila_load: RefCell<Option<Rc<kaliru::extras::filament_switch_sensor::SwitchSensor>>>,
    load_debounce_button: RefCell<Option<Rc<DebounceButton>>>,

    pub connect_done: Cell<bool>,
    pub prep_active: Cell<bool>,
    prep_recheck_pending: Cell<bool>,
    last_prep_time: Cell<f64>,

    /// The Canvas lane's part (AFC_canvas_lane)
    pub canvas: RefCell<Option<CanvasLane>>,
    /// The lane's own stepper (AFC_stepper)
    pub stepper: RefCell<Option<Rc<super::stepper::StepperLane>>>,
}

/// [AFC_lane <name>]
pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<kaliru::printer::PObj> {
    Ok(AfcLane::new(config, false)?)
}

impl AfcLane {
    /// AFCLane.__init__
    pub fn new(config: &ConfigWrapper, canvas: bool) -> KResult<Rc<AfcLane>> {
        let printer = config.get_printer();
        let afc = printer.lookup_or_load::<Afc>(config, "AFC")?;
        let reactor = printer.get_reactor();
        let mutex = reactor.mutex(false);
        let fullname = config.get_name().to_string();
        let name = fullname.split_whitespace().last().unwrap_or("").to_string();

        let unit_cfg: String = config.get("unit", Dflt::Required)?;
        let hub = config.get_opt("hub")?;
        let buffer_name = config.get_opt("buffer")?;
        let unit = unit_cfg.split(':').next().unwrap_or("").to_string();
        let index = unit_cfg.split(':').nth(1).and_then(|s| utils::py_int_of(&PyVal::Str(s.to_string())).ok()).unwrap_or(0);
        let afc_extruder_name = config.get_opt("extruder")?;
        let standalone_lane = config.getboolean("standalone", false)?;
        let remember_spool = config.getboolean_opt("remember_spool")?;
        let mut map = config.getlist("cmd", PyVal::List(vec![]), ",", None)?.unwrap_or_default();
        map = config.getlist("map", PyVal::List(map.iter().map(|m| PyVal::Str(m.clone())).collect()), ",", None)?.unwrap_or_default();
        let config_map = map.clone();

        let led_index = config.get_opt("led_index")?;
        let mut settings = LaneSettings {
            led_fault: config.get_opt("led_fault")?,
            led_ready: config.get_opt("led_ready")?,
            led_not_ready: config.get_opt("led_not_ready")?,
            led_loading: config.get_opt("led_loading")?,
            led_prep_loaded: config.get_opt("led_loading")?,
            led_unloading: config.get_opt("led_unloading")?,
            led_tool_loaded: config.get_opt("led_tool_loaded")?,
            led_tool_loaded_idle: config.get_opt("led_tool_loaded_idle")?,
            led_tool_unloaded: config.get_opt("led_tool_unloaded")?,
            ..Default::default()
        };
        let led_spool_index = config.get_opt("led_spool_index")?;
        settings.led_spool_illum = config.get_opt("led_spool_illuminate")?;
        settings.led_use_filament_color = config.getboolean_opt("led_use_filament_color")?;
        settings.long_moves_speed = config.getfloat_opt("long_moves_speed", Lim::new())?;
        settings.long_moves_accel = config.getfloat_opt("long_moves_accel", Lim::new())?;
        settings.short_moves_speed = config.getfloat_opt("short_moves_speed", Lim::new())?;
        settings.short_moves_accel = config.getfloat_opt("short_moves_accel", Lim::new())?;
        settings.short_move_dis = config.getfloat_opt("short_move_dis", Lim::new())?;
        settings.max_move_dis = config.getfloat_opt("max_move_dis", Lim::new())?;
        settings.n20_break_delay_time = config.getfloat_opt("n20_break_delay_time", Lim::new())?;
        settings.homing_overshoot = config.getfloat_opt("homing_overshoot", Lim::new())?;
        settings.homing_delta = config.getfloat_opt("homing_delta", Lim::new())?;
        settings.load_then_home_var = config.getboolean_opt("load_then_home")?;
        settings.load_undershoot = config.getfloat_opt("load_undershoot", Lim::new())?;
        settings.extruder_clear_dis = config.getfloat_opt("extruder_clear_dis", Lim::new())?;
        settings.tool_max_unload_attempts = config.getint_opt("tool_max_unload_attempts", Lim::new().min(0))?;
        let custom_load_cmd = config.get_opt("custom_load_cmd")?;
        let custom_unload_cmd = config.get_opt("custom_unload_cmd")?;
        settings.rev_long_moves_speed_factor = config.getfloat_opt("rev_long_moves_speed_factor", Lim::new())?;
        let dist_hub = config.getfloat("dist_hub", 60.0, Lim::new())?;
        let _park_dist = config.getfloat("park_dist", 10.0, Lim::new())?;
        let load_to_hub = config.getboolean("load_to_hub", afc.load_to_hub)?;
        let enable_sensors_in_gui = config.getboolean("enable_sensors_in_gui", afc.enable_sensors_in_gui)?;
        let debounce_delay = config.getfloat("debounce_delay", afc.debounce_delay, Lim::new())?;
        let enable_runout = config.getboolean("enable_hub_runout", afc.enable_hub_runout)?;
        let sensor_to_show = config.get_opt("sensor_to_show")?;
        settings.assisted_unload = config.getboolean_opt("assisted_unload")?;
        settings.td1_when_loaded = config.getboolean_opt("capture_td1_when_loaded")?;
        let td1_device_id = config.get_opt("td1_device_id")?;
        settings.td1_bowden_length = config.getfloat_opt("td1_bowden_length", Lim::new())?;
        let calibrated_lane = config.getboolean("calibrated_lane", false)?;
        settings.post_prep_macro = config.get_opt("post_prep_macro")?;

        let buttons = printer.lookup_or_load::<PrinterButtons>(config, "buttons")?;
        let prep = config.get_opt("prep")?;
        let load = config.get_opt("load")?;
        let selector = config.get_opt("selector")?;
        let selector_cal_dis = config.getfloat("selector_cal_distance", 0.0, Lim::new())?;
        // lane triggers: registered before the espooler sets up its motor
        // pins, as python does (the first button on an mcu takes its oid);
        // the callbacks reach the lane once it exists
        let lane_ref: Rc<RefCell<Weak<AfcLane>>> = Rc::default();
        if let Some(p) = &prep {
            let l = lane_ref.clone();
            buttons.register_buttons(&[p.as_str()], Rc::new(move |et, state| l.borrow().upgrade().map_or(Ok(()), |l| l.prep_callback(et, state != 0))))?;
        }
        if let Some(p) = &load {
            let l = lane_ref.clone();
            buttons.register_buttons(&[p.as_str()], Rc::new(move |et, state| l.borrow().upgrade().map_or(Ok(()), |l| l.load_callback(et, state != 0))))?;
        }
        let espooler = Espooler::new(&name, config, &afc)?;
        let filament_diameter = config.getfloat("filament_diameter", 1.75, Lim::new())?;
        let filament_density = config.getfloat("filament_density", 1.24, Lim::new())?;
        let inner_diameter = config.getfloat("spool_inner_diameter", 75.0, Lim::new().min(1))?;
        let outer_diameter = config.getfloat("spool_outer_diameter", 200.0, Lim::new().min(100))?;
        // the int default stays an int, as python keeps it (190, not 190.0)
        let empty_spool_weight = config.getfloat_value("empty_spool_weight", &kaliru::configfile::Dflt::Val(PyVal::Int(190)), &Lim::new().min(1))?.unwrap_or(PyVal::Int(190));
        let max_motor_rpm = super::getfloat_pv(config, "assist_max_motor_rpm", PyVal::Int(465))?;
        let rwd_speed_multi = config.getfloat("rwd_speed_multiplier", 0.5, Lim::new())?;
        let fwd_speed_multi = config.getfloat("fwd_speed_multiplier", 0.5, Lim::new())?;

        let lane = Rc::new_cyclic(|me: &Weak<AfcLane>| {
            let w = me.clone();
            let cb_update_weight = reactor.register_timer(
                Box::new(move |et| match w.upgrade() {
                    Some(l) => l.update_weight_callback(Some(et)).unwrap_or(NEVER),
                    None => NEVER,
                }),
                NEVER,
            );
            AfcLane {
                me: me.clone(),
                config: config.clone(),
                printer: printer.clone(),
                afc: afc.clone(),
                reactor: reactor.clone(),
                mutex,
                cb_update_weight,
                unit_obj: RefCell::new(None),
                hub_obj: RefCell::new(None),
                direct_hub: Cell::new(false),
                extruder_obj: RefCell::new(None),
                endstops: RefCell::new(IndexMap::default()),
                fullname: fullname.clone(),
                name: name.clone(),
                tool_loaded: Cell::new(false),
                loaded_to_hub: Cell::new(false),
                spool_id: RefCell::new(PyVal::None),
                color: RefCell::new(PyVal::Str(String::new())),
                multi_color: RefCell::new(PyVal::List(vec![])),
                spool_vendor: RefCell::new(PyVal::Str(String::new())),
                filament_name: RefCell::new(PyVal::Str(String::new())),
                weight: RefCell::new(PyVal::Float(0.0)),
                auto_switch_triggered: Cell::new(false),
                material: RefCell::new(PyVal::None),
                extruder_temp: RefCell::new(PyVal::None),
                bed_temp: RefCell::new(PyVal::None),
                td1_data: RefCell::new(PyVal::dict()),
                runout_lane: RefCell::new(None),
                status: Cell::new(LaneState::None),
                need_purge: Cell::new(false),
                afc_staged_spool_id: RefCell::new(PyVal::None),
                load_suppressed: Cell::new(false),
                multi_hubs_found: Cell::new(false),
                unit: unit.clone(),
                hub: RefCell::new(hub.clone()),
                buffer_name: RefCell::new(buffer_name.clone()),
                buffer_obj: RefCell::new(None),
                index,
                afc_extruder_name: RefCell::new(afc_extruder_name.clone()),
                standalone_lane,
                remember_spool: RefCell::new(remember_spool),
                map: RefCell::new(map),
                config_map,
                current_map: RefCell::new(String::new()),
                sent_lane_data_keys: RefCell::new(vec![]),
                led_index,
                led_spool_index,
                current_led_state: RefCell::new(String::new()),
                settings: RefCell::new(settings),
                custom_load_cmd: RefCell::new(custom_load_cmd),
                custom_unload_cmd: RefCell::new(custom_unload_cmd),
                dist_hub: Cell::new(dist_hub),
                config_dist_hub: dist_hub,
                load_to_hub,
                enable_sensors_in_gui,
                debounce_delay,
                td1_device_id: RefCell::new(td1_device_id),
                calibrated_lane: Cell::new(calibrated_lane),
                only_lane: Cell::new(false),
                prep: prep.clone(),
                prep_state: Cell::new(false),
                load: load.clone(),
                load_state_raw: Cell::new(load.is_none()),
                selector: selector.clone(),
                selector_state: Cell::new(None),
                selector_cal_dis,
                fila_selector: RefCell::new(None),
                drive_stepper: RefCell::new(None),
                espooler,
                lane_load_count: RefCell::new(None),
                filament_diameter: RefCell::new(PyVal::Float(filament_diameter)),
                filament_density: RefCell::new(PyVal::Float(filament_density)),
                inner_diameter,
                outer_diameter,
                empty_spool_weight: RefCell::new(empty_spool_weight),
                max_motor_rpm,
                rwd_speed_multi: Cell::new(rwd_speed_multi),
                fwd_speed_multi: Cell::new(fwd_speed_multi),
                past_extruder_position: Cell::new(-1.0),
                save_counter: Cell::new(-1),
                afc_prep_done: Cell::new(false),
                fila_prep: RefCell::new(None),
                prep_debounce_button: RefCell::new(None),
                fila_load: RefCell::new(None),
                load_debounce_button: RefCell::new(None),
                connect_done: Cell::new(false),
                prep_active: Cell::new(false),
                prep_recheck_pending: Cell::new(false),
                last_prep_time: Cell::new(0.0),
                canvas: RefCell::new(None),
                stepper: RefCell::new(None),
            }
        });
        let l = lane.clone();
        kaliru::events::KlippyReady::on(&printer, move || l.handle_ready());
        let l = lane.clone();
        printer.register_event_handler(utils::MOONRAKER_CONNECT_EVENT, Rc::new(move |_| l.handle_moonraker_connect()));
        let l = lane.clone();
        printer.register_event_handler(
            &super::unit::connect_event(&unit),
            Rc::new(move |args| {
                let u: Rc<AfcUnit> = kaliru::events::arg("AFC_unit:connect", args, 0)?;
                l.handle_unit_connect(&u)
            }),
        );

        *lane_ref.borrow_mut() = Rc::downgrade(&lane);
        let query_endstops = printer.lookup_or_load::<QueryEndstops>(config, "query_endstops")?;
        let ppins = printer.lookup::<PinsObject>("pins")?;

        if let Some(p) = &prep {
            let show = enable_sensors_in_gui && sensor_to_show.as_deref().is_none_or(|s| s.contains("prep"));
            let (fila, button) = add_filament_switch(
                &format!("{}_prep", name),
                p,
                &printer,
                &SwitchOptions { show_sensor: show, enable_runout, debounce_delay, ..Default::default() },
            )?;
            let l = Rc::downgrade(&lane);
            button.set_action(Rc::new(move |et, state| l.upgrade().map_or(Ok(()), |l| l.handle_prep_runout(et, state))));
            button.debounce_delay.set(0.0);
            *lane.fila_prep.borrow_mut() = Some(fila);
            *lane.prep_debounce_button.borrow_mut() = Some(button);
            lane.set_homing_endstop(&query_endstops, &ppins, p, HOMING_PREP)?;
        }
        if let Some(p) = &load {
            let show = enable_sensors_in_gui && sensor_to_show.as_deref().is_none_or(|s| s.contains("load"));
            let (fila, button) = add_filament_switch(
                &format!("{}_load", name),
                p,
                &printer,
                &SwitchOptions { show_sensor: show, enable_runout, debounce_delay, ..Default::default() },
            )?;
            let l = Rc::downgrade(&lane);
            button.set_action(Rc::new(move |et, state| l.upgrade().map_or(Ok(()), |l| l.handle_load_runout(et, state))));
            button.debounce_delay.set(0.0);
            *lane.fila_load.borrow_mut() = Some(fila);
            *lane.load_debounce_button.borrow_mut() = Some(button);
            lane.set_homing_endstop(&query_endstops, &ppins, p, HOMING_LOAD)?;
        }
        if let Some(p) = &selector {
            let show = enable_sensors_in_gui && sensor_to_show.as_deref().is_none_or(|s| s.contains("selector"));
            let (fila, _) = add_filament_switch(&format!("{}_selector", name), p, &printer, &SwitchOptions { show_sensor: show, ..Default::default() })?;
            *lane.fila_selector.borrow_mut() = Some(fila);
            lane.set_homing_endstop(&query_endstops, &ppins, p, HOMING_SELECTOR)?;
            let l = Rc::downgrade(&lane);
            buttons.register_buttons(
                &[p.as_str()],
                Rc::new(move |_et, state| {
                    if let Some(l) = l.upgrade() {
                        l.selector_state.set(Some(state != 0));
                    }
                    Ok(())
                }),
            )?;
        }
        if let Some(h) = hub.as_deref().filter(|h| !h.contains("direct")) {
            lane.get_hub_object(h)?;
            let pin = lane.hub_obj_opt().and_then(|h| h.switch_pin.clone());
            if let Some(pin) = pin {
                lane.set_homing_endstop(&query_endstops, &ppins, &pin, HOMING_HUB)?;
            }
        }
        if buffer_name.is_some() {
            lane.get_buffer_object()?;
            let pin = lane.buffer_obj().and_then(|b| b.advance_pin.clone());
            if let Some(pin) = pin {
                lane.set_homing_endstop(&query_endstops, &ppins, &pin, HOMING_BUFFER)?;
            }
        }
        if afc_extruder_name.is_some() && !standalone_lane {
            lane.get_extruder_object()?;
            let pin = lane.extruder_obj_opt().and_then(|e| e.tool_start.clone());
            if let Some(pin) = pin.filter(|p| !p.contains("buffer") && p.to_lowercase() != "virtual") {
                lane.set_homing_endstop(&query_endstops, &ppins, &pin, HOMING_TOOL)?;
            }
        }

        let show_macros = afc.show_macros;
        let l = lane.clone();
        afc.function.register_mux_command(
            show_macros,
            "SET_LANE_LOADED",
            "LANE",
            &name,
            Rc::new(move |g| l.cmd_set_lane_loaded(g)),
            CMD_SET_LANE_LOADED_HELP,
            Some(&[("LANE", "string", "lane1")]),
        )?;
        let l = lane.clone();
        afc.function.register_mux_command(
            show_macros,
            "AFC_RECOVER_LANE",
            "LANE",
            &name,
            Rc::new(move |g| l.cmd_afc_recover_lane(g)),
            CMD_AFC_RECOVER_LANE_HELP,
            Some(&[("LANE", "string", "lane1")]),
        )?;

        if fullname.contains("AFC_stepper") {
            // its endstops go on its stepper once the MCUs are identified
            let l = lane.clone();
            let cfg = config.clone();
            kaliru::events::KlippyMcuIdentify::on(&printer, move || l.get_steppers(&cfg));
            return Ok(lane);
        }
        // Should only get this far for AFC_lane configs
        lane.get_steppers(config)?;
        super::vivid::register_select_lane(&lane)?;
        let _ = canvas;
        Ok(lane)
    }

    pub fn me_weak(&self) -> Weak<AfcLane> {
        self.me.clone()
    }

    /// the unit's drive stepper, for a lane without its own
    fn drive(&self) -> Option<Rc<AfcLane>> {
        self.drive_stepper.borrow().clone()
    }

    pub fn selector_endstop_name(&self) -> Endstop {
        self.lookup_endstop(HOMING_SELECTOR)
    }

    pub fn load_endstop_name(&self) -> Endstop {
        self.lookup_endstop(HOMING_LOAD)
    }

    pub fn lookup_endstop_pub(&self, name: &str) -> Endstop {
        self.lookup_endstop(name)
    }

    pub fn rc(&self) -> Rc<AfcLane> {
        self.me.upgrade().expect("AFC lane")
    }

    pub fn unit_obj(&self) -> KResult<Rc<AfcUnit>> {
        self.unit_obj.borrow().clone().ok_or_else(|| no_attr("NoneType", "unit_obj"))
    }

    pub fn unit_obj_opt(&self) -> Option<Rc<AfcUnit>> {
        self.unit_obj.borrow().clone()
    }

    /// hub_obj (the direct-hub stand-in is is_direct_hub(): hub_obj() errors)
    pub fn hub_obj(&self) -> KResult<Rc<AfcHub>> {
        self.hub_obj.borrow().clone().ok_or_else(|| no_attr("NoneType", "state"))
    }

    pub fn hub_obj_opt(&self) -> Option<Rc<AfcHub>> {
        self.hub_obj.borrow().clone()
    }

    pub fn extruder_obj(&self) -> KResult<Rc<AfcExtruder>> {
        self.extruder_obj.borrow().clone().ok_or_else(|| no_attr("AFCLane", "extruder_obj"))
    }

    pub fn extruder_obj_opt(&self) -> Option<Rc<AfcExtruder>> {
        self.extruder_obj.borrow().clone()
    }

    pub fn buffer_obj(&self) -> Option<Rc<AfcBuffer>> {
        self.buffer_obj.borrow().clone()
    }

    /// _get_buffer_object: the [AFC_buffer] the lane names, loaded if need be
    fn get_buffer_object(&self) -> KResult<()> {
        let name = self.buffer_name.borrow().clone().unwrap_or_default();
        let section = format!("AFC_buffer {}", name);
        let obj = self.printer.load_object(&self.config, &section, true).ok().flatten().and_then(kaliru::printer::downcast_rc::<AfcBuffer>);
        match obj {
            Some(b) => {
                *self.buffer_obj.borrow_mut() = Some(b);
                Ok(())
            }
            None => Err(KError::config(format!(
                "Error: No config found for buffer: {} in [{}]. Please make sure [AFC_buffer {}] section exists in your config",
                name, self.fullname, name
            ))),
        }
    }

    pub fn lane_load_count(&self) -> KResult<Rc<StatVar>> {
        self.lane_load_count.borrow().clone().ok_or_else(|| no_attr("NoneType", "increase_count"))
    }

    pub fn status(&self) -> LaneState {
        self.status.get()
    }

    pub fn set_status(&self, s: LaneState) {
        self.status.set(s);
    }

    pub fn settings(&self) -> std::cell::Ref<'_, LaneSettings> {
        self.settings.borrow()
    }

    pub fn short_move_dis(&self) -> f64 {
        self.settings().short_move_dis.unwrap_or(0.0)
    }

    pub fn short_moves_speed(&self) -> f64 {
        self.settings().short_moves_speed.unwrap_or(0.0)
    }

    pub fn short_moves_accel(&self) -> f64 {
        self.settings().short_moves_accel.unwrap_or(0.0)
    }

    pub fn long_moves_speed(&self) -> f64 {
        self.settings().long_moves_speed.unwrap_or(0.0)
    }

    pub fn tool_max_unload_attempts(&self) -> i64 {
        self.settings().tool_max_unload_attempts.unwrap_or(0)
    }

    pub fn assisted_unload(&self) -> bool {
        self.settings().assisted_unload.unwrap_or(false)
    }

    pub fn led_ready(&self) -> String {
        self.settings().led_ready.clone().unwrap_or_default()
    }
    pub fn led_not_ready(&self) -> String {
        self.settings().led_not_ready.clone().unwrap_or_default()
    }
    pub fn led_prep_loaded(&self) -> String {
        self.settings().led_prep_loaded.clone().unwrap_or_default()
    }
    pub fn led_tool_loaded(&self) -> String {
        self.settings().led_tool_loaded.clone().unwrap_or_default()
    }
    pub fn led_fault(&self) -> String {
        self.settings().led_fault.clone().unwrap_or_default()
    }
    pub fn led_loading(&self) -> String {
        self.settings().led_loading.clone().unwrap_or_default()
    }
    pub fn led_unloading(&self) -> String {
        self.settings().led_unloading.clone().unwrap_or_default()
    }
    pub fn led_tool_loaded_idle(&self) -> String {
        self.settings().led_tool_loaded_idle.clone().unwrap_or_default()
    }
    pub fn led_tool_unloaded(&self) -> String {
        self.settings().led_tool_unloaded.clone().unwrap_or_default()
    }
    pub fn led_spool_illum(&self) -> String {
        self.settings().led_spool_illum.clone().unwrap_or_default()
    }
    pub fn led_use_filament_color(&self) -> bool {
        self.settings().led_use_filament_color.unwrap_or(false)
    }
    pub fn remember_spool(&self) -> bool {
        self.remember_spool.borrow().unwrap_or(false)
    }

    fn format_map(mapping: &[String], empty: &str) -> String {
        if mapping.is_empty() {
            return empty.to_string();
        }
        let mut m = mapping.to_vec();
        m.sort_by_key(|a| utils::natural_sort_key(a));
        m.join(", ")
    }

    pub fn map_to_string(&self) -> String {
        Self::format_map(&self.map.borrow(), "NONE")
    }

    /// 'load', or the lane's own load endstop for lanes without a stepper
    pub fn load_es(&self) -> Endstop {
        if self.only_lane.get() {
            self.lookup_endstop(HOMING_LOAD)
        } else {
            Endstop::Name(HOMING_LOAD.into())
        }
    }

    fn lookup_endstop(&self, name: &str) -> Endstop {
        match self.endstops.borrow().get(name) {
            Some(e) => Endstop::Name(e.endstop_name.clone()),
            None => Endstop::Name(name.to_string()),
        }
    }

    pub fn hub_endstop_name(&self) -> Endstop {
        self.lookup_endstop(HOMING_HUB)
    }
    pub fn tool_endstop_name(&self) -> Endstop {
        self.lookup_endstop(HOMING_TOOL)
    }
    pub fn buffer_endstop_name(&self) -> Endstop {
        self.lookup_endstop(HOMING_BUFFER)
    }

    pub fn material_value(&self) -> Option<PyVal> {
        match &*self.material.borrow() {
            PyVal::None => None,
            v => Some(v.clone()),
        }
    }

    pub fn material(&self) -> PyVal {
        self.material.borrow().clone()
    }

    /// material setter: also the density of a known material
    pub fn set_material(&self, value: PyVal) -> KResult<()> {
        *self.material.borrow_mut() = value.clone();
        if !value.truthy() {
            *self.filament_density.borrow_mut() = PyVal::Float(1.24);
            return Ok(());
        }
        let v = value.str();
        for density in &self.afc.common_density_values {
            let parts: Vec<&str> = density.split(':').collect();
            if v.contains(parts[0]) {
                let d = parts.get(1).ok_or_else(|| KError::Py("IndexError".into(), "list index out of range".into()))?;
                *self.filament_density.borrow_mut() = PyVal::Float(utils::py_float_of(&PyVal::Str(d.to_string()))?);
                break;
            }
        }
        Ok(())
    }

    pub fn lane_extruder_index(&self) -> i64 {
        let Some(e) = self.extruder_obj_opt() else { return 0 };
        let th = &e.th_extruder_name;
        if th == "extruder" {
            return 0;
        }
        th.replace("extruder", "").parse().unwrap_or(0)
    }

    fn handle_ready(&self) -> KResult<()> {
        if self.unit_obj_opt().is_none() {
            return Err(KError::config(format!("Unit {} is not defined in your configuration file. Please defined unit ex. [AFC_BoxTurtle {}]", self.unit, self.unit)));
        }
        if let Some(idx) = &self.led_index {
            for (led_name, _) in self.afc.function.parse_led_groups(idx) {
                let (error_string, led) = self.afc.function.verify_led_object(&led_name);
                if led.is_none() {
                    return Err(KError::config(error_string));
                }
            }
        }
        self.espooler.handle_ready();
        if let Some(b) = self.prep_debounce_button.borrow().as_ref() {
            b.debounce_delay.set(self.debounce_delay);
        }
        if let Some(b) = self.load_debounce_button.borrow().as_ref() {
            b.debounce_delay.set(self.debounce_delay);
        }
        // AFC_stepper: an FPS_PSF buffer's software endstops home the stepper
        if self.stepper.borrow().is_some() {
            if let Some(fps) = self.buffer_obj().filter(|b| b.kind == "FPS_PSF") {
                if let Some(f) = fps.fps.as_ref() {
                    if self.extruder_obj()?.tool_start.as_deref() == Some("buffer") {
                        super::stepper::add_prebuilt_endstop(self, "tool_start", &format!("{}_tool_start", self.name), f.fps_endstop.clone())?;
                    }
                    super::stepper::add_prebuilt_endstop(self, "buffer_advance", &format!("{}_buffer_adv", self.name), f.fps_endstop.clone())?;
                    super::stepper::add_prebuilt_endstop(self, "buffer_trailing", &format!("{}_buffer_trailing", self.name), f.fps_trailing_endstop.clone())?;
                }
            }
        }
        Ok(())
    }

    fn get_hub_object(&self, hub: &str) -> KResult<()> {
        match self.printer.load_object(&self.config, &format!("AFC_hub {}", hub), true).ok().flatten().and_then(kaliru::printer::downcast_rc::<AfcHub>) {
            Some(h) => {
                *self.hub_obj.borrow_mut() = Some(h);
                Ok(())
            }
            None => Err(KError::config(format!(
                "Error: No config found for hub: {} in [{}]. Please make sure [AFC_hub {}] section exists in your config",
                hub, self.fullname, hub
            ))),
        }
    }

    fn get_extruder_object(&self) -> KResult<()> {
        let name = self.afc_extruder_name.borrow().clone().unwrap_or_default();
        match self.printer.load_object(&self.config, &format!("AFC_extruder {}", name), true) {
            Ok(Some(o)) => match kaliru::printer::downcast_rc::<AfcExtruder>(o) {
                Some(e) => {
                    *self.extruder_obj.borrow_mut() = Some(e);
                    Ok(())
                }
                None => Err(KError::config(format!("AFC_extruder {} is not an AFC extruder", name))),
            },
            Ok(None) => Err(KError::config(format!("Unable to load module 'AFC_extruder {}'", name))),
            Err(e) => Err(KError::config(format!(
                "Error: No config found for extruder: {} in [{}]. Please make sure [AFC_extruder {}] section exists in your config\nKlippy error: {}",
                name,
                self.fullname,
                name,
                e.message()
            ))),
        }
    }

    /// The pin as an endstop to home to, registered with query_endstops
    fn set_homing_endstop(&self, query_endstops: &QueryEndstops, ppins: &PinsObject, pin: &str, name: &str) -> KResult<()> {
        if pin.trim().to_lowercase() == "virtual" {
            return Ok(());
        }
        let pin_params = {
            let mut pp = ppins.0.borrow_mut();
            pp.allow_multi_use_pin(pin.trim_matches(|c| c == '!' || c == '^')).map_err(|e| KError::Pins(e.0))?;
            pp.parse_pin(pin, true, true).map_err(|e| KError::Pins(e.0))?;
            pp.lookup_pin(pin, true, true, None).map_err(|e| KError::Pins(e.0))?
        };
        let chip = pin_params.borrow().chip.clone();
        let endstop = chip
            .setup_pin("endstop", &pin_params)
            .map_err(KError::Pins)
            .and_then(|o| kaliru::mcu::any_to_endstop(o).ok_or_else(|| KError::Other(format!("{} is not an endstop", pin))))?;
        let endstop_name = format!("{}_{}", self.name, name);
        query_endstops.register_endstop(endstop.clone(), &endstop_name);
        self.endstops.borrow_mut().insert(name.to_string(), LaneEndstop { endstop, endstop_name });
        Ok(())
    }

    fn handle_moonraker_connect(&self) -> KResult<()> {
        let unit = self.unit_obj()?;
        let t = unit.type_name();
        if !EXCLUDE_TYPES.contains(&t.as_str()) || self.fullname.contains("AFC_lane") {
            let moonraker = self.afc.moonraker.borrow().clone().ok_or_else(|| no_attr("NoneType", "get_afc_stats"))?;
            let values = moonraker.get_afc_stats();
            *self.lane_load_count.borrow_mut() = Some(Rc::new(StatVar::new(&self.name, "load_count", values.as_ref(), moonraker.clone(), "", false)));
            self.espooler.handle_moonraker_connect()?;
            let s = self.settings();
            let v = s.td1_when_loaded.unwrap_or(false) && self.afc.td1_defined.get();
            drop(s);
            self.settings.borrow_mut().td1_when_loaded = Some(v);
        }
        Ok(())
    }

    /// <unit>:connect: the unit, hub, extruder and the unit's defaults
    fn handle_unit_connect(&self, unit_obj: &Rc<AfcUnit>) -> KResult<()> {
        let me = self.rc();
        *self.unit_obj.borrow_mut() = Some(unit_obj.clone());
        let mut add_to_other_obj = false;
        if self.remember_spool.borrow().is_none() {
            *self.remember_spool.borrow_mut() = Some(unit_obj.remember_spool);
        }
        let t = unit_obj.type_name();
        if !EXCLUDE_TYPES.contains(&t.as_str()) || self.fullname.contains("AFC_lane") {
            add_to_other_obj = true;
            unit_obj.lanes.borrow_mut().insert(self.name.clone(), me.clone());
            self.afc.lanes.borrow_mut().insert(self.name.clone(), me.clone());
        }
        if self.hub.borrow().is_none() {
            *self.hub_obj.borrow_mut() = unit_obj.hub_obj();
        }
        if !self.is_direct_hub() {
            let hub = self.hub.borrow().clone();
            if let Some(h) = hub {
                self.get_hub_object(&h)?;
            }
            if let Some(h) = self.hub_obj_opt() {
                *self.hub.borrow_mut() = Some(h.name.clone());
                if add_to_other_obj {
                    h.lanes.borrow_mut().insert(self.name.clone(), me.clone());
                }
            }
        } else {
            *self.hub_obj.borrow_mut() = None;
            self.direct_hub.set(true);
        }
        *self.extruder_obj.borrow_mut() = unit_obj.extruder_obj();
        if self.afc_extruder_name.borrow().is_some() {
            self.get_extruder_object()?;
        } else if self.extruder_obj_opt().is_none() {
            return Err(KError::config(format!(
                "Error: Extruder has not been configured for stepper {}, please add extruder variable to either [AFC_stepper {}] or [AFC_{} {}] in your config file",
                self.name,
                self.name,
                t.replace('_', ""),
                unit_obj.name
            )));
        }
        let extruder = self.extruder_obj()?;
        *self.afc_extruder_name.borrow_mut() = Some(extruder.name.clone());
        if add_to_other_obj {
            extruder.lanes.borrow_mut().insert(self.name.clone(), me.clone());
            extruder.check_lanes();
        }
        // Use buffer defined in stepper and override buffers that maybe set at the UNIT or extruder levels
        *self.buffer_obj.borrow_mut() = unit_obj.buffer_obj.borrow().clone();
        if self.buffer_name.borrow().is_some() {
            self.get_buffer_object()?;
        } else if self.buffer_obj().is_none() && extruder.tool_start.as_deref() == Some("buffer") && self.name != extruder.name {
            if let Some(b) = &extruder.buffer_name {
                let obj = self.printer.lookup_object_opt(&format!("AFC_buffer {}", b)).and_then(kaliru::printer::downcast_rc::<AfcBuffer>);
                *self.buffer_obj.borrow_mut() = Some(obj.ok_or_else(|| KError::Other(format!("Unknown config object 'AFC_buffer {}'", b)))?);
            } else {
                return Err(KError::config(format!(
                "Error: Buffer was defined as tool_start in [AFC_extruder {}] config, but buffer variable has not been configured. Please add buffer variable to either [AFC_extruder {}], [AFC_stepper {}] or [AFC_{} {}] section in your config file",
                extruder.name,
                extruder.name,
                self.name,
                t.replace('_', ""),
                unit_obj.name
            )));
            }
        }
        // Valid to not have a buffer defined, check to make sure object exists before adding lane to buffer
        if let Some(b) = self.buffer_obj().filter(|_| add_to_other_obj) {
            b.lanes.borrow_mut().insert(self.name.clone(), me.clone());
            // Assigning buffer name just in case stepper is using buffer defined in units/extruder config
            *self.buffer_name.borrow_mut() = Some(b.name.clone());
        }
        {
            let u = &unit_obj.settings;
            let mut s = self.settings.borrow_mut();
            // a speed the lane leaves unset is the unit's value, int or float
            s.move_ints = super::MoveInts {
                long_speed: s.long_moves_speed.is_none() && u.move_ints.long_speed,
                long_accel: s.long_moves_accel.is_none() && u.move_ints.long_accel,
                short_speed: s.short_moves_speed.is_none() && u.move_ints.short_speed,
                short_accel: s.short_moves_accel.is_none() && u.move_ints.short_accel,
            };
            macro_rules! inherit {
                ($($f:ident),*) => {$( if s.$f.is_none() { s.$f = Some(u.$f.clone()); } )*};
            }
            inherit!(
                led_fault,
                led_ready,
                led_not_ready,
                led_loading,
                led_prep_loaded,
                led_unloading,
                led_tool_loaded,
                led_tool_loaded_idle,
                led_tool_unloaded,
                led_spool_illum,
                led_use_filament_color,
                rev_long_moves_speed_factor,
                long_moves_speed,
                long_moves_accel,
                short_moves_speed,
                short_moves_accel,
                short_move_dis,
                max_move_dis,
                homing_overshoot,
                homing_delta,
                load_then_home_var,
                load_undershoot,
                td1_when_loaded,
                extruder_clear_dis,
                post_prep_macro,
                tool_max_unload_attempts
            );
            if self.td1_device_id.borrow().is_none() {
                *self.td1_device_id.borrow_mut() = u.td1_device_id.clone();
            }
            if s.td1_bowden_length.is_none() && !self.is_direct_hub() {
                s.td1_bowden_length = self.hub_obj_opt().map(|h| h.td1_bowden_length.get());
            }
            let f = s.rev_long_moves_speed_factor.unwrap_or(0.0);
            if f < 0.5 {
                s.rev_long_moves_speed_factor = Some(0.5);
            }
            if s.rev_long_moves_speed_factor.unwrap_or(0.0) > 1.2 {
                s.rev_long_moves_speed_factor = Some(1.2);
            }
        }
        self.espooler.handle_connect(&me)?;
        {
            let mut s = self.settings.borrow_mut();
            let long = self.dist_hub.get() >= 200.0;
            s.dist_hub_move_speed = if long { s.long_moves_speed.unwrap_or(0.0) } else { s.short_moves_speed.unwrap_or(0.0) };
            s.dist_hub_move_accel = if long { s.long_moves_accel.unwrap_or(0.0) } else { s.short_moves_accel.unwrap_or(0.0) };
        }
        type LaneCmd = fn(&AfcLane, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, LaneCmd, &str); 6] = [
            ("SET_LONG_MOVE_SPEED", AfcLane::cmd_set_long_move_speed, CMD_SET_LONG_MOVE_SPEED_HELP),
            ("SET_SPEED_MULTIPLIER", AfcLane::cmd_set_speed_multiplier, CMD_SET_SPEED_MULTIPLIER_HELP),
            ("SAVE_SPEED_MULTIPLIER", AfcLane::cmd_save_speed_multiplier, CMD_SAVE_SPEED_MULTIPLIER_HELP),
            ("SET_HUB_DIST", AfcLane::cmd_set_hub_dist, CMD_SET_HUB_DIST_HELP),
            ("SAVE_HUB_DIST", AfcLane::cmd_save_hub_dist, CMD_SAVE_HUB_DIST_HELP),
            ("AFC_SET_REMEMBER_SPOOL", AfcLane::cmd_afc_set_remember_spool, CMD_AFC_SET_REMEMBER_SPOOL_HELP),
        ];
        for (cmd, f, help) in cmds {
            let l = me.clone();
            self.afc.gcode.register_mux_command(cmd, "LANE", Some(&self.name), Rc::new(move |g| f(&l, g)), Some(help))?;
        }
        {
            let mut s = self.settings.borrow_mut();
            if s.assisted_unload.is_none() {
                s.assisted_unload = Some(unit_obj.settings.assisted_unload);
            }
        }
        self.printer.send_event(REGISTER_MACROS_EVENT, &[&me])?;
        self.connect_done.set(true);
        Ok(())
    }

    /// _get_steppers: the lane's unit, and its endstops on the drive stepper
    fn get_steppers(&self, config: &ConfigWrapper) -> KResult<()> {
        let c = &AFC_LANE_AFCLANE__GET_STEPPERS;
        let res = (|| -> KResult<()> {
            let sections = config.fileconfig.borrow().sections();
            let unit_cfg = sections
                .iter()
                .find(|s| s.contains(&self.unit) && s.starts_with("AFC_") && !INVALID_UNIT_NAMES.iter().any(|x| s.contains(x)))
                .cloned()
                .ok_or_else(|| KError::Py("StopIteration".into(), String::new()))?;
            let unit = self.printer.load_object(config, &unit_cfg, true)?.and_then(kaliru::printer::downcast_rc::<AfcUnit>);
            *self.unit_obj.borrow_mut() = unit.clone();
            let unit = unit.ok_or_else(|| no_attr("NoneType", "stepperless_drive"))?;
            if unit.stepperless_drive.get() {
                return Ok(());
            }
            let mut drive = None;
            if config.get_opt("step_pin")?.is_none_or(|s| s.is_empty()) {
                self.only_lane.set(true);
                drive = unit.selector.borrow().as_ref().map(|s| s.drive_stepper_obj.clone());
                *self.drive_stepper.borrow_mut() = drive.clone();
            } else {
                // drive_stepper is the lane itself (AFC_stepper)
                super::stepper::add_lane_endstops(self)?;
            }
            match &drive {
                Some(d) => super::stepper::add_endstops_to_drive(d, self)?,
                // drive_stepper is the lane itself: its endstops go on its
                // extruder_stepper, which a lane without a stepper does not have
                None if self.stepper.borrow().is_none() && !self.endstops.borrow().values().all(|es| es.endstop_name.contains(HOMING_SELECTOR)) => {
                    return Err(no_attr("NoneType", "stepper"));
                }
                None => {}
            }
            if EXCLUDE_TYPES.contains(&unit.type_name().as_str()) && self.fullname.contains("AFC_lane") {
                let sel = unit.selector.borrow().clone();
                *self.drive_stepper.borrow_mut() = sel.as_ref().map(|s| s.drive_stepper_obj.clone());
                if self.selector.is_some() {
                    if let Some(s) = sel {
                        super::stepper::add_selector_endstop(&s.selector_stepper_obj, self)?;
                    }
                }
            }
            Ok(())
        })();
        if let Err(e) = res {
            let msg = e.message().to_string();
            self.afc.logger.info(c, &format!("Couldn't find unit for {} {}", self.name, msg));
        }
        Ok(())
    }

    pub fn get_color(&self) -> String {
        if let PyVal::Dict(d) = self.td1_data.borrow().get() {
            if let Some(c) = d.get("color") {
                return format!("#{}", c.str());
            }
        }
        self.color.borrow().str()
    }

    /// assist_move(speed, rewind, assist_active): espooler assist around f
    pub fn assist_move<T>(&self, speed: f64, rewind: bool, assist_active: bool, f: impl FnOnce() -> KResult<T>) -> KResult<T> {
        if assist_active {
            let mut value = if rewind { -self.calculate_pwm_value(speed, true)? } else { self.calculate_pwm_value(speed, false)? };
            if value > 1.0 {
                value = 1.0;
            }
            self.espooler.assist(value)?;
        }
        let r = f();
        if assist_active {
            self.espooler.assist(0.0)?;
        }
        r
    }

    /// (speed, accel, assist) for a distance
    fn get_speed_accel_auto(&self, distance: f64) -> (f64, f64, bool) {
        let s = self.settings();
        if distance.abs() > 200.0 {
            (s.long_moves_speed.unwrap_or(0.0), s.long_moves_accel.unwrap_or(0.0), true)
        } else {
            (s.short_moves_speed.unwrap_or(0.0), s.long_moves_accel.unwrap_or(0.0), false)
        }
    }

    pub fn move_auto_speed(&self, distance: f64) -> KResult<()> {
        let (speed, accel, assist) = self.get_speed_accel_auto(distance);
        self.move_lane(distance, speed, accel, assist)
    }

    /// (speed, accel) of a speed mode
    pub fn get_speed_accel(&self, mode: SpeedMode) -> (f64, f64) {
        let s = self.settings();
        if self.afc.get_quiet_mode() {
            return (self.afc.quiet_moves_speed.get(), s.short_moves_accel.unwrap_or(0.0));
        }
        match mode {
            SpeedMode::Long => (s.long_moves_speed.unwrap_or(0.0), s.long_moves_accel.unwrap_or(0.0)),
            SpeedMode::DistHub => (s.dist_hub_move_speed, s.dist_hub_move_accel),
            _ => (s.short_moves_speed.unwrap_or(0.0), s.short_moves_accel.unwrap_or(0.0)),
        }
    }

    /// get_speed_accel's (speed, accel) as python numbers: ints where they
    /// are still AFC's int defaults
    pub fn get_speed_accel_pv(&self, mode: SpeedMode) -> (PyVal, PyVal) {
        let (speed, accel) = self.get_speed_accel(mode);
        let s = self.settings();
        let mi = s.move_ints;
        let (si, ai) = if self.afc.get_quiet_mode() {
            (self.afc.quiet_moves_int.get(), mi.short_accel)
        } else {
            match mode {
                SpeedMode::Long => (mi.long_speed, mi.long_accel),
                SpeedMode::DistHub => {
                    if self.dist_hub.get() >= 200.0 {
                        (mi.long_speed, mi.long_accel)
                    } else {
                        (mi.short_speed, mi.short_accel)
                    }
                }
                _ => (mi.short_speed, mi.short_accel),
            }
        };
        let pv = |v: f64, int: bool| if int { PyVal::Int(v as i64) } else { PyVal::Float(v) };
        (pv(speed, si), pv(accel, ai))
    }

    pub fn get_active_assist(&self, distance: f64, assist_active: AssistActive) -> bool {
        match assist_active {
            AssistActive::Yes => true,
            AssistActive::Dynamic => distance.abs() > 200.0,
            AssistActive::No => false,
        }
    }

    pub fn is_direct_hub(&self) -> bool {
        self.hub.borrow().as_deref().is_some_and(|h| VALID_DIRECT_HUB.contains(&h))
    }

    pub fn is_direct_dist(&self) -> bool {
        self.is_direct_hub() || self.hub_obj_opt().is_some_and(|h| h.use_dist_hub())
    }

    pub fn select_lane(&self) -> KResult<()> {
        self.unit_obj()?.select_lane(&self.rc(), false).map(|_| ())
    }

    /// move(distance, speed, accel, assist_active)
    pub fn move_lane(&self, distance: f64, speed: f64, accel: f64, assist_active: bool) -> KResult<()> {
        if self.stepper.borrow().is_some() {
            return super::stepper::lane_move(self, distance, speed, accel, assist_active);
        }
        if self.canvas.borrow().is_some() {
            return super::canvas::lane_move(self, distance, speed, accel, true);
        }
        self.unit_obj()?.select_lane(&self.rc(), false)?;
        let drive = self.drive_stepper.borrow().clone();
        self.assist_move(speed, distance < 0.0, assist_active, || match drive {
            Some(d) => d.move_lane(distance, speed, accel, assist_active),
            None => Ok(()),
        })
    }

    /// move_to: homing to an endstop, or a plain move
    pub fn move_to(&self, distance: f64, speed_mode: SpeedMode, endstop: &Endstop, assist_active: AssistActive, use_homing: bool) -> KResult<(bool, f64, MoveWarning)> {
        if self.canvas.borrow().is_some() {
            return super::canvas::lane_move_to(self, distance, speed_mode, endstop, use_homing);
        }
        let drive = self.drive_stepper.borrow().clone();
        if self.stepper.borrow().is_some() || drive.is_some() {
            if use_homing {
                // Add extra distance to homing move to guarantee that endstop is hit
                let overshoot = self.settings().homing_overshoot.unwrap_or(0.0);
                let new_distance = if distance < 0.0 { distance - overshoot } else { distance + overshoot };
                self.unit_obj()?.select_lane(&self.rc(), false)?;
                let (speed, accel) = self.get_speed_accel_pv(speed_mode);
                let assist = self.get_active_assist(distance, assist_active);
                let (homed, mov_dis, error) = match &drive {
                    Some(d) => super::stepper::home_to_for(d, self, endstop, new_distance, &speed, &accel, distance > 0.0, true, assist)?,
                    None => super::stepper::home_to(&self.rc(), endstop, new_distance, &speed, &accel, distance > 0.0, true, assist)?,
                };
                let warn = if error {
                    MoveWarning::Error
                } else if (distance.abs() - mov_dis) > self.settings().homing_delta.unwrap_or(0.0) {
                    MoveWarning::Warn
                } else {
                    MoveWarning::None
                };
                return Ok((homed, mov_dis, warn));
            }
            self.move_advanced(distance, speed_mode, assist_active)?;
            return Ok((true, 0.0, MoveWarning::None));
        }
        if self.extruder_obj()?.is_standalone() {
            Ok((true, 0.0, MoveWarning::None))
        } else {
            Ok((false, 0.0, MoveWarning::Error))
        }
    }

    pub fn move_advanced(&self, distance: f64, speed_mode: SpeedMode, assist_active: AssistActive) -> KResult<()> {
        // Stepperless units have no drive stepper: the unit's firmware-driven
        // lane_move does the move
        if let Some(unit) = self.unit_obj_opt().filter(|u| u.stepperless_drive.get()) {
            if let super::unit::UnitKind::Ace(_) = unit.kind {
                return super::ace::lane_move(&unit, self, distance);
            }
        }
        let (speed, accel) = self.get_speed_accel(speed_mode);
        let assist = self.get_active_assist(distance, assist_active);
        self.move_lane(distance, speed, accel, assist)
    }

    pub fn set_afc_prep_done(&self) {
        self.afc_prep_done.set(true);
    }

    fn perform_infinite_runout(&self) -> KResult<()> {
        let c = &AFC_LANE_AFCLANE__PERFORM_INFINITE_RUNOUT;
        self.status.set(LaneState::None);
        self.afc.logger.info(c, &format!("Infinite Spool triggered for {}", self.name));
        let current = self.afc.current();
        let Some(empty_lane) = current.as_deref().and_then(|c| self.afc.lane(c)) else {
            self.afc.error.afc_error(c, &format!("Error when looking up current lane:{}", py_opt_str(current.as_deref())), true, None)?;
            return Ok(());
        };
        let runout = self.runout_lane.borrow().clone();
        let Some(change_lane) = runout.as_deref().and_then(|r| self.afc.lane(r)) else {
            self.afc.error.afc_error(c, &format!("Error when looking up runout lane:{} for lane:{}", py_opt_str(runout.as_deref()), self.name), true, None)?;
            return Ok(());
        };
        change_lane.status.set(LaneState::InfiniteRunout);
        self.afc.error.pause_resume()?.send_pause_command()?;
        self.afc.save_pos()?;
        self.afc.change_tool(&change_lane, None, false, None)?;
        if !self.afc.error_state.get() {
            let r = self.afc.gcode.run_script_from_command(&format!("AFC_SWAP_MAPPING FROM={} TO={}", empty_lane.name, change_lane.name));
            match r {
                Err(e) if e.is_command_error() => {
                    self.afc.error.afc_error(c, &format!("Error when trying swap mapping from {} to {}", empty_lane.name, change_lane.name), true, None)?;
                    return Ok(());
                }
                r => r?,
            }
            if !Rc::ptr_eq(&empty_lane.extruder_obj()?, &change_lane.extruder_obj()?) {
                empty_lane.extruder_obj()?.set_print_leds(0, true)?;
                change_lane.extruder_obj()?.set_print_leds(1, true)?;
                change_lane.unit_obj()?.lane_tool_loaded(&change_lane)?;
            }
            self.afc.restore_pos(true)?;
            self.afc.error.pause_resume()?.send_resume_command()?;
            self.unit_obj()?.lane_not_ready(&self.rc())?;
        }
        Ok(())
    }

    fn handle_auto_spool_switch(&self) -> KResult<()> {
        if self.afc.error_state.get() || !self.afc.function.is_printing(false)? {
            return Ok(());
        }
        if self.runout_lane.borrow().is_some() {
            self.perform_infinite_runout()
        } else {
            self.perform_pause_runout()
        }
    }

    fn perform_pause_runout(&self) -> KResult<()> {
        let c = &AFC_LANE_AFCLANE__PERFORM_PAUSE_RUNOUT;
        let me = self.rc();
        if self.unit_obj()?.unload_on_runout {
            self.afc.error.pause_resume()?.send_pause_command()?;
            self.afc.save_pos()?;
            let unloaded = self.afc.tool_unload(&me, true, false)?;
            if unloaded && !self.afc.error_state.get() {
                self.afc.lane_unload(&me)?;
            } else {
                self.afc.afc_stats()?.increase_unload_error_count(&self.afc)?;
            }
        }
        self.status.set(LaneState::None);
        let runout_method = if self.auto_switch_triggered.get() { "Minimum weight threshold reached" } else { "Runout triggered" };
        let mut msg = format!("{} for lane {} and runout lane is not setup to switch to another lane.", runout_method, self.name);
        msg += "\nPlease manually load next spool into toolhead and then hit resume to continue.";
        self.unit_obj()?.lane_not_ready(&me)?;
        self.afc.error.afc_error(c, &msg, true, None)
    }

    fn prep_capture_td1(&self) -> KResult<()> {
        if self.settings().td1_when_loaded.unwrap_or(false) {
            if !self.hub_obj()?.state() && self.afc.function.get_current_lane_obj().is_none() {
                self.get_td1_data()?;
            } else {
                self.afc.logger.info(
                    &AFC_LANE_AFCLANE__PREP_CAPTURE_TD1,
                    &format!("Cannot get TD-1 data for {}, either toolhead is loaded or hub shows filament in path", self.name),
                );
            }
        }
        Ok(())
    }

    /// load_state (a virtual hub pin: loaded_to_hub)
    pub fn load_state(&self) -> bool {
        if self.hub_obj_opt().is_some_and(|h| h.is_virtual_pin()) {
            self.loaded_to_hub.get()
        } else {
            self.load_state_raw.get()
        }
    }

    pub fn raw_load_state(&self) -> bool {
        self.load_state_raw.get()
    }

    fn load_callback(&self, _eventtime: f64, state: bool) -> KResult<()> {
        self.load_state_raw.set(state);
        if self.printer.state_message() == "Printer is ready" && ONLY_LOAD_TYPES.contains(&self.unit_obj()?.type_name().as_str()) {
            self.prep_state.set(state);
        }
        Ok(())
    }

    pub fn handle_load_runout(&self, eventtime: f64, load_state: bool) -> KResult<()> {
        let c = &AFC_LANE_AFCLANE_HANDLE_LOAD_RUNOUT;
        let me = self.rc();
        if let Some(b) = self.load_debounce_button.borrow().clone() {
            b.old_note_filament_present(eventtime, load_state)?;
        }
        if self.printer.state_message() == "Printer is ready" && ONLY_LOAD_TYPES.contains(&self.unit_obj()?.type_name().as_str()) && self.afc_prep_done.get() {
            if load_state {
                *self.afc_staged_spool_id.borrow_mut() = self.afc.spool.next_spool_id.borrow().clone();
                self.set_loaded()?;
                if !self.load_suppressed.get() {
                    self.unit_obj()?.on_filament_insert(&me)?;
                }
                self.load_suppressed.set(false);
                if self.td1_device_id.borrow().is_some() && !self.tool_loaded.get() {
                    self.prep_capture_td1()?;
                }
                if self.hub.borrow().as_deref() == Some("direct_load") {
                    if self.afc.function.is_printing(true)? {
                        self.afc.error.afc_error(c, "Cannot load spool to toolhead while printer is actively moving or homing", false, None)?;
                    } else if !self.afc.tool_load(&me, None, false)? {
                        self.afc.afc_stats()?.increase_load_error_count(&self.afc)?;
                    }
                }
                self.post_prep_user_macro()?;
            } else {
                self.unit_obj()?.on_filament_remove(&me)?;
                let disabled = self.fila_load.borrow().as_ref().is_some_and(|f| !f.runout_helper.sensor_enabled.get());
                if disabled && self.afc.function.is_printing(false)? {
                    self.afc.logger.warning(c, "Load runout has been detected, but pause and runout detection has been disabled");
                } else if self.unit_obj()?.check_runout(&me)? {
                    if self.runout_lane.borrow().is_some() {
                        self.perform_infinite_runout()?;
                    } else {
                        self.perform_pause_runout()?;
                    }
                } else if self.status.get().as_str() != "calibrating" {
                    self.set_unloaded()?;
                }
            }
        }
        self.afc.save_vars()
    }

    /// The prep sensor: loads the lane when filament is inserted
    fn prep_callback(&self, eventtime: f64, state: bool) -> KResult<()> {
        let c = &AFC_LANE_AFCLANE_PREP_CALLBACK;
        let me = self.rc();
        self.prep_state.set(state);
        let delta_time = eventtime - self.last_prep_time.get();
        self.last_prep_time.set(eventtime);
        if self.prep_active.get() {
            return Ok(());
        }
        let ready = self.printer.state_message() == "Printer is ready";
        if ready && self.afc_prep_done.get() && self.hub.borrow().as_deref() == Some("direct_load") && !self.afc.auto_home && !self.afc.function.is_homed(false)? {
            self.afc.error.afc_error(c, "Please home printer before directly loading to toolhead", false, None)?;
            return Ok(());
        }
        self.prep_active.set(true);
        // `with self.mutex:` - held for the body, released on every exit
        let res = self.mutex.lock().map_err(KError::from).and_then(|()| {
            let r = (|| -> KResult<bool> {
                if self.printer.state_message() == "Printer is ready" && self.afc_prep_done.get() && self.status.get() != LaneState::ToolUnloading {
                    if self.prep_state.get() && !self.raw_load_state() {
                        self.afc.logger.debug(c, &format!("Prep: callback triggered {}", self.name));
                        if delta_time < 1.0 {
                            return Ok(true);
                        }
                        let other_active = self.afc.lanes.borrow().values().any(|l| !Rc::ptr_eq(l, &me) && l.prep_active.get());
                        if self.afc.function.is_printing(true)? || other_active {
                            self.afc.error.afc_error(c, &format!("Cannot load {} spool while printer is actively moving or homing", self.name), false, None)?;
                            return Ok(false);
                        }
                        self.afc.last_prep_activity_time.set(eventtime);
                        let unit = self.unit_obj()?;
                        unit.lane_loading(&me)?;
                        unit.prep_load(&me)?;
                        self.status.set(LaneState::None);
                        self.afc.logger.debug(c, &format!("Prep: Load Done-{}", self.name));
                        if self.hub.borrow().as_deref() == Some("direct_load") && self.prep_state.get() {
                            self.afc.logger.debug(c, &format!("Prep: direct load logic-{}-{}", self.name, py_opt_str(self.hub.borrow().as_deref())));
                            if !self.afc.tool_load(&me, None, false)? {
                                self.afc.afc_stats()?.increase_load_error_count(&self.afc)?;
                            }
                            let l = me.clone();
                            self.afc.spool.set_values(&me, Some(Box::new(move || l.afc.spool.set_active_spool(&l.spool_id.borrow().clone()))))?;
                            self.afc.logger.debug(c, &format!("Prep: direct load logic done-{}-{}", self.name, py_opt_str(self.hub.borrow().as_deref())));
                            return Ok(true);
                        }
                        unit.prep_post_load(&me)?;
                        self.do_enable(false)?;
                        if self.load_state() && self.prep_state.get() {
                            self.set_loaded()?;
                            self.post_prep_user_macro()?;
                            if self.td1_device_id.borrow().is_some() {
                                self.prep_capture_td1()?;
                            }
                        }
                    } else if self.prep_state.get() && self.raw_load_state() && !self.afc.function.is_printing(false)? {
                        let mut message = format!("Cannot load {} load sensor is triggered.", self.name);
                        message += "\n    Make sure filament is not stuck in load sensor or check to make sure load sensor is not stuck triggered.";
                        if self.unit_obj()?.type_name() == "ViViD" {
                            message += &format!("\n    If filament is not stuck in sensor run AFC_RECOVER_LANE LANE={}", self.name);
                            message += " to reset internal AFC state.";
                        }
                        message += "\n    Once cleared try loading again";
                        self.afc.error.afc_error(c, &message, false, None)?;
                    }
                }
                Ok(true)
            })();
            self.mutex.unlock();
            r
        });
        self.prep_active.set(false);
        if res? {
            self.afc.save_vars()?;
        }
        Ok(())
    }

    /// The prep sensor's delayed runout event
    fn handle_prep_runout(&self, eventtime: f64, prep_state: bool) -> KResult<()> {
        let c = &AFC_LANE_AFCLANE_HANDLE_PREP_RUNOUT;
        if let Some(b) = self.prep_debounce_button.borrow().clone() {
            b.old_note_filament_present(eventtime, prep_state)?;
        }
        if self.printer.state_message() == "Printer is ready" && self.afc_prep_done.get() && self.status.get() != LaneState::ToolUnloading {
            if !prep_state
                && self.afc.current().as_deref() == Some(self.name.as_str())
                && self.afc.function.is_printing(false)?
                && self.raw_load_state()
                && self.status.get() != LaneState::Ejecting
            {
                let enabled = self.fila_prep.borrow().as_ref().map(|f| f.runout_helper.sensor_enabled.get()).ok_or_else(|| no_attr("AFCLane", "fila_prep"))?;
                if !enabled {
                    self.afc.logger.warning(c, "Prep runout has been detected, but pause and runout detection has been disabled");
                } else if self.runout_lane.borrow().is_some() {
                    self.perform_infinite_runout()?;
                } else {
                    self.perform_pause_runout()?;
                }
            } else if !prep_state {
                if self.any_lane_prep_active() || (eventtime - self.afc.last_prep_activity_time.get()) < PREP_GUARD_WINDOW {
                    if !self.prep_recheck_pending.get() {
                        self.prep_recheck_pending.set(true);
                        let l = self.rc();
                        self.reactor.register_callback(
                            move |et| {
                                if let Err(e) = l.recheck_prep_runout(et) {
                                    kaliru::logging::exception_err("AFC prep recheck", &e);
                                }
                            },
                            self.reactor.monotonic() + PREP_GUARD_WINDOW,
                        );
                    }
                    return Ok(());
                }
                self.set_unloaded()?;
            }
        }
        self.afc.save_vars()
    }

    fn any_lane_prep_active(&self) -> bool {
        self.afc.lanes.borrow().values().any(|l| l.prep_active.get())
    }

    fn recheck_prep_runout(&self, eventtime: f64) -> KResult<()> {
        if self.prep_state.get() {
            self.prep_recheck_pending.set(false);
            return Ok(());
        }
        if self.any_lane_prep_active() || (eventtime - self.afc.last_prep_activity_time.get()) < PREP_GUARD_WINDOW {
            let l = self.rc();
            self.reactor.register_callback(
                move |et| {
                    if let Err(e) = l.recheck_prep_runout(et) {
                        kaliru::logging::exception_err("AFC prep recheck", &e);
                    }
                },
                self.reactor.monotonic() + PREP_GUARD_WINDOW,
            );
            return Ok(());
        }
        self.prep_recheck_pending.set(false);
        self.set_unloaded()?;
        self.afc.save_vars()
    }

    fn post_prep_user_macro(&self) -> KResult<()> {
        let m = self.settings().post_prep_macro.clone();
        if self.afc.function.check_macro_present(m.as_deref()) {
            self.afc.gcode.run_script_from_command(&format!("{} LANE={}", m.unwrap_or_default(), self.name))?;
        }
        Ok(())
    }

    pub fn do_enable(&self, enable: bool) -> KResult<()> {
        if self.stepper.borrow().is_some() {
            return super::stepper::lane_do_enable(self, enable);
        }
        if let Some(d) = self.drive() {
            return d.do_enable(enable);
        }
        if self.canvas.borrow().is_some() {
            return super::canvas::lane_do_enable(self, enable);
        }
        Ok(())
    }

    pub fn sync_to_extruder(&self, update_current: bool) -> KResult<()> {
        if self.stepper.borrow().is_some() {
            let th = self.extruder_obj()?.th_extruder_name.clone();
            return super::stepper::lane_sync_to_extruder(self, update_current, Some(&th));
        }
        if let Some(d) = self.drive() {
            let th = self.extruder_obj()?.th_extruder_name.clone();
            return super::stepper::lane_sync_to_extruder(&d, update_current, Some(&th));
        }
        Ok(())
    }

    pub fn unsync_to_extruder(&self, update_current: bool) -> KResult<()> {
        if self.stepper.borrow().is_some() {
            return super::stepper::lane_unsync_to_extruder(self, update_current);
        }
        if let Some(d) = self.drive() {
            return d.unsync_to_extruder(update_current);
        }
        Ok(())
    }

    pub fn set_load_current(&self) -> KResult<()> {
        if self.stepper.borrow().is_some() {
            return super::stepper::set_load_current(self);
        }
        if let Some(d) = self.drive() {
            return d.set_load_current();
        }
        Ok(())
    }

    pub fn set_print_current(&self) -> KResult<()> {
        if self.stepper.borrow().is_some() {
            return super::stepper::set_print_current(self);
        }
        if let Some(d) = self.drive() {
            return d.set_print_current();
        }
        Ok(())
    }

    pub fn update_rotation_distance(&self, multiplier: f64) -> KResult<()> {
        if self.stepper.borrow().is_some() {
            return super::stepper::update_rotation_distance(self, multiplier);
        }
        if let Some(d) = self.drive() {
            return d.update_rotation_distance(multiplier);
        }
        Ok(())
    }

    #[allow(clippy::approx_constant)] // (python's 3.14159)
    fn calculate_effective_diameter(&self, weight_g: f64, spool_width_mm: f64) -> KResult<f64> {
        let density_g_mm3 = utils::py_float_of(&self.filament_density.borrow())? / 1000.0;
        let filament_volume_mm3 = weight_g / density_g_mm3;
        let package_corrected_volume_mm3 = filament_volume_mm3 / 0.785;
        let filament_area_mm2 = package_corrected_volume_mm3 / spool_width_mm;
        let spool_outer_diameter_mm2 = (4.0 * filament_area_mm2 / 3.14159) + self.inner_diameter.powi(2);
        Ok(spool_outer_diameter_mm2.powf(0.5))
    }

    fn calculate_rpm(&self, feed_rate: f64) -> KResult<f64> {
        let weight = utils::py_float_of(&self.weight.borrow())? + utils::py_float_of(&self.empty_spool_weight.borrow())?;
        let effective_diameter = self.calculate_effective_diameter(weight, 60.0)?;
        let rpm = (feed_rate * 60.0) / (std::f64::consts::PI * effective_diameter);
        Ok(kaliru::pyfmt::py_min(rpm, self.max_motor_rpm.as_f64().unwrap_or(0.0)))
    }

    fn calculate_pwm_value(&self, feed_rate: f64, rewind: bool) -> KResult<f64> {
        let rpm = self.calculate_rpm(feed_rate)?;
        let max_motor_rpm = self.max_motor_rpm.as_f64().unwrap_or(0.0);
        let pwm_value = if !rewind {
            rpm / (max_motor_rpm / (1.0 + 9.0 * self.fwd_speed_multi.get()))
        } else {
            rpm / (max_motor_rpm / (15.0 + 15.0 * self.rwd_speed_multi.get()))
        };
        Ok(kaliru::pyfmt::py_max(0.0, kaliru::pyfmt::py_min(pwm_value, 1.0)))
    }

    pub fn enable_weight_timer(&self) -> KResult<()> {
        let past = self.afc.function.get_extruder_pos(None, Some(self.past_extruder_position.get()), Some(self.extruder_obj()?.toolhead_extruder()?))?;
        self.past_extruder_position.set(past);
        self.reactor.update_timer(&self.cb_update_weight, self.reactor.monotonic() + UPDATE_WEIGHT_DELAY);
        Ok(())
    }

    pub fn disable_weight_timer(&self) -> KResult<()> {
        self.update_weight_callback(None)?;
        self.reactor.update_timer(&self.cb_update_weight, NEVER);
        self.past_extruder_position.set(-1.0);
        self.save_counter.set(-1);
        self.afc.save_vars()
    }

    fn update_weight_callback(&self, eventtime: Option<f64>) -> KResult<f64> {
        let c = &AFC_LANE_AFCLANE_UPDATE_WEIGHT_CALLBACK;
        let extruder_pos = self.afc.function.get_extruder_pos(eventtime, Some(self.past_extruder_position.get()), Some(self.extruder_obj()?.toolhead_extruder()?))?;
        let delta_length = extruder_pos - self.past_extruder_position.get();
        if self.past_extruder_position.get() == -1.0 {
            self.past_extruder_position.set(extruder_pos);
        }
        self.save_counter.set(self.save_counter.get() + 1);
        if extruder_pos > self.past_extruder_position.get() {
            self.update_remaining_weight(delta_length)?;
            self.past_extruder_position.set(extruder_pos);
            let weight = utils::py_float_of(&self.weight.borrow())?;
            if self.afc.auto_spool_switch
                && !self.auto_switch_triggered.get()
                && weight > 0.0
                && weight <= self.afc.auto_spool_switch_threshold
                && self.afc.current().as_deref() == Some(self.name.as_str())
                && self.afc.function.is_printing(false)?
                && !self.afc.error_state.get()
            {
                self.auto_switch_triggered.set(true);
                self.afc.logger.info(
                    c,
                    &format!(
                        "Auto spool switch: {} weight ({:.1}g) at or below threshold ({:.1}g), triggering switch",
                        self.name, weight, self.afc.auto_spool_switch_threshold
                    ),
                );
                let l = self.rc();
                self.reactor.register_callback(
                    move |_| {
                        if let Err(e) = l.handle_auto_spool_switch() {
                            kaliru::logging::exception_err("AFC auto spool switch", &e);
                        }
                    },
                    kaliru::reactor::NOW,
                );
            }
            if self.save_counter.get() as f64 > 120.0 / UPDATE_WEIGHT_DELAY {
                self.afc.save_vars()?;
                self.save_counter.set(0);
            }
        }
        Ok(self.reactor.monotonic() + UPDATE_WEIGHT_DELAY)
    }

    fn update_remaining_weight(&self, distance_moved: f64) -> KResult<()> {
        let diameter = utils::py_float_of(&self.filament_diameter.borrow())?;
        let filament_volume_mm3 = std::f64::consts::PI * (diameter / 2.0).powi(2) * distance_moved;
        let filament_weight_change = filament_volume_mm3 * utils::py_float_of(&self.filament_density.borrow())? / 1000.0;
        let w = self.weight.borrow().clone();
        let mut new = utils::py_sub(&w, &PyVal::Float(filament_weight_change))?;
        if new.as_f64().is_some_and(|v| v < 0.0) {
            new = PyVal::Int(0);
        }
        *self.weight.borrow_mut() = new;
        Ok(())
    }

    pub fn set_loaded(&self) -> KResult<()> {
        let me = self.rc();
        self.status.set(LaneState::Loaded);
        self.unit_obj()?.lane_loaded(&me, false)?;
        self.afc.spool.set_values(&me, None)
    }

    pub fn set_unloaded(&self) -> KResult<()> {
        let me = self.rc();
        self.tool_loaded.set(false);
        self.status.set(LaneState::None);
        self.loaded_to_hub.set(false);
        *self.td1_data.borrow_mut() = PyVal::dict();
        if !self.remember_spool.borrow().unwrap_or(false) {
            self.afc.spool.clear_values(&me)?;
        }
        self.unit_obj()?.lane_not_ready(&me)
    }

    pub fn set_tool_loaded(&self, normal_toolchange: bool) -> KResult<()> {
        let me = self.rc();
        self.tool_loaded.set(true);
        *self.extruder_obj()?.lane_loaded.borrow_mut() = Some(self.name.clone());
        self.status.set(LaneState::Tooled);
        if normal_toolchange {
            *self.afc.current_loading.borrow_mut() = None;
            self.afc.spool.set_active_spool(&self.spool_id.borrow().clone())?;
        }
        self.unit_obj()?.lane_tool_loaded(&me)
    }

    pub fn set_tool_unloaded(&self, normal_toolchange: bool) -> KResult<()> {
        let me = self.rc();
        self.tool_loaded.set(false);
        *self.extruder_obj()?.lane_loaded.borrow_mut() = None;
        self.status.set(LaneState::None);
        if normal_toolchange {
            *self.afc.current_loading.borrow_mut() = None;
            self.afc.spool.set_active_spool(&PyVal::None)?;
        }
        self.unit_obj()?.lane_tool_unloaded(&me)
    }

    pub fn enable_buffer(&self, disable_fault: bool) -> KResult<()> {
        if let Some(b) = self.buffer_obj() {
            if disable_fault {
                b.disable_fault_sensitivity();
            }
            b.enable_buffer(&self.rc())?;
        }
        self.espooler.enable_timer()?;
        self.enable_weight_timer()
    }

    /// The buffer's fault sensitivity back on, re-enabling the buffer
    pub fn enable_fault_detection(&self) -> KResult<()> {
        if let Some(b) = self.buffer_obj() {
            b.restore_fault_sensitivity();
            b.enable_buffer(&self.rc())?;
        }
        Ok(())
    }

    pub fn disable_buffer(&self) -> KResult<()> {
        if let Some(b) = self.buffer_obj() {
            b.disable_buffer()?;
        }
        self.espooler.disable_timer();
        self.disable_weight_timer()
    }

    pub fn buffer_status(&self) -> Option<String> {
        self.buffer_obj().map(|b| b.buffer_status())
    }

    pub fn get_toolhead_pre_sensor_state(&self) -> KResult<bool> {
        let e = self.extruder_obj()?;
        if e.tool_start.as_deref() == Some("buffer") {
            return Ok(self.buffer_obj().ok_or_else(|| no_attr("NoneType", "advance_state"))?.advance_state.get());
        }
        Ok(e.tool_start_state.get())
    }

    pub fn get_toolhead_endstop(&self) -> KResult<Endstop> {
        if self.extruder_obj()?.tool_start.as_deref() == Some("buffer") {
            Ok(self.buffer_endstop_name())
        } else {
            Ok(self.tool_endstop_name())
        }
    }

    pub fn get_trailing(&self) -> Option<bool> {
        self.buffer_obj().map(|b| b.trailing_state.get())
    }

    fn is_normal_printing_state(&self) -> bool {
        if self.afc.in_toolchange.get() {
            return false;
        }
        matches!(self.status.get(), LaneState::Tooled | LaneState::Loaded)
    }

    /// The toolhead sensor's runout for the loaded lane
    pub fn handle_toolhead_runout(&self, sensor: &str) -> KResult<()> {
        let c = &AFC_LANE_AFCLANE_HANDLE_TOOLHEAD_RUNOUT;
        let extruder = self.extruder_obj()?;
        if !(self.is_normal_printing_state() && self.afc.function.is_printing(false)? && extruder.on_shuttle()) {
            return Ok(());
        }
        let prep_ok = self.prep_state.get();
        let load_ok = self.raw_load_state();
        let hub_ok = self.hub_obj_opt().map(|h| h.state()).unwrap_or(true);
        if prep_ok && load_ok && hub_ok && !extruder.is_standalone() {
            let msg = format!(
                "Toolhead runout detected by {} sensor, but upstream sensors still detect filament.\nPossible filament break or jam at the toolhead. Please clear the jam and reload filament manually, then resume the print.",
                sensor
            );
            self.afc.error.pause_resume()?.send_pause_command()?;
            self.afc.save_pos()?;
            self.afc.error.afc_error(c, &msg, true, None)?;
        } else {
            let enabled = extruder.tool_sensor_enabled(sensor)?;
            let sensor_name = if sensor.is_empty() { "toolhead" } else { sensor };
            if !enabled {
                self.afc.logger.warning(c, &format!("{} runout has been detected, but pause and runout detection has been disabled", sensor_name));
            } else if self.runout_lane.borrow().is_some() && extruder.is_standalone() {
                self.perform_infinite_runout()?;
            } else {
                self.perform_pause_runout()?;
                if extruder.is_standalone() {
                    self.set_tool_unloaded(false)?;
                    self.set_unloaded()?;
                    self.afc.save_vars()?;
                }
            }
        }
        Ok(())
    }

    /// The hub sensor's runout for the loaded lane (a break or jam at the hub)
    pub fn handle_hub_runout(&self, sensor: &str) -> KResult<()> {
        let c = &AFC_LANE_AFCLANE_HANDLE_HUB_RUNOUT;
        if !(self.is_normal_printing_state() && self.afc.function.is_printing(false)?) {
            return Ok(());
        }
        let prep_ok = self.prep_state.get();
        let load_ok = self.raw_load_state();
        let hub_ok = self.hub_obj_opt().map(|h| h.state()).unwrap_or(false);
        if prep_ok && load_ok && !hub_ok {
            let msg = format!(
                "Hub runout detected by {} sensor, but upstream sensors still detect filament.\nPossible filament break or jam at the hub. Please clear the jam and reload filament manually, then resume the print.",
                if sensor.is_empty() { "hub" } else { sensor }
            );
            self.afc.error.pause_resume()?.send_pause_command()?;
            self.afc.save_pos()?;
            self.afc.error.afc_error(c, &msg, true, None)?;
        }
        Ok(())
    }

    /// tool_swap: the toolchanger picks up this lane's tool
    pub fn tool_swap(&self) -> KResult<()> {
        let unit = self.extruder_obj()?.tc_unit_obj.borrow().clone();
        if let Some(u) = unit {
            super::toolchanger::tool_swap(&u, &self.rc(), false)?;
        }
        Ok(())
    }

    /// activate_toolhead_extruder: the lane's extruder as the toolhead's
    pub fn activate_toolhead_extruder(&self) -> KResult<()> {
        let toolhead = self.afc.toolhead()?;
        let ext = self.extruder_obj()?.toolhead_extruder()?;
        if Rc::ptr_eq(&toolhead.get_extruder(), &ext) {
            return Ok(());
        }
        // Code below is pulled exactly from klippy/kinematics/extruder.py file without the prints
        toolhead.flush_step_generation()?;
        toolhead.set_extruder(ext, 0.0);
        self.printer.send_event("extruder:activate_extruder", &[])
    }

    /// the lane's unit, set by the unit's connect (or, for a tool's own lane,
    /// by its extruder)
    pub fn set_unit_obj(&self, unit: &Rc<AfcUnit>) {
        *self.unit_obj.borrow_mut() = Some(unit.clone());
    }

    fn mapped_keys(&self) -> Vec<String> {
        self.map.borrow().iter().filter(|m| *m != "NONE").cloned().collect()
    }

    /// One moonraker lane_data record per mapped T(n)
    pub fn send_lane_data(&self) -> KResult<()> {
        let Some(moonraker) = self.afc.moonraker.borrow().clone() else { return Ok(()) };
        let current_keys = self.mapped_keys();
        for key in self.sent_lane_data_keys.borrow().iter() {
            if current_keys.contains(key) {
                continue;
            }
            let still_claimed = self.afc.lanes.borrow().values().any(|l| l.map.borrow().contains(key));
            if !still_claimed {
                moonraker.remove_database_entry("lane_data", key);
            }
        }
        let td1 = |k: &str| -> PyVal {
            match self.td1_data.borrow().get() {
                PyVal::Dict(d) => d.get(k).cloned().unwrap_or(PyVal::Str(String::new())),
                _ => PyVal::Str(String::new()),
            }
        };
        for key in &current_keys {
            let value = kaliru::pydict! {
                "color" => self.color.borrow().clone(),
                "material" => self.material(),
                "vendor_name" => self.spool_vendor.borrow().clone(),
                "name" => self.filament_name.borrow().clone(),
                "bed_temp" => self.bed_temp.borrow().clone(),
                "nozzle_temp" => self.extruder_temp.borrow().clone(),
                "scan_time" => td1("scan_time"),
                "td" => td1("td"),
                "lane" => PyVal::Str(key.replace('T', "")),
                "extruder_index" => PyVal::Int(self.lane_extruder_index()),
                "spool_id" => self.spool_id.borrow().clone(),
                "weight" => self.weight.borrow().clone(),
                "initial_weight" => self.espooler.full_weight(),
            };
            let lane_data = kaliru::pydict! {
                "namespace" => PyVal::from("lane_data"),
                "key" => PyVal::Str(key.clone()),
                "value" => PyVal::Dict(value),
            };
            moonraker.send_lane_data(PyVal::Dict(lane_data));
        }
        *self.sent_lane_data_keys.borrow_mut() = current_keys;
        Ok(())
    }

    /// Empty lane_data records for every mapped T(n)
    pub fn clear_lane_data(&self) -> KResult<()> {
        let Some(moonraker) = self.afc.moonraker.borrow().clone() else { return Ok(()) };
        let current_keys = self.mapped_keys();
        for key in &current_keys {
            let e = || PyVal::Str(String::new());
            let value = kaliru::pydict! {
                "color" => e(),
                "material" => e(),
                "vendor_name" => e(),
                "name" => e(),
                "bed_temp" => e(),
                "nozzle_temp" => e(),
                "scan_time" => e(),
                "td" => e(),
                "lane" => PyVal::Str(key.replace('T', "")),
                "extruder_index" => PyVal::Int(self.lane_extruder_index()),
                "spool_id" => PyVal::None,
                "weight" => PyVal::Int(0),
                "initial_weight" => PyVal::Int(0),
            };
            moonraker.send_lane_data(PyVal::Dict(kaliru::pydict! {
                "namespace" => PyVal::from("lane_data"),
                "key" => PyVal::Str(key.clone()),
                "value" => PyVal::Dict(value),
            }));
        }
        *self.sent_lane_data_keys.borrow_mut() = current_keys;
        Ok(())
    }

    pub fn get_td1_data_load(&self) -> KResult<()> {
        if !self.afc.td1_present()? || self.td1_device_id.borrow().as_deref().is_none_or(|s| s.is_empty()) {
            return Ok(());
        }
        let Some(moonraker) = self.afc.moonraker.borrow().clone() else { return Ok(()) };
        let l = self.rc();
        moonraker.get_td1_data_async(Box::new(move |devices| {
            let id = l.td1_device_id.borrow().clone();
            let (valid, _) = l.afc.function.check_td1_id_in_data(devices.as_ref(), id.as_deref());
            if valid {
                l.unit_obj()?.apply_td1_data(&l, utils::datetime_now(), devices.as_ref(), true)?;
            }
            Ok(())
        }));
        Ok(())
    }

    /// Captures TD-1 data for the lane: (status, message)
    pub fn get_td1_data(&self) -> KResult<(bool, String)> {
        let c = &AFC_LANE_AFCLANE_GET_TD1_DATA;
        let me = self.rc();
        let mut status = true;
        let mut msg = String::new();
        let td1_id = self.td1_device_id.borrow().clone();
        if td1_id.is_none() {
            let msg = format!("Cannot grab TD-1 data for {}, td1_device_id is a required field in AFC_hub or per AFC_lane", self.name);
            self.afc.error.afc_error(c, &msg, false, None)?;
            return Ok((false, msg));
        }
        let td1_bowden_length = self.settings().td1_bowden_length;
        let Some(td1_bowden_length) = td1_bowden_length else {
            let msg = format!(
                "td1_bowden_length is not set for {}. Please run AFC_CALIBRATION to calibrate TD1 length for lane before trying to capture TD1 data.",
                self.name
            );
            self.afc.error.afc_error(c, &msg, false, None)?;
            return Ok((false, msg));
        };
        if !self.load_state() && !self.prep_state.get() {
            let msg = format!("{} not loaded, cannot capture TD-1 data for lane", self.name);
            self.afc.error.afc_error(c, &msg, false, None)?;
            return Ok((false, msg));
        }
        let hub = self.hub_obj()?;
        if hub.state() {
            let msg = format!("Hub for {} detects filament, cannot capture TD-1 data for lane", self.name);
            self.afc.error.afc_error(c, &msg, false, None)?;
            return Ok((false, msg));
        }
        if self.is_direct_hub() && self.tool_loaded.get() {
            let msg = format!("{} loaded to toolhead, unload from toolhead before trying to capture TD1 data.", self.name);
            self.afc.error.afc_error(c, &msg, false, None)?;
            return Ok((false, msg));
        }
        let (valid, m) = self.afc.function.check_for_td1_id(td1_id.as_deref())?;
        if !valid {
            self.afc.error.afc_error(c, &m, false, None)?;
            return Ok((false, m));
        }
        let (error, m) = self.afc.function.check_for_td1_error(None, true)?;
        if error {
            return Ok((false, m));
        }
        if !hub.state() {
            let mut max_move_tries = 0;
            if !self.is_direct_hub() {
                if !self.loaded_to_hub.get() {
                    let mut move_dis = self.dist_hub.get();
                    if self.afc.homing_enabled {
                        move_dis += hub.move_dis();
                    }
                    self.unit_obj()?.move_to_hub(&me, move_dis, MoveDirection::Pos, self.afc.homing_enabled, SpeedMode::Hub)?;
                    self.loaded_to_hub.set(true);
                }
                while !hub.state() {
                    if max_move_tries >= self.afc.max_move_tries {
                        let mut fail_message = format!("Failed to trigger hub {} for {}\n", hub.name, self.name);
                        fail_message += "Cannot capture TD-1 data, verify that hub switch is properly working before continuing";
                        self.afc.error.afc_error(c, &fail_message, false, None)?;
                        self.do_enable(false)?;
                        return Ok((false, fail_message));
                    }
                    if max_move_tries == 0 {
                        self.move_auto_speed(hub.move_dis())?;
                    } else {
                        self.move_auto_speed(self.short_move_dis())?;
                    }
                    max_move_tries += 1;
                }
            }
            let compare_time = utils::datetime_now();
            self.move_auto_speed(td1_bowden_length)?;
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 5.0)?;
            let success = self.unit_obj()?.get_td1_data(&me, compare_time, false)?;
            if !success {
                msg = format!("Not able to gather TD-1 data after moving {}mm", py_float_repr(td1_bowden_length));
                self.afc.error.afc_error(c, &msg, false, None)?;
                status = false;
            }
            self.unit_obj()?.move_to_hub(&me, hub.td1_bowden_length.get(), MoveDirection::Neg, self.afc.homing_enabled, SpeedMode::Long)?;
            if success {
                self.send_lane_data()?;
            }
            if !self.is_direct_hub() {
                let mut max_move_tries = 0;
                while hub.state() {
                    if max_move_tries >= self.afc.max_move_tries {
                        let mut fail_message = format!("Failed to un-trigger hub {} for {}\n", hub.name, self.name);
                        fail_message += "Verify that hub switch is properly working before continuing";
                        self.afc.error.afc_error(c, &fail_message, false, None)?;
                        self.do_enable(false)?;
                        return Ok((false, fail_message));
                    }
                    self.move_auto_speed(-self.short_move_dis())?;
                    max_move_tries += 1;
                }
                self.move_auto_speed(-hub.hub_clear_move_dis())?;
            }
            self.unit_obj()?.return_to_home(false)?;
            self.do_enable(false)?;
        } else {
            msg = "Cannot gather TD-1 data, hub sensor not clear. Please clear hub and try again.".into();
            self.afc.error.afc_error(c, &msg, false, None)?;
            status = false;
        }
        Ok((status, msg))
    }

    fn cmd_set_lane_loaded(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_LANE_AFCLANE_CMD_SET_LANE_LOADED;
        if !self.load_state() {
            self.afc.error.afc_error(
                c,
                &format!("Lane:{} is not loaded, ensure that the LOAD switch is properly configured and filament is detected.", self.name),
                false,
                None,
            )?;
            return Ok(());
        }
        if self.afc.get_bypass_state() {
            let mut disable_msg = "";
            let mut detected_msg = " detects filament";
            let mut msg = format!("Cannot set {} as loaded, ", self.name);
            if self.afc.bypass.borrow().as_ref().is_some_and(|b| b.name.contains("virtual")) {
                msg += "virtual ";
                detected_msg = " is enabled";
                disable_msg = " and disable";
            }
            msg += &format!("bypass{}.\nPlease unload{} before trying to set lanes as loaded.", detected_msg, disable_msg);
            self.afc.logger.error(c, &msg);
            return Ok(());
        }
        self.afc.function.unset_lane_loaded()?;
        self.afc.function.handle_activate_extruder()?;
        self.set_tool_loaded(true)?;
        self.sync_to_extruder(true)?;
        self.afc.save_vars()?;
        self.unit_obj()?.select_lane(&self.rc(), false)?;
        self.afc.logger.info(c, &format!("Manually set {} loaded to toolhead", self.name));
        Ok(())
    }

    fn cmd_set_long_move_speed(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_LANE_AFCLANE_CMD_SET_LONG_MOVE_SPEED;
        let update = gcmd.get_int("SAVE", Some(0), Lim::new().min(0).max(2))?;
        let (old_speed, old_factor) = {
            let s = self.settings();
            (s.long_moves_speed.unwrap_or(0.0), s.rev_long_moves_speed_factor.unwrap_or(0.0))
        };
        let speed = gcmd.get_float("FWD_SPEED", Some(old_speed), Lim::new().min(50.0).max(500.0))?;
        let mut factor = gcmd.get_float("RWD_FACTOR", Some(old_factor), Lim::new().min(0.0).max(1.2))?;
        if factor < 0.5 {
            factor = 0.5;
        }
        if factor > 1.2 {
            factor = 1.2;
        }
        {
            let mut s = self.settings.borrow_mut();
            s.long_moves_speed = Some(speed);
            s.rev_long_moves_speed_factor = Some(factor);
        }
        if speed != old_speed {
            self.afc.logger.info(c, &format!("{} forward speed set, New: {}, Old: {}", self.name, py_float_repr(speed), py_float_repr(old_speed)));
        } else {
            self.afc.logger.info(c, &format!("{} forward speed currently set to {}", self.name, py_float_repr(speed)));
        }
        if factor != old_factor {
            self.afc.logger.info(c, &format!("{} reverse speed multiplier set, New: {}, Old: {}", self.name, py_float_repr(factor), py_float_repr(old_factor)));
        } else {
            self.afc.logger.info(c, &format!("{} reverse speed multiplier currently set to {}", self.name, py_float_repr(factor)));
        }
        if update == 1 {
            self.afc.function.config_rewrite(&self.fullname, "long_moves_speed", &PyVal::Float(speed), "")?;
            self.afc.function.config_rewrite(&self.fullname, "rev_long_moves_speed_factor", &PyVal::Float(factor), "")?;
        }
        Ok(())
    }

    fn cmd_set_speed_multiplier(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_LANE_AFCLANE_CMD_SET_SPEED_MULTIPLIER;
        let mut updated = false;
        let old_fwd = self.fwd_speed_multi.get();
        let old_rwd = self.rwd_speed_multi.get();
        self.fwd_speed_multi.set(gcmd.get_float("FWD", Some(old_fwd), Lim::new().min(0.0))?);
        self.rwd_speed_multi.set(gcmd.get_float("RWD", Some(old_rwd), Lim::new().min(0.0))?);
        if self.fwd_speed_multi.get() != old_fwd {
            self.afc.logger.info(c, &format!("{} forward speed multiplier set, New: {}, Old: {}", self.name, py_float_repr(self.fwd_speed_multi.get()), py_float_repr(old_fwd)));
            updated = true;
        }
        if self.rwd_speed_multi.get() != old_rwd {
            self.afc.logger.info(c, &format!("{} reverse speed multiplier set, New: {}, Old: {}", self.name, py_float_repr(self.rwd_speed_multi.get()), py_float_repr(old_rwd)));
            updated = true;
        }
        if updated {
            self.afc.logger.info(c, &format!("Run SAVE_SPEED_MULTIPLIER LANE={} to save values to config file", self.name));
        }
        Ok(())
    }

    fn cmd_save_speed_multiplier(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.afc.function.config_rewrite(&self.fullname, "fwd_speed_multiplier", &PyVal::Float(self.fwd_speed_multi.get()), "")?;
        self.afc.function.config_rewrite(&self.fullname, "rwd_speed_multiplier", &PyVal::Float(self.rwd_speed_multi.get()), "")
    }

    fn cmd_set_hub_dist(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_LANE_AFCLANE_CMD_SET_HUB_DIST;
        let old_dist_hub = self.dist_hub.get();
        // (gcmd.get("LENGTH", self.dist_hub): the float default compares
        // unequal to every string, so a given LENGTH always recalculates)
        if let Some(length) = gcmd.get_opt("LENGTH") {
            self.dist_hub.set(self.afc.function.calc_length(self.config_dist_hub, self.dist_hub.get(), &length)?);
        }
        let mut msg = format!("//{} dist_hub:\n", self.name);
        msg += &format!("//   Config Length:   {}\n", py_float_repr(self.config_dist_hub));
        msg += &format!("//   Previous Length: {}\n", py_float_repr(old_dist_hub));
        msg += &format!("//   New Length:      {}\n", py_float_repr(self.dist_hub.get()));
        self.afc.logger.raw(c, &msg);
        self.afc.logger.info(c, &format!("Run SAVE_HUB_DIST LANE={} to save value to config file", self.name));
        Ok(())
    }

    fn cmd_save_hub_dist(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.afc.function.config_rewrite(&self.fullname, "dist_hub", &PyVal::Float(self.dist_hub.get()), "")
    }

    fn cmd_afc_set_remember_spool(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_LANE_AFCLANE_CMD_AFC_SET_REMEMBER_SPOOL;
        let old = self.remember_spool.borrow().unwrap_or(false);
        let new = gcmd.get_int("REMEMBER_SPOOL", Some(old as i64), Lim::new().min(0).max(1))? != 0;
        if new == old {
            self.afc.logger.info(c, &format!("{} remember_spool already set to {}", self.name, py_bool(new)));
            return Ok(());
        }
        *self.remember_spool.borrow_mut() = Some(new);
        if !new && self.status.get() == LaneState::None {
            self.afc.spool.set_spool_id(&self.rc(), PyVal::None, true, None)?;
        }
        match self.afc.function.config_rewrite(&self.fullname, "remember_spool", &PyVal::Bool(new), "") {
            Ok(()) => self.afc.logger.info(c, &format!("{} remember_spool changed: {} -> {}", self.name, py_bool(old), py_bool(new))),
            Err(e) => self.afc.logger.error(c, &format!("{} failed to save remember_spool to config: {}", self.name, e.message())),
        }
        Ok(())
    }

    fn cmd_afc_recover_lane(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let me = self.rc();
        self.tool_loaded.set(false);
        self.status.set(LaneState::None);
        self.loaded_to_hub.set(false);
        *self.td1_data.borrow_mut() = PyVal::dict();
        if !self.remember_spool.borrow().unwrap_or(false) {
            self.afc.spool.clear_values(&me)?;
        }
        self.unit_obj()?.lane_not_ready(&me)?;
        self.afc.logger.info(&AFC_LANE_AFCLANE_CMD_AFC_RECOVER_LANE, &format!("{} reset", self.name));
        self.afc.save_vars()
    }

    /// get_status(save_to_file): the webhooks / AFC.var.unit view of the lane
    pub fn get_status(&self, save_to_file: bool) -> KResult<PyVal> {
        let mut r = PyDict::default();
        if !self.connect_done.get() {
            return Ok(PyVal::Dict(r));
        }
        let opt = |v: Option<String>| v.map(PyVal::Str).unwrap_or(PyVal::None);
        r.insert("name".into(), PyVal::Str(self.name.clone()));
        r.insert("unit".into(), PyVal::Str(self.unit.clone()));
        r.insert("hub".into(), opt(self.hub.borrow().clone()));
        r.insert("extruder".into(), opt(self.afc_extruder_name.borrow().clone()));
        r.insert("buffer".into(), opt(self.buffer_name.borrow().clone()));
        r.insert("buffer_status".into(), self.buffer_status().map_or(PyVal::None, PyVal::Str));
        r.insert("lane".into(), PyVal::Int(self.index));
        if !save_to_file {
            r.insert("map".into(), PyVal::List(self.map.borrow().iter().map(|m| PyVal::Str(m.clone())).collect()));
        } else {
            r.insert("map".into(), PyVal::Str(self.map_to_string()));
        }
        r.insert("current_map".into(), PyVal::Str(self.current_map.borrow().clone()));
        r.insert("load".into(), PyVal::Bool(self.load_state()));
        r.insert("prep".into(), PyVal::Bool(self.prep_state.get()));
        if let Some(s) = self.selector_state.get() {
            r.insert("selector".into(), PyVal::Bool(s));
        }
        r.insert("tool_loaded".into(), PyVal::Bool(self.tool_loaded.get()));
        r.insert("loaded_to_hub".into(), PyVal::Bool(self.loaded_to_hub.get()));
        r.insert("material".into(), self.material());
        if save_to_file {
            r.insert("density".into(), self.filament_density.borrow().clone());
            r.insert("diameter".into(), self.filament_diameter.borrow().clone());
            r.insert("empty_spool_weight".into(), self.empty_spool_weight.borrow().clone());
            r.insert("need_purge".into(), PyVal::Bool(self.need_purge.get()));
        }
        r.insert("remember_spool".into(), PyVal::Bool(self.remember_spool.borrow().unwrap_or(false)));
        let spool_id = self.spool_id.borrow().clone();
        r.insert("spool_id".into(), if spool_id.truthy() { PyVal::Int(utils::py_int_of(&spool_id)?) } else { PyVal::None });
        r.insert("color".into(), self.color.borrow().clone());
        if !save_to_file {
            r.insert("filament_name".into(), self.filament_name.borrow().clone());
            r.insert("spool_vendor".into(), self.spool_vendor.borrow().clone());
            r.insert("multi_color_hexes".into(), self.multi_color.borrow().clone());
            r.insert("initial_weight".into(), self.espooler.full_weight());
        }
        r.insert("weight".into(), self.weight.borrow().clone());
        r.insert("extruder_temp".into(), self.extruder_temp.borrow().clone());
        r.insert("bed_temp".into(), self.bed_temp.borrow().clone());
        r.insert("runout_lane".into(), opt(self.runout_lane.borrow().clone()));
        let filament_stat = self.afc.function.get_filament_status(self)?;
        let mut parts = filament_stat.split(':');
        r.insert("filament_status".into(), PyVal::Str(parts.next().unwrap_or("").to_string()));
        r.insert("filament_status_led".into(), PyVal::Str(parts.next().unwrap_or("").to_string()));
        r.insert("status".into(), PyVal::from(self.status.get().as_str()));
        r.insert("dist_hub".into(), PyVal::Float(self.dist_hub.get()));
        let td1 = |k: &str| -> PyVal {
            match self.td1_data.borrow().get() {
                PyVal::Dict(d) => d.get(k).cloned().unwrap_or(PyVal::Str(String::new())),
                _ => PyVal::Str(String::new()),
            }
        };
        if save_to_file {
            r.insert("td1_data".into(), self.td1_data.borrow().clone());
        } else {
            r.insert("td1_td".into(), td1("td"));
            r.insert("td1_color".into(), td1("color"));
            r.insert("td1_scan_time".into(), td1("scan_time"));
        }
        if self.canvas.borrow().is_some() {
            super::canvas::lane_status_extra(self, &mut r)?;
        }
        Ok(PyVal::Dict(r))
    }
}

pub const CMD_SET_LANE_LOADED_HELP: &str =
    "Sets current lane as loaded to toolhead, useful when manually loading lanes during prints if AFC detects an error when trying to unload/load a lane";
pub const CMD_SET_LONG_MOVE_SPEED_HELP: &str = "Gives ability to set long_moves_speed or rev_long_moves_speed_factor values without having to update config and restart";
pub const CMD_SET_SPEED_MULTIPLIER_HELP: &str = "Gives ability to set fwd_speed_multiplier or rwd_speed_multiplier values without having to update config and restart";
pub const CMD_SAVE_SPEED_MULTIPLIER_HELP: &str = "Saves fwd_speed_multiplier and rwd_speed_multiplier values to config file ";
pub const CMD_SET_HUB_DIST_HELP: &str = "Helper to dynamically set distance between a lanes extruder and hub";
pub const CMD_SAVE_HUB_DIST_HELP: &str = "Saves dist_hub value to config file ";
pub const CMD_AFC_SET_REMEMBER_SPOOL_HELP: &str = "Set lane to remember ejected spool";
pub const CMD_AFC_RECOVER_LANE_HELP: &str =
    "Command to help recover a lane without manually editing files when filament was removed when power was removed. This should only be called as last resort.";

impl PrinterObject for AfcLane {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        AfcLane::get_status(self, false).ok()
    }
}

