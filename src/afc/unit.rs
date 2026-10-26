// An AFC unit: a box of lanes (port of AFC_unit.py; the Canvas unit's
// overrides are in canvas.rs)
//
// Copyright (C) 2024-2026 Armored Turtle
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::ace::AceUnit;
use super::canvas::CanvasUnit;
use super::respond::{button, AfcPrompt, Button};
use super::codes::*;
use super::extruder::AfcExtruder;
use super::hub::AfcHub;
use super::lane::{AfcLane, AssistActive, Endstop, MoveDirection, MoveWarning, SpeedMode};
use super::utils::DateTime;
use super::Afc;
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::gcode::GCodeCommand;
use kaliru::printer::{Printer, PrinterObject};
use kaliru::pyval::PyVal;
use kaliru::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub const CALI_WARN: &str = "The following lanes ({lanes}) have already been calibrated, if you continue with calibration then the filament for selected lanes will be ejected. Once filament is reinserted, then lanes will be calibrated.\n";
const HOMING_DELTA: f64 = 300.0;

pub const CMD_UNIT_CALIBRATION_HELP: &str = "open prompt to calibrate the dist hub for lanes in selected unit";
pub const CMD_UNIT_LANE_CALIBRATION_HELP: &str = "open prompt to calibrate the length from extruder to hub";
pub const CMD_UNIT_BOW_CALIBRATION_HELP: &str = "open prompt to calibrate the afc_bowden_length from a lane in the unit";
pub const CMD_AFC_UNIT_TD_ONE_CALIBRATION_HELP: &str = "open prompt to calibrate the td1_bowden_length from a lane in the unit";

/// "AFC_unit_<name>:connect"
pub fn connect_event(name: &str) -> String {
    format!("AFC_unit_{}:connect", name)
}

/// The unit's values its lanes inherit
pub struct UnitSettings {
    pub led_fault: String,
    pub led_ready: String,
    pub led_not_ready: String,
    pub led_loading: String,
    pub led_prep_loaded: String,
    pub led_unloading: String,
    pub led_tool_loaded: String,
    pub led_tool_loaded_idle: String,
    pub led_tool_unloaded: String,
    pub led_spool_illum: String,
    pub led_use_filament_color: bool,
    pub long_moves_speed: f64,
    pub long_moves_accel: f64,
    pub short_moves_speed: f64,
    pub short_moves_accel: f64,
    pub short_move_dis: f64,
    pub max_move_dis: f64,
    pub homing_overshoot: f64,
    pub homing_delta: f64,
    pub load_then_home_var: bool,
    pub load_undershoot: f64,
    pub rev_long_moves_speed_factor: f64,
    pub move_ints: super::MoveInts,
    pub extruder_clear_dis: f64,
    pub tool_max_unload_attempts: i64,
    pub n20_break_delay_time: f64,
    pub enable_assist: bool,
    pub enable_assist_weight: f64,
    pub timer_delay: PyVal,
    pub enable_kick_start: bool,
    pub spool_ratio: PyVal,
    pub full_weight: PyVal,
    pub espool_rot_dist: f64,
    pub kick_start_time: f64,
    pub delta_movement: PyVal,
    pub scaling: f64,
    pub assisted_unload: bool,
    pub td1_when_loaded: bool,
    pub td1_device_id: Option<String>,
    pub post_prep_macro: String,
    pub debug: bool,
}

/// The unit type's part
pub enum UnitKind {
    Canvas(CanvasUnit),
    Ace(Box<AceUnit>),
    /// BoxTurtle, NightOwl, QuattroBox (lanes with their own steppers)
    BoxTurtle(super::boxturtle::Family),
    /// AFC_Toolchanger: its tools (toolchanger.rs)
    Toolchanger,
    OpenAms(Box<super::openams::OpenAmsUnit>),
}

/// [AFC_canvas]: a unit
pub struct AfcUnit {
    me: Weak<AfcUnit>,
    pub printer: Rc<Printer>,
    pub afc: Rc<Afc>,
    pub full_name: Vec<String>,
    pub name: String,
    pub lanes: RefCell<IndexMap<String, Rc<AfcLane>>>,
    eject_to_calibrate: bool,
    hub_obj: RefCell<Option<Rc<AfcHub>>>,
    extruder_obj: RefCell<Option<Rc<AfcExtruder>>>,
    pub stepperless_drive: Cell<bool>,
    hub: Option<String>,
    extruder: Option<String>,
    buffer_name: Option<String>,
    pub buffer_obj: RefCell<Option<Rc<super::buffer::AfcBuffer>>>,
    /// a ViViD's / Claymore's drive and selector steppers
    pub selector: RefCell<Option<Rc<super::vivid::Selector>>>,
    pub remember_spool: bool,
    pub settings: UnitSettings,
    pub unload_on_runout: bool,
    pub enable_buffer_tool_check: bool,
    pub kind: UnitKind,
    /// type (config "type")
    type_name: String,
    pub logo: RefCell<String>,
    pub logo_error: RefCell<String>,
    pub led_logo_index: Option<String>,
    pub led_logo_color: String,
    pub led_logo_loading: String,
}

impl AfcUnit {
    /// afcUnit.__init__ + the unit type's __init__
    pub fn new(config: &ConfigWrapper, make_kind: impl FnOnce(&ConfigWrapper, &UnitSettings) -> KResult<(UnitKind, String)>) -> KResult<Rc<AfcUnit>> {
        let printer = config.get_printer();
        let afc = printer.lookup_or_load::<Afc>(config, "AFC")?;
        let full_name: Vec<String> = config.get_name().split_whitespace().map(|s| s.to_string()).collect();
        let name = full_name.last().cloned().unwrap_or_default();
        let _screen_mac = config.get_opt("screen_mac")?;
        let hub = config.get_opt("hub")?;
        let extruder = config.get_opt("extruder")?;
        let buffer_name = config.get_opt("buffer")?;
        let remember_spool = config.getboolean("remember_spool", false)?;
        let led_loading = config.get("led_loading", afc.led_loading.as_str())?;
        let settings = UnitSettings {
            led_fault: config.get("led_fault", afc.led_fault.as_str())?,
            led_ready: config.get("led_ready", afc.led_ready.as_str())?,
            led_not_ready: config.get("led_not_ready", afc.led_not_ready.as_str())?,
            led_loading: led_loading.clone(),
            led_prep_loaded: config.get("led_loading", afc.led_loading.as_str())?,
            led_unloading: config.get("led_unloading", afc.led_unloading.as_str())?,
            led_tool_loaded: config.get("led_tool_loaded", afc.led_tool_loaded.as_str())?,
            led_tool_loaded_idle: config.get("led_tool_loaded_idle", afc.led_tool_loaded_idle.as_str())?,
            led_tool_unloaded: config.get("led_tool_unloaded", afc.led_tool_unloaded.as_str())?,
            led_spool_illum: config.get("led_spool_illuminate", afc.led_spool_illum.as_str())?,
            led_use_filament_color: false,
            long_moves_speed: 0.0,
            long_moves_accel: 0.0,
            short_moves_speed: 0.0,
            short_moves_accel: 0.0,
            short_move_dis: 0.0,
            max_move_dis: 0.0,
            homing_overshoot: 0.0,
            homing_delta: 0.0,
            load_then_home_var: false,
            load_undershoot: 0.0,
            rev_long_moves_speed_factor: 0.0,
            move_ints: afc.move_ints,
            extruder_clear_dis: 0.0,
            tool_max_unload_attempts: 0,
            n20_break_delay_time: 0.0,
            enable_assist: false,
            enable_assist_weight: 0.0,
            timer_delay: PyVal::Int(0),
            enable_kick_start: false,
            spool_ratio: PyVal::Int(0),
            full_weight: PyVal::Int(0),
            espool_rot_dist: 0.0,
            kick_start_time: 0.0,
            delta_movement: PyVal::Int(0),
            scaling: 0.0,
            assisted_unload: false,
            td1_when_loaded: false,
            td1_device_id: None,
            post_prep_macro: String::new(),
            debug: false,
        };
        let led_logo_index = config.get_opt("led_logo_index")?;
        let led_logo_color = afc.function.hex_convert(&config.get("led_logo_color", "0,0,0,0")?)?;
        let led_logo_loading = afc.function.hex_convert(&config.get("led_logo_loading", led_loading.as_str())?)?;
        let mut settings = settings;
        settings.led_use_filament_color = config.getboolean("led_use_filament_color", afc.led_use_filament_color)?;
        // (a default comes back unchanged: still an int when AFC's is)
        let dflt = |v: f64, int: bool| if int { PyVal::Int(v as i64) } else { PyVal::Float(v) };
        let mi = afc.move_ints;
        (settings.long_moves_speed, settings.move_ints.long_speed) = super::getfloat_int(config, "long_moves_speed", dflt(afc.long_moves_speed, mi.long_speed))?;
        (settings.long_moves_accel, settings.move_ints.long_accel) = super::getfloat_int(config, "long_moves_accel", dflt(afc.long_moves_accel, mi.long_accel))?;
        (settings.short_moves_speed, settings.move_ints.short_speed) = super::getfloat_int(config, "short_moves_speed", dflt(afc.short_moves_speed, mi.short_speed))?;
        (settings.short_moves_accel, settings.move_ints.short_accel) = super::getfloat_int(config, "short_moves_accel", dflt(afc.short_moves_accel, mi.short_accel))?;
        settings.short_move_dis = config.getfloat("short_move_dis", afc.short_move_dis, Lim::new())?;
        settings.max_move_dis = config.getfloat("max_move_dis", afc.max_move_dis, Lim::new())?;
        settings.homing_overshoot = config.getfloat("homing_overshoot", 50.0, Lim::new())?;
        settings.homing_delta = config.getfloat("homing_delta", HOMING_DELTA, Lim::new())?;
        settings.load_then_home_var = config.getboolean("load_then_home", afc.load_then_home_var)?;
        settings.load_undershoot = config.getfloat("load_undershoot", afc.load_undershoot, Lim::new())?;
        settings.debug = config.getboolean("debug", false)?;
        settings.rev_long_moves_speed_factor = config.getfloat("rev_long_moves_speed_factor", afc.rev_long_moves_speed_factor, Lim::new())?;
        settings.extruder_clear_dis = config.getfloat("extruder_clear_dis", 50.0, Lim::new())?;
        let enable_buffer_tool_check = config.getboolean("enable_buffer_tool_check", false)?;
        settings.tool_max_unload_attempts = config.getint("tool_max_unload_attempts", afc.tool_max_unload_attempts, Lim::new().min(0))?;
        settings.n20_break_delay_time = config.getfloat("n20_break_delay_time", afc.n20_break_delay_time, Lim::new())?;
        settings.enable_assist = config.getboolean("enable_assist", afc.enable_assist)?;
        settings.enable_assist_weight = config.getfloat("enable_assist_weight", afc.enable_assist_weight, Lim::new())?;
        settings.timer_delay = super::getfloat_pv(config, "timer_delay", PyVal::Int(5))?;
        settings.enable_kick_start = config.getboolean("enable_kick_start", true)?;
        settings.spool_ratio = super::getfloat_pv(config, "spool_ratio", PyVal::Int(2))?;
        settings.full_weight = super::getfloat_pv(config, "full_weight", PyVal::Int(1000))?;
        settings.espool_rot_dist = config.getfloat("espool_rot_dist", 132.9, Lim::new())?;
        settings.kick_start_time = config.getfloat("kick_start_time", 0.070, Lim::new())?;
        settings.delta_movement = super::getfloat_pv(config, "delta_movement", PyVal::Int(150))?;
        settings.scaling = config.getfloat("spoolrate", 1.0, Lim::new())?;
        settings.assisted_unload = config.getboolean("assisted_unload", afc.assisted_unload)?;
        let unload_on_runout = config.getboolean("unload_on_runout", afc.unload_on_runout)?;
        settings.td1_when_loaded = config.getboolean("capture_td1_when_loaded", afc.td1_when_loaded)?;
        settings.td1_device_id = config.get_opt("td1_device_id")?;
        settings.post_prep_macro = config.get("post_prep_macro", "AFC_POST_PREP")?;

        let (kind, type_name) = make_kind(config, &settings)?;
        let unit = Rc::new_cyclic(|me| AfcUnit {
            me: me.clone(),
            printer: printer.clone(),
            afc: afc.clone(),
            full_name,
            name,
            lanes: RefCell::new(IndexMap::default()),
            eject_to_calibrate: matches!(kind, UnitKind::BoxTurtle(super::boxturtle::Family::Vivid | super::boxturtle::Family::Claymore)),
            hub_obj: RefCell::new(None),
            extruder_obj: RefCell::new(None),
            stepperless_drive: Cell::new(false),
            hub,
            extruder,
            buffer_name,
            buffer_obj: RefCell::new(None),
            selector: RefCell::new(None),
            remember_spool,
            settings,
            unload_on_runout,
            enable_buffer_tool_check,
            kind,
            type_name,
            logo: RefCell::new(String::new()),
            logo_error: RefCell::new(String::new()),
            led_logo_index,
            led_logo_color,
            led_logo_loading,
        });
        let u = unit.clone();
        kaliru::events::KlippyConnect::on(&printer, move || u.handle_connect());
        let u = unit.clone();
        kaliru::events::KlippyReady::on(&printer, move || u.handle_ready());
        let u = unit.clone();
        printer.register_event_handler(super::utils::MOONRAKER_CONNECT_EVENT, Rc::new(move |_| u.handle_moonraker_connect()));
        Ok(unit)
    }

    pub fn rc(&self) -> Rc<AfcUnit> {
        self.me.upgrade().expect("AFC unit")
    }

    pub fn type_name(&self) -> String {
        self.type_name.clone()
    }

    pub fn hub_obj(&self) -> Option<Rc<AfcHub>> {
        self.hub_obj.borrow().clone()
    }

    pub fn extruder_obj(&self) -> Option<Rc<AfcExtruder>> {
        self.extruder_obj.borrow().clone()
    }

    pub fn canvas(&self) -> &CanvasUnit {
        match &self.kind {
            UnitKind::Canvas(c) => c,
            _ => unreachable!("not a Canvas unit"),
        }
    }

    /// klippy:connect: the unit's hub / extruder / buffer, then <unit>:connect
    fn handle_connect(&self) -> KResult<()> {
        let me = self.rc();
        self.afc.units.borrow_mut().insert(self.name.clone(), me.clone());
        let tn = self.type_name.replace('_', "");
        if let Some(h) = &self.hub {
            match self.printer.lookup::<AfcHub>(&format!("AFC_hub {}", h)) {
                Ok(o) => *self.hub_obj.borrow_mut() = Some(o),
                Err(_) => {
                    return Err(KError::config(format!(
                        "Error: No config found for hub: {} in [AFC_{} {}]. Please make sure [AFC_hub {}] section exists in your config",
                        h, tn, self.name, h
                    )))
                }
            }
        }
        if let Some(e) = &self.extruder {
            match self.printer.lookup::<AfcExtruder>(&format!("AFC_extruder {}", e)) {
                Ok(o) => *self.extruder_obj.borrow_mut() = Some(o),
                Err(_) => {
                    return Err(KError::config(format!(
                        "Error: No config found for extruder: {} in [AFC_{} {}]. Please make sure [AFC_extruder {}] section exists in your config",
                        e, tn, self.name, e
                    )))
                }
            }
        }
        if let Some(b) = &self.buffer_name {
            match self.printer.lookup_object_opt(&format!("AFC_buffer {}", b)).and_then(kaliru::printer::downcast_rc::<super::buffer::AfcBuffer>) {
                Some(o) => *self.buffer_obj.borrow_mut() = Some(o),
                None => return Err(KError::config(format!(
                "Error: No config found for buffer: {} in [AFC_{} {}]. Please make sure [AFC_buffer {}] section exists in your config",
                b, tn, self.name, b
            ))),
            }
        }
        self.printer.send_event(&connect_event(&self.name), &[&me])?;
        let cmds: [(&str, fn(&AfcUnit, &GCodeCommand) -> KResult<()>, &str); 3] = [
            ("UNIT_CALIBRATION", AfcUnit::cmd_unit_calibration, CMD_UNIT_CALIBRATION_HELP),
            ("UNIT_LANE_CALIBRATION", AfcUnit::cmd_unit_lane_calibration, CMD_UNIT_LANE_CALIBRATION_HELP),
            ("UNIT_BOW_CALIBRATION", AfcUnit::cmd_unit_bow_calibration, CMD_UNIT_BOW_CALIBRATION_HELP),
        ];
        for (cmd, f, help) in cmds {
            let u = me.clone();
            self.afc.gcode.register_mux_command(cmd, "UNIT", Some(&self.name), Rc::new(move |g| f(&u, g)), Some(help))?;
        }
        match &self.kind {
            UnitKind::Canvas(_) => super::canvas::unit_handle_connect(self),
            UnitKind::Ace(_) => super::ace::unit_handle_connect(self),
            UnitKind::BoxTurtle(f) => {
                super::boxturtle::unit_handle_connect(self, *f);
                if *f == super::boxturtle::Family::QuattroBox {
                    self.set_logo_color(&self.led_logo_color)?;
                }
            }
            // (its logos are set when it is made)
            UnitKind::Toolchanger => {}
            UnitKind::OpenAms(_) => super::openams::unit_handle_connect(self),
        }
        Ok(())
    }

    /// klippy:ready: lanes in natural order
    fn handle_ready(&self) -> KResult<()> {
        let mut lanes: Vec<(String, Rc<AfcLane>)> = self.lanes.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect();
        lanes.sort_by_key(|(k, _)| super::first_number(k).unwrap_or(9999));
        *self.lanes.borrow_mut() = lanes.into_iter().collect();
        match &self.kind {
            UnitKind::Ace(_) => super::ace::unit_handle_ready(self),
            UnitKind::OpenAms(_) => super::openams::unit_handle_ready(self),
            UnitKind::Canvas(_) | UnitKind::BoxTurtle(_) | UnitKind::Toolchanger => Ok(()),
        }
    }

    fn handle_moonraker_connect(&self) -> KResult<()> {
        if self.afc.td1_defined.get() {
            let u = self.rc();
            self.afc.gcode.register_mux_command(
                "AFC_UNIT_TD_ONE_CALIBRATION",
                "UNIT",
                Some(&self.name),
                Rc::new(move |g| u.cmd_afc_unit_td_one_calibration(g)),
                Some(CMD_AFC_UNIT_TD_ONE_CALIBRATION_HELP),
            )?;
        }
        Ok(())
    }

    pub fn get_status(&self) -> PyVal {
        let mut extruders = vec![];
        let mut hubs = vec![];
        let mut buffers: Vec<PyVal> = vec![];
        for lane in self.lanes.borrow().values() {
            if let Some(h) = lane.hub.borrow().clone() {
                if !lane.is_direct_hub() && !hubs.contains(&PyVal::Str(h.clone())) {
                    hubs.push(PyVal::Str(h));
                }
            }
            if let Some(e) = lane.afc_extruder_name.borrow().clone() {
                if !extruders.contains(&PyVal::Str(e.clone())) {
                    extruders.push(PyVal::Str(e));
                }
            }
            if let Some(b) = lane.buffer_name.borrow().clone() {
                if !buffers.contains(&PyVal::Str(b.clone())) {
                    buffers.push(PyVal::Str(b));
                }
            }
        }
        let mut r = kaliru::pydict! {
            "lanes" => PyVal::List(self.lanes.borrow().values().map(|l| PyVal::Str(l.name.clone())).collect()),
            "extruders" => PyVal::List(extruders),
            "hubs" => PyVal::List(hubs),
            "buffers" => PyVal::List(buffers),
        };
        if let UnitKind::Ace(_) = &self.kind {
            super::ace::status_extra(self, &mut r);
        }
        PyVal::Dict(r)
    }

    fn cmd_unit_calibration(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if let UnitKind::Canvas(_) = &self.kind {
            return super::canvas::unit_calibration_unsupported(self, gcmd);
        }
        if self.lanes.borrow().is_empty() {
            self.afc.logger.warning(&AFC_UNIT_AFCUNIT_CMD_UNIT_CALIBRATION, &format!("No lanes to calibrate for {}", self.name));
            return Ok(());
        }
        let prompt = AfcPrompt::new(&self.afc.logger);
        let mut buttons = vec![button("Calibrate Lanes", format!("UNIT_LANE_CALIBRATION UNIT={}", self.name), "primary")];
        let lanes: Vec<Rc<AfcLane>> = self.lanes.borrow().values().cloned().collect();
        let direct_hubs = lanes.iter().any(|l| l.is_direct_dist());
        let lanes_loaded = lanes.iter().any(|l| (l.load_state() && !l.hub_obj_opt().is_some_and(|h| h.use_dist_hub())) && !l.is_direct_hub());
        let any_td1 = lanes.iter().any(|l| l.td1_device_id.borrow().is_some());
        if !direct_hubs || lanes_loaded {
            buttons.push(button("Calibrate afc_bowden_length", format!("UNIT_BOW_CALIBRATION UNIT={}", self.name), "secondary"));
        }
        if self.afc.td1_defined.get() && any_td1 {
            buttons.push(button("Calibrate TD-1 Length", format!("AFC_UNIT_TD_ONE_CALIBRATION UNIT={}", self.name), "primary"));
        }
        let back = [button("Back to unit selection", "AFC_CALIBRATION", "info")];
        prompt.create_custom_p(
            &format!("{} Calibration", self.name),
            Some("Select to calibrate the distance from extruder to hub or bowden length"),
            Some(&buttons),
            true,
            None,
            Some(&back),
        );
        Ok(())
    }

    /// Lane buttons in groups of two
    fn lane_button_groups(&self, include: impl Fn(&AfcLane) -> bool, command: impl Fn(&AfcLane) -> String) -> Vec<Vec<Button>> {
        let lanes: Vec<Rc<AfcLane>> = self.lanes.borrow().values().cloned().collect();
        let mut buttons = vec![];
        let mut group = vec![];
        let mut index = 0usize;
        for lane in &lanes {
            if include(lane) {
                group.push(button(lane.name.clone(), command(lane), if index.is_multiple_of(2) { "primary" } else { "secondary" }));
                if (index + 1).is_multiple_of(2) || index == lanes.len() - 1 {
                    buttons.push(std::mem::take(&mut group));
                }
                index += 1;
            }
        }
        if !group.is_empty() {
            buttons.push(group);
        }
        buttons
    }

    fn cmd_unit_lane_calibration(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if let UnitKind::Canvas(_) = &self.kind {
            return self.cmd_unit_calibration(gcmd);
        }
        let prompt = AfcPrompt::new(&self.afc.logger);
        let mut text = String::new();
        if let Some(lanes) = self.get_calibrated_lanes() {
            text = CALI_WARN.replace("{lanes}", &lanes.join(", "));
        }
        text += &format!("Select a loaded lane from {} to calibrate length from extruder to hub. Config option: dist_hub", self.name);
        let buttons = self.lane_button_groups(
            |l| l.load_state() && !l.tool_loaded.get(),
            |l| {
                if l.is_direct_hub() || l.hub_obj_opt().is_some_and(|h| h.use_dist_hub()) {
                    format!("CALIBRATE_AFC BOWDEN={}", l.name)
                } else {
                    format!("CALIBRATE_AFC LANE={}", l.name)
                }
            },
        );
        let total: usize = buttons.iter().map(|g| g.len()).sum();
        let all_lanes = [button("All lanes", format!("CALIBRATE_AFC LANE=all UNIT={}", self.name), "default")];
        if total == 0 {
            text = "No lanes are loaded, please load before calibration".into();
        }
        let back = [button("Back", format!("UNIT_CALIBRATION UNIT={}", self.name), "info")];
        prompt.create_custom_p(&format!("{} Lane Calibration", self.name), Some(&text), if total > 1 { Some(&all_lanes) } else { None }, true, Some(&buttons), Some(&back));
        Ok(())
    }

    fn cmd_unit_bow_calibration(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if let UnitKind::Canvas(_) = &self.kind {
            return self.cmd_unit_calibration(gcmd);
        }
        let prompt = AfcPrompt::new(&self.afc.logger);
        let mut text = format!(
            "Select a loaded lane from {} to measure Bowden length. ONLY CALIBRATE BOWDEN USING 1 LANE PER UNIT/hub.Config option: afc_bowden_length",
            self.name
        );
        let buttons = self.lane_button_groups(
            |l| l.load_state() && !l.is_direct_hub() && !l.hub_obj_opt().is_some_and(|h| h.use_dist_hub()),
            |l| format!("CALIBRATE_AFC BOWDEN={}", l.name),
        );
        if buttons.iter().map(|g| g.len()).sum::<usize>() == 0 {
            text = "No lanes are loaded, please load before calibration".into();
        }
        let back = [button("Back", format!("UNIT_CALIBRATION UNIT={}", self.name), "info")];
        prompt.create_custom_p(&format!("Bowden Calibration {}", self.name), Some(&text), None, true, Some(&buttons), Some(&back));
        Ok(())
    }

    fn cmd_afc_unit_td_one_calibration(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if let UnitKind::Canvas(_) = &self.kind {
            return self.cmd_unit_calibration(gcmd);
        }
        let prompt = AfcPrompt::new(&self.afc.logger);
        let mut text = format!(
            "Select a loaded lane from {} to measure Bowden length to your TD-1 Device. ONLY CALIBRATE BOWDEN USING 1 LANE PER UNIT/hub. WARNING: This could take some time to complete. Config option: td1_bowden_length",
            self.name
        );
        let buttons = self.lane_button_groups(|l| l.td1_device_id.borrow().is_some() && l.load_state(), |l| format!("CALIBRATE_AFC TD1={} DISTANCE=50", l.name));
        if buttons.iter().map(|g| g.len()).sum::<usize>() == 0 {
            text = "No lanes are loaded, please load before calibration".into();
        }
        let back = [button("Back", format!("UNIT_CALIBRATION UNIT={}", self.name), "info")];
        prompt.create_custom_p(&format!("TD-1 Bowden Calibration {}", self.name), Some(&text), None, true, Some(&buttons), Some(&back));
        Ok(())
    }

    /// _check_led_state: false when the lane already shows this state
    fn check_led_state(&self, lane: &AfcLane, state: &str) -> bool {
        if *lane.current_led_state.borrow() == state {
            return false;
        }
        *lane.current_led_state.borrow_mut() = state.to_string();
        true
    }

    /// _get_lane_color: the filament color when the lane uses it
    fn get_lane_color(&self, lane: &AfcLane, fallback: &str) -> KResult<PyVal> {
        if lane.led_use_filament_color() && lane.color.borrow().truthy() {
            let hex = lane.color.borrow().str().replace('#', "");
            let hex = kaliru::pyfmt::py_strip(&hex).to_uppercase();
            if (hex.len() == 6 || hex.len() == 8) && hex.chars().all(|c| "0123456789ABCDEF".contains(c)) {
                return Ok(PyVal::List(self.afc.function.hex_to_led_string(&hex)?.into_iter().map(PyVal::Float).collect()));
            }
        }
        Ok(PyVal::Str(fallback.to_string()))
    }

    /// _trigger_led_state with a color that may be a hex-derived list
    fn trigger_led(&self, lane: &AfcLane, extruder: Option<&AfcExtruder>, color: Option<&PyVal>, extruder_color: Option<&str>, effect: &str) -> KResult<()> {
        let mut names = vec![lane.name.clone()];
        if let Some(e) = extruder {
            names.push(e.name.clone());
        }
        self.stop_led_effects(Some(&names));
        if let Some(color) = color {
            self.afc.function.afc_led(color, lane.led_index.as_deref())?;
            if let Some(e) = extruder {
                e.set_status_led(extruder_color.map(|s| s.to_string()).unwrap_or_else(|| color.str()).as_str())?;
            }
        }
        self.call_led_effect(&lane.name, effect);
        if let Some(e) = extruder {
            self.call_led_effect(&e.name, effect);
        }
        Ok(())
    }

    // The lane LED states: the Canvas unit sets its two LED pins
    pub fn lane_not_ready(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        match &self.kind {
            UnitKind::Canvas(_) => super::canvas::unit_lane_not_ready(self, lane),
            UnitKind::Ace(_) | UnitKind::BoxTurtle(_) | UnitKind::Toolchanger | UnitKind::OpenAms(_) => {
                if !self.check_led_state(lane, "not_ready") {
                    return Ok(());
                }
                self.trigger_led(lane, None, Some(&PyVal::Str(lane.led_not_ready())), None, "not_ready")?;
                if let Some(i) = &lane.led_spool_index {
                    self.afc.function.afc_led(&PyVal::Str(self.afc.led_off.clone()), Some(i))?;
                }
                Ok(())
            }
        }
    }

    pub fn lane_loaded(&self, lane: &Rc<AfcLane>, force: bool) -> KResult<()> {
        match &self.kind {
            UnitKind::Canvas(_) => super::canvas::unit_lane_led(self, lane, "lane_loaded", &self.afc.led_ready),
            UnitKind::Ace(_) | UnitKind::BoxTurtle(_) | UnitKind::Toolchanger | UnitKind::OpenAms(_) => {
                if !self.check_led_state(lane, "loaded") && !force {
                    return Ok(());
                }
                let color = self.get_lane_color(lane, &lane.led_ready())?;
                self.trigger_led(lane, None, Some(&color), None, "loaded")?;
                if let Some(i) = &lane.led_spool_index {
                    self.afc.function.afc_led(&PyVal::Str(lane.led_spool_illum()), Some(i))?;
                }
                Ok(())
            }
        }
    }

    pub fn lane_unloading(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        match &self.kind {
            UnitKind::Canvas(_) => super::canvas::unit_lane_led(self, lane, "lane_unloading", &self.afc.led_unloading),
            UnitKind::Ace(_) | UnitKind::BoxTurtle(_) | UnitKind::Toolchanger => self.base_lane_unloading(lane),
            UnitKind::OpenAms(_) => {
                self.base_lane_unloading(lane)?;
                super::openams::lane_unloading(self);
                Ok(())
            }
        }
    }

    fn base_lane_unloading(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        if !self.check_led_state(lane, "unloading") {
            return Ok(());
        }
        self.trigger_led(lane, None, Some(&PyVal::Str(lane.led_unloading())), None, "unloading")
    }

    pub fn lane_unloaded(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        match &self.kind {
            UnitKind::Canvas(_) => super::canvas::unit_lane_unloaded(self, lane),
            UnitKind::Ace(_) | UnitKind::BoxTurtle(_) | UnitKind::Toolchanger | UnitKind::OpenAms(_) => {
                if !self.check_led_state(lane, "unloaded") {
                    return Ok(());
                }
                self.trigger_led(lane, None, Some(&PyVal::Str(lane.led_not_ready())), None, "unloaded")
            }
        }
    }

    /// the base unit's lane_loading (AFC_ACE calls it first)
    pub fn base_lane_loading(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        if !self.check_led_state(lane, "loading") {
            return Ok(());
        }
        self.trigger_led(lane, None, Some(&PyVal::Str(lane.led_loading())), None, "loading")
    }

    pub fn lane_loading(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        match &self.kind {
            UnitKind::Canvas(_) => super::canvas::unit_lane_led(self, lane, "lane_loading", &self.afc.led_loading),
            UnitKind::Ace(_) => super::ace::lane_loading(self, lane),
            UnitKind::BoxTurtle(f) => {
                self.base_lane_loading(lane)?;
                if *f == super::boxturtle::Family::QuattroBox {
                    self.set_logo_color(&self.led_logo_loading)?;
                }
                Ok(())
            }
            UnitKind::Toolchanger | UnitKind::OpenAms(_) => self.base_lane_loading(lane),
        }
    }

    fn is_quattro_box(&self) -> bool {
        matches!(self.kind, UnitKind::BoxTurtle(super::boxturtle::Family::QuattroBox))
    }

    /// set_logo_color: the unit's logo LEDs (a hex value or a color list)
    pub fn set_logo_color(&self, color: &str) -> KResult<()> {
        if !color.is_empty() {
            let led_color = self.afc.function.hex_to_led_string(&color.replace('#', ""))?;
            self.afc.function.afc_led(&PyVal::List(led_color.into_iter().map(PyVal::Float).collect()), self.led_logo_index.as_deref())?;
        }
        Ok(())
    }

    /// (the base unit's: no LED state for the Canvas, which has no override)
    pub fn lane_tool_loaded_gears(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        if !self.check_led_state(lane, "tool_loaded_gears") {
            return Ok(());
        }
        self.trigger_led_state(Some(lane), lane.extruder_obj_opt().as_deref(), None, None, Some("tool_loaded_gears"))
    }

    pub fn lane_tool_loaded(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        let r = match &self.kind {
            UnitKind::Canvas(_) => super::canvas::unit_lane_tool_led(self, lane, "lane_tool_loaded", &self.afc.led_tool_loaded, &self.afc.led_tool_loaded),
            UnitKind::Ace(_) | UnitKind::BoxTurtle(_) | UnitKind::Toolchanger | UnitKind::OpenAms(_) => {
                if !self.check_led_state(lane, "tool_loaded") {
                    Ok(())
                } else {
                let ext = lane.extruder_obj_opt();
                self.trigger_led(lane, ext.as_deref(), Some(&PyVal::Str(lane.led_tool_loaded())), None, "tool_loaded")
                }
            }
        };
        if self.is_quattro_box() {
            r?;
            return self.set_logo_color(&lane.color.borrow().str());
        }
        r
    }

    pub fn lane_tool_unloaded(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        let r = match &self.kind {
            UnitKind::Canvas(_) => super::canvas::unit_lane_tool_led(self, lane, "lane_tool_unloaded", &self.afc.led_ready, &self.afc.led_tool_unloaded),
            UnitKind::Ace(_) | UnitKind::BoxTurtle(_) | UnitKind::Toolchanger | UnitKind::OpenAms(_) => {
                if !self.check_led_state(lane, "tool_unloaded") {
                    Ok(())
                } else {
                let color = self.get_lane_color(lane, &lane.led_ready())?;
                let ext = lane.extruder_obj_opt();
                self.trigger_led(lane, ext.as_deref(), Some(&color), Some(&lane.led_tool_unloaded()), "tool_unloaded")
                }
            }
        };
        if self.is_quattro_box() {
            r?;
            return self.set_logo_color(&self.led_logo_color);
        }
        r
    }

    pub fn lane_tool_loaded_idle(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        match &self.kind {
            UnitKind::Canvas(_) => {
                super::canvas::unit_lane_tool_led(self, lane, "lane_tool_loaded_idle", &self.afc.led_tool_loaded_idle, &self.afc.led_tool_loaded_idle)
            }
            UnitKind::Ace(_) | UnitKind::BoxTurtle(_) | UnitKind::Toolchanger | UnitKind::OpenAms(_) => {
                if !self.check_led_state(lane, "tool_loaded_idle") {
                    return Ok(());
                }
                let color = self.get_lane_color(lane, &lane.led_tool_loaded_idle())?;
                let ext = lane.extruder_obj_opt();
                self.trigger_led(lane, ext.as_deref(), Some(&color), None, "tool_loaded_idle")
            }
        }
    }

    pub fn lane_fault(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        match &self.kind {
            UnitKind::Canvas(_) => super::canvas::unit_lane_led(self, lane, "lane_fault", &self.afc.led_fault),
            UnitKind::Ace(_) | UnitKind::BoxTurtle(_) | UnitKind::Toolchanger | UnitKind::OpenAms(_) => {
                if !self.check_led_state(lane, "fault") {
                    return Ok(());
                }
                self.trigger_led(lane, None, Some(&PyVal::Str(lane.led_fault())), None, "fault")
            }
        }
    }

    pub fn lane_illuminate_spool(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        match &self.kind {
            UnitKind::Canvas(_) => super::canvas::unit_lane_led(self, lane, "lane_illuminate_spool", &self.afc.led_spool_illum),
            UnitKind::Ace(_) | UnitKind::BoxTurtle(_) | UnitKind::Toolchanger | UnitKind::OpenAms(_) => {
                if let Some(i) = &lane.led_spool_index {
                    self.afc.function.afc_led(&PyVal::Str(lane.led_spool_illum()), Some(i))?;
                }
                Ok(())
            }
        }
    }

    /// _stop_led_effects (targets: names whose effects stop)
    fn stop_led_effects(&self, targets: Option<&[String]>) {
        let effects = self.afc.active_led_effects.borrow().clone();
        let mut remaining = vec![];
        for effect in effects {
            if let Some(t) = targets {
                if !t.iter().any(|t| effect.starts_with(&format!("{}_", t))) {
                    remaining.push(effect);
                    continue;
                }
            }
            let _ = self.afc.gcode.run_script_from_command(&format!("SET_LED_EFFECT EFFECT={} STOP=1", effect));
        }
        *self.afc.active_led_effects.borrow_mut() = remaining;
    }

    fn call_led_effect(&self, name: &str, effect_suffix: &str) {
        let effect_name = format!("{}_{}", name, effect_suffix);
        if self.printer.lookup_object_opt(&format!("led_effect {}", effect_name)).is_some()
            && self.afc.gcode.run_script_from_command(&format!("SET_LED_EFFECT EFFECT={}", effect_name)).is_ok()
        {
            self.afc.active_led_effects.borrow_mut().push(effect_name);
        }
    }

    /// _trigger_led_state: the static color, then an led_effect overlay
    fn trigger_led_state(&self, lane: Option<&AfcLane>, extruder: Option<&AfcExtruder>, static_color: Option<&str>, extruder_static_color: Option<&str>, effect_suffix: Option<&str>) -> KResult<()> {
        if lane.is_none() && extruder.is_none() {
            return Ok(());
        }
        let mut names = vec![];
        if let Some(l) = lane {
            names.push(l.name.clone());
        }
        if let Some(e) = extruder {
            names.push(e.name.clone());
        }
        self.stop_led_effects(Some(&names));
        if let Some(color) = static_color {
            if let Some(l) = lane {
                self.afc.function.afc_led(&PyVal::Str(color.to_string()), l.led_index.as_deref())?;
            }
            if let Some(e) = extruder {
                e.set_status_led(extruder_static_color.unwrap_or(color))?;
            }
        }
        if let Some(suffix) = effect_suffix.filter(|s| !s.is_empty()) {
            if let Some(l) = lane {
                self.call_led_effect(&l.name, suffix);
            }
            if let Some(e) = extruder {
                self.call_led_effect(&e.name, suffix);
            }
        }
        Ok(())
    }

    /// select_lane: (homed, distance); units without a selector: (True, 0.0)
    pub fn select_lane(&self, lane: &Rc<AfcLane>, sel_prep: bool) -> KResult<(bool, f64)> {
        if super::vivid::is_selector_unit(self) {
            return super::vivid::select_lane(self, lane, sel_prep);
        }
        if super::htlf::is_htlf(self) {
            // (python's second argument is HTLF's disable_selector)
            return super::htlf::select_lane(self, lane, sel_prep);
        }
        Ok((true, 0.0))
    }

    pub fn return_to_home(&self, prep: bool) -> KResult<()> {
        if super::htlf::is_htlf(self) {
            super::htlf::return_to_home(self, prep, true)?;
        }
        Ok(())
    }

    pub fn check_runout(&self, lane: &Rc<AfcLane>) -> KResult<bool> {
        match &self.kind {
            UnitKind::Ace(_) => super::ace::check_runout(self, lane),
            UnitKind::OpenAms(_) => super::openams::check_runout(self, lane),
            UnitKind::BoxTurtle(super::boxturtle::Family::Htlf) => super::htlf::check_runout(self, lane),
            UnitKind::Canvas(_) | UnitKind::BoxTurtle(_) | UnitKind::Toolchanger => Ok(false),
        }
    }

    pub fn system_test(&self, cur_lane: &Rc<AfcLane>, delay: f64, assign_tcmd: bool, enable_movement: bool) -> KResult<bool> {
        match &self.kind {
            UnitKind::Canvas(_) => super::canvas::unit_system_test(self, cur_lane, delay, assign_tcmd, enable_movement),
            UnitKind::Ace(_) => super::ace::system_test(self, cur_lane, assign_tcmd),
            UnitKind::BoxTurtle(super::boxturtle::Family::Htlf) => super::htlf::system_test(self, cur_lane, delay, assign_tcmd, enable_movement),
            UnitKind::BoxTurtle(_) => super::boxturtle::unit_system_test(self, cur_lane, delay, assign_tcmd, enable_movement),
            UnitKind::Toolchanger => super::toolchanger::system_test(self, cur_lane, assign_tcmd),
            UnitKind::OpenAms(_) => super::openams::system_test(self, cur_lane, assign_tcmd),
        }
    }

    /// _print_function_not_defined, where the caller uses no result
    fn print_not_defined(&self, name: &str) -> KResult<()> {
        self.afc.logger.error(&AFC_UNIT_AFCUNIT__PRINT_FUNCTION_NOT_DEFINED, &format!("{} function not defined for {}", name, self.name));
        Ok(())
    }

    /// _print_function_not_defined, and the TypeError of unpacking its None
    fn not_defined(&self, name: &str) -> KError {
        self.afc.logger.error(&AFC_UNIT_AFCUNIT__PRINT_FUNCTION_NOT_DEFINED, &format!("{} function not defined for {}", name, self.name));
        KError::Py("TypeError".into(), "cannot unpack non-iterable NoneType object".into())
    }

    pub fn calibrate_bowden(&self, lane: &Rc<AfcLane>, dis: f64, tol: f64) -> KResult<(bool, String, f64)> {
        match &self.kind {
            UnitKind::Canvas(_) => Ok((false, super::canvas::CALIBRATION_UNSUPPORTED.into(), 0.0)),
            UnitKind::Ace(_) => Ok((false, super::ace::CALIBRATION_LANE_MESSAGE.into(), 0.0)),
            UnitKind::BoxTurtle(_) => super::boxturtle::unit_calibrate_bowden(self, lane, dis, tol),
            UnitKind::Toolchanger => Err(self.not_defined("calibrate_bowden")),
            UnitKind::OpenAms(_) => super::openams::calibrate_bowden(self, lane),
        }
    }

    /// (success, message, distance: the python number the unit returns)
    pub fn calibrate_td1(&self, lane: &Rc<AfcLane>, dis: f64, tol: f64) -> KResult<(bool, String, PyVal)> {
        match &self.kind {
            UnitKind::Canvas(_) => Ok((false, super::canvas::CALIBRATION_UNSUPPORTED.into(), PyVal::Float(0.0))),
            UnitKind::Ace(_) => Err(self.not_defined("calibrate_td1")),
            UnitKind::BoxTurtle(_) => super::boxturtle::unit_calibrate_td1(self, lane, dis, tol),
            UnitKind::Toolchanger => Err(self.not_defined("calibrate_td1")),
            UnitKind::OpenAms(_) => super::openams::calibrate_td1(self, lane),
        }
    }

    pub fn calibrate_lane(&self, lane: &Rc<AfcLane>, tol: f64) -> KResult<(bool, String, f64)> {
        match &self.kind {
            UnitKind::Canvas(_) => Ok((false, super::canvas::CALIBRATION_UNSUPPORTED.into(), 0.0)),
            UnitKind::Ace(_) => Ok((true, "calibration_lane".into(), 0.0)),
            UnitKind::BoxTurtle(_) => match super::vivid::calibrate_lane(self, lane)? {
                Some(r) => Ok(r),
                None => super::boxturtle::unit_calibrate_lane(self, lane, tol),
            },
            UnitKind::Toolchanger => Err(self.not_defined("calibrate_lane")),
            UnitKind::OpenAms(_) => super::openams::calibrate_lane(self, lane),
        }
    }

    pub fn calibration_lane_message(&self) -> String {
        match &self.kind {
            UnitKind::BoxTurtle(_) if super::vivid::is_selector_unit(self) => super::vivid::calibration_lane_message(self),
            UnitKind::Canvas(_) | UnitKind::BoxTurtle(_) | UnitKind::Toolchanger => String::new(),
            UnitKind::Ace(_) => super::ace::CALIBRATION_LANE_MESSAGE.into(),
            UnitKind::OpenAms(_) => super::openams::CALIBRATION_LANE_MESSAGE.into(),
        }
    }

    /// hasattr(unit, "get_lane_reset_command"): Some(the command AFC_LANE_RESET
    /// runs, None when the unit has no distance-based reset)
    pub fn lane_reset_command(&self, lane: &AfcLane) -> Option<Option<String>> {
        match &self.kind {
            UnitKind::Ace(_) => Some(Some(super::ace::get_lane_reset_command(self, lane))),
            UnitKind::OpenAms(_) => Some(None),
            UnitKind::Canvas(_) | UnitKind::BoxTurtle(_) | UnitKind::Toolchanger => None,
        }
    }

    /// hasattr(unit, "unit_load_lane"): Some(the unit's load result)
    pub fn unit_load_lane(&self, lane: &Rc<AfcLane>, extruder: &AfcExtruder) -> KResult<Option<bool>> {
        match &self.kind {
            UnitKind::Ace(_) => super::ace::unit_load_lane(self, lane, extruder, false).map(Some),
            UnitKind::OpenAms(_) => super::openams::unit_load_lane(self, lane, extruder).map(Some),
            UnitKind::Canvas(_) | UnitKind::BoxTurtle(_) | UnitKind::Toolchanger => Ok(None),
        }
    }

    /// hasattr(unit, "unit_unload_lane"): Some(the unit's unload result)
    pub fn unit_unload_lane(&self, lane: &Rc<AfcLane>, extruder: &AfcExtruder) -> KResult<Option<bool>> {
        match &self.kind {
            UnitKind::Ace(_) => super::ace::unit_unload_lane(self, lane, extruder).map(Some),
            UnitKind::OpenAms(_) => super::openams::unit_unload_lane(self, lane, extruder).map(Some),
            UnitKind::Canvas(_) | UnitKind::BoxTurtle(_) | UnitKind::Toolchanger => Ok(None),
        }
    }

    /// _buffer_toolhead_load_check
    pub fn buffer_toolhead_load_check(&self, lane: &Rc<AfcLane>) -> KResult<bool> {
        super::boxturtle::buffer_toolhead_load_check(self, lane)
    }

    pub fn get_calibrated_lanes(&self) -> Option<Vec<String>> {
        if self.eject_to_calibrate {
            return Some(self.lanes.borrow().values().filter(|l| l.load_state() && l.calibrated_lane.get() && !l.tool_loaded.get()).map(|l| l.name.clone()).collect());
        }
        None
    }

    /// Queries moonraker's TD-1 data and applies it when it is newer
    pub fn get_td1_data(&self, cur_lane: &Rc<AfcLane>, compare_time: DateTime, ignore_time: bool) -> KResult<bool> {
        let moonraker = self.afc.moonraker.borrow().clone().ok_or_else(|| super::no_attr("NoneType", "get_td1_data"))?;
        let td1_data = moonraker.get_td1_data();
        self.apply_td1_data(cur_lane, compare_time, td1_data.as_ref(), ignore_time)
    }

    pub fn apply_td1_data(&self, cur_lane: &Rc<AfcLane>, compare_time: DateTime, td1_data: Option<&PyVal>, ignore_time: bool) -> KResult<bool> {
        let c = &AFC_UNIT_AFCUNIT__APPLY_TD1_DATA;
        let Some(PyVal::Dict(devices)) = td1_data.map(|v| v.get()) else { return Ok(false) };
        if devices.is_empty() {
            return Ok(false);
        }
        self.afc.logger.debug(c, &format!("Data: {}, Compare_time: {}", td1_data.map(|v| v.repr()).unwrap_or_default(), compare_time.str()));
        let id = cur_lane.td1_device_id.borrow().clone().unwrap_or_default();
        let Some(data) = devices.get(id.as_str()).cloned() else {
            self.afc.error.afc_error(c, &format!("TD-1 Device ID ({}) supplied, but ID not found.", super::py_opt_str(cur_lane.td1_device_id.borrow().as_deref())), false, None)?;
            return Ok(false);
        };
        let PyVal::Dict(d) = data.get() else { return Ok(false) };
        let scan = d.get("scan_time").cloned().unwrap_or(PyVal::None);
        if matches!(scan, PyVal::None) {
            return Ok(false);
        }
        let s = scan.str();
        let scan_time = if s.ends_with("+00:00Z") { s[..s.len() - 1].to_string() } else { format!("{}+00:00", &s[..s.len().saturating_sub(1)]) };
        let scan_time = match DateTime::fromisoformat(&scan_time) {
            Some(t) => t,
            None => {
                self.afc.logger.error_opt(c, "Error trying to format TD-1 scan time, check AFC.log for more information", None, "");
                return Ok(false);
            }
        };
        let valid_data = scan_time.after(&compare_time) || compare_time.seconds_since(&scan_time) < 10.0;
        let td_ok = d.get("td").is_some_and(|v| !matches!(v, PyVal::None));
        let color_ok = d.get("color").is_some_and(|v| !matches!(v, PyVal::None));
        if (valid_data || ignore_time) && td_ok && color_ok {
            *cur_lane.td1_data.borrow_mut() = data.clone();
            self.afc.logger.info(c, &format!("{} TD-1 data captured", cur_lane.name));
            self.afc.save_vars()?;
            return Ok(true);
        }
        Ok(false)
    }

    pub fn prep_load(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        match &self.kind {
            UnitKind::Canvas(_) => super::canvas::unit_prep_load(self, lane),
            // ACE firmware stages filament at the slot on insert
            UnitKind::Ace(_) => Ok(()),
            UnitKind::BoxTurtle(_) if super::vivid::is_selector_unit(self) => super::vivid::prep_load(self, lane),
            // (HTLF has no prep switches)
            UnitKind::BoxTurtle(super::boxturtle::Family::Htlf) => Ok(()),
            UnitKind::BoxTurtle(_) => super::boxturtle::unit_prep_load(self, lane),
            UnitKind::Toolchanger => self.print_not_defined("prep_load"),
            // (the OpenAMS firmware drives filament to the load sensor)
            UnitKind::OpenAms(_) => Ok(()),
        }
    }

    pub fn prep_post_load(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        match &self.kind {
            UnitKind::Canvas(_) => super::canvas::unit_prep_post_load(self, lane),
            UnitKind::Ace(_) => Ok(()),
            // (ViViD, Claymore: nothing)
            UnitKind::BoxTurtle(_) if super::vivid::is_selector_unit(self) || super::htlf::is_htlf(self) => Ok(()),
            UnitKind::BoxTurtle(_) => super::boxturtle::unit_prep_post_load(self, lane),
            UnitKind::Toolchanger => self.print_not_defined("prep_post_load"),
            UnitKind::OpenAms(_) => Ok(()),
        }
    }

    pub fn eject_lane(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        match &self.kind {
            UnitKind::Canvas(_) => super::canvas::unit_eject_lane(self, lane),
            UnitKind::Ace(_) => super::ace::eject_lane(self, lane),
            UnitKind::BoxTurtle(_) => {
                if super::vivid::eject_lane(self, lane)? {
                    return Ok(());
                }
                super::boxturtle::unit_eject_lane(self, lane)
            }
            UnitKind::Toolchanger => self.print_not_defined("eject_lane"),
            UnitKind::OpenAms(_) => super::openams::eject_lane(self, lane),
        }
    }

    pub fn on_filament_insert(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        if let UnitKind::OpenAms(_) = self.kind {
            super::openams::on_filament_insert(self, lane)?;
        }
        self.printer.send_event("afc:lane_inserted", &[lane])
    }

    pub fn on_filament_remove(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        match &self.kind {
            UnitKind::OpenAms(_) => super::openams::on_filament_remove(self, lane),
            _ => Ok(()),
        }
    }

    /// getattr(unit, "handle_runout"): true when the unit handled the runout
    pub fn handle_runout(&self, lane: &Rc<AfcLane>) -> KResult<bool> {
        match &self.kind {
            UnitKind::OpenAms(_) => super::openams::handle_runout(self, lane),
            _ => Ok(false),
        }
    }

    /// getattr(unit, "prep_capture_td1"): Some(result) when the unit captured
    pub fn prep_capture_td1(&self, lane: &Rc<AfcLane>) -> KResult<Option<(bool, String)>> {
        match &self.kind {
            UnitKind::OpenAms(_) => super::openams::prep_capture_td1(self, lane),
            _ => Ok(None),
        }
    }

    /// getattr(unit, "capture_td1_data"): Some(result) from the unit's capture
    pub fn capture_td1_data(&self, lane: &Rc<AfcLane>) -> KResult<Option<(bool, String)>> {
        match &self.kind {
            UnitKind::OpenAms(_) => super::openams::capture_td1_data(self, lane).map(Some),
            _ => Ok(None),
        }
    }

    pub fn move_to_hub(&self, lane: &Rc<AfcLane>, dist: f64, dir: MoveDirection, use_homing: bool, speed_mode: SpeedMode) -> KResult<(bool, f64, MoveWarning)> {
        self.move_to_hub_assist(lane, dist, dir, use_homing, speed_mode, AssistActive::Dynamic)
    }

    /// move_to_hub with its assist_active argument
    pub fn move_to_hub_assist(&self, lane: &Rc<AfcLane>, dist: f64, dir: MoveDirection, use_homing: bool, speed_mode: SpeedMode, assist_active: AssistActive) -> KResult<(bool, f64, MoveWarning)> {
        match &self.kind {
            UnitKind::Canvas(_) => super::canvas::unit_move_to_hub(self, lane, dist, dir, use_homing, speed_mode),
            UnitKind::BoxTurtle(_) => super::boxturtle::unit_move_to_hub(self, lane, dist, dir, use_homing, speed_mode, assist_active),
            UnitKind::Ace(_) | UnitKind::OpenAms(_) => lane.move_to(dist * dir.value(), speed_mode, &lane.hub_endstop_name(), assist_active, use_homing),
            UnitKind::Toolchanger => Ok(super::toolchanger::no_move()),
        }
    }

    pub fn move_to_load(&self, lane: &Rc<AfcLane>, dist: f64, dir: MoveDirection, use_homing: bool, speed_mode: SpeedMode) -> KResult<(bool, f64, MoveWarning)> {
        if let UnitKind::Toolchanger = self.kind {
            return Ok(super::toolchanger::no_move());
        }
        lane.move_to(dist * dir.value(), speed_mode, &lane.load_es(), AssistActive::Dynamic, use_homing)
    }

    pub fn load_then_home(&self, lane: &Rc<AfcLane>, distance: f64, assist_active: AssistActive, endstop: &Endstop) -> KResult<(bool, f64, MoveWarning)> {
        match &self.kind {
            UnitKind::Toolchanger => Ok(super::toolchanger::no_move()),
            UnitKind::Canvas(_) => super::canvas::unit_load_then_home(self, lane, distance, endstop),
            UnitKind::Ace(_) => {
                let home_to_tool = self.afc.homing_enabled && self.afc.home_to_tool;
                lane.move_to(distance, SpeedMode::Long, endstop, assist_active, home_to_tool)
            }
            UnitKind::BoxTurtle(_) | UnitKind::OpenAms(_) => {
                let mut total_distance = distance;
                let mut speed_mode = SpeedMode::Long;
                // Only do load then home if user has buffer defined and load then home enabled
                let home_to_tool = self.afc.homing_enabled && self.afc.home_to_tool;
                // load_and_home only when the distance is over load_undershoot
                let (load_then_home, load_undershoot) = {
                    let s = lane.settings();
                    (s.load_then_home_var.unwrap_or(false), s.load_undershoot.unwrap_or(0.0))
                };
                let load_and_home = load_then_home && distance > load_undershoot;
                if load_and_home && lane.extruder_obj()?.tool_start.as_deref() == Some("buffer") {
                    let load_length = distance - load_undershoot;
                    lane.move_to(load_length, speed_mode, endstop, assist_active, false)?;
                    total_distance -= load_length;
                    speed_mode = SpeedMode::Short;
                }
                lane.move_to(total_distance, speed_mode, endstop, assist_active, home_to_tool)
            }
        }
    }
}

impl PrinterObject for AfcUnit {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        Some(AfcUnit::get_status(self))
    }
}
