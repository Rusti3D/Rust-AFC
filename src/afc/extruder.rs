// An AFC extruder: the toolhead's sensors and lengths (port of
// AFC_extruder.py, without the toolchanger, U1 and LED strip parts:
// native_scope leaves those configs to the python AFC)
//
// Copyright (C) 2024-2026 Armored Turtle
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::lane::AfcLane;
use super::stats::StatVar;
use super::utils::{add_filament_switch, SwitchOptions};
use super::{no_attr, Afc, State};
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::extras::buttons::PrinterButtons;
use kaliru::extras::filament_switch_sensor::SwitchSensor;
use kaliru::gcode::GCodeCommand;
use kaliru::printer::{PObj, Printer, PrinterObject};
use kaliru::pyfmt::py_float_repr;
use kaliru::pyval::PyVal;
use kaliru::toolhead::ExtraAxisRef;
use kaliru::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub const CMD_UPDATE_TOOLHEAD_SENSORS_HELP: &str = "Gives ability to update tool_stn, tool_stn_unload, tool_sensor_after_extruder values without restarting klipper";
pub const CMD_SAVE_EXTRUDER_VALUES_HELP: &str = "Saves tool_stn, tool_stn_unload and tool_sensor_after_extruder values to config file.";

/// AFCExtruderStats: cut and toolchange counts (local until moonraker connects)
pub struct ExtruderStats {
    name: String,
    logger: Rc<super::logger::AfcLogger>,
    cut_total: RefCell<Rc<StatVar>>,
    cut_total_since_changed: RefCell<Rc<StatVar>>,
    last_blade_changed: RefCell<Rc<StatVar>>,
    pub cut_threshold_for_warning: i64,
    threshold_warning_sent: Cell<bool>,
    threshold_error_sent: Cell<bool>,
    tc_total: RefCell<Rc<StatVar>>,
    tc_tool_unload: RefCell<Rc<StatVar>>,
    tc_tool_load: RefCell<Rc<StatVar>>,
    tool_selected: RefCell<Rc<StatVar>>,
    tool_unselected: RefCell<Rc<StatVar>>,
}

macro_rules! stat_getters {
    ($($f:ident),*) => {$(
        pub fn $f(&self) -> Rc<StatVar> {
            self.$f.borrow().clone()
        }
    )*};
}

impl ExtruderStats {
    fn new(name: &str, logger: Rc<super::logger::AfcLogger>, cut_threshold: i64) -> ExtruderStats {
        let z = || RefCell::new(Rc::new(StatVar::local(PyVal::Int(0))));
        ExtruderStats {
            name: name.to_string(),
            logger,
            cut_total: z(),
            cut_total_since_changed: z(),
            last_blade_changed: RefCell::new(Rc::new(StatVar::local(PyVal::Str("N/A".into())))),
            cut_threshold_for_warning: cut_threshold,
            threshold_warning_sent: Cell::new(false),
            threshold_error_sent: Cell::new(false),
            tc_total: z(),
            tc_tool_unload: z(),
            tc_tool_load: z(),
            tool_selected: z(),
            tool_unselected: z(),
        }
    }

    stat_getters!(cut_total, cut_total_since_changed, last_blade_changed, tc_total, tc_tool_unload, tc_tool_load, tool_selected, tool_unselected);

    fn handle_moonraker_stats(&self, afc: &Afc) -> KResult<()> {
        let moonraker = afc.moonraker.borrow().clone().ok_or_else(|| no_attr("NoneType", "get_afc_stats"))?;
        let values = moonraker.get_afc_stats();
        let v = values.as_ref();
        let m = || moonraker.clone();
        let cut_parent_name = format!("{}.cut", self.name);
        let tool_parent_name = format!("{}.change_count", self.name);
        let set = |slot: &RefCell<Rc<StatVar>>, s: StatVar| *slot.borrow_mut() = Rc::new(s);
        if self.name == "extruder" {
            set(&self.cut_total, StatVar::new("cut", "cut_total", v, m(), &cut_parent_name, false));
            set(&self.cut_total_since_changed, StatVar::new("cut", "cut_total_since_changed", v, m(), &cut_parent_name, false));
            set(&self.last_blade_changed, StatVar::new("cut", "last_blade_changed", v, m(), &cut_parent_name, false));
            set(&self.tc_total, StatVar::new("toolchange_count", "total", v, m(), &tool_parent_name, false));
            set(&self.tc_tool_unload, StatVar::new("toolchange_count", "tool_unload", v, m(), &tool_parent_name, false));
            set(&self.tc_tool_load, StatVar::new("toolchange_count", "tool_load", v, m(), &tool_parent_name, false));
        } else {
            set(&self.cut_total, StatVar::new(&cut_parent_name, "cut_total", v, m(), "", false));
            set(&self.cut_total_since_changed, StatVar::new(&cut_parent_name, "cut_total_since_changed", v, m(), "", false));
            set(&self.last_blade_changed, StatVar::new(&cut_parent_name, "last_blade_changed", v, m(), "", false));
            set(&self.tc_total, StatVar::new(&cut_parent_name, "total", v, m(), "", false));
            set(&self.tc_tool_unload, StatVar::new(&cut_parent_name, "tool_unload", v, m(), "", false));
            set(&self.tc_tool_load, StatVar::new(&cut_parent_name, "tool_load", v, m(), "", false));
        }
        set(&self.tool_selected, StatVar::new(&tool_parent_name, "tool_selected", v, m(), "", false));
        set(&self.tool_unselected, StatVar::new(&tool_parent_name, "tool_unselected", v, m(), "", false));
        let lbc = self.last_blade_changed();
        if lbc.value().py_eq(&PyVal::Int(0)) {
            lbc.set_value(PyVal::Str("N/A".into()));
            lbc.update_database();
        }
        Ok(())
    }

    /// Warns near / past the blade's cut threshold (once each)
    pub fn check_cut_threshold(&self, afc: &Afc) -> KResult<()> {
        let c = &AFC_EXTRUDER_AFCEXTRUDERSTATS_CHECK_CUT_THRESHOLD;
        let lbc = self.last_blade_changed();
        let blade_changed_date_string = if lbc.value().py_eq(&PyVal::Int(0)) { "N/A".to_string() } else { lbc.str() };
        let since = self.cut_total_since_changed().value().as_f64().unwrap_or(0.0);
        let threshold = self.cut_threshold_for_warning as f64;
        let (warning_msg_time, warning_msg_threshold, span_start, message_type, send_message);
        if since >= threshold {
            warning_msg_time = "Time";
            warning_msg_threshold = "have exceeded";
            span_start = "<span class=error--text>";
            message_type = "error";
            send_message = !self.threshold_error_sent.replace(true);
        } else if since >= threshold - 1000.0 {
            warning_msg_time = "Almost time";
            warning_msg_threshold = "is about to exceeded";
            span_start = "<span class=warning--text>";
            message_type = "warning";
            send_message = !self.threshold_warning_sent.replace(true);
        } else {
            return Ok(());
        }
        let mut warning_msg = format!(
            "{} to change cutting blade as your blade has performed {} cuts\n",
            warning_msg_time,
            self.cut_total_since_changed().str()
        );
        warning_msg += &format!(
            "since changed on {}. Number of cuts {} set threshold of {}.\n",
            blade_changed_date_string, warning_msg_threshold, self.cut_threshold_for_warning
        );
        warning_msg += "Once blade is changed, execute AFC_CHANGE_BLADE macro to reset count and date changed.\n";
        if send_message {
            self.logger.raw(c, &format!("{}{}</span>", span_start, warning_msg));
            afc.logger.message_queue.borrow_mut().push((warning_msg, message_type));
        }
        Ok(())
    }

    pub fn increase_cut_total(&self) -> KResult<()> {
        self.cut_total().increase_count()?;
        self.cut_total_since_changed().increase_count()
    }

    pub fn increase_toolcount_change(&self, afc: &Afc) -> KResult<()> {
        self.tc_total().increase_count()?;
        if let Some(s) = afc.afc_stats.borrow().clone() {
            s.increase_toolchange_wo_error()?;
        }
        Ok(())
    }

    pub fn reset_stats(&self) -> KResult<()> {
        self.tc_total().reset_count()?;
        self.tc_tool_unload().reset_count()?;
        self.tc_tool_load().reset_count()?;
        self.tool_selected().reset_count()?;
        self.tool_unselected().reset_count()
    }
}

/// [AFC_extruder <name>]
pub struct AfcExtruder {
    me: Weak<AfcExtruder>,
    printer: Rc<Printer>,
    afc: Rc<Afc>,
    pub fullname: String,
    pub name: String,
    pub th_extruder_name: String,
    pub tool_start: Option<String>,
    pub tool_end: Option<String>,
    pub tool_stn: Cell<f64>,
    pub tool_stn_unload: Cell<f64>,
    pub tool_sensor_after_extruder: Cell<f64>,
    pub tool_unload_speed: Cell<f64>,
    pub tool_load_speed: Cell<f64>,
    pub buffer_name: Option<String>,
    pub enable_runout: Cell<bool>,
    pub enable_runout_in_bypass: Cell<bool>,
    pub deadband: f64,
    pub toolchange_temp_drop: f64,
    next_pickup: Cell<bool>,
    status: Cell<State>,
    pub lane_loaded: RefCell<Option<String>>,
    pub lanes: RefCell<IndexMap<String, Rc<AfcLane>>>,
    no_lanes: Cell<bool>,
    pub estats: ExtruderStats,
    pub tool_start_state: Cell<bool>,
    pub tool_end_state: Cell<bool>,
    pub fila_tool_start: RefCell<Option<Rc<SwitchSensor>>>,
    /// AFC_ACE bypass: (enable_runout, enable_runout_in_bypass, sensor_enabled)
    pub afc_ace_saved: Cell<Option<(bool, bool, bool)>>,
    fila_tool_end: RefCell<Option<Rc<SwitchSensor>>>,
    common_save_msg: String,
    toolhead_extruder: RefCell<Option<ExtraAxisRef>>,
}

impl AfcExtruder {
    fn new(config: &ConfigWrapper) -> KResult<Rc<AfcExtruder>> {
        let printer = config.get_printer();
        let buttons = printer.lookup_or_load::<PrinterButtons>(config, "buttons")?;
        let afc = printer.lookup_or_load::<Afc>(config, "AFC")?;
        let fullname = config.get_name().to_string();
        let name = fullname.split(' ').next_back().unwrap_or("").to_string();
        let th_extruder_name = config.get("extruder_name", name.clone())?;
        if !th_extruder_name.contains("extruder") {
            return Err(KError::config(format!(
                "Missing extruder reference in [{}] section. The word extruder must appear either in the section name ([AFC_extruder extruder(n)]) or in the extruder_name variable (extruder_name: extruder(n)).",
                fullname
            )));
        }
        let tool_start = config.get_opt("pin_tool_start")?;
        let tool_end = config.get_opt("pin_tool_end")?;
        let tool_stn = config.getfloat("tool_stn", 72.0, Lim::new())?;
        let tool_stn_unload = config.getfloat("tool_stn_unload", 100.0, Lim::new())?;
        let tool_sensor_after_extruder = config.getfloat("tool_sensor_after_extruder", 0.0, Lim::new())?;
        let tool_unload_speed = config.getfloat("tool_unload_speed", 25.0, Lim::new())?;
        let tool_load_speed = config.getfloat("tool_load_speed", 25.0, Lim::new())?;
        let buffer_name = config.get_opt("buffer")?;
        let enable_sensors_in_gui = config.getboolean("enable_sensors_in_gui", afc.enable_sensors_in_gui)?;
        let enable_runout = config.getboolean("enable_tool_runout", afc.enable_tool_runout)?;
        let enable_runout_in_bypass = config.getboolean("enable_runout_in_bypass", afc.enable_runout_in_bypass)?;
        let debounce_delay = config.getfloat("debounce_delay", afc.debounce_delay, Lim::new())?;
        let deadband = config.getfloat("deadband", 2.0, Lim::new())?;
        let toolchange_temp_drop = config.getfloat("toolchange_temp_drop", afc.toolchange_temp_drop, Lim::new())?;
        for opt in ["led_name", "status_led_idx", "nozzle_led_idx", "u1_filament_sensor_name", "u1_park_detector_name", "toolchanger_unit", "tool"] {
            if config.get_opt(opt)?.is_some() {
                return Err(KError::config(format!("AFC_extruder {} is not native", opt)));
            }
        }
        let _map = config.getlist("map", PyVal::List(vec![]), ",", None)?;
        let _custom_tool_swap = config.get_opt("custom_tool_swap")?;
        let _custom_unselect = config.get_opt("custom_unselect")?;
        let _enable_standalone_purge = config.getboolean("enable_standalone_purge", afc.enable_standalone_purge)?;
        let _standalone_auto_load_unload = config.getboolean("standalone_auto_load_unload", afc.standalone_auto_load_unload)?;
        let estats = ExtruderStats::new(&th_extruder_name, afc.logger.clone(), afc.tool_cut_threshold);

        let ext = Rc::new_cyclic(|me| AfcExtruder {
            me: me.clone(),
            printer: printer.clone(),
            afc: afc.clone(),
            fullname: fullname.clone(),
            name: name.clone(),
            th_extruder_name,
            tool_start: tool_start.clone(),
            tool_end: tool_end.clone(),
            tool_stn: Cell::new(tool_stn),
            tool_stn_unload: Cell::new(tool_stn_unload),
            tool_sensor_after_extruder: Cell::new(tool_sensor_after_extruder),
            tool_unload_speed: Cell::new(tool_unload_speed),
            tool_load_speed: Cell::new(tool_load_speed),
            buffer_name,
            enable_runout: Cell::new(enable_runout),
            enable_runout_in_bypass: Cell::new(enable_runout_in_bypass),
            deadband,
            toolchange_temp_drop,
            next_pickup: Cell::new(false),
            status: Cell::new(State::Idle),
            lane_loaded: RefCell::new(None),
            lanes: RefCell::new(IndexMap::default()),
            no_lanes: Cell::new(false),
            estats,
            tool_start_state: Cell::new(false),
            tool_end_state: Cell::new(false),
            fila_tool_start: RefCell::new(None),
            afc_ace_saved: Cell::new(None),
            fila_tool_end: RefCell::new(None),
            common_save_msg: format!("\nRun SAVE_EXTRUDER_VALUES EXTRUDER={} once done to update values in config", name),
            toolhead_extruder: RefCell::new(None),
        });
        let e = ext.clone();
        kaliru::events::KlippyConnect::on(&printer, move || e.handle_connect());
        let e = ext.clone();
        printer.register_event_handler(super::utils::MOONRAKER_CONNECT_EVENT, Rc::new(move |_| e.estats.handle_moonraker_stats(&e.afc)));
        let e = ext.clone();
        kaliru::events::KlippyReady::on(&printer, move || e.handle_ready());

        if let Some(pin) = &tool_start {
            if pin.to_lowercase() == "unknown" {
                return Err(KError::config(format!("Unknown is not valid for pin_tool_start in [{}] config.", fullname)));
            }
            if pin == "buffer" || pin == "virtual" {
                return Err(KError::config(format!("AFC_extruder pin_tool_start {} is not native", pin)));
            }
            let w = Rc::downgrade(&ext);
            let (fila, _b) = add_filament_switch(
                &format!("{}_tool_start", name),
                pin,
                &printer,
                &SwitchOptions {
                    show_sensor: enable_sensors_in_gui,
                    runout_callback: Some(Rc::new(move |et| w.upgrade().map_or(Ok(()), |e| e.handle_start_runout(et)))),
                    enable_runout,
                    debounce_delay,
                },
            )?;
            *ext.fila_tool_start.borrow_mut() = Some(fila);
            let w = Rc::downgrade(&ext);
            buttons.register_buttons(
                &[pin.as_str()],
                Rc::new(move |_et, state| {
                    if let Some(e) = w.upgrade() {
                        e.tool_start_state.set(state != 0);
                    }
                    Ok(())
                }),
            )?;
        }
        if let Some(pin) = &tool_end {
            if pin.to_lowercase() == "unknown" {
                return Err(KError::config(format!("Unknown is not valid for pin_tool_end in [{}] config.", fullname)));
            }
            let w = Rc::downgrade(&ext);
            buttons.register_buttons(
                &[pin.as_str()],
                Rc::new(move |_et, state| {
                    if let Some(e) = w.upgrade() {
                        e.tool_end_state.set(state != 0);
                    }
                    Ok(())
                }),
            )?;
            let w = Rc::downgrade(&ext);
            let (fila, _b) = add_filament_switch(
                &format!("{}_tool_end", name),
                pin,
                &printer,
                &SwitchOptions {
                    show_sensor: enable_sensors_in_gui,
                    runout_callback: Some(Rc::new(move |et| w.upgrade().map_or(Ok(()), |e| e.handle_end_runout(et)))),
                    enable_runout,
                    debounce_delay,
                },
            )?;
            *ext.fila_tool_end.borrow_mut() = Some(fila);
        }
        let show_macros = afc.show_macros;
        let e = ext.clone();
        afc.function.register_mux_command(
            show_macros,
            "UPDATE_TOOLHEAD_SENSORS",
            "EXTRUDER",
            &name,
            Rc::new(move |g| e.cmd_update_toolhead_sensors(g)),
            CMD_UPDATE_TOOLHEAD_SENSORS_HELP,
            Some(&[("EXTRUDER", "string", "extruder"), ("TOOL_STN", "float", "0"), ("TOOL_STN_UNLOAD", "float", "0"), ("TOOL_AFTER_EXTRUDER", "float", "0")]),
        )?;
        let e = ext.clone();
        afc.function.register_mux_command(
            show_macros,
            "SAVE_EXTRUDER_VALUES",
            "EXTRUDER",
            &name,
            Rc::new(move |g| e.cmd_save_extruder_values(g)),
            CMD_SAVE_EXTRUDER_VALUES_HELP,
            Some(&[("EXTRUDER", "string", "extruder")]),
        )?;
        Ok(ext)
    }

    pub fn toolhead_extruder(&self) -> KResult<ExtraAxisRef> {
        self.toolhead_extruder.borrow().clone().ok_or_else(|| no_attr("AFCExtruder", "toolhead_extruder"))
    }

    pub fn get_heater(&self) -> KResult<Rc<kaliru::extras::heaters::Heater>> {
        self.toolhead_extruder()?.get_heater()
    }

    /// (no toolchanger lane to drop)
    pub fn check_lanes(&self) {}

    fn handle_ready(&self) -> KResult<()> {
        if self.lanes.borrow().contains_key(&self.name) {
            self.no_lanes.set(true);
            self.afc.logger.info(&AFC_EXTRUDER_AFCEXTRUDER_HANDLE_READY, &format!("{} no lanes", self.name));
        }
        Ok(())
    }

    fn handle_connect(&self) -> KResult<()> {
        let me = self.me.upgrade().expect("AFC extruder");
        if let Some(existing) = self.afc.tools.borrow().get(&self.th_extruder_name) {
            if !Rc::ptr_eq(existing, &me) {
                return Err(KError::config(format!(
                    "Duplicate toolhead extruder mapping '{}' between [{}] and [{}]. Each AFC_extruder section must map to a unique toolhead extruder.",
                    self.th_extruder_name, existing.fullname, self.fullname
                )));
            }
        }
        self.afc.tools.borrow_mut().insert(self.th_extruder_name.clone(), me);
        let found = self
            .printer
            .lookup_object_opt(&self.th_extruder_name)
            .and_then(kaliru::printer::downcast_rc::<kaliru::kinematics::extruder::PrinterExtruder>);
        match found {
            Some(e) => *self.toolhead_extruder.borrow_mut() = Some(e),
            None => return Err(KError::config(format!("[{}] not found in config file for {}", self.th_extruder_name, self.fullname))),
        }
        Ok(())
    }

    /// Runout at a toolhead sensor: to the loaded lane, or (bypass) pause
    fn handle_toolhead_sensor_runout(&self, state: bool, sensor_name: &str) -> KResult<()> {
        if state {
            return Ok(());
        }
        let loaded = self.lane_loaded.borrow().clone();
        if let Some(lane) = loaded.and_then(|l| self.lanes.borrow().get(&l).cloned()) {
            lane.handle_toolhead_runout(sensor_name)?;
        } else if self.enable_runout.get()
            && self.enable_runout_in_bypass.get()
            && !self.afc.in_toolchange.get()
            && !self.afc.error_state.get()
            && self.on_shuttle()
            && self.afc.function.is_printing(false)?
        {
            let msg = format!("Toolhead runout detected by {} sensor in bypass/manual mode.", sensor_name);
            self.afc.error.afc_error(&AFC_EXTRUDER_AFCEXTRUDER__HANDLE_TOOLHEAD_SENSOR_RUNOUT, &msg, true, None)?;
        }
        Ok(())
    }

    fn handle_start_runout(&self, _eventtime: f64) -> KResult<()> {
        let fila = self.fila_tool_start.borrow().clone().ok_or_else(|| no_attr("NoneType", "runout_helper"))?;
        self.handle_toolhead_sensor_runout(fila.runout_helper.filament_present.get(), "tool_start")?;
        let rh = &fila.runout_helper;
        rh.min_event_systime.set(self.afc.reactor.monotonic() + rh.event_delay.get());
        Ok(())
    }

    fn handle_end_runout(&self, _eventtime: f64) -> KResult<()> {
        let fila = self.fila_tool_end.borrow().clone().ok_or_else(|| no_attr("AFCExtruder", "fila_tool_end"))?;
        self.handle_toolhead_sensor_runout(fila.runout_helper.filament_present.get(), "tool_end")?;
        let rh = &fila.runout_helper;
        rh.min_event_systime.set(self.afc.reactor.monotonic() + rh.event_delay.get());
        Ok(())
    }

    /// The sensor's runout switch (the gui's SET_FILAMENT_SENSOR ENABLE)
    pub fn tool_sensor_enabled(&self, sensor: &str) -> KResult<bool> {
        let fila = if sensor == "tool_start" { self.fila_tool_start.borrow().clone() } else { self.fila_tool_end.borrow().clone() };
        fila.map(|f| f.runout_helper.sensor_enabled.get()).ok_or_else(|| no_attr("NoneType", "runout_helper"))
    }

    /// (standalone extruders need a toolchanger lane: outside the native scope)
    pub fn load_unload_sequence(&self, _distance: f64) -> KResult<()> {
        Err(no_attr("NoneType", "unit_obj"))
    }

    fn update_tool_stn(&self, length: f64) {
        let c = &AFC_EXTRUDER_AFCEXTRUDER__UPDATE_TOOL_STN;
        if length > 0.0 {
            let msg = format!("tool_stn updated old: {}, new: {}{}", py_float_repr(self.tool_stn.get()), py_float_repr(length), self.common_save_msg);
            self.tool_stn.set(length);
            self.afc.logger.info(c, &msg);
        } else {
            self.afc.logger.error(c, "tool_stn length should be greater than zero");
        }
    }

    fn update_tool_stn_unload(&self, length: f64) {
        let c = &AFC_EXTRUDER_AFCEXTRUDER__UPDATE_TOOL_STN_UNLOAD;
        if length >= 0.0 {
            let msg = format!("tool_stn_unload updated old: {}, new: {}{}", py_float_repr(self.tool_stn_unload.get()), py_float_repr(length), self.common_save_msg);
            self.tool_stn_unload.set(length);
            self.afc.logger.info(c, &msg);
        } else {
            self.afc.logger.error(c, "tool_stn_unload length should be greater than or equal to zero");
        }
    }

    fn update_tool_after_extr(&self, length: f64) {
        let c = &AFC_EXTRUDER_AFCEXTRUDER__UPDATE_TOOL_AFTER_EXTR;
        if length > 0.0 {
            let msg = format!(
                "tool_sensor_after_extruder updated old: {}, new: {}{}",
                py_float_repr(self.tool_sensor_after_extruder.get()),
                py_float_repr(length),
                self.common_save_msg
            );
            self.tool_sensor_after_extruder.set(length);
            self.afc.logger.info(c, &msg);
        } else {
            self.afc.logger.error(c, "tool_sensor_after_extruder length should be greater than zero");
        }
    }

    /// The toolhead status LEDs (none: led_name is outside the native scope)
    pub fn set_status_led(&self, _color: &str) -> KResult<()> {
        Ok(())
    }

    /// set_print_leds: (success, error) - there is no toolhead LED object
    pub fn set_print_leds(&self, _state: i64, quiet: bool) -> KResult<(bool, String)> {
        let error_string = format!("led_name variable not set in [{}] config section", self.fullname);
        if !quiet {
            self.afc.logger.error(&AFC_EXTRUDER_AFCEXTRUDER_SET_PRINT_LEDS, &error_string);
        }
        Ok((false, error_string))
    }

    /// Single toolhead printers: always on the shuttle
    pub fn on_shuttle(&self) -> bool {
        true
    }

    /// prep_on_shuttle_check: PREP's " in ToolHead" (a toolchanger tool
    /// also reports whether it is on the shuttle)
    pub fn prep_on_shuttle_check(&self) -> String {
        "<span class=primary--text> in ToolHead</span>".to_string()
    }

    pub fn is_standalone(&self) -> bool {
        self.no_lanes.get()
    }

    /// repr() of the lanes dict, as AFC.log prints it (objects by address)
    pub fn lanes_repr(&self) -> String {
        let items: Vec<String> = self
            .lanes
            .borrow()
            .values()
            .map(|l| {
                let cls = if l.canvas.borrow().is_some() { "extras.AFC_canvas_lane.AFCCanvasLane" } else { "extras.AFC_lane.AFCLane" };
                format!("{}: <{} object at {:#x}>", kaliru::pyfmt::py_str_repr(&l.name), cls, Rc::as_ptr(l) as usize)
            })
            .collect();
        format!("{{{}}}", items.join(", "))
    }

    fn cmd_update_toolhead_sensors(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let tool_stn = gcmd.get_float("TOOL_STN", Some(self.tool_stn.get()), Lim::new())?;
        let tool_stn_unload = gcmd.get_float("TOOL_STN_UNLOAD", Some(self.tool_stn_unload.get()), Lim::new())?;
        let tool_sensor_after_extruder = gcmd.get_float("TOOL_AFTER_EXTRUDER", Some(self.tool_sensor_after_extruder.get()), Lim::new())?;
        if tool_stn != self.tool_stn.get() {
            self.update_tool_stn(tool_stn);
        }
        if tool_stn_unload != self.tool_stn_unload.get() {
            self.update_tool_stn_unload(tool_stn_unload);
        }
        if tool_sensor_after_extruder != self.tool_sensor_after_extruder.get() {
            self.update_tool_after_extr(tool_sensor_after_extruder);
        }
        Ok(())
    }

    fn cmd_save_extruder_values(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let f = &self.afc.function;
        f.config_rewrite(&self.fullname, "tool_stn", &PyVal::Float(self.tool_stn.get()), "")?;
        f.config_rewrite(&self.fullname, "tool_stn_unload", &PyVal::Float(self.tool_stn_unload.get()), "")?;
        f.config_rewrite(&self.fullname, "tool_sensor_after_extruder", &PyVal::Float(self.tool_sensor_after_extruder.get()), "")
    }

    pub fn get_status(&self) -> KResult<PyVal> {
        let opt = |v: &Option<String>| v.clone().map(PyVal::Str).unwrap_or(PyVal::None);
        Ok(PyVal::Dict(kaliru::pydict! {
            "tool_stn" => PyVal::Float(self.tool_stn.get()),
            "tool_stn_unload" => PyVal::Float(self.tool_stn_unload.get()),
            "tool_sensor_after_extruder" => PyVal::Float(self.tool_sensor_after_extruder.get()),
            "tool_unload_speed" => PyVal::Float(self.tool_unload_speed.get()),
            "tool_load_speed" => PyVal::Float(self.tool_load_speed.get()),
            "buffer" => opt(&self.buffer_name),
            "lane_loaded" => opt(&self.lane_loaded.borrow()),
            "tool_start" => opt(&self.tool_start),
            "tool_start_status" => PyVal::Bool(self.tool_start_state.get()),
            "tool_end" => opt(&self.tool_end),
            "tool_end_status" => PyVal::Bool(self.tool_end_state.get()),
            "lanes" => PyVal::List(self.lanes.borrow().values().map(|l| PyVal::Str(l.name.clone())).collect()),
            "on_shuttle" => PyVal::Bool(self.on_shuttle()),
            "is_standalone" => PyVal::Bool(self.is_standalone()),
            "next_pickup" => PyVal::Bool(self.next_pickup.get()),
            "status" => PyVal::from(self.status.get().as_str()),
        }))
    }
}

impl PrinterObject for AfcExtruder {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        AfcExtruder::get_status(self).ok()
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(AfcExtruder::new(config)?)
}
