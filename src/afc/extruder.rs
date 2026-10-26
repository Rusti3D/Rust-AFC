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
use super::lane::LaneState;
use super::utils::{add_filament_switch, PyNum, SwitchOptions, VirtualSensor};
use super::{no_attr, Afc, State};
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::chelper::itersolve::SkRef;
use kaliru::chelper::kin_cartesian::cartesian_stepper_alloc;
use kaliru::chelper::trapq::TrapQRef;
use kaliru::extras::buttons::PrinterButtons;
use kaliru::extras::force_move::calc_move_time;
use kaliru::extras::motion_queuing::PrinterMotionQueuing;
use kaliru::kinematics::extruder::PrinterExtruder;
use kaliru::reactor::{Timer, NEVER};
use kaliru::extras::led::LEDHelper;
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
pub const CMD_AFC_SET_EXTRUDER_LED_HELP: &str = "Turns on toolhead leds for specified extruder name, does not affect status led if status_led_idx variable is provided";

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
    pub tool_stn: PyNum,
    pub tool_stn_unload: PyNum,
    pub tool_sensor_after_extruder: PyNum,
    pub tool_unload_speed: PyNum,
    pub tool_load_speed: PyNum,
    pub buffer_name: Option<String>,
    pub enable_runout: Cell<bool>,
    pub enable_runout_in_bypass: Cell<bool>,
    pub deadband: f64,
    pub toolchange_temp_drop: f64,
    pub next_pickup: Cell<bool>,
    pub status: Cell<State>,
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
    toolhead_leds: Option<String>,
    toolhead_status_index: Option<Vec<i64>>,
    toolhead_nozzle_index: Option<Vec<i64>>,
    /// the led_name object's LEDHelper (its _set_color / _check_transmit)
    toolhead_led: RefCell<Option<Rc<LEDHelper>>>,
    toolhead_led_found: Cell<bool>,
    status_led_count: Cell<i64>,
    /// toolchanger_unit: this toolhead is a tool of an AFC_Toolchanger, its
    /// own lane (tc_lane) when no unit feeds it
    pub tc_unit_name: Option<String>,
    pub tc_unit_obj: RefCell<Option<Rc<super::unit::AfcUnit>>>,
    pub tc_lane: RefCell<Option<Rc<AfcLane>>>,
    pub custom_tool_swap: Option<String>,
    pub custom_unselect: Option<String>,
    enable_standalone_purge: bool,
    standalone_auto_load_unload: bool,
    pub load_active: Cell<bool>,
    current_move_distance: Cell<f64>,
    captured_toolhead_temp: RefCell<Option<super::TempState>>,
    /// pin_tool_start virtual: the sensor its gui switch stands in for
    pub virtual_tool_start: RefCell<Option<Rc<VirtualSensor>>>,
    tc_motion: RefCell<Option<TcMotion>>,
    printer_extruder: RefCell<Option<Rc<PrinterExtruder>>>,
    extruder_move_timer: Timer,
    temp_check_timer: Timer,
}

/// A standalone tool's own move of its toolhead extruder (load / unload to
/// the nozzle): the stepper's kinematics and trapq while it moves
struct TcMotion {
    motion_queuing: Rc<PrinterMotionQueuing>,
    stepper_kinematics: SkRef,
    trapq: TrapQRef,
    prev_sk: RefCell<Option<SkRef>>,
    prev_trapq: RefCell<Option<TrapQRef>>,
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
        let tool_stn = super::getfloat_pv(config, "tool_stn", PyVal::Int(72))?;
        let tool_stn_unload = super::getfloat_pv(config, "tool_stn_unload", PyVal::Int(100))?;
        let tool_sensor_after_extruder = super::getfloat_pv(config, "tool_sensor_after_extruder", PyVal::Int(0))?;
        let tool_unload_speed = super::getfloat_pv(config, "tool_unload_speed", PyVal::Int(25))?;
        let tool_load_speed = super::getfloat_pv(config, "tool_load_speed", PyVal::Int(25))?;
        let buffer_name = config.get_opt("buffer")?;
        let enable_sensors_in_gui = config.getboolean("enable_sensors_in_gui", afc.enable_sensors_in_gui)?;
        let enable_runout = config.getboolean("enable_tool_runout", afc.enable_tool_runout)?;
        let enable_runout_in_bypass = config.getboolean("enable_runout_in_bypass", afc.enable_runout_in_bypass)?;
        let debounce_delay = config.getfloat("debounce_delay", afc.debounce_delay, Lim::new())?;
        let deadband = config.getfloat("deadband", 2.0, Lim::new())?;
        let toolchange_temp_drop = config.getfloat("toolchange_temp_drop", afc.toolchange_temp_drop, Lim::new())?;
        let toolhead_leds = config.get_opt("led_name")?;
        let toolhead_status_index = config.get_opt("status_led_idx")?;
        let toolhead_nozzle_index = config.get_opt("nozzle_led_idx")?;
        for opt in ["u1_filament_sensor_name", "u1_park_detector_name"] {
            if config.get_opt(opt)?.is_some() {
                return Err(KError::config(format!("AFC_extruder {} is not native", opt)));
            }
        }
        let toolhead_status_index = toolhead_status_index.filter(|s| !s.is_empty()).map(|s| afc.function.get_led_indexes(&s)).transpose()?;
        let toolhead_nozzle_index = toolhead_nozzle_index.filter(|s| !s.is_empty()).map(|s| afc.function.get_led_indexes(&s)).transpose()?;
        // If both status and nozzle indexes are provided, verify that they do not overlap
        if let (Some(n), Some(st)) = (&toolhead_nozzle_index, &toolhead_status_index) {
            if !n.is_empty() && !st.is_empty() {
                let mut both: Vec<i64> = st.iter().filter(|i| n.contains(i)).copied().collect();
                both.sort();
                both.dedup();
                if !both.is_empty() {
                    let list = both.iter().map(|i| i.to_string()).collect::<Vec<_>>().join(", ");
                    return Err(KError::config(format!("{} have overlapping led index(s) [{}]. Please fix and restart.", fullname, list)));
                }
            }
        }
        let tc_unit_name = config.get_opt("toolchanger_unit")?;
        // (a klipper-toolchanger tool object: native_scope leaves it to the python AFC)
        if config.get_opt("tool")?.is_some() {
            return Err(KError::config("AFC_extruder tool is not native"));
        }
        let _map = config.getlist("map", PyVal::List(vec![]), ",", None)?;
        let custom_tool_swap = config.get_opt("custom_tool_swap")?;
        let custom_unselect = config.get_opt("custom_unselect")?;
        let enable_standalone_purge = config.getboolean("enable_standalone_purge", afc.enable_standalone_purge)?;
        let standalone_auto_load_unload = config.getboolean("standalone_auto_load_unload", afc.standalone_auto_load_unload)?;
        let estats = ExtruderStats::new(&th_extruder_name, afc.logger.clone(), afc.tool_cut_threshold);

        let reactor = printer.get_reactor();
        let ext = Rc::new_cyclic(|me: &Weak<AfcExtruder>| {
            let w = me.clone();
            let extruder_move_timer = reactor.register_timer(
                Box::new(move |et| match w.upgrade() {
                    Some(e) => e.extruder_move_cb(et).unwrap_or_else(|err| {
                        kaliru::logging::exception_err("AFC extruder move", &err);
                        NEVER
                    }),
                    None => NEVER,
                }),
                NEVER,
            );
            let w = me.clone();
            let temp_check_timer = reactor.register_timer(
                Box::new(move |et| match w.upgrade() {
                    Some(e) => e.temp_check_cb(et).unwrap_or_else(|err| {
                        kaliru::logging::exception_err("AFC extruder temp check", &err);
                        NEVER
                    }),
                    None => NEVER,
                }),
                NEVER,
            );
            AfcExtruder {
            me: me.clone(),
            printer: printer.clone(),
            afc: afc.clone(),
            fullname: fullname.clone(),
            name: name.clone(),
            th_extruder_name,
            tool_start: tool_start.clone(),
            tool_end: tool_end.clone(),
            tool_stn: PyNum::new(&tool_stn),
            tool_stn_unload: PyNum::new(&tool_stn_unload),
            tool_sensor_after_extruder: PyNum::new(&tool_sensor_after_extruder),
            tool_unload_speed: PyNum::new(&tool_unload_speed),
            tool_load_speed: PyNum::new(&tool_load_speed),
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
            toolhead_leds: toolhead_leds.clone(),
            toolhead_status_index,
            toolhead_nozzle_index,
            toolhead_led: RefCell::new(None),
            toolhead_led_found: Cell::new(false),
            status_led_count: Cell::new(0),
            tc_unit_name: tc_unit_name.clone(),
            tc_unit_obj: RefCell::new(None),
            tc_lane: RefCell::new(None),
            custom_tool_swap,
            custom_unselect,
            enable_standalone_purge,
            standalone_auto_load_unload,
            load_active: Cell::new(false),
            current_move_distance: Cell::new(0.0),
            captured_toolhead_temp: RefCell::new(None),
            virtual_tool_start: RefCell::new(None),
            tc_motion: RefCell::new(None),
            printer_extruder: RefCell::new(None),
            extruder_move_timer,
            temp_check_timer,
        }
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
            if pin == "buffer" {
                return Err(KError::config(format!("AFC_extruder pin_tool_start {} is not native", pin)));
            }
        }
        if tool_start.as_deref() == Some("virtual") {
            // Virtual toolhead sensor for toolheads without a physical sensor (standalone
            // toolchanger lanes only)
            let w = Rc::downgrade(&ext);
            let vs = VirtualSensor::new(
                &printer,
                &format!("{}_tool_start", name),
                enable_sensors_in_gui,
                Rc::new(move |enabled| w.upgrade().map_or(Ok(()), |e| e.virtual_tool_start_toggle(enabled))),
            )?;
            *ext.virtual_tool_start.borrow_mut() = Some(vs);
        } else if let Some(pin) = &tool_start {
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
                Rc::new(move |_et, state| w.upgrade().map_or(Ok(()), |e| e.tool_start_callback(state != 0))),
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
        if let Some(tc) = &tc_unit_name {
            {
                let mut fc = config.fileconfig.borrow_mut();
                fc.set(&fullname, "unit", tc.split_whitespace().last().unwrap_or(""))?;
                fc.set(&fullname, "extruder", &name)?;
                fc.set(&fullname, "hub", "direct")?;
                fc.set(&fullname, "standalone", "True")?;
            }
            let lane = AfcLane::new(config, false)?;
            printer.objects_set(&format!("AFC_lane {}", name), lane.clone());
            *ext.tc_lane.borrow_mut() = Some(lane);
            let motion_queuing = printer.load_as::<PrinterMotionQueuing>(config, "motion_queuing")?;
            let stepper_kinematics = cartesian_stepper_alloc('x').expect("axis");
            let trapq = motion_queuing.allocate_trapq();
            *ext.tc_motion.borrow_mut() = Some(TcMotion { motion_queuing, stepper_kinematics, trapq, prev_sk: RefCell::new(None), prev_trapq: RefCell::new(None) });
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
        if toolhead_leds.is_some() {
            let e = ext.clone();
            afc.function.register_mux_command(
                show_macros,
                "AFC_SET_EXTRUDER_LED",
                "EXTRUDER",
                &name,
                Rc::new(move |g| e.cmd_afc_set_extruder_led(g)),
                CMD_AFC_SET_EXTRUDER_LED_HELP,
                Some(&[("EXTRUDER", "string", "extruder"), ("TURN_ON", "int", "1")]),
            )?;
        }
        Ok(ext)
    }

    pub fn toolhead_extruder(&self) -> KResult<ExtraAxisRef> {
        self.toolhead_extruder.borrow().clone().ok_or_else(|| no_attr("AFCExtruder", "toolhead_extruder"))
    }

    pub fn get_heater(&self) -> KResult<Rc<kaliru::extras::heaters::Heater>> {
        self.toolhead_extruder()?.get_heater()
    }

    /// check_lanes: a toolhead other lanes feed drops its own (tc) lane
    pub fn check_lanes(&self) {
        let Some(tc_lane) = self.tc_lane.borrow().clone() else { return };
        if self.lanes.borrow().len() > 1 && self.lanes.borrow().contains_key(&tc_lane.name) {
            if let Some(u) = tc_lane.unit_obj_opt() {
                u.lanes.borrow_mut().shift_remove(&tc_lane.name);
            }
            self.lanes.borrow_mut().shift_remove(&tc_lane.name);
            self.afc.lanes.borrow_mut().shift_remove(&tc_lane.name);
            self.printer.objects_remove(&format!("AFC_lane {}", self.name));
        }
    }

    fn handle_ready(&self) -> KResult<()> {
        // a toolhead that is its own only lane is "standalone"
        if self.lanes.borrow().contains_key(&self.name) {
            self.no_lanes.set(true);
            self.afc.logger.info(&AFC_EXTRUDER_AFCEXTRUDER_HANDLE_READY, &format!("{} no lanes", self.name));
            // Due to race conditions at startup, these variables might not be set correctly,
            //  set to current tool start state
            if let Some(tc) = self.tc_lane.borrow().clone() {
                tc.load_state_raw.set(self.tool_start_state.get());
                tc.prep_state.set(self.tool_start_state.get());
                if self.tool_start_state.get() {
                    tc.set_loaded()?;
                    tc.set_tool_loaded(false)?;
                }
            }
            if self.tool_start.as_deref() == Some("buffer") {
                return Err(KError::config(format!("buffer is not valid config for pin_tool_start when using {} as a standalone extruder", self.name)));
            }
        } else if self.tool_start.as_deref() == Some("virtual") {
            return Err(KError::config(format!(
                "{} config section cannot have a virtual toolhead sensor when lanes are configured for this toolhead.",
                self.fullname
            )));
        }
        Ok(())
    }

    /// tool_start_callback: the toolhead sensor; a standalone tool loads
    /// (or unloads) its nozzle by itself
    pub fn tool_start_callback(&self, state: bool) -> KResult<()> {
        let c = &AFC_EXTRUDER_AFCEXTRUDER_TOOL_START_CALLBACK;
        let tc_lane = self.tc_lane.borrow().clone();
        if let (Some(_), Some(tc_lane), true) = (&self.tc_unit_name, &tc_lane, self.is_standalone()) {
            if state != self.tool_start_state.get() {
                tc_lane.load_state_raw.set(state);
                tc_lane.prep_state.set(state);
                // Check to verify that toolhead is not actively printing to trigger the auto
                // load/unload logic.
                let actively_printing = self.afc.function.is_printing(false)? && self.on_shuttle();
                if self.printer.state_message() == "Printer is ready" {
                    if tc_lane.afc_prep_done.get() && !actively_printing {
                        if state {
                            if !self.load_active.get() {
                                self.load_unload_sequence(self.tool_stn.get())?;
                            }
                        } else {
                            tc_lane.set_tool_unloaded(false)?;
                            tc_lane.set_unloaded()?;
                        }
                    } else if tc_lane.afc_prep_done.get() && actively_printing {
                        self.afc.logger.info(c, "Cannot trigger auto load/unload when toolhead is actively printing");
                    }
                    if tc_lane.afc_prep_done.get() {
                        self.afc.save_vars()?;
                    }
                }
            } else {
                self.afc.logger.info(c, "Not loading State matches tool_start_state");
            }
        }
        self.tool_start_state.set(state);
        Ok(())
    }

    /// note_tool_start_callback: the virtual sensor's state, then the callback
    fn note_tool_start_callback(&self, state: bool) -> KResult<()> {
        // (python passes (state, force) as (eventtime, is_filament_present):
        // the helper only ever sees "no filament")
        if let Some(vs) = self.virtual_tool_start.borrow().clone() {
            vs.note_filament_present(false);
        }
        self.tool_start_callback(state)
    }

    /// _set_standalone_lane_states: the tool's lane loaded or not with it
    fn set_standalone_lane_states(&self, enabled: bool) -> KResult<()> {
        let Some(lane) = self.tc_lane.borrow().clone() else { return Ok(()) };
        if enabled {
            lane.set_loaded()?;
            lane.set_tool_loaded(false)?;
            if !self.on_shuttle() {
                lane.unit_obj()?.lane_tool_loaded_idle(&lane)?;
            }
        } else {
            lane.set_tool_unloaded(false)?;
            if self.tool_start.as_deref() == Some("virtual") {
                lane.set_unloaded()?;
            }
        }
        Ok(())
    }

    /// _apply_virtual_tool_start_state: the virtual sensor's switch, state
    /// and tool_start_state as one
    fn apply_virtual_tool_start_state(&self, enabled: bool) {
        if self.tool_start.as_deref() != Some("virtual") {
            return;
        }
        if let Some(vs) = self.virtual_tool_start.borrow().as_ref() {
            vs.sensor_enabled.set(enabled);
            vs.filament_present.set(enabled);
        }
        self.tool_start_state.set(enabled);
    }

    /// the virtual sensor's gui switch (SET_FILAMENT_SENSOR ENABLE=)
    fn virtual_tool_start_toggle(&self, enabled: bool) -> KResult<()> {
        self.note_tool_start_callback(enabled)?;
        self.apply_virtual_tool_start_state(enabled);
        self.afc.logger.info(
            &AFC_EXTRUDER_AFCEXTRUDER__VIRTUAL_TOOL_START_TOGGLE,
            &format!("Virtual toolhead sensor for {} {}", self.name, if enabled { "enabled." } else { "disabled" }),
        );
        if self.afc.prep_done.get() {
            self.afc.save_vars()?;
        }
        Ok(())
    }

    /// restore_virtual_tool_start: the saved virtual sensor state (PREP)
    pub fn restore_virtual_tool_start(&self, enabled: bool) -> KResult<()> {
        if enabled == self.tool_start_state.get() {
            return Ok(());
        }
        let Some(tc) = self.tc_lane.borrow().clone() else { return Ok(()) };
        self.apply_virtual_tool_start_state(enabled);
        self.set_standalone_lane_states(enabled)?;
        tc.load_state_raw.set(enabled);
        tc.prep_state.set(enabled);
        self.afc.logger.info(
            &AFC_EXTRUDER_AFCEXTRUDER_RESTORE_VIRTUAL_TOOL_START,
            &format!("Virtual toolhead sensor for {} restored as {}", self.name, if enabled { "enabled, filament loaded" } else { "disabled" }),
        );
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
            Some(e) => {
                *self.printer_extruder.borrow_mut() = Some(e.clone());
                *self.toolhead_extruder.borrow_mut() = Some(e);
            }
            None => return Err(KError::config(format!("[{}] not found in config file for {}", self.th_extruder_name, self.fullname))),
        }
        if let Some(tc) = &self.tc_unit_name {
            let tc_str = format!("AFC_Toolchanger {}", tc);
            let unit = self.printer.lookup_object_opt(&tc_str).and_then(kaliru::printer::downcast_rc::<super::unit::AfcUnit>);
            let Some(unit) = unit else {
                return Err(KError::config(format!("[{}] not found in config file for {}", tc_str, self.fullname)));
            };
            if let Some(lane) = self.tc_lane.borrow().as_ref() {
                lane.set_unit_obj(&unit);
            }
            *self.tc_unit_obj.borrow_mut() = Some(unit);
        }
        // Looking up led object if user supplied variable
        if let Some(leds) = &self.toolhead_leds {
            let Some(obj) = self.printer.lookup_object_opt(leds) else {
                return Err(KError::config(format!("{} not found in config file for led_name variable in {} config section", leds, self.fullname)));
            };
            self.toolhead_led_found.set(true);
            let helper = match kaliru::printer::downcast_rc::<super::led::AfcLed>(obj.clone()) {
                Some(l) => l.led_helper(),
                None => kaliru::extras::led::led_helper_of(&obj),
            };
            if let Some(h) = helper {
                self.status_led_count.set(h.led_count());
                *self.toolhead_led.borrow_mut() = Some(h);
            }
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
        if sensor == "tool_start" {
            if let Some(vs) = self.virtual_tool_start.borrow().as_ref() {
                return Ok(vs.sensor_enabled.get());
            }
        }
        let fila = if sensor == "tool_start" { self.fila_tool_start.borrow().clone() } else { self.fila_tool_end.borrow().clone() };
        fila.map(|f| f.runout_helper.sensor_enabled.get()).ok_or_else(|| no_attr("NoneType", "runout_helper"))
    }

    fn tc_lane(&self) -> KResult<Rc<AfcLane>> {
        self.tc_lane.borrow().clone().ok_or_else(|| no_attr("NoneType", "status"))
    }

    /// load_unload_sequence: a standalone tool's load to (or unload from)
    /// the nozzle: heat, then move the extruder, without blocking
    pub fn load_unload_sequence(&self, distance: f64) -> KResult<()> {
        let c = &AFC_EXTRUDER_AFCEXTRUDER_LOAD_UNLOAD_SEQUENCE;
        let info_str = if distance > 0.0 { "Loading" } else { "Unloading" };
        let loaded = distance > 0.0;
        let tc_lane = self.tc_lane()?;
        if !self.standalone_auto_load_unload {
            tc_lane.set_status(if loaded { LaneState::Tooled } else { LaneState::None });
            tc_lane.need_purge.set(false);
            self.set_standalone_lane_states(loaded)?;
            self.apply_virtual_tool_start_state(loaded);
            self.afc.logger.info(c, &format!("{} {} skipped: standalone_auto_load_unload is disabled, hotend heating and filament move were not performed", info_str, self.name));
            return Ok(());
        }
        self.afc.logger.info(c, &format!("{} {}", info_str, self.name));
        self.load_active.set(true);
        self.current_move_distance.set(distance);
        if distance > 0.0 {
            tc_lane.unit_obj()?.lane_loading(&tc_lane)?;
            tc_lane.set_status(LaneState::ToolLoading);
        } else {
            tc_lane.set_status(LaneState::ToolUnloading);
        }
        *self.captured_toolhead_temp.borrow_mut() = self.afc.capture_toolhead_temp(Some(self), true)?;
        self.afc.check_extruder_temp(&tc_lane, true)?;
        self.afc.reactor.update_timer(&self.temp_check_timer, self.afc.reactor.monotonic() + 1.0);
        Ok(())
    }

    /// move_extruder: the toolhead extruder by distance on its own trapq;
    /// extruder_move_cb puts it back once the move is done
    fn move_extruder(&self, distance: f64) -> KResult<()> {
        if distance == 0.0 {
            return Ok(());
        }
        self.load_active.set(true);
        let toolhead = self.afc.toolhead()?;
        let pe = self.printer_extruder.borrow().clone().ok_or_else(|| no_attr("NoneType", "extruder_stepper"))?;
        let es = pe.extruder_stepper.clone().ok_or_else(|| no_attr("NoneType", "stepper"))?;
        let stepper = &es.stepper;
        let motion = self.tc_motion.borrow();
        let m = motion.as_ref().ok_or_else(|| no_attr("AFCExtruder", "stepper_kinematics"))?;
        *m.prev_sk.borrow_mut() = stepper.set_stepper_kinematics(Some(m.stepper_kinematics.clone()));
        *m.prev_trapq.borrow_mut() = stepper.set_trapq(Some(m.trapq.clone()));
        stepper.set_position(&[0.0, 0.0, 0.0]);
        let (axis_r, accel_t, cruise_t, cruise_v) = calc_move_time(distance, self.tool_load_speed.get(), 5.0);
        let mut print_time = toolhead.get_last_move_time()?;
        m.trapq.borrow_mut().append(print_time, accel_t, cruise_t, accel_t, 0.0, 0.0, 0.0, axis_r, 0.0, 0.0, 0.0, cruise_v, 5.0);
        print_time += accel_t + cruise_t + accel_t;
        m.motion_queuing.note_mcu_movequeue_activity(print_time, true);
        let dwell_time = accel_t + cruise_t + accel_t;
        self.afc.reactor.update_timer(&self.extruder_move_timer, self.afc.reactor.monotonic() + dwell_time.ceil());
        Ok(())
    }

    /// extruder_move_cb: the extruder's own kinematics back, the motor off,
    /// the temperature restored, the lane's new state
    fn extruder_move_cb(&self, _eventtime: f64) -> KResult<f64> {
        let c = &AFC_EXTRUDER_AFCEXTRUDER_EXTRUDER_MOVE_CB;
        let toolhead = self.afc.toolhead()?;
        let pe = self.printer_extruder.borrow().clone().ok_or_else(|| no_attr("NoneType", "extruder_stepper"))?;
        let es = pe.extruder_stepper.clone().ok_or_else(|| no_attr("NoneType", "stepper"))?;
        toolhead.flush_step_generation()?;
        {
            let motion = self.tc_motion.borrow();
            let m = motion.as_ref().ok_or_else(|| no_attr("AFCExtruder", "trapq"))?;
            es.stepper.set_trapq(m.prev_trapq.borrow_mut().take());
            es.stepper.set_stepper_kinematics(m.prev_sk.borrow_mut().take());
            m.motion_queuing.wipe_trapq(&m.trapq);
        }
        self.afc.function.do_enable(false, &self.th_extruder_name)?;
        self.load_active.set(false);
        let captured = self.captured_toolhead_temp.borrow_mut().take();
        self.afc.restore_toolhead_temp(captured.as_ref(), true)?;
        let tc_lane = self.tc_lane()?;
        let info_str = if self.current_move_distance.get() > 0.0 {
            tc_lane.set_status(LaneState::Tooled);
            tc_lane.need_purge.set(self.enable_standalone_purge);
            "loading"
        } else {
            tc_lane.set_status(LaneState::None);
            tc_lane.need_purge.set(false);
            "unloading"
        };
        self.afc.logger.info(c, &format!("{} {} done", self.name, info_str));
        self.current_move_distance.set(0.0);
        self.afc.save_vars()?;
        Ok(NEVER)
    }

    /// temp_check_cb: once the extruder is up to temperature, the move
    fn temp_check_cb(&self, eventtime: f64) -> KResult<f64> {
        let c = &AFC_EXTRUDER_AFCEXTRUDER_TEMP_CHECK_CB;
        let heater = self.get_heater()?;
        let (current_temp, target_temp) = heater.get_temp(eventtime);
        let tol = self.afc.temp_wait_tolerance;
        if current_temp >= target_temp - tol && current_temp <= target_temp + tol {
            let tc_lane = self.tc_lane()?;
            if self.tool_start_state.get() {
                let info_str = if self.current_move_distance.get() > 0.0 { "loading to" } else { "unloading from" };
                self.afc.logger.info(c, &format!("{} temp within range, {} nozzle", self.th_extruder_name, info_str));
                self.move_extruder(self.current_move_distance.get())?;
                let loaded = self.current_move_distance.get() > 0.0;
                self.set_standalone_lane_states(loaded)?;
                self.apply_virtual_tool_start_state(loaded);
                if self.tool_start.as_deref() == Some("virtual") {
                    tc_lane.load_state_raw.set(loaded);
                    tc_lane.prep_state.set(loaded);
                }
            } else {
                self.load_active.set(false);
                tc_lane.set_unloaded()?;
                self.afc.save_vars()?;
                self.afc.logger.error(c, &format!("Filament is no longer detected at tool start sensor for {}.\nNot loading filament to nozzle.", self.name));
            }
            return Ok(NEVER);
        }
        self.afc.logger.debug(c, &format!("{}: waiting for temp: {}", self.th_extruder_name, py_float_repr(current_temp)));
        Ok(self.afc.reactor.monotonic() + 1.0)
    }

    fn update_tool_stn(&self, length: f64) {
        let c = &AFC_EXTRUDER_AFCEXTRUDER__UPDATE_TOOL_STN;
        if length > 0.0 {
            let msg = format!("tool_stn updated old: {}, new: {}{}", self.tool_stn.str(), py_float_repr(length), self.common_save_msg);
            self.tool_stn.set(length);
            self.afc.logger.info(c, &msg);
        } else {
            self.afc.logger.error(c, "tool_stn length should be greater than zero");
        }
    }

    fn update_tool_stn_unload(&self, length: f64) {
        let c = &AFC_EXTRUDER_AFCEXTRUDER__UPDATE_TOOL_STN_UNLOAD;
        if length >= 0.0 {
            let msg = format!("tool_stn_unload updated old: {}, new: {}{}", self.tool_stn_unload.str(), py_float_repr(length), self.common_save_msg);
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
                self.tool_sensor_after_extruder.str(),
                py_float_repr(length),
                self.common_save_msg
            );
            self.tool_sensor_after_extruder.set(length);
            self.afc.logger.info(c, &msg);
        } else {
            self.afc.logger.error(c, "tool_sensor_after_extruder length should be greater than zero");
        }
    }

    /// set_status_led: the status_led_idx LEDs of the toolhead in a color
    pub fn set_status_led(&self, color: &str) -> KResult<()> {
        if !self.toolhead_led_found.get() {
            return Ok(());
        }
        let Some(index) = &self.toolhead_status_index else { return Ok(()) };
        let Some(helper) = self.toolhead_led.borrow().clone() else { return Ok(()) };
        let color: Vec<f64> = color.split(',').map(|v| super::utils::py_float_of(&PyVal::Str(v.to_string()))).collect::<KResult<_>>()?;
        for idx in index {
            helper.set_color(Some(*idx), &color)?;
        }
        helper.check_transmit(None)
    }

    /// set_print_leds: the toolhead's print LEDs (nozzle_led_idx, else all
    /// but status_led_idx) on or off; (success, error)
    pub fn set_print_leds(&self, state: i64, quiet: bool) -> KResult<(bool, String)> {
        let c = &AFC_EXTRUDER_AFCEXTRUDER_SET_PRINT_LEDS;
        if !self.toolhead_led_found.get() {
            let error_string = format!("led_name variable not set in [{}] config section", self.fullname);
            if !quiet {
                self.afc.logger.error(c, &error_string);
            }
            return Ok((false, error_string));
        }
        let Some(helper) = self.toolhead_led.borrow().clone() else {
            let error_string = "Cannot set print leds as status or check_transmit function are None".to_string();
            self.afc.logger.error(c, &error_string);
            return Ok((false, error_string));
        };
        let color = vec![state as f64; 4];
        for idx in 1..self.status_led_count.get() + 1 {
            if self.toolhead_status_index.as_ref().is_some_and(|s| !s.is_empty() && s.contains(&idx)) {
                continue;
            }
            match &self.toolhead_nozzle_index {
                Some(n) if !n.is_empty() => {
                    if n.contains(&idx) {
                        helper.set_color(Some(idx), &color)?;
                    }
                }
                _ => helper.set_color(Some(idx), &color)?,
            }
        }
        helper.check_transmit(None)?;
        Ok((true, String::new()))
    }

    fn cmd_afc_set_extruder_led(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let state = gcmd.get_int("TURN_ON", Some(1), Lim::new())?;
        let (success, error_string) = self.set_print_leds(state, false)?;
        if !success {
            return Err(gcmd.error(error_string));
        }
        Ok(())
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
        f.config_rewrite(&self.fullname, "tool_stn", &self.tool_stn.pv(), "")?;
        f.config_rewrite(&self.fullname, "tool_stn_unload", &self.tool_stn_unload.pv(), "")?;
        f.config_rewrite(&self.fullname, "tool_sensor_after_extruder", &self.tool_sensor_after_extruder.pv(), "")
    }

    pub fn get_status(&self) -> KResult<PyVal> {
        let opt = |v: &Option<String>| v.clone().map(PyVal::Str).unwrap_or(PyVal::None);
        Ok(PyVal::Dict(kaliru::pydict! {
            "tool_stn" => self.tool_stn.pv(),
            "tool_stn_unload" => self.tool_stn_unload.pv(),
            "tool_sensor_after_extruder" => self.tool_sensor_after_extruder.pv(),
            "tool_unload_speed" => self.tool_unload_speed.pv(),
            "tool_load_speed" => self.tool_load_speed.pv(),
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
