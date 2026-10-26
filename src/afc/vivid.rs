// ViViD and Claymore: units whose lanes (AFC_lane, no stepper of their own)
// share the unit's drive stepper and are picked by a cam on a selector
// stepper (port of AFC_vivid.py, AFC_Claymore.py, and AFC_unit.py's
// drive/selector parts)
//
// Copyright (C) 2024-2026 Armored Turtle
// Copyright (C) 2026 AFCProject
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::boxturtle::Family;
use super::codes::*;
use super::lane::{AfcLane, AssistActive, Endstop, LaneState, MoveDirection, SpeedMode, HOMING_PREP};
use super::unit::{AfcUnit, UnitKind};
use super::utils;
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::gcode::GCodeCommand;
use kaliru::pyval::PyVal;
use std::rc::Rc;

const CALIBRATION_DISTANCE: f64 = 5000.0;
const LANE_OVERSHOOT: f64 = 200.0;

const CMD_AFC_SELECT_LANE_HELP: &str = "Command to home to lane selector for specified lane in selector style units.";
const CMD_AFC_UNSELECT_LANE_HELP: &str =
    "Command to move selector to free filament from currently selected lane. Selector only moves if any lanes selector is currently triggered unless force is passed in.";

/// A selector unit's drive and selector steppers
pub struct Selector {
    pub drive_stepper: String,
    pub selector_stepper: String,
    pub drive_stepper_obj: Rc<AfcLane>,
    pub selector_stepper_obj: Rc<AfcLane>,
    pub mode: Mode,
}

/// how the unit's cam finds a lane
pub enum Mode {
    /// ViViD / Claymore: homing to the lane's selector switch
    Vivid { selector_homing_speed: PyVal, selector_homing_accel: PyVal, max_selector_movement: PyVal },
    /// HTLF: lobe angles from a home switch
    Htlf(super::htlf::Htlf),
}

impl Selector {
    fn vivid(&self) -> KResult<(&PyVal, &PyVal, &PyVal)> {
        match &self.mode {
            Mode::Vivid { selector_homing_speed, selector_homing_accel, max_selector_movement } => Ok((selector_homing_speed, selector_homing_accel, max_selector_movement)),
            Mode::Htlf(_) => Err(super::no_attr("AFC_HTLF", "selector_homing_speed")),
        }
    }
}

/// _lookup_objects: the [AFC_stepper] sections of the unit's drive and
/// selector steppers, loaded; a config error names each one missing
pub fn lookup_objects(unit: &AfcUnit, config: &ConfigWrapper, drive_stepper: &str, selector_stepper: &str) -> KResult<(Rc<AfcLane>, Rc<AfcLane>)> {
    // (full_name is a list: the message shows it as python prints one)
    let full_name = format!("[{}]", unit.full_name.iter().map(|n| super::py_repr(n)).collect::<Vec<_>>().join(", "));
    let mut error_string = String::new();
    let mut lookup = |name: &str, variable: &str| -> KResult<Option<Rc<AfcLane>>> {
        let config_name = format!("AFC_stepper {}", name);
        let mut obj = None;
        if config.fileconfig.borrow().has_section(&config_name) {
            obj = unit.printer.load_object(config, &config_name, false)?.and_then(kaliru::printer::downcast_rc::<AfcLane>);
        }
        if obj.is_none() {
            error_string += &format!(
                "Error: [{}] config not found for {} in {} config section. Please make sure [{}] section exists in your config.\n",
                config_name, variable, full_name, config_name
            );
        }
        Ok(obj)
    };
    let drive = lookup(drive_stepper, "drive_stepper")?;
    let sel = lookup(selector_stepper, "selector_stepper")?;
    match (drive, sel) {
        (Some(d), Some(s)) => Ok((d, s)),
        _ => Err(KError::config(error_string)),
    }
}

/// The unit is a ViViD or a Claymore
pub fn is_selector_unit(unit: &AfcUnit) -> bool {
    matches!(unit.kind, UnitKind::BoxTurtle(Family::Vivid | Family::Claymore))
}

fn is_claymore(unit: &AfcUnit) -> bool {
    matches!(unit.kind, UnitKind::BoxTurtle(Family::Claymore))
}

pub fn selector(unit: &AfcUnit) -> KResult<Rc<Selector>> {
    unit.selector.borrow().clone().ok_or_else(|| super::no_attr("afcUnit", "selector_stepper_obj"))
}

/// AFC_vivid.__init__ after the BoxTurtle part: the drive and selector
/// steppers (_lookup_objects loads their sections) and AFC_UNSELECT_LANE
pub fn init(unit: &Rc<AfcUnit>, config: &ConfigWrapper) -> KResult<()> {
    let drive_stepper = config.get("drive_stepper", kaliru::configfile::Dflt::Required)?;
    let selector_stepper = config.get("selector_stepper", kaliru::configfile::Dflt::Required)?;
    let _enable_sensors_in_gui = config.getboolean("enable_sensors_in_gui", unit.afc.enable_sensors_in_gui)?;
    let selector_homing_speed = super::getfloat_pv(config, "selector_homing_speed", PyVal::Int(150))?;
    let selector_homing_accel = super::getfloat_pv(config, "selector_homing_accel", PyVal::Int(150))?;
    let max_selector_movement = super::getfloat_pv(config, "max_selector_movement", PyVal::Int(800))?;
    let (drive_stepper_obj, selector_stepper_obj) = lookup_objects(unit, config, &drive_stepper, &selector_stepper)?;
    *unit.selector.borrow_mut() = Some(Rc::new(Selector {
        drive_stepper,
        selector_stepper,
        drive_stepper_obj,
        selector_stepper_obj,
        mode: Mode::Vivid { selector_homing_speed, selector_homing_accel, max_selector_movement },
    }));
    let u = unit.clone();
    unit.afc.function.register_mux_command(
        unit.afc.show_macros,
        "AFC_UNSELECT_LANE",
        "UNIT",
        &unit.name,
        Rc::new(move |g| cmd_afc_unselect_lane(&u, g)),
        CMD_AFC_UNSELECT_LANE_HELP,
        Some(&[("UNIT", "string", ""), ("FORCE", "int", "0"), ("MOVE_DIST", "int", "100")]),
    )
}

/// AFC_SELECT_LANE for an AFC_lane of a unit with a selector
pub fn register_select_lane(lane: &Rc<AfcLane>) -> KResult<()> {
    let Some(unit) = lane.unit_obj_opt() else { return Ok(()) };
    if unit.selector.borrow().is_none() {
        return Ok(());
    }
    let u = unit.clone();
    lane.afc.function.register_mux_command(
        lane.afc.show_macros,
        "AFC_SELECT_LANE",
        "LANE",
        &lane.name,
        Rc::new(move |g| cmd_afc_select_lane(&u, g)),
        CMD_AFC_SELECT_LANE_HELP,
        Some(&[("LANE", "string", "lane1")]),
    )
}

/// handle_connect: the unit's logos
pub fn logos(unit: &AfcUnit) -> (String, String) {
    if is_claymore(unit) {
        let t = unit.type_name();
        (format!("<span class=success--text>{} Ready\n</span>", t), format!("<span class=error--text>{} Not Ready</span>\n", t))
    } else {
        ("<span class=success--text>ViViD Ready\n</span>".into(), "<span class=error--text>ViViD Not Ready</span>\n".into())
    }
}

/// _move_lane: home the lane to its load sensor and back off; whether it
/// homed (Claymore: the load sensor's state)
pub fn move_lane(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<bool> {
    if is_claymore(unit) {
        return Ok(lane.load_state());
    }
    let mut loaded = false;
    let mut homed = false;
    if lane.prep_state.get() {
        (homed, _, _) = unit.move_to_load(lane, lane.dist_hub.get(), MoveDirection::Pos, true, SpeedMode::Short)?;
        if homed {
            loaded = true;
            lane.loaded_to_hub.set(true);
            if !lane.tool_loaded.get() {
                if let Some(hub) = lane.hub_obj_opt() {
                    lane.move_to(hub.hub_clear_move_dis() * MoveDirection::Neg.value(), SpeedMode::Short, &Endstop::None, AssistActive::No, false)?;
                }
            }
        }
    }
    // Resetting internal states when prep sensor is not triggered but loaded to hub is still
    // set, or when failed to home to load sensor
    if (!lane.prep_state.get() && lane.loaded_to_hub.get()) || !homed {
        unit.lane_unloaded(lane)?;
        lane.tool_loaded.set(false);
        lane.set_status(LaneState::None);
        lane.loaded_to_hub.set(false);
        *lane.td1_data.borrow_mut() = PyVal::dict();
        if !lane.remember_spool.borrow().unwrap_or(false) {
            lane.afc.spool.clear_values(lane)?;
        }
    }
    Ok(loaded)
}

/// Claymore's system_Test takes the load sensor as the prep sensor
pub fn system_test_prepare(unit: &AfcUnit, lane: &AfcLane) {
    if is_claymore(unit) {
        lane.prep_state.set(lane.load_state());
    }
}

fn lane_selector_state(lane: &AfcLane) -> bool {
    lane.fila_selector.borrow().as_ref().is_some_and(|f| f.runout_helper.filament_present.get())
}

fn selector_enabled(unit: &AfcUnit, sel: &Selector) -> bool {
    let Some(se) = unit.printer.lookup_object_opt("stepper_enable").and_then(kaliru::printer::downcast_rc::<kaliru::extras::stepper_enable::PrinterStepperEnable>) else {
        return false;
    };
    se.lookup_enable(&format!("AFC_stepper {}", sel.selector_stepper)).map(|e| e.is_motor_enabled()).unwrap_or(false)
}

/// select_lane: the cam homed to the lane's selector switch (sel_prep:
/// turning the other way, so new filament finds the PTFE); (homed, distance)
pub fn select_lane(unit: &AfcUnit, lane: &Rc<AfcLane>, sel_prep: bool) -> KResult<(bool, f64)> {
    let c = &AFC_VIVID_AFC_VIVID_SELECT_LANE;
    let logger = &unit.afc.logger;
    let Endstop::Name(endstop) = lane.selector_endstop_name() else { return Ok((false, 0.0)) };
    let sel = selector(unit)?;
    let sel_dir = if sel_prep { MoveDirection::Neg } else { MoveDirection::Pos };
    let selector_state = lane_selector_state(lane);
    let enabled = selector_enabled(unit, &sel);
    if selector_state && enabled {
        logger.debug(c, &format!("{} already selected", lane.name));
        return Ok((true, 0.0));
    }
    if !enabled && selector_state {
        unselect_lane(unit, None)?;
    }
    let t = unit.type_name();
    logger.debug(c, &format!("{}: Selecting {}", t, lane.name));
    let (speed, accel, max_selector_movement) = sel.vivid()?;
    let movepos = utils::py_float_of(max_selector_movement)? * sel_dir.value();
    let (homed, distance) = super::stepper::do_homing_move_pub(&sel.selector_stepper_obj, movepos, speed, accel, &endstop, true, true, false)?;
    // Applying selector_cal_dis move if specified in users config
    selector_cal_dis_adjust(unit, lane)?;
    logger.debug(c, &format!("{}: Homing done, success:{}, distance:{}", t, super::py_bool(homed), kaliru::pyfmt::py_float_repr(distance)));
    Ok((homed, kaliru::pyfmt::py_round(distance, 2)))
}

/// _selector_cal_dis_adjust: the configured fine adjustment after selecting
pub fn selector_cal_dis_adjust(unit: &AfcUnit, lane: &AfcLane) -> KResult<()> {
    if let Some(sel) = unit.selector.borrow().clone() {
        if lane.selector_cal_dis != 0.0 {
            sel.selector_stepper_obj.move_lane(lane.selector_cal_dis, lane.short_moves_speed(), lane.short_moves_accel(), false)?;
        }
    }
    Ok(())
}

/// unselect_lane: the cam turned off the lane to free its filament
/// (ViViD 50 mm, Claymore 5 mm unless given)
pub fn unselect_lane(unit: &AfcUnit, move_distance: Option<f64>) -> KResult<()> {
    let sel = selector(unit)?;
    let d = move_distance.unwrap_or(if is_claymore(unit) { 5.0 } else { 50.0 });
    let (speed, accel, _) = sel.vivid()?;
    sel.selector_stepper_obj.move_lane(d, utils::py_float_of(speed)?, utils::py_float_of(accel)?, false)
}

/// prep_load: select, home the new spool to its load sensor (calibrating
/// dist_hub on a first load), back off it
pub fn prep_load(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<()> {
    let c = &AFC_VIVID_AFC_VIVID_PREP_LOAD;
    let afc = &unit.afc;
    let sel = selector(unit)?;
    unit.lane_loading(lane)?;
    select_lane(unit, lane, true)?;
    let mut num_tries = 0;
    let (mut distance, move_speed) = if !lane.calibrated_lane.get() { (CALIBRATION_DISTANCE, SpeedMode::Short) } else { (lane.dist_hub.get(), SpeedMode::Long) };
    let mut homed = false;
    while num_tries < 2 && lane.prep_state.get() && !lane.raw_load_state() {
        (homed, distance, _) = lane.move_to(distance, move_speed, &lane.load_endstop_name(), AssistActive::No, true)?;
        num_tries += 1;
    }
    if homed {
        lane.loaded_to_hub.set(true);
        if !lane.calibrated_lane.get() {
            lane.calibrated_lane.set(true);
            lane.dist_hub.set(kaliru::pyfmt::py_round(distance, 2) + LANE_OVERSHOOT);
            afc.function.config_rewrite(&lane.fullname, "dist_hub", &PyVal::Float(lane.dist_hub.get()), &format!("{} calibrated, updating dist_hub", lane.name))?;
            afc.function.config_rewrite(&lane.fullname, "calibrated_lane", &PyVal::Bool(true), "")?;
        }
        // Retract so load sensor is not triggered
        if let Some(hub) = lane.hub_obj_opt() {
            lane.move_to(hub.hub_clear_move_dis() * MoveDirection::Neg.value(), SpeedMode::Short, &Endstop::None, AssistActive::No, false)?;
        }
        unit.lane_loaded(lane, false)?;
    } else {
        afc.logger.error(c, &format!("Failed to move {} to load sensor.", lane.name));
    }
    sel.selector_stepper_obj.do_enable(false)?;
    sel.drive_stepper_obj.do_enable(false)?;
    afc.function.select_loaded_lane()
}

/// eject_lane: select, back to the prep sensor, free the filament; false
/// where the BoxTurtle eject applies (other units, Claymore)
pub fn eject_lane(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<bool> {
    if !is_selector_unit(unit) || is_claymore(unit) {
        return Ok(false);
    }
    let sel = selector(unit)?;
    select_lane(unit, lane, false)?;
    let mut move_dis = lane.dist_hub.get();
    if move_dis > 400.0 {
        let hub_clear_move_dis = lane.hub_obj_opt().map_or(0.0, |h| h.hub_clear_move_dis());
        move_dis = lane.dist_hub.get() - (LANE_OVERSHOOT + 100.0) - hub_clear_move_dis - lane.settings().homing_overshoot.unwrap_or(0.0);
    }
    lane.move_to(move_dis * MoveDirection::Neg.value(), SpeedMode::Long, &lane.lookup_endstop_pub(HOMING_PREP), AssistActive::No, true)?;
    unselect_lane(unit, None)?;
    sel.selector_stepper_obj.do_enable(false)?;
    sel.drive_stepper_obj.do_enable(false)?;
    Ok(true)
}

/// calibrate_lane: eject, and calibrate on the next insert; None where the
/// BoxTurtle calibration applies (other units, Claymore)
pub fn calibrate_lane(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<Option<(bool, String, f64)>> {
    if !is_selector_unit(unit) || is_claymore(unit) {
        return Ok(None);
    }
    eject_lane(unit, lane)?;
    lane.loaded_to_hub.set(false);
    lane.set_status(LaneState::None);
    lane.calibrated_lane.set(false);
    lane.unit_obj()?.lane_unloaded(lane)?;
    Ok(Some((true, "calibration_lane".into(), 0.0)))
}

pub fn calibration_lane_message(unit: &AfcUnit) -> String {
    if is_claymore(unit) {
        return String::new();
    }
    let mut msg = String::from("\nThe following lanes ({lanes}) were ejected and calibration flag set to false. ");
    msg += "Please reinsert filament into ViViD to automatically calibrate distance.\n";
    msg
}

fn cmd_afc_select_lane(unit: &Rc<AfcUnit>, gcmd: &GCodeCommand) -> KResult<()> {
    let c = &AFC_UNIT_AFCUNIT_CMD_AFC_SELECT_LANE;
    let lane = gcmd.get("LANE")?;
    let Some(lane_obj) = unit.afc.lane(&lane) else {
        // (python builds the error and drops it)
        let _ = gcmd.error(format!("Invalid lane {}", lane));
        return Ok(());
    };
    let (homed, distance) = unit.select_lane(&lane_obj, false)?;
    if homed {
        unit.afc.logger.info(c, &format!("Successfully homed to {} selector after {}mm", lane_obj.name, kaliru::pyfmt::py_float_repr(distance)));
    } else {
        unit.afc.logger.error(c, &format!("Failed to home to {}", lane_obj.name));
    }
    Ok(())
}

fn cmd_afc_unselect_lane(unit: &Rc<AfcUnit>, gcmd: &GCodeCommand) -> KResult<()> {
    let force = gcmd.get_int("FORCE", Some(0), Lim::new())? != 0;
    let move_dist = gcmd.get_float("MOVE_DIST", Some(100.0), Lim::new())?;
    let any_selected = unit.lanes.borrow().values().any(|l| l.selector_state.get().unwrap_or(false));
    if any_selected || force {
        unselect_lane(unit, Some(move_dist))?;
        unit.afc.logger.info(&AFC_UNIT_AFCUNIT_CMD_AFC_UNSELECT_LANE, &format!("{} selector moved", unit.name));
    }
    Ok(())
}
