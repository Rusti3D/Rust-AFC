// HTLF: lanes (AFC_lane) on the unit's drive stepper, picked by a lobed cam
// the selector stepper turns from its home switch (port of AFC_HTLF.py)
//
// Copyright (C) 2024-2026 Armored Turtle
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::boxturtle::Family;
use super::codes::*;
use super::lane::{AfcLane, LaneState, MoveDirection};
use super::unit::{AfcUnit, UnitKind};
use super::utils::{self, add_filament_switch, SwitchOptions};
use super::vivid::{self, Mode, Selector};
use kaliru::configfile::{ConfigWrapper, Dflt, Lim};
use kaliru::error::{KError, KResult};
use kaliru::extras::buttons::PrinterButtons;
use kaliru::extras::query_endstops::QueryEndstops;
use kaliru::gcode::GCodeCommand;
use kaliru::pins::PinsObject;
use kaliru::pyval::PyVal;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

const VALID_CAM_ANGLES: [i64; 3] = [30, 45, 60];
const CMD_AFC_HOME_UNIT_HELP: &str = "Command to move lane selector back to home position for specified in selector style units that utilizes a home sensor.";

/// the HTLF's cam: its geometry, home switch and where it is
pub struct Htlf {
    mm_move_per_rotation: i64,
    cam_angle: i64,
    max_angle_movement: i64,
    selector_movement_speed: PyVal,
    selector_movement_accel: PyVal,
    home_endstop_name: String,
    current_selected_lane: RefCell<Option<Rc<AfcLane>>>,
    home_state: Rc<Cell<bool>>,
    prep_homed: Cell<bool>,
    failed_to_home: Cell<bool>,
    homed_distance: Cell<f64>,
}

pub fn is_htlf(unit: &AfcUnit) -> bool {
    matches!(unit.kind, UnitKind::BoxTurtle(Family::Htlf))
}

/// the unit's selector, with its HTLF cam
fn parts(unit: &AfcUnit) -> KResult<Rc<Selector>> {
    let sel = vivid::selector(unit)?;
    if !matches!(sel.mode, Mode::Htlf(_)) {
        return Err(super::no_attr("AFC_vivid", "home_endstop_name"));
    }
    Ok(sel)
}

fn htlf(sel: &Selector) -> &Htlf {
    match &sel.mode {
        Mode::Htlf(h) => h,
        Mode::Vivid { .. } => unreachable!("parts() checked the mode"),
    }
}

/// AFC_HTLF.__init__ after the BoxTurtle part
pub fn init(unit: &Rc<AfcUnit>, config: &ConfigWrapper) -> KResult<()> {
    let printer = &unit.printer;
    let drive_stepper = config.get("drive_stepper", Dflt::Required)?;
    let selector_stepper = config.get("selector_stepper", Dflt::Required)?;
    let mm_move_per_rotation = config.getint("mm_move_per_rotation", 32, Lim::new())?;
    let cam_angle = config.getint("cam_angle", Dflt::Required, Lim::new())?;
    let home_pin = config.get("home_pin", Dflt::Required)?;
    let max_angle_movement = config.getint("MAX_ANGLE_MOVEMENT", 215, Lim::new())?;
    let enable_sensors_in_gui = config.getboolean("enable_sensors_in_gui", unit.afc.enable_sensors_in_gui)?;
    let selector_movement_speed = super::getfloat_pv(config, "selector_movement_speed", PyVal::Int(50))?;
    let selector_movement_accel = super::getfloat_pv(config, "selector_movement_accel", PyVal::Int(50))?;
    if !VALID_CAM_ANGLES.contains(&cam_angle) {
        return Err(KError::config(format!("{} is not a valid cam angle, please choose from the following [30, 45, 60]", cam_angle)));
    }
    let home_state = Rc::new(Cell::new(false));
    let buttons = printer.lookup_or_load::<PrinterButtons>(config, "buttons")?;
    let hs = home_state.clone();
    buttons.register_buttons(
        &[home_pin.as_str()],
        Rc::new(move |_et, state| {
            hs.set(state != 0);
            Ok(())
        }),
    )?;
    let query_endstops = printer.lookup_or_load::<QueryEndstops>(config, "query_endstops")?;
    add_filament_switch(&format!("{}_home_pin", unit.name), &home_pin, printer, &SwitchOptions { show_sensor: enable_sensors_in_gui, ..Default::default() })?;
    let home_endstop = {
        let ppins = printer.lookup::<PinsObject>("pins")?;
        let mut pp = ppins.0.borrow_mut();
        pp.allow_multi_use_pin(home_pin.trim_matches(|c| c == '!' || c == '^')).map_err(|e| KError::Pins(e.0))?;
        pp.parse_pin(&home_pin, true, true).map_err(|e| KError::Pins(e.0))?;
        drop(pp);
        let obj = kaliru::pins::setup_pin(printer, "endstop", &home_pin).map_err(|e| KError::Other(e.message().to_string()))?;
        kaliru::mcu::any_to_endstop(obj).ok_or_else(|| KError::Other(format!("{} is not an endstop", home_pin)))?
    };
    let home_endstop_name = format!("{}_home", unit.name);
    query_endstops.register_endstop(home_endstop.clone(), &home_endstop_name);
    let (drive_stepper_obj, selector_stepper_obj) = vivid::lookup_objects(unit, config, &drive_stepper, &selector_stepper)?;
    // Adding home endstop to selector
    super::stepper::add_named_endstop(&selector_stepper_obj, &home_endstop, &home_endstop_name)?;
    *unit.selector.borrow_mut() = Some(Rc::new(Selector {
        drive_stepper,
        selector_stepper,
        drive_stepper_obj,
        selector_stepper_obj,
        mode: Mode::Htlf(Htlf {
            mm_move_per_rotation,
            cam_angle,
            max_angle_movement,
            selector_movement_speed,
            selector_movement_accel,
            home_endstop_name,
            current_selected_lane: RefCell::new(None),
            home_state,
            prep_homed: Cell::new(false),
            failed_to_home: Cell::new(false),
            homed_distance: Cell::new(0.0),
        }),
    }));
    let u = unit.clone();
    unit.afc.function.register_commands(
        unit.afc.show_macros,
        "AFC_HOME_UNIT",
        Rc::new(move |g| cmd_afc_home_unit(&u, g)),
        CMD_AFC_HOME_UNIT_HELP,
        Some(&[("UNIT", "string", "HTLF_1")]),
    )
}

pub fn logos() -> (String, String) {
    ("<span class=success--text>HTLF Ready\n</span>".into(), "<span class=error--text>HTLF Not Ready</span>\n".into())
}

/// system_Test: home the cam first (once) and after the BoxTurtle test
pub fn system_test(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, delay: f64, assign_tcmd: bool, enable_movement: bool) -> KResult<bool> {
    let sel = parts(unit)?;
    let h = htlf(&sel);
    cur_lane.prep_state.set(cur_lane.load_state());
    if !h.prep_homed.get() {
        return_to_home(unit, true, false)?;
    }
    let status = super::boxturtle::unit_system_test(unit, cur_lane, delay, assign_tcmd, enable_movement)?;
    return_to_home(unit, false, true)?;
    Ok(h.prep_homed.get() && status)
}

fn cmd_afc_home_unit(unit: &Rc<AfcUnit>, _gcmd: &GCodeCommand) -> KResult<()> {
    return_to_home(unit, false, true).map(|_| ())
}

/// _move_selector_home: towards home, homing to the home switch when homing
/// is enabled
fn move_selector_home(unit: &AfcUnit, sel: &Selector, distance: f64) -> KResult<()> {
    let h = htlf(sel);
    let distance = distance * MoveDirection::Neg.value();
    if unit.afc.homing_enabled {
        let (homed, homed_distance) =
            super::stepper::do_homing_move_pub(&sel.selector_stepper_obj, distance, &h.selector_movement_speed, &h.selector_movement_accel, &h.home_endstop_name, true, true, false)?;
        h.homed_distance.set(homed_distance);
        unit.afc.logger.debug(
            &AFC_HTLF_AFC_HTLF__MOVE_SELECTOR_HOME,
            &format!("HTLF: Homing done, success:{}, distance:{}", super::py_bool(homed), kaliru::pyfmt::py_float_repr(homed_distance)),
        );
        Ok(())
    } else {
        sel.selector_stepper_obj.move_lane(distance, utils::py_float_of(&h.selector_movement_speed)?, utils::py_float_of(&h.selector_movement_accel)?, false)
    }
}

/// return_to_home: the cam back to its home switch; whether it got there
pub fn return_to_home(unit: &AfcUnit, prep: bool, disable_selector: bool) -> KResult<bool> {
    let sel = parts(unit)?;
    let h = htlf(&sel);
    let mut total_moved = 0.0;
    let mut move_distance = 200.0;
    let current = h.current_selected_lane.borrow().clone();
    if let Some(cur) = current.filter(|_| !h.home_state.get() && !prep) {
        if !unit.afc.homing_enabled {
            move_distance = calculate_lobe_movement(unit, h, cur.index);
        }
        move_selector_home(unit, &sel, move_distance)?;
    }
    while !h.home_state.get() && !h.failed_to_home.get() {
        if !unit.afc.homing_enabled {
            move_distance = 1.0;
        }
        move_selector_home(unit, &sel, move_distance)?;
        total_moved += if unit.afc.homing_enabled { h.homed_distance.get() } else { move_distance };
        if total_moved > (h.mm_move_per_rotation as f64 / 360.0) * (h.max_angle_movement + h.cam_angle) as f64 {
            h.failed_to_home.set(true);
            unit.afc.error.afc_error(&AFC_HTLF_AFC_HTLF_RETURN_TO_HOME, &format!("Failed to home {}", unit.name), false, None)?;
            return Ok(false);
        }
    }
    h.prep_homed.set(true);
    if disable_selector {
        sel.selector_stepper_obj.do_enable(false)?;
    }
    *h.current_selected_lane.borrow_mut() = None;
    Ok(true)
}

/// calculate_lobe_movement: the cam's travel in mm to engage a lane
fn calculate_lobe_movement(unit: &AfcUnit, h: &Htlf, lane_index: i64) -> f64 {
    let angle_movement = h.max_angle_movement - ((lane_index - 1) * h.cam_angle);
    unit.afc.logger.debug(&AFC_HTLF_AFC_HTLF_CALCULATE_LOBE_MOVEMENT, &format!("HTLF: Lobe Movement angle : {}", angle_movement));
    (h.mm_move_per_rotation as f64 / 360.0) * angle_movement as f64
}

/// select_lane: home the cam, then turn it to the lane's lobe
pub fn select_lane(unit: &AfcUnit, lane: &Rc<AfcLane>, disable_selector: bool) -> KResult<(bool, f64)> {
    let c = &AFC_HTLF_AFC_HTLF_SELECT_LANE;
    if lane.fullname.to_lowercase().contains("stepper") {
        return Ok((false, 0.0));
    }
    let sel = parts(unit)?;
    let h = htlf(&sel);
    let r = (|| -> KResult<Option<(bool, f64)>> {
        let current = h.current_selected_lane.borrow().clone();
        if current.is_none_or(|c| !Rc::ptr_eq(&c, lane)) {
            unit.afc.logger.debug(c, &format!("HTLF: {} Homing to endstop.", unit.name));
            if return_to_home(unit, false, false)? {
                sel.selector_stepper_obj.move_lane(calculate_lobe_movement(unit, h, lane.index), utils::py_float_of(&h.selector_movement_speed)?, utils::py_float_of(&h.selector_movement_accel)?, false)?;
                // Applying selector_cal_dis move if specified in users config
                vivid::selector_cal_dis_adjust(unit, lane)?;
                unit.afc.logger.debug(c, &format!("HTLF: Selecting {}", lane.name));
                *h.current_selected_lane.borrow_mut() = Some(lane.clone());
                return Ok(Some((true, h.homed_distance.get())));
            }
            unit.afc.logger.error(c, &format!("HTLF: failed to home when selecting {}", lane.name));
            return Ok(Some((false, 0.0)));
        }
        Ok(None)
    })();
    if disable_selector {
        sel.selector_stepper_obj.do_enable(false)?;
    }
    Ok(r?.unwrap_or((true, 0.0)))
}

/// check_runout: the loaded lane while printing, unless ejecting or calibrating
pub fn check_runout(unit: &AfcUnit, lane: &AfcLane) -> KResult<bool> {
    Ok(unit.afc.function.get_current_lane().as_deref() == Some(lane.name.as_str())
        && unit.afc.function.is_printing(false)?
        && lane.status() != LaneState::Ejecting
        && lane.status() != LaneState::Calibrating)
}
