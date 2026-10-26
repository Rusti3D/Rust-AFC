// The BoxTurtle unit and its siblings (port of AFC_BoxTurtle.py: afcBoxTurtle;
// AFC_NightOwl.py, AFC_QuattroBox.py)
//
// Copyright (C) 2024-2026 Armored Turtle
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::error::LaneRef;
use super::lane::{AfcLane, AssistActive, LaneState, MoveDirection, MoveWarning, SpeedMode, HOMING_BUFFER};
use super::unit::{AfcUnit, UnitKind, UnitSettings};
use super::utils;
use kaliru::configfile::ConfigWrapper;
use kaliru::error::KResult;
use kaliru::printer::PObj;
use kaliru::pyfmt::py_float_repr;
use kaliru::pyval::PyVal;
use std::rc::Rc;

const MAX_NUM_MOVES: i64 = 40;

/// Which BoxTurtle-family unit (all are afcBoxTurtle with their own logo;
/// EMU also loads, homes to the hub and ejects its own way)
#[derive(Clone, Copy, PartialEq)]
pub enum Family {
    BoxTurtle,
    NightOwl,
    QuattroBox,
    Emu,
}

/// A python number in a message: an int stays "0", a float is "0.0"
#[derive(Clone, Copy)]
struct Num {
    v: f64,
    int: bool,
}

impl Num {
    fn i(v: i64) -> Num {
        Num { v: v as f64, int: true }
    }
    fn f(v: f64) -> Num {
        Num { v, int: false }
    }
    fn add(self, o: Num) -> Num {
        Num { v: self.v + o.v, int: self.int && o.int }
    }
    fn sub(self, o: Num) -> Num {
        Num { v: self.v - o.v, int: self.int && o.int }
    }
    /// round(x, 2)
    fn round2(self) -> Num {
        if self.int {
            self
        } else {
            Num::f(kaliru::pyfmt::py_round(self.v, 2))
        }
    }
    fn str(self) -> String {
        if self.int {
            format!("{}", self.v as i64)
        } else {
            py_float_repr(self.v)
        }
    }
    fn pv(self) -> PyVal {
        if self.int {
            PyVal::Int(self.v as i64)
        } else {
            PyVal::Float(self.v)
        }
    }
}

fn make(config: &ConfigWrapper, family: Family) -> KResult<PObj> {
    let unit = AfcUnit::new(config, |config, _settings: &UnitSettings| {
        let default_type = match family {
            Family::BoxTurtle => "Box_Turtle",
            Family::NightOwl => "Night_Owl",
            Family::QuattroBox => "Quattro_Box",
            Family::Emu => "EMU",
        };
        let type_name = config.get("type", default_type)?;
        Ok((UnitKind::BoxTurtle(family), type_name))
    })?;
    Ok(unit)
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    make(config, Family::BoxTurtle)
}

pub fn load_config_prefix_night_owl(config: &ConfigWrapper) -> KResult<PObj> {
    make(config, Family::NightOwl)
}

pub fn load_config_prefix_quattro_box(config: &ConfigWrapper) -> KResult<PObj> {
    make(config, Family::QuattroBox)
}

pub fn load_config_prefix_emu(config: &ConfigWrapper) -> KResult<PObj> {
    make(config, Family::Emu)
}

/// an EMU's virtual hub: no hub switch, the load switch stands in
fn emu_virtual_hub(unit: &AfcUnit, lane: &AfcLane) -> bool {
    matches!(unit.kind, UnitKind::BoxTurtle(Family::Emu)) && lane.hub_obj_opt().is_some_and(|h| h.is_virtual_pin())
}

/// move_to_hub: to the hub switch (an EMU with a virtual hub: the load switch)
pub fn unit_move_to_hub(unit: &AfcUnit, lane: &Rc<AfcLane>, dist: f64, dir: MoveDirection, use_homing: bool, speed_mode: SpeedMode, assist_active: AssistActive) -> KResult<(bool, f64, MoveWarning)> {
    let endstop = if emu_virtual_hub(unit, lane) { lane.load_es() } else { lane.hub_endstop_name() };
    lane.move_to(dist * dir.value(), speed_mode, &endstop, assist_active, use_homing)
}

/// handle_connect: the unit's logos
pub fn unit_handle_connect(unit: &AfcUnit, family: Family) {
    let (logo, logo_error) = match family {
        Family::BoxTurtle => {
            let first_leg = "<span class=warning--text>|</span><span class=error--text>_</span>";
            let second_leg = format!("{}<span class=warning--text>|</span>", first_leg);
            let mut logo = "<span class=success--text>R  _____     ____\n".to_string();
            logo += "E /      \\  |  </span><span class=info--text>o</span><span class=success--text> | \n";
            logo += "A |       |/ ___/ \n";
            logo += "D |_________/     \n";
            logo += &format!("Y {first}{second} {first}{second}\n", first = first_leg, second = second_leg);
            logo += &format!("  {}\n", unit.name);
            let mut logo_error = "<span class=error--text>E  _ _   _ _\n".to_string();
            logo_error += "R |_|_|_|_|_|\n";
            logo_error += "R |         \\____\n";
            logo_error += "O |              \\ \n";
            logo_error += "R |          |\\ <span class=secondary--text>X</span> |\n";
            logo_error += "! \\_________/ |___|</span>\n";
            logo_error += &format!("  {}\n", unit.name);
            (logo, logo_error)
        }
        Family::NightOwl => {
            let mut logo = "<span class=success--text>R  ,     ,\n".to_string();
            logo += "E  )\\___/(\n";
            logo += "A {(@)v(@)}\n";
            logo += "D  {|~~~|}\n";
            logo += "Y  {/^^^\\}\n";
            logo += "!   `m-m`</span>\n";
            (logo, "<span class=error--text>Night Owl Not Ready</span>\n".to_string())
        }
        Family::QuattroBox => ("<span class=success--text>Quattro Box Ready</span>".to_string(), "<span class=error--text>Quattro Box Not Ready</span>\n".to_string()),
        Family::Emu => ("<span class=success--text>EMU Ready\n</span>".to_string(), "<span class=error--text>EMU Not Ready</span>\n".to_string()),
    };
    *unit.logo.borrow_mut() = logo;
    *unit.logo_error.borrow_mut() = logo_error;
}

/// _move_lane: forward then back by 5 mm; the lane's load state
fn move_lane(unit: &AfcUnit, lane: &Rc<AfcLane>, delay: f64, enable_movement: bool) -> KResult<bool> {
    let afc = &unit.afc;
    if enable_movement {
        lane.move_lane(5.0, afc.short_moves_speed, afc.short_moves_accel, true)?;
        afc.reactor.pause(afc.reactor.monotonic() + delay)?;
        lane.move_lane(-5.0, afc.short_moves_speed, afc.short_moves_accel, true)?;
    } else {
        afc.reactor.pause(afc.reactor.monotonic() + 0.7)?;
    }
    Ok(lane.load_state())
}

pub fn unit_system_test(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, delay: f64, assign_tcmd: bool, enable_movement: bool) -> KResult<bool> {
    let afc = &unit.afc;
    let mut msg = String::new();
    let mut succeeded = true;

    // Run test reverse/forward on each lane
    cur_lane.unsync_to_extruder(false)?;
    let loaded = move_lane(unit, cur_lane, delay, enable_movement)?;

    if !cur_lane.prep_state.get() {
        if !loaded {
            cur_lane.unit_obj()?.lane_not_ready(cur_lane)?;
            msg += "EMPTY READY FOR SPOOL";
        } else {
            unit.lane_fault(cur_lane)?;
            cur_lane.do_enable(false)?;
            msg = "<span class=error--text>CHECK FILAMENT Prep: False - Load: True</span>".into();
            succeeded = false;
        }
    } else {
        unit.lane_loaded(cur_lane, false)?;
        msg += "<span class=success--text>LOCKED</span>";
        if !loaded {
            msg += "<span class=error--text> NOT LOADED</span>";
            unit.lane_not_ready(cur_lane)?;
            succeeded = false;
        } else {
            cur_lane.set_status(LaneState::Loaded);
            msg += "<span class=success--text> AND LOADED</span>";
            unit.lane_illuminate_spool(cur_lane)?;
            let ext = cur_lane.extruder_obj()?;
            if cur_lane.tool_loaded.get() && ext.lane_loaded.borrow().as_deref() == Some(cur_lane.name.as_str()) {
                let mut ramming_loaded = false;
                if ext.tool_start.as_deref() == Some("buffer") {
                    ramming_loaded = unit.buffer_toolhead_load_check(cur_lane)?;
                }
                if cur_lane.get_toolhead_pre_sensor_state()? || ramming_loaded || ext.tool_end_state.get() {
                    if ext.lane_loaded.borrow().as_deref() == Some(cur_lane.name.as_str()) {
                        cur_lane.sync_to_extruder(true)?;
                        msg += &ext.prep_on_shuttle_check();
                        if ext.tool_start.as_deref() == Some("buffer") && (!afc.homing_enabled || !unit.enable_buffer_tool_check) {
                            msg += "<span class=warning--text>\n Ram sensor enabled, confirm tool is loaded</span>";
                        }
                        if afc.current().as_deref() == Some(cur_lane.name.as_str()) {
                            afc.spool.set_active_spool(&cur_lane.spool_id.borrow().clone())?;
                            unit.lane_tool_loaded(cur_lane)?;
                            cur_lane.set_status(LaneState::Tooled);
                        } else {
                            unit.lane_tool_loaded_idle(cur_lane)?;
                        }
                        cur_lane.enable_buffer(false)?;
                    } else if cur_lane.get_toolhead_pre_sensor_state()? || ext.tool_end_state.get() {
                        msg += "<span class=error--text> error in ToolHead. \nLane identified as loaded \n but not identified as loaded in extruder</span>";
                        succeeded = false;
                    }
                } else {
                    // send to error handling
                    let lane_check = afc.error.fix(Some("toolhead"), Some(LaneRef::Lane(cur_lane.clone())))?;
                    if !lane_check {
                        return Ok(false);
                    }
                }
            }
        }
    }

    if assign_tcmd {
        afc.function.tcmd_assign(cur_lane)?;
    }
    // Now that a T command is assigned, send lane data to moonraker
    cur_lane.send_lane_data()?;
    cur_lane.do_enable(false)?;
    afc.logger.info(&AFC_BOXTURTLE_AFCBOXTURTLE_SYSTEM_TEST, &format!("{} tool cmd: {:<3} {}", cur_lane.name, cur_lane.map_to_string(), msg));
    cur_lane.set_afc_prep_done();
    Ok(succeeded)
}

/// move_until_state: to the sensor, back off it, then onto it in tolerance steps
#[allow(clippy::too_many_arguments)]
fn move_until_state(lane: &AfcLane, state: &dyn Fn() -> KResult<bool>, move_dis: Num, tolerance: Num, short_move: Num, mut pos: Num, fault_dis: Num, checkpoint: &str) -> KResult<(Num, String, bool)> {
    let afc = &lane.afc;
    let (speed, accel) = (lane.short_moves_speed(), lane.short_moves_accel());
    // moves filament until specified sensor, returns values for further calibration
    while !state()? {
        lane.move_lane(move_dis.v, speed, accel, false)?;
        pos = pos.add(move_dis);
        if pos.v >= fault_dis.v {
            // return if pos exceeds fault_dis
            return Ok((fault_dis, checkpoint.to_string(), false));
        }
    }
    afc.reactor.pause(afc.reactor.monotonic() + 0.1)?;

    let mut state_retracts;
    while state()? {
        // retract off of sensor (python's "state_retracts =+ 1" is 1 every pass)
        state_retracts = 1;
        lane.move_lane(-short_move.v, speed, accel, true)?;
        pos = pos.sub(short_move);
        let check_p = format!("{} switch did not go false, reset lane and check switch", checkpoint);
        if state_retracts >= 4 {
            // fault if it takes more than 4 attempts
            return Ok((Num { v: short_move.v * 4.0, int: short_move.int }, check_p, false));
        }
    }
    afc.reactor.pause(afc.reactor.monotonic() + 0.1)?;

    let mut tol_checks = 0;
    while !state()? {
        // move back to sensor in short steps
        tol_checks += 1;
        lane.move_lane(tolerance.v, speed, accel, false)?;
        pos = pos.add(tolerance);
        let check_p = format!("{} switch failed to become true during tolerance check, reset lane and check switch", checkpoint);
        if tol_checks >= 15 {
            // fault if tol_checks exceed 15
            return Ok((fault_dis, check_p, false));
        }
    }
    Ok((pos, checkpoint.to_string(), true))
}

/// calc_position: off and back onto the sensor to find the end position
fn calc_position(lane: &AfcLane, state: &dyn Fn() -> KResult<bool>, mut pos: Num, short_move: Num, tolerance: Num, fault_dis: Num, checkpoint: &str) -> KResult<(Num, String, bool)> {
    let afc = &lane.afc;
    let (speed, accel) = (lane.short_moves_speed(), lane.short_moves_accel());
    let mut check_pos = Num::i(0);
    while state()? {
        // retract from sensor
        lane.move_lane(-short_move.v, speed, accel, true)?;
        pos = pos.sub(short_move);
        check_pos = check_pos.sub(short_move);
        if check_pos.v.abs() >= fault_dis.v {
            // fault if absolute value you check_pos exceeds fault_dis
            return Ok((fault_dis, checkpoint.to_string(), false));
        }
    }
    afc.reactor.pause(afc.reactor.monotonic() + 0.1)?;

    let checkpoint = format!("{}, tolerance check,", checkpoint);
    let mut tol_checks = 0;
    while !state()? {
        // move back to sensor to confirm distance
        tol_checks += 1;
        lane.move_lane(tolerance.v, speed, accel, false)?;
        pos = pos.add(tolerance);
        if tol_checks >= 15 {
            // fault if tol_checks exceeds 15
            return Ok((pos, checkpoint, false));
        }
    }
    Ok((pos, checkpoint, true))
}

/// calibrate_hub: the lane's distance to the hub sensor
fn calibrate_hub(lane: &Rc<AfcLane>, tol: f64) -> KResult<(bool, String, Num)> {
    let hub = lane.hub_obj()?;
    let hub_fault_dis = Num::f(lane.dist_hub.get() + 150.0);
    let checkpoint = format!("hub calibration {}", lane.name);
    let h = hub.clone();
    let state = move || -> KResult<bool> { Ok(h.state()) };
    // move until hub sensor is triggered and get information
    let (hub_pos, checkpoint, success) = move_until_state(lane, &state, Num::f(hub.move_dis()), Num::f(tol), Num::f(lane.short_move_dis()), Num::i(0), hub_fault_dis, &checkpoint)?;
    if !success {
        // fault if check is not successful
        let mut msg = format!("Failed to calibrate dist_hub for {} after moving {}mm. ", lane.name, hub_pos.str());
        msg += "If filament stopped short of the hub during calibration use the following command to increase dist_hub value.";
        msg += &format!(" SET_HUB_DIST LANE={} LENGTH=+(distance the filament was short from the hub)", lane.name);
        return Ok((false, msg, hub_pos));
    }
    let hub_dist = Num::f(lane.dist_hub.get() + 500.0);
    // verify hub distance
    let (tuned_hub_pos, checkpoint, success) = calc_position(lane, &state, hub_pos, Num::f(lane.short_move_dis()), Num::f(tol), hub_dist, &checkpoint)?;
    if !success {
        // fault if check is not successful
        return Ok((false, format!("failed {} after {}mm", checkpoint, tuned_hub_pos.str()), tuned_hub_pos));
    }
    // when successful return values to calibration macro
    Ok((true, String::new(), tuned_hub_pos))
}

pub fn unit_calibrate_bowden(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, dis: f64, tol: f64) -> KResult<(bool, String, f64)> {
    let c = &AFC_BOXTURTLE_AFCBOXTURTLE_CALIBRATE_BOWDEN;
    let afc = &unit.afc;
    let cur_extruder = cur_lane.extruder_obj()?;
    let cur_hub = cur_lane.hub_obj()?;
    if cur_lane.is_direct_hub() {
        afc.logger.raw(c, &format!("Calibrating dist_hub Length for {}", cur_lane.name));
    } else {
        afc.logger.raw(c, &format!("Calibrating Bowden Length with {}", cur_lane.name));
    }
    // Store variable locally so check only happens once
    let is_direct_dist = cur_lane.is_direct_dist();

    let mut checkpoint = "Moving to hub".to_string();
    let (hub_pos, success, bowden_length, variable_name, fullname, fault_dis);
    if !is_direct_dist {
        // move to hub and retrieve that distance, the checkpoint returned and if successful
        if !afc.homing_enabled {
            let h = cur_hub.clone();
            let state = move || -> KResult<bool> { Ok(h.state()) };
            let r = move_until_state(cur_lane, &state, Num::f(cur_hub.move_dis()), Num::f(tol), Num::f(cur_lane.short_move_dis()), Num::i(0), Num::f(cur_lane.dist_hub.get() + 200.0), &checkpoint)?;
            hub_pos = r.0;
            checkpoint = r.1;
            success = r.2;
        } else {
            let r = cur_lane.unit_obj()?.move_to_hub(cur_lane, cur_lane.dist_hub.get() + 200.0, MoveDirection::Pos, afc.homing_enabled, SpeedMode::Calibration)?;
            success = r.0;
            hub_pos = Num::f(r.1);
        }
        bowden_length = Num::f(cur_hub.afc_bowden_length.get());
        variable_name = "afc_bowden_length";
        fullname = cur_hub.fullname.clone();
        fault_dis = bowden_length.v + 500.0;
    } else {
        checkpoint = "retract to extruder".into();
        if !afc.homing_enabled {
            let l = cur_lane.clone();
            let state = move || -> KResult<bool> { Ok(l.raw_load_state()) };
            let r = calc_position(cur_lane, &state, Num::i(0), Num::f(cur_lane.short_move_dis()), Num::f(tol), Num::f(cur_lane.dist_hub.get() + 100.0), &checkpoint)?;
            hub_pos = r.0;
            checkpoint = r.1;
            success = r.2;
        } else {
            let r = cur_lane.unit_obj()?.move_to_load(cur_lane, cur_lane.dist_hub.get() + 100.0, MoveDirection::Neg, afc.homing_enabled, SpeedMode::Calibration)?;
            success = r.0;
            hub_pos = Num::f(r.1);
        }
        bowden_length = Num::f(cur_lane.dist_hub.get());
        variable_name = "dist_hub";
        fullname = cur_lane.fullname.clone();
        fault_dis = bowden_length.v + 500.0;
    }

    if !success {
        // if movement does not succeed fault and return values to calibration macro
        return Ok((false, format!("Failed {} after {}mm", checkpoint, hub_pos.str()), hub_pos.v));
    }

    let mut bow_pos = Num::i(0);
    if cur_extruder.tool_start.as_deref().is_some_and(|s| !s.is_empty()) {
        // if tool_start is defined move and confirm distance
        let mut dis = Num::f(dis);
        while !cur_lane.get_toolhead_pre_sensor_state()? {
            if afc.homing_enabled {
                dis = Num::f(fault_dis);
            }
            let (_homed, distance, warn) = cur_lane.move_to(dis.v, SpeedMode::Calibration, &cur_lane.get_toolhead_endstop()?, AssistActive::No, afc.homing_enabled)?;
            // Check for error and return, if error state is set then AFC tried pausing
            // during the homing
            if warn == MoveWarning::Error {
                return Ok((false, "Error occurred when trying to move lane".into(), fault_dis));
            }
            bow_pos = bow_pos.add(if afc.homing_enabled { Num::f(distance) } else { Num::i(distance as i64) });
            afc.reactor.pause(afc.reactor.monotonic() + 0.1)?;
            if bow_pos.v >= fault_dis {
                // fault if move to bowden length does not reach toolhead sensor return to calibration macro
                let mut msg = format!("while moving to toolhead. Failed after {}mm, ", bow_pos.str());
                msg += "if filament stopped short of the toolhead sensor/ramming during calibration";
                msg += "use the following command to increase bowden length";
                if !is_direct_dist {
                    msg += &format!("\n SET_BOWDEN_LENGTH HUB={} LENGTH=+(distance the filament was short from the toolhead)", cur_hub.name);
                } else {
                    msg += &format!("\n SET_HUB_DIST LANE={} LENGTH=+(distance the filament was short from the toolhead)", cur_lane.name);
                }
                return Ok((false, msg, bow_pos.v));
            }
        }

        let mut success = success;
        if cur_extruder.tool_start.as_deref() != Some("buffer") && !afc.homing_enabled {
            // is using ramming, only use first trigger of sensor
            let l = cur_lane.clone();
            let state = move || -> KResult<bool> { l.get_toolhead_pre_sensor_state() };
            let r = calc_position(cur_lane, &state, bow_pos, Num::f(cur_lane.short_move_dis()), Num::f(tol), Num::i(100), "retract from toolhead sensor")?;
            bow_pos = r.0;
            checkpoint = r.1;
            success = r.2;
        }
        if !success {
            // fault if check is not successful
            return Ok((false, format!("Failed {} after {}mm", checkpoint, bow_pos.str()), bow_pos.v));
        }

        let success = if !is_direct_dist {
            cur_lane.unit_obj()?.move_to_hub(cur_lane, bow_pos.v, MoveDirection::Neg, afc.homing_enabled, SpeedMode::Long)?.0
        } else {
            cur_lane.unit_obj()?.move_to_load(cur_lane, bow_pos.v, MoveDirection::Neg, afc.homing_enabled, SpeedMode::Long)?.0
        };
        if !success {
            return Ok((false, "Failed to home filament back to hub".into(), fault_dis));
        }

        if !afc.homing_enabled && !is_direct_dist {
            let (success, message, hub_dis) = calibrate_hub(cur_lane, tol)?;
            if !success {
                return Ok((false, message, hub_dis.v));
            }
        }

        if !cur_lane.is_direct_hub() {
            cur_lane.move_lane(-cur_hub.hub_clear_move_dis(), cur_lane.short_moves_speed(), cur_lane.short_moves_accel(), true)?;
        } else {
            // When direct lane move forwards so that load sensor is still triggered
            cur_lane.move_lane(cur_lane.short_move_dis() * 4.0, cur_lane.short_moves_speed(), cur_lane.short_moves_accel(), true)?;
        }

        let mut bowden_dist = bow_pos.round2();
        if !afc.homing_enabled {
            if cur_extruder.tool_start.as_deref() == Some("buffer") {
                bowden_dist = bow_pos.sub(Num::f(cur_lane.short_move_dis() * 2.0)).round2();
            } else {
                bowden_dist = bow_pos.sub(Num::f(cur_lane.short_move_dis())).round2();
            }
        }

        if bowden_dist.v < 0.0 {
            afc.error.afc_error(c, &format!("'{}' is not a valid length. Please check your setup and re-run calibration.", bowden_dist.str()), false, None)?;
            return Ok((false, "Invalid bowden length".into(), bowden_dist.v));
        }

        let mut unload_cal_msg = String::new();
        let cal_msg = format!("\n {}: New: {} Old: {}", variable_name, bowden_dist.str(), bowden_length.str());
        if !is_direct_dist {
            unload_cal_msg = format!("\n afc_unload_bowden_length: New: {} Old: {}", bowden_dist.str(), py_float_repr(cur_hub.afc_unload_bowden_length.get()));
            cur_hub.afc_unload_bowden_length.set(bowden_dist.v);
            cur_hub.afc_bowden_length.set(bowden_dist.v);
        } else {
            cur_lane.dist_hub.set(bowden_dist.v);
        }

        afc.function.config_rewrite(&fullname, variable_name, &bowden_dist.pv(), &cal_msg)?;
        if !is_direct_dist {
            afc.function.config_rewrite(&fullname, "afc_unload_bowden_length", &PyVal::Float(cur_hub.afc_unload_bowden_length.get()), &unload_cal_msg)?;
            cur_lane.loaded_to_hub.set(true);
        }

        cur_lane.do_enable(false)?;
        cur_lane.unit_obj()?.return_to_home(false)?;
        afc.save_vars()?;
        Ok((true, format!("{} successful", variable_name), bowden_dist.v))
    } else {
        afc.logger.info(c, "CALIBRATE_AFC is not currently supported without tool start sensor");
        Ok((false, "CALIBRATE_AFC is not currently supported without tool start sensor".into(), 0.0))
    }
}

pub fn unit_calibrate_td1(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, dis: f64, tol: f64) -> KResult<(bool, String, f64)> {
    let c = &AFC_BOXTURTLE_AFCBOXTURTLE_CALIBRATE_TD1;
    let afc = &unit.afc;
    let mut bow_pos = Num::i(0);
    let cur_hub = cur_lane.hub_obj()?;

    let Some(td1_device_id) = cur_lane.td1_device_id.borrow().clone() else {
        let mut msg = format!("Cannot calibrate TD-1 for {}, td1_device_id is a required ", cur_lane.name);
        msg += "field in AFC_hub or per AFC_lane";
        return Ok((false, msg, 0.0));
    };

    // Store variable locally so check only happens once
    let is_direct_dist = cur_lane.is_direct_dist();

    // Verify TD-1 is still connected before trying to get data
    let (valid, _msg) = afc.function.check_for_td1_id(Some(&td1_device_id))?;
    if !valid {
        let mut msg = format!("TD-1 device(SN: {}) not detected anymore, ", td1_device_id);
        msg += "please check before continuing to calibrate TD-1 bowden length";
        return Ok((valid, msg, 0.0));
    }

    afc.logger.raw(c, &format!("Calibrating bowden length to TD-1 device with {}", cur_lane.name));
    let mut checkpoint = String::new();
    if !cur_lane.is_direct_hub() {
        let fault_dis = Num::f(cur_lane.dist_hub.get() + cur_hub.move_dis() + 200.0);
        let h = cur_hub.clone();
        let state = move || -> KResult<bool> { Ok(h.state()) };
        let (hub_pos, cp, success) = move_until_state(cur_lane, &state, Num::f(cur_hub.move_dis()), Num::f(tol), Num::f(cur_lane.short_move_dis()), Num::i(0), fault_dis, "Moving to hub")?;
        checkpoint = cp;
        if !success {
            // if movement does not succeed fault and return values to calibration macro
            cur_lane.do_enable(false)?;
            return Ok((false, format!("Failed {} after {}mm", checkpoint, hub_pos.str()), hub_pos.v));
        }
    }

    let mut compare_time = utils::datetime_now();
    let max_bowden_length = if is_direct_dist { cur_lane.dist_hub.get() } else { cur_hub.afc_bowden_length.get() };
    while !unit.get_td1_data(cur_lane, compare_time, false)? {
        if bow_pos.v > max_bowden_length {
            // fault if move to TD-1 is not detected
            cur_lane.do_enable(false)?;
            return Ok((false, format!("TD-1 failed to detect filament after moving {}mm", bow_pos.str()), bow_pos.v));
        }
        compare_time = utils::datetime_now();
        bow_pos = bow_pos.add(Num::f(dis));
        cur_lane.move_lane(dis, unit.settings.short_moves_speed, unit.settings.short_moves_accel, false)?;
        afc.reactor.pause(afc.reactor.monotonic() + 5.0)?;
    }
    if !is_direct_dist {
        cur_lane.unit_obj()?.move_to_hub(cur_lane, bow_pos.v, MoveDirection::Neg, afc.homing_enabled, SpeedMode::Long)?;
    } else {
        cur_lane.unit_obj()?.move_to_load(cur_lane, bow_pos.v, MoveDirection::Neg, afc.homing_enabled, SpeedMode::Long)?;
    }

    if !afc.homing_enabled && !is_direct_dist {
        // Reset to hub
        let h = cur_lane.hub_obj()?;
        let state = move || -> KResult<bool> { Ok(h.state()) };
        calc_position(cur_lane, &state, Num::i(0), Num::f(cur_lane.short_move_dis()), Num::f(tol), Num::i(200), &checkpoint)?;
    }

    if !cur_lane.is_direct_hub() {
        cur_lane.move_lane(-cur_hub.hub_clear_move_dis(), cur_lane.short_moves_speed(), cur_lane.short_moves_accel(), true)?;
    } else {
        // When direct lane move forwards so that load sensor is still triggered
        cur_lane.move_lane(cur_lane.short_move_dis() * 4.0, cur_lane.short_moves_speed(), cur_lane.short_moves_accel(), true)?;
    }

    let old = cur_lane.settings().td1_bowden_length;
    let cal_msg = format!("\n td1_bowden_length: New: {} Old: {}", bow_pos.str(), old.map(py_float_repr).unwrap_or_else(|| "None".into()));
    let fullname = if is_direct_dist {
        cur_lane.settings.borrow_mut().td1_bowden_length = Some(bow_pos.v);
        cur_lane.fullname.clone()
    } else {
        cur_hub.td1_bowden_length.set(bow_pos.v);
        cur_hub.fullname.clone()
    };
    afc.function.config_rewrite(&fullname, "td1_bowden_length", &bow_pos.pv(), &cal_msg)?;
    cur_lane.do_enable(false)?;
    cur_lane.unit_obj()?.return_to_home(false)?;
    afc.save_vars()?;
    Ok((true, "td1_bowden_length calibration successful".into(), bow_pos.v))
}

pub fn unit_calibrate_lane(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, tol: f64) -> KResult<(bool, String, f64)> {
    let c = &AFC_BOXTURTLE_AFCBOXTURTLE_CALIBRATE_LANE;
    let afc = &unit.afc;
    // function to calibrate distance from secondary extruder to hub
    let cur_hub = cur_lane.hub_obj()?;
    if cur_hub.state() {
        return Ok((false, "Hub is not clear, check before calibration".into(), 0.0));
    }
    if !cur_lane.load_state() {
        return Ok((false, format!("{} not loaded, load before calibration", cur_lane.name), 0.0));
    }
    if !cur_lane.prep_state.get() {
        return Ok((false, format!("{} is loaded but not prepped, check prep before calibration", cur_lane.name), 0.0));
    }

    afc.logger.info(c, &format!("Calibrating {}", cur_lane.name));
    cur_lane.set_status(LaneState::Calibrating);
    // reset to extruder
    let move_dis = cur_lane.dist_hub.get() + 100.0;
    let (pos, checkpoint, success);
    if afc.homing_enabled {
        let r = cur_lane.move_to(-move_dis, SpeedMode::Calibration, &cur_lane.load_es(), AssistActive::Yes, true)?;
        checkpoint = "retract to extruder".to_string();
        success = r.0;
        pos = Num::f(r.1);
        // Check for error and return, if error state is set then AFC tried pausing
        // during the homing
        if r.2 == MoveWarning::Error {
            return Ok((false, "Error occurred when trying to move lane".into(), 0.0));
        }
    } else {
        let l = cur_lane.clone();
        let state = move || -> KResult<bool> { Ok(l.raw_load_state()) };
        let r = calc_position(cur_lane, &state, Num::i(0), Num::f(cur_lane.short_move_dis()), Num::f(tol), Num::f(move_dis), "retract to extruder")?;
        pos = r.0;
        checkpoint = r.1;
        success = r.2;
    }

    if !success {
        let msg = if checkpoint == "retract to extruder" {
            format!(
                "\n{} failed during calibration after {}mm. Check position of filament and reset filament using BT_LANE_MOVE macro if necessary. If filament is between the extruder and the hub, and is moving smoothly, you may need to increase the dist_hub value. Once adjusted, please try again.\nThis can be adjusted by using the SET_HUB_DIST LANE={} LENGTH=+/-distance macro. Once you are satisfied, you can save the values with SAVE_HUB_DIST LANE={} macro.",
                cur_lane.name,
                pos.round2().str(),
                cur_lane.name,
                cur_lane.name
            )
        } else {
            format!("Lane failed to calibrate {} after {}mm", checkpoint, pos.str())
        };
        cur_lane.set_status(LaneState::None);
        cur_lane.unit_obj()?.return_to_home(false)?;
        return Ok((false, msg, 0.0));
    }

    let (success, message, hub_pos) = if afc.homing_enabled {
        let (success, hub_pos, _) = cur_lane.unit_obj()?.move_to_hub_assist(cur_lane, move_dis, MoveDirection::Pos, true, SpeedMode::Calibration, AssistActive::No)?;
        let mut message = format!("\nFailed to calibrate dist_hub for {} after moving {}mm. ", cur_lane.name, py_float_repr(hub_pos));
        message += "If filament stopped short of the hub during calibration use the following command to increase dist_hub value";
        message += &format!(" SET_HUB_DIST LANE={} LENGTH=+(distance the filament was short from the hub)", cur_lane.name);
        (success, message, Num::f(hub_pos))
    } else {
        calibrate_hub(cur_lane, tol)?
    };

    if !success {
        cur_lane.set_status(LaneState::None);
        cur_lane.unit_obj()?.return_to_home(false)?;
        return Ok((false, message, hub_pos.v));
    }

    // Always run hub clear
    cur_lane.move_lane(-cur_hub.hub_clear_move_dis(), cur_lane.short_moves_speed(), cur_lane.short_moves_accel(), true)?;

    let cal_dist = hub_pos.sub(Num::f(cur_hub.hub_clear_move_dis())).round2();
    let cal_msg = format!("\n{} dist_hub: New: {} Old: {}", cur_lane.name, cal_dist.str(), py_float_repr(cur_lane.dist_hub.get()));
    cur_lane.loaded_to_hub.set(true);
    cur_lane.do_enable(false)?;
    cur_lane.dist_hub.set(cal_dist.v);
    afc.function.config_rewrite(&cur_lane.fullname, "dist_hub", &cal_dist.pv(), &cal_msg)?;
    cur_lane.set_status(LaneState::None);
    cur_lane.unit_obj()?.return_to_home(false)?;
    Ok((true, cal_msg, cal_dist.v))
}

/// prep_load: home to (or step up to) the load sensor after a spool goes in
pub fn unit_prep_load(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<()> {
    let c = &AFC_BOXTURTLE_AFCBOXTURTLE_PREP_LOAD;
    let afc = &unit.afc;
    let short_move_dis = unit.settings.short_move_dis;
    if afc.homing_enabled {
        lane.move_to(short_move_dis * MAX_NUM_MOVES as f64, SpeedMode::Short, &lane.load_es(), AssistActive::No, true)?;
    }
    let mut x = 0;
    while !lane.raw_load_state() && lane.prep_state.get() && lane.load.is_some() {
        x += 1;
        lane.move_lane(short_move_dis, 500.0, 400.0, false)?;
        afc.reactor.pause(afc.reactor.monotonic() + 0.1)?;
        if x > MAX_NUM_MOVES {
            let msg = " FAILED TO LOAD, CHECK FILAMENT AT TRIGGER\n||==>--||----||------||\nTRG   LOAD   HUB    TOOL";
            afc.error.afc_error(c, msg, false, None)?;
            lane.unit_obj()?.lane_fault(lane)?;
            lane.set_status(LaneState::None);
            break;
        }
    }
    Ok(())
}

/// prep_post_load: on to the hub (load_to_hub) once the spool is in; an
/// EMU with a virtual hub backs off its load switch instead
pub fn unit_prep_post_load(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<()> {
    if emu_virtual_hub(unit, lane) {
        let clear = lane.hub_obj()?.hub_clear_move_dis();
        let mut num_tries = 0;
        while lane.raw_load_state() && num_tries < 10 {
            lane.move_to(clear * MoveDirection::Neg.value(), SpeedMode::Short, &super::lane::Endstop::None, AssistActive::No, false)?;
            num_tries += 1;
        }
        lane.loaded_to_hub.set(true);
        return Ok(());
    }
    if !lane.is_direct_hub() && lane.load_to_hub && !lane.loaded_to_hub.get() && lane.load_state() && lane.prep_state.get() {
        let (speed, accel) = {
            let s = lane.settings();
            (s.dist_hub_move_speed, s.dist_hub_move_accel)
        };
        let dist_hub = lane.dist_hub.get();
        lane.move_lane(dist_hub, speed, accel, dist_hub > 200.0)?;
        lane.loaded_to_hub.set(true);
    }
    Ok(())
}

/// eject_lane: back to the load sensor, off it, and clear of the extruder
pub fn unit_eject_lane(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<()> {
    let c = &AFC_BOXTURTLE_AFCBOXTURTLE_EJECT_LANE;
    let afc = &unit.afc;
    if matches!(unit.kind, UnitKind::BoxTurtle(Family::Emu)) {
        // EMU: back to the load switch, then clear of the extruder
        if lane.loaded_to_hub.get() {
            lane.move_to(-lane.dist_hub.get(), SpeedMode::DistHub, &lane.load_es(), AssistActive::Dynamic, afc.homing_enabled)?;
        }
        let clear = lane.settings().extruder_clear_dis.unwrap_or(0.0);
        lane.move_advanced(-clear, SpeedMode::Short, AssistActive::No)?;
        return lane.do_enable(false);
    }
    let r = (|| -> KResult<()> {
        if lane.loaded_to_hub.get() {
            lane.move_to(-lane.dist_hub.get(), SpeedMode::DistHub, &lane.load_es(), AssistActive::Dynamic, afc.homing_enabled)?;
        }
        let mut max_tries = 0;
        while lane.load_state() {
            max_tries += 1;
            lane.move_advanced(-lane.hub_obj()?.move_dis(), SpeedMode::Short, AssistActive::Yes)?;
            if max_tries >= MAX_NUM_MOVES {
                afc.error.afc_error(c, &format!(" Failed to eject {}", lane.name), false, None)?;
                break;
            }
        }
        let clear = lane.settings().extruder_clear_dis.unwrap_or(0.0);
        lane.move_advanced(-clear, SpeedMode::Short, AssistActive::No)
    })();
    lane.do_enable(false)?;
    r
}

/// The base unit's _buffer_toolhead_load_check
pub fn buffer_toolhead_load_check(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<bool> {
    let c = &AFC_UNIT_AFCUNIT__BUFFER_TOOLHEAD_LOAD_CHECK;
    let afc = &unit.afc;
    if !afc.homing_enabled || !unit.enable_buffer_tool_check {
        return Ok(lane.tool_loaded.get());
    }
    let (homed, move_dis, _) = lane.move_to(200.0, SpeedMode::Short, &super::lane::Endstop::Name(HOMING_BUFFER.into()), AssistActive::No, true)?;
    if !homed {
        let mut msg = format!("Buffer toolhead loaded check failed for {}. Please verify", lane.name);
        msg += &format!(" that {} is loaded to toolhead. If lane is not loaded to ", lane.name);
        msg += &format!("toolhead then run AFC_RESET and choose {} to reset back to hub.", lane.name);
        msg += " Once lane is reset run UNSET_LANE_LOADED macro.";
        afc.error.afc_error(c, &msg, false, None)?;
    } else {
        lane.move_lane(-move_dis, lane.short_moves_speed(), lane.short_moves_accel(), false)?;
    }
    Ok(homed)
}
