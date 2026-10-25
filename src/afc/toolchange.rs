// AFC.py, continued: the load and unload sequences, CHANGE_TOOL, status,
// M104/M109 and the remaining AFC commands
//
// Copyright (C) 2024-2026 Armored Turtle
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::extruder::AfcExtruder;
use super::hub::AfcHub;
use super::lane::{self, AfcLane, AssistActive, LaneState, MoveDirection, MoveWarning, SpeedMode};
use super::{error, no_attr, py_bool, py_opt_str, py_repr, Afc, State, AFC_VERSION};
use kaliru::configfile::Lim;
use kaliru::error::{KError, KResult};
use kaliru::extras::heaters::{Heater, PrinterHeaters};
use kaliru::webhooks::WebRequest;
use kaliru::gcode::GCodeCommand;
use kaliru::pyfmt::py_float_repr;
use kaliru::pyval::{PyDict, PyVal};
use std::rc::Rc;

impl Afc {
    /// load_sequence without a custom command or unit hook: to the hub,
    /// past it, to the toolhead sensors and into the nozzle
    pub(super) fn standard_load_sequence(&self, cur_lane: &Rc<AfcLane>, cur_hub: &AfcHub, cur_extruder: &AfcExtruder) -> KResult<bool> {
        let c = &AFC_AFC_LOAD_SEQUENCE;
        let use_direct_dist = cur_lane.hub_obj_opt().is_some_and(|h| h.use_dist_hub());
        let mut warn = MoveWarning::None;
        // Move filament to the hub if it's not already loaded there.
        if !cur_lane.loaded_to_hub.get() || cur_lane.is_direct_hub() {
            let mut dist_to_hub = cur_lane.dist_hub.get();
            if self.homing_enabled && !cur_lane.is_direct_hub() {
                dist_to_hub += cur_hub.move_dis();
            }
            if cur_lane.is_direct_hub() {
                let home_endstop = cur_lane.get_toolhead_endstop()?;
                warn = cur_lane.unit_obj()?.load_then_home(cur_lane, dist_to_hub, AssistActive::Dynamic, &home_endstop)?.2;
            } else {
                warn = cur_lane.unit_obj()?.move_to_hub(cur_lane, dist_to_hub, MoveDirection::Pos, self.homing_enabled, SpeedMode::Long)?.2;
            }
            if warn == MoveWarning::Error {
                self.error.afc_error(c, &format!("Homing error occurred when trying to move to hub sensor for {}", cur_lane.name), false, None)?;
                return Ok(false);
            }
            self.delta_time.log_with_time(c, &format!("Loaded to {}", if !cur_lane.is_direct_hub() { "hub" } else { "toolhead" }), true);
        }
        cur_lane.loaded_to_hub.set(true);
        let mut hub_attempts = 0;
        // Ensure filament moves past the hub.
        while !cur_hub.state() && !cur_lane.is_direct_hub() {
            warn = cur_lane.unit_obj()?.move_to_hub(cur_lane, cur_hub.move_dis(), MoveDirection::Pos, self.homing_enabled, SpeedMode::Hub)?.2;
            if warn == MoveWarning::Error {
                self.error.afc_error(c, &format!("Homing error occurred when trying to short move to hub sensor for {}", cur_lane.name), false, None)?;
                return Ok(false);
            }
            hub_attempts += 1;
            if hub_attempts > 20 {
                let mut message = String::from("filament did not trigger hub sensor, CHECK FILAMENT PATH\n||=====||==>--||-----||\nTRG   LOAD   HUB   TOOL.");
                if self.function.in_print()? {
                    message += &format!("\nOnce issue is resolved please manually load {} ", cur_lane.name);
                    message += &format!("with {} macro and click resume to continue ", cur_lane.current_map.borrow());
                    message += "printing. \nIf you have to retract filament back, use ";
                    message += &format!("LANE_MOVE macro for {}.", cur_lane.name);
                }
                self.error.handle_lane_failure(c, cur_lane, &message, true)?;
                return Ok(false);
            }
        }
        self.delta_time.log_with_time(c, "Filament loaded to hub", true);

        // Move filament towards the toolhead.
        if !cur_lane.is_direct_hub() {
            let distance = if use_direct_dist { cur_lane.dist_hub.get() } else { cur_hub.afc_bowden_length.get() };
            warn = cur_lane.unit_obj()?.load_then_home(cur_lane, distance, AssistActive::Dynamic, &cur_lane.get_toolhead_endstop()?)?.2;
            if warn == MoveWarning::Error {
                self.error.afc_error(c, &format!("Homing error occurred when trying to move to toolhead for {}", cur_lane.name), false, None)?;
                return Ok(false);
            }
        }
        // Ensure filament reaches the toolhead.
        let mut tool_attempts = 0;
        let tool_start = cur_extruder.tool_start.clone();
        // (a virtual tool_start has no hardware: the move above is the load)
        if tool_start.as_deref().is_some_and(|t| !t.is_empty() && t != "virtual") {
            while !cur_lane.get_toolhead_pre_sensor_state()? || warn == MoveWarning::Warn {
                tool_attempts += 1;
                let mut move_distance = cur_lane.short_move_dis();
                let mut max_attempts = (self.tool_homing_distance / cur_lane.short_move_dis()).trunc() as i64;
                if self.homing_enabled && self.home_to_tool {
                    move_distance = if !cur_lane.is_direct_hub() && !use_direct_dist { cur_hub.afc_bowden_length.get() } else { cur_lane.dist_hub.get() };
                    max_attempts = 2;
                    self.logger.info(c, "Distance stopped short of commanded distance to toolhead, backing up and retrying load.");
                    cur_lane.move_to(100.0 * MoveDirection::Neg.value(), SpeedMode::Short, &lane::Endstop::None, AssistActive::No, false)?;
                }
                let (_, dist, w) =
                    cur_lane.move_to(move_distance, SpeedMode::Short, &cur_lane.get_toolhead_endstop()?, AssistActive::No, self.homing_enabled && self.home_to_tool)?;
                warn = w;
                if warn == MoveWarning::Error {
                    self.error.afc_error(c, &format!("Homing error occurred when trying to slow move to toolhead for {}", cur_lane.name), false, None)?;
                    return Ok(false);
                }
                if dist > 50.0 && self.homing_enabled {
                    warn = MoveWarning::None;
                }
                if tool_attempts >= max_attempts {
                    let mut message = String::from(
                        "filament failed to trigger pre extruder gear toolhead sensor, CHECK FILAMENT PATH\n||=====||====||==>--||\nTRG   LOAD   HUB   TOOL",
                    );
                    message += &format!("\nTo resolve set lane loaded with `SET_LANE_LOADED LANE={}` macro.", cur_lane.name);
                    message += &format!("\nManually move filament with LANE_MOVE macro for {} until filament is right before toolhead extruder gears,", cur_lane.name);
                    message += " then load into extruder gears with extrude button in your gui of choice until the color fully changes";
                    if self.homing_enabled {
                        message += "\nFilament can also be reset back to hub by running AFC_RESET ";
                        message += &format!("command then select {} to reset", cur_lane.name);
                        message += &format!("back to hub. Once lane is reset try reload lane with {} macro.", cur_lane.current_map.borrow());
                    }
                    if self.function.in_print()? {
                        message += "\nOnce filament is fully loaded click resume to continue printing";
                    }
                    self.error.handle_lane_failure(c, cur_lane, &message, true)?;
                    return Ok(false);
                }
            }
        }
        self.delta_time.log_with_time(c, "Filament loaded to pre-sensor", true);

        // Synchronize lane's extruder stepper and finalize tool loading.
        cur_lane.set_status(LaneState::ToolLoaded);
        self.save_vars()?;
        cur_lane.sync_to_extruder(true)?;
        cur_lane.unit_obj()?.lane_tool_loaded_gears(cur_lane)?;

        if cur_extruder.tool_end.is_some() {
            while !cur_extruder.tool_end_state.get() {
                tool_attempts += 1;
                self.move_e_pos(cur_lane.short_move_dis(), cur_extruder.tool_load_speed.get(), "Tool end", true)?;
                if tool_attempts > 20 {
                    let mut message = String::from(
                        "filament failed to trigger post extruder gear toolhead sensor, CHECK FILAMENT PATH\n||=====||====||==>--||\nTRG   LOAD   HUB   TOOL",
                    );
                    message += &format!("\nTo resolve set lane loaded with `SET_LANE_LOADED LANE={}` macro.", cur_lane.name);
                    message += "\nAlso might be a good idea to verify that post extruder gear toolhead sensor is working.";
                    if self.function.in_print()? {
                        message += "\nOnce issue is resolved click resume to continue printing";
                    }
                    self.error.handle_lane_failure(c, cur_lane, &message, true)?;
                    return Ok(false);
                }
            }
            self.delta_time.log_with_time(c, "Filament loaded to post-sensor", true);
        }

        // Adjust tool position for loading.
        self.move_e_pos(cur_extruder.tool_stn.get(), cur_extruder.tool_load_speed.get(), "tool stn", false)?;
        self.delta_time.log_with_time(c, "Filament loaded to nozzle", true);

        // Ramming: load until the advance sensor, then retract off it
        if tool_start.as_deref() == Some("buffer") {
            cur_lane.unsync_to_extruder(true)?;
            let mut load_checks = 0;
            while cur_lane.get_toolhead_pre_sensor_state()? {
                cur_lane.move_advanced(-cur_lane.short_move_dis(), SpeedMode::Short, AssistActive::No)?;
                load_checks += 1;
                self.reactor.pause(self.reactor.monotonic() + 0.1)?;
                if load_checks > self.tool_max_load_checks {
                    let mut msg = String::new();
                    msg += &format!("Buffer did not become compressed after {} short moves.\n", self.tool_max_load_checks);
                    msg += "Setting and increasing 'tool_max_load_checks' in AFC.cfg may improve loading reliability.\n\n";
                    msg += "Check that the filament is properly loaded into the toolhead extruder. If filament is loaded\n";
                    msg += &format!("into toolhead extruders gears, then manually run SET_LANE_LOADED LANE={} then\n", cur_lane.name);
                    msg += "manually extrude filament and clean nozzle.";
                    if self.function.in_print()? {
                        msg += "\nOnce issue is resolved click resume to continue printing";
                    }
                    self.error.handle_lane_failure(c, cur_lane, &msg, true)?;
                    return Ok(false);
                }
            }
            cur_lane.sync_to_extruder(true)?;
        }
        Ok(true)
    }

    pub(super) fn cmd_tool_unload(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_AFC_CMD_TOOL_UNLOAD;
        if self.check_bypass(true, c) {
            return Ok(());
        }
        let lane = match gcmd.get_opt("LANE") {
            Some(l) => Some(l),
            None => self.current(),
        };
        let Some(lane) = lane else { return Ok(()) };
        let Some(cur_lane) = self.lane(&lane) else {
            self.logger.info(c, &format!("{} Unknown", lane));
            return Ok(());
        };
        if !self.tool_unload(&cur_lane, true, false)? {
            self.afc_stats()?.increase_unload_error_count(self)?;
        }
        // User manually unloaded spool from toolhead, remove spool from active status
        self.spool.set_active_spool(&PyVal::None)
    }

    /// Unloads a lane from its tool; Ok(false) when an error was reported
    pub fn tool_unload(&self, cur_lane: &Rc<AfcLane>, set_start_time: bool, force_unload: bool) -> KResult<bool> {
        let c = &AFC_AFC_TOOL_UNLOAD;
        if self.check_bypass(true, c) {
            return Ok(false);
        }
        if set_start_time {
            self.delta_time.set_start_time();
        }
        if !self.function.check_homed(None, c)? {
            return Ok(false);
        }
        let error_str = self.verify_macro_positions();
        if !error_str.is_empty() {
            self.error.afc_error(c, &format!("Error occurred when verifying macro positions, TOOL_UNLOAD aborted.\n{}", error_str), self.function.in_print()?, None)?;
            return Ok(false);
        }
        let mut cur_extruder = Some(cur_lane.extruder_obj()?);
        let mut cur_lane = Some(cur_lane.clone());
        self.function.check_absolute_mode("TOOL_UNLOAD")?;

        // Perform Z-hop to avoid collisions during unloading.
        let z = {
            let gm = self.gcode_move()?;
            let mut lp = gm.last_position.borrow_mut();
            lp[2] += self.z_hop;
            lp[2]
        };
        self.move_z_pos(z, "Tool_Unload quick pull", true)?;

        let mut unload_toolhead = true;
        if !force_unload {
            let next_lookup_lane_name = self.next_lane_load.borrow().clone().unwrap_or_else(|| cur_lane.as_ref().map(|l| l.name.clone()).unwrap_or_default());
            let Some(next_lane) = self.lane(&next_lookup_lane_name) else {
                self.error.afc_error(c, &format!("Lane '{}' not found in AFC lane mapping during unload operation.", next_lookup_lane_name), self.function.in_print()?, None)?;
                return Ok(false);
            };
            let next_extruder = next_lane.extruder_obj()?.th_extruder_name.clone();
            if self.function.get_current_extruder()? != Some(next_extruder) {
                next_lane.tool_swap()?;
                cur_extruder = self.function.get_current_extruder_obj()?;
                cur_lane = match &cur_extruder {
                    Some(e) if e.lane_loaded.borrow().is_some() => self.function.get_current_lane_obj(),
                    _ => None,
                };
                self.logger.debug(
                    c,
                    &format!(
                        "Current extruder: {}, current lane:{}",
                        py_opt_str(cur_extruder.as_ref().map(|e| e.name.as_str())),
                        py_opt_str(cur_lane.as_ref().map(|l| l.name.as_str()))
                    ),
                );
            }
            let ext = cur_extruder.clone().ok_or_else(|| no_attr("NoneType", "lanes"))?;
            if let Some(next) = self.next_lane_load.borrow().clone() {
                unload_toolhead = ext.lanes.borrow().contains_key(&next) && ext.lane_loaded.borrow().as_deref() != Some(next.as_str());
            }
            self.logger.debug(
                c,
                &format!(
                    "Next lane load:{}, lanes:{}, current lane:{}, unload_toolhead:{}",
                    py_opt_str(self.next_lane_load.borrow().as_deref()),
                    ext.lanes_repr(),
                    py_opt_str(cur_lane.as_ref().map(|l| l.name.as_str())),
                    py_bool(unload_toolhead)
                ),
            );
        }

        if self.current().is_some() && unload_toolhead {
            let cur_lane = cur_lane.clone().ok_or_else(|| no_attr("NoneType", "name"))?;
            let cur_extruder = cur_extruder.clone().ok_or_else(|| no_attr("NoneType", "name"))?;
            self.current_state.set(State::Unloading);
            *self.current_loading.borrow_mut() = Some(cur_lane.name.clone());
            self.logger.info(c, &format!("Unloading {}", cur_lane.name));
            cur_lane.set_status(LaneState::ToolUnloading);
            self.save_vars()?;
            let cur_hub = cur_lane.hub_obj()?;
            let temp_state = self.capture_toolhead_temp(None, false)?;
            self.set_display_status("retraction", true);
            let r = self.unload_sequence(&cur_lane, &cur_hub, &cur_extruder);
            self.set_display_status("retraction", false);
            self.restore_toolhead_temp(temp_state.as_ref(), false)?;
            if !r? {
                return Ok(false);
            }
            let unload_time = self.delta_time.log_major_delta(c, &format!("Lane {} unload done", cur_lane.name), true);
            self.afc_stats()?.average_tool_unload_time.average_time(unload_time);
            if cur_lane.hub.borrow().as_deref() == Some("direct_load") {
                self.lane_unload(&cur_lane)?;
            }
        }
        self.current_state.set(State::Idle);
        Ok(true)
    }

    /// Cutting / parking / tip forming during unload, when enabled
    pub fn do_tool_cut_tip_form(&self, cur_lane: &AfcLane, cur_extruder: &AfcExtruder) -> KResult<()> {
        let c = &AFC_AFC_DO_TOOL_CUT_TIP_FORM;
        if self.tool_cut.get() {
            cur_lane.extruder_obj()?.estats.increase_cut_total()?;
            self.gcode.run_script_from_command(&format!("{} EXTRUDER={}", py_opt_str(self.tool_cut_cmd.as_deref()), cur_extruder.name))?;
            self.delta_time.log_with_time(c, "TOOL_UNLOAD: After cut", true);
            self.function.log_toolhead_pos("")?;
            if self.park.get() {
                self.gcode.run_script_from_command(&format!("{} EXTRUDER={}", py_opt_str(self.park_cmd.as_deref()), cur_extruder.name))?;
                self.delta_time.log_with_time(c, "TOOL_UNLOAD: After park", true);
                self.function.log_toolhead_pos("")?;
            }
        }
        if self.form_tip.get() {
            if self.park.get() {
                self.gcode.run_script_from_command(&format!("{} EXTRUDER={}", py_opt_str(self.park_cmd.as_deref()), cur_extruder.name))?;
                self.delta_time.log_with_time(c, "TOOL_UNLOAD: After form tip park", true);
                self.function.log_toolhead_pos("")?;
            }
            if self.form_tip_cmd.as_deref() == Some("AFC") {
                // (native_scope leaves form_tip_cmd AFC to the python AFC)
                return Err(no_attr("afc", "tip"));
            }
            self.gcode.run_script_from_command(&py_opt_str(self.form_tip_cmd.as_deref()))?;
            self.delta_time.log_with_time(c, "TOOL_UNLOAD: After custom form tip", true);
            self.function.log_toolhead_pos("")?;
        }
        Ok(())
    }

    fn unload_failure_next_lane(&self, message: &mut String, prefix: &str) -> KResult<()> {
        if let Some(next) = self.next_lane_load.borrow().clone() {
            let map_str = self.lane(&next).map(|l| l.current_map.borrow().clone()).ok_or_else(|| KError::Py("KeyError".into(), py_repr(&next)))?;
            *message += &format!("{}{} with {} macro", prefix, next, map_str);
            if self.function.in_print()? {
                *message += "\nOnce lane is loaded click resume to continue printing";
            }
        }
        Ok(())
    }

    /// The unload sequence (or the lane's custom unload command)
    fn unload_sequence(&self, cur_lane: &Rc<AfcLane>, cur_hub: &AfcHub, cur_extruder: &AfcExtruder) -> KResult<bool> {
        let c = &AFC_AFC_UNLOAD_SEQUENCE;
        cur_lane.unit_obj()?.lane_unloading(cur_lane)?;
        if self.check_extruder_temp(cur_lane, false)? {
            self.delta_time.log_with_time(c, "Done heating toolhead", true);
        }
        let custom_unload_cmd = cur_lane.custom_unload_cmd.borrow().clone();
        if let Some(cmd) = custom_unload_cmd.filter(|c| !c.is_empty()) {
            self.logger.info(c, &format!("Running custom unload command for lane {}", cur_lane.name));
            cur_lane.set_status(LaneState::ToolUnloading);
            self.gcode.run_script_from_command(&cmd)?;
            if let Some(m) = &self.post_unload_macro {
                self.gcode.run_script_from_command(m)?;
            }
            cur_lane.set_tool_unloaded(true)?;
            cur_lane.set_status(LaneState::None);
            self.save_vars()?;
        } else if let Some(ok) = cur_lane.unit_obj()?.unit_unload_lane(cur_lane, cur_extruder)? {
            if !ok {
                return Ok(false);
            }
        } else {
            let use_direct_dist = cur_lane.hub_obj_opt().is_some_and(|h| h.use_dist_hub());
            // Quick pull to prevent oozing.
            self.move_e_pos(-2.0, cur_extruder.tool_unload_speed.get(), "Quick Pull", false)?;
            self.function.log_toolhead_pos("TOOL_UNLOAD quick pull: ")?;
            cur_lane.disable_buffer()?;
            cur_lane.sync_to_extruder(true)?;
            cur_lane.select_lane()?;
            self.do_tool_cut_tip_form(cur_lane, cur_extruder)?;

            let mut num_tries: i64 = 0;
            if cur_extruder.tool_start.as_deref() == Some("buffer") {
                cur_lane.unsync_to_extruder(true)?;
                while !cur_lane.get_trailing().unwrap_or(false) && cur_lane.tool_max_unload_attempts() > 0 {
                    num_tries += 1;
                    self.delta_time.log_with_time(c, &format!("TOOL_UNLOAD: Retracting Buffer, Try:{}", num_tries), true);
                    cur_lane.move_advanced(-cur_lane.short_move_dis(), SpeedMode::Short, AssistActive::No)?;
                    self.reactor.pause(self.reactor.monotonic() + 0.5)?;
                    if num_tries > cur_lane.tool_max_unload_attempts() {
                        let mut msg = String::new();
                        msg += &format!("Buffer did not become compressed after {} short moves.\n", cur_lane.tool_max_unload_attempts());
                        msg += "Setting and increasing 'tool_max_unload_attempts' in AFC.cfg may improve unloading reliability\n\n";
                        msg += "Please check to make sure filament is unloaded from the toolhead's extruder. If filament is still\n";
                        msg += "loaded manually retract back until its free, then run UNSET_LANE_LOADED and then do manual\n";
                        msg += "moves with BT_LANE_MOVE until filament is retracted behind your hub. Or you can run AFC_RESET\n";
                        msg += &format!("and select {}, then AFC will slowly move lane until hub is no longer triggered.\n", cur_lane.name);
                        if let Some(next) = self.next_lane_load.borrow().clone() {
                            msg += &format!("\nOnce lane is behind hub and hub is no longer triggered manually load {}\n", next);
                            let map_str = self.lane(&next).map(|l| l.current_map.borrow().clone()).ok_or_else(|| KError::Py("KeyError".into(), py_repr(&next)))?;
                            msg += &format!("with {} macro.\n", map_str);
                            if self.function.in_print()? {
                                msg += "Once lane is loaded click resume to continue printing";
                            }
                        }
                        self.error.handle_lane_failure(c, cur_lane, &msg, true)?;
                        return Ok(false);
                    }
                }
                cur_lane.sync_to_extruder(false)?;
                if cur_extruder.tool_stn_unload.get() > 0.0 {
                    self.delta_time.log_with_time(c, "TOOL_UNLOAD: Buffer-Unloading from toolhead(tool_stn_unload)", true);
                    cur_lane.assist_move(cur_extruder.tool_unload_speed.get(), true, cur_lane.assisted_unload(), || {
                        self.move_e_pos(-cur_extruder.tool_stn_unload.get(), cur_extruder.tool_unload_speed.get(), "Buffer Move", false)
                    })?;
                }
                self.function.log_toolhead_pos("Buffer move after ")?;
            } else {
                if cur_extruder.tool_stn_unload.get() == 0.0 {
                    cur_lane.unsync_to_extruder(true)?;
                    while cur_lane.get_toolhead_pre_sensor_state()? {
                        num_tries += 1;
                        self.delta_time.log_with_time(c, &format!("TOOL_UNLOAD: Sensor-Unloading from toolhead(tool_stn_unload==0), Try:{}", num_tries), true);
                        cur_lane.move_advanced(-cur_lane.short_move_dis(), SpeedMode::Short, AssistActive::No)?;
                        if num_tries > cur_lane.tool_max_unload_attempts() {
                            break;
                        }
                        self.reactor.pause(self.reactor.monotonic() + 0.1)?;
                    }
                }
                while cur_lane.get_toolhead_pre_sensor_state()? || cur_extruder.tool_end_state.get() {
                    num_tries += 1;
                    if num_tries > cur_lane.tool_max_unload_attempts() {
                        let mut message = String::from("Failed to unload filament from toolhead. Filament stuck in toolhead.");
                        if self.function.in_print()? {
                            message += "\nRetract filament fully with retract button in gui of choice to remove from extruder gears if needed,";
                            message += "\n  and then use LANE_MOVE to fully retract behind hub so its not triggered anymore.";
                            message += "\nThen Manually run UNSET_LANE_LOADED to let AFC know nothing is loaded into toolhead";
                            self.unload_failure_next_lane(&mut message, "\nThen manually load ")?;
                        }
                        self.error.handle_lane_failure(c, cur_lane, &message, true)?;
                        return Ok(false);
                    }
                    self.delta_time.log_with_time(c, &format!("TOOL_UNLOAD: Sensor-Unloading from toolhead(tool_stn_unload), Try:{}", num_tries), true);
                    cur_lane.sync_to_extruder(true)?;
                    cur_lane.assist_move(cur_extruder.tool_unload_speed.get(), true, cur_lane.assisted_unload(), || {
                        self.move_e_pos(-cur_extruder.tool_stn_unload.get(), cur_extruder.tool_unload_speed.get(), "Sensor move", true)
                    })?;
                    self.function.log_toolhead_pos("Sensor move after ")?;
                    if cur_lane.extruder_obj()?.is_standalone() {
                        break;
                    }
                }
            }
            self.delta_time.log_with_time(c, "Unloaded from toolhead", true);

            // Move filament past the sensor after the extruder, if applicable.
            if cur_extruder.tool_sensor_after_extruder.get() > 0.0 {
                cur_lane.assist_move(cur_extruder.tool_unload_speed.get(), true, cur_lane.assisted_unload(), || {
                    self.move_e_pos(-cur_extruder.tool_sensor_after_extruder.get(), cur_extruder.tool_unload_speed.get(), "After extruder", false)
                })?;
                self.delta_time.log_with_time(c, "Tool sensor after extruder move done", true);
            }
            self.save_vars()?;
            // Synchronize and move filament out of the hub.
            cur_lane.unsync_to_extruder(true)?;
            if !cur_lane.is_direct_hub() {
                let distance = if use_direct_dist { cur_lane.dist_hub.get() } else { cur_hub.afc_unload_bowden_length.get() };
                let warn = cur_lane.unit_obj()?.move_to_hub(cur_lane, distance, MoveDirection::Neg, self.homing_enabled, SpeedMode::Long)?.2;
                if warn == MoveWarning::Error {
                    self.error.afc_error(c, &format!("Homing error occurred when trying to move back to hub sensor for {}", cur_lane.name), true, None)?;
                    return Ok(false);
                }
            } else {
                let warn = cur_lane.move_to(-cur_lane.dist_hub.get(), SpeedMode::Long, &cur_lane.load_es(), AssistActive::Dynamic, self.homing_enabled)?.2;
                if warn == MoveWarning::Error {
                    self.error.afc_error(c, &format!("Homing error occurred when trying to move back to load sensor for {}", cur_lane.name), true, None)?;
                    return Ok(false);
                }
            }
            self.delta_time.log_with_time(c, "Long retract done", true);

            // Clear toolhead's loaded state for easier error handling later.
            cur_lane.set_tool_unloaded(true)?;
            self.save_vars()?;

            // Ensure filament is fully cleared from the hub.
            num_tries = 0;
            while cur_hub.state() {
                let mut max_attempts = cur_hub.afc_unload_bowden_length.get() / cur_lane.short_move_dis();
                let mut move_dist = cur_lane.short_move_dis();
                if self.homing_enabled {
                    max_attempts = 2.0;
                    move_dist = cur_hub.afc_unload_bowden_length.get();
                }
                let warn = cur_lane.unit_obj()?.move_to_hub(cur_lane, move_dist, MoveDirection::Neg, self.homing_enabled, SpeedMode::Hub)?.2;
                if warn == MoveWarning::Error {
                    self.error.afc_error(c, &format!("Homing error occurred when trying to move back to hub sensor for {}", cur_lane.name), false, None)?;
                    return Ok(false);
                }
                num_tries += 1;
                if num_tries as f64 >= max_attempts {
                    let mut message = String::from("Hub is not clearing, filament may be stuck in hub");
                    message += "\nPlease check to make sure filament has not broken off and caused the sensor to stay stuck";
                    message += &format!("\nIf you have to retract filament back, use LANE_MOVE macro for {}.", cur_lane.name);
                    if self.function.in_print()? {
                        self.unload_failure_next_lane(&mut message, "\nOnce hub is clear, manually load ")?;
                    }
                    self.error.handle_lane_failure(c, cur_lane, &message, true)?;
                    return Ok(false);
                }
            }
            self.delta_time.log_with_time(c, "Hub cleared", true);

            // Move to make sure hub path is clear based on the move_clear_dis var
            if !cur_lane.is_direct_hub() {
                cur_lane.move_advanced(-cur_hub.hub_clear_move_dis(), SpeedMode::Short, AssistActive::Yes)?;
                if cur_hub.cut {
                    if cur_hub.cut_cmd.as_deref() == Some("AFC") {
                        cur_hub.hub_cut(cur_lane)?;
                    } else {
                        self.gcode.run_script_from_command(&py_opt_str(cur_hub.cut_cmd.as_deref()))?;
                    }
                    while cur_hub.state() {
                        cur_lane.move_advanced(-cur_lane.short_move_dis(), SpeedMode::Short, AssistActive::Yes)?;
                        num_tries += 1;
                        if num_tries as f64 > (cur_hub.afc_unload_bowden_length.get() / cur_lane.short_move_dis()) {
                            self.error.handle_lane_failure(c, cur_lane, "HUB NOT CLEARING after hub cut\n", true)?;
                            return Ok(false);
                        }
                    }
                    self.delta_time.log_with_time(c, "Hub cut done", true);
                }
            }

            // Finalize unloading and reset lane state.
            cur_lane.loaded_to_hub.set(true);
            cur_lane.unit_obj()?.lane_loaded(cur_lane, false)?;
            cur_lane.set_status(LaneState::None);

            if cur_lane.is_direct_hub() && !cur_lane.extruder_obj()?.is_standalone() {
                let mut park_tries = 0;
                while !cur_lane.raw_load_state() && cur_lane.prep_state.get() {
                    park_tries += 1;
                    cur_lane.move_advanced(cur_lane.short_move_dis(), SpeedMode::Short, AssistActive::No)?;
                    if park_tries >= 5 {
                        break;
                    }
                }
            }
            if let Some(m) = &self.post_unload_macro {
                self.gcode.run_script_from_command(m)?;
            }
            cur_lane.do_enable(false)?;
            cur_lane.unit_obj()?.return_to_home(false)?;
            cur_lane.espooler.stats()?.update_database();
            self.save_vars()?;
        }

        // Update tool and lane status.
        cur_lane.disable_buffer()?;
        cur_lane.do_enable(false)?;
        cur_lane.extruder_obj()?.estats.tc_tool_unload().increase_count()?;
        self.save_vars()?;
        Ok(true)
    }

    pub(super) fn cmd_change_tool(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_AFC_CMD_CHANGE_TOOL;
        // (the snapmaker A= path: snapmaker_printer is never true on kalico)
        if self.check_bypass(false, c) {
            return Ok(());
        }
        if !self.function.check_homed(None, c)? {
            return Ok(());
        }
        let mut purge_length = None;
        if let Some(p) = gcmd.get_opt("PURGE_LENGTH") {
            match kaliru::pyfmt::py_float(p.trim_start_matches('=')) {
                Some(v) => purge_length = Some(PyVal::Float(v)),
                None => {
                    self.error.afc_error(c, "PURGE_LENGTH must be a numeric value", false, None)?;
                    return Ok(());
                }
            }
        }
        let mut new_extruder_temp = None;
        if let Some(t) = gcmd.get_opt("NEW_EXTRUDER_TEMP") {
            match kaliru::pyfmt::py_float(t.trim_start_matches('=')) {
                Some(v) => new_extruder_temp = Some(v),
                None => {
                    self.error.afc_error(c, "NEW_EXTRUDER_TEMP must be a numeric value", false, None)?;
                    return Ok(());
                }
            }
        }
        let command_line = gcmd.get_commandline().to_string();
        self.logger.debug(c, &format!("CHANGE_TOOL: cmd-{}", command_line));
        // Remove everything after ; since it could contain CHANGE in a comment
        let command = match command_line.find(';') {
            Some(i) => &command_line[..i],
            None => &command_line,
        };
        let command = command.split(' ').next().unwrap_or("").to_uppercase();
        let cmd = command_line.to_uppercase();
        let mut tcmd = String::new();
        if command.contains("CHANGE") {
            if let Some(lane) = gcmd.get_opt("LANE") {
                for (key, value) in self.tool_cmds.borrow().iter() {
                    if value.to_uppercase() == lane.to_uppercase() {
                        tcmd = key.clone();
                        break;
                    }
                }
            }
        } else {
            tcmd = command;
        }
        if tcmd.is_empty() {
            self.error.afc_error(c, &format!("I did not understand the change -- {}", cmd), self.function.in_print()?, None)?;
            return Ok(());
        }
        let change_to_lane_name = self.tool_cmds.borrow().get(&tcmd).cloned().unwrap_or_default();
        match self.lane(&change_to_lane_name) {
            Some(change_to_lane) => {
                *change_to_lane.current_map.borrow_mut() = tcmd;
                self.change_tool(&change_to_lane, purge_length.as_ref(), true, new_extruder_temp)
            }
            None => {
                self.error.afc_error(c, &format!("Error trying to lookup lane for {}", tcmd), true, None)?;
                Ok(())
            }
        }
    }

    /// CHANGE_TOOL: unload the current lane, load cur_lane; every error is
    /// caught and reported
    pub fn change_tool(&self, cur_lane: &Rc<AfcLane>, purge_length: Option<&PyVal>, restore_pos: bool, new_extruder_temp: Option<f64>) -> KResult<()> {
        let c = &AFC_AFC_CHANGE_TOOL;
        let res = match self.change_tool_inner(cur_lane, purge_length, restore_pos, new_extruder_temp) {
            // (python's `except Exception`: not a gcode wait interruption)
            Err(e) => {
                let trace = kaliru::logging::format_traceback(&e);
                self.logger.error_opt(c, "Unexpected error during CHANGE_TOOL:\n", Some(&trace), "");
                let in_print = self.function.in_print()?;
                self.error.afc_error(
                    c,
                    "An unexpected error occurred during CHANGE_TOOL. Please check the logs and report this issue to developers.",
                    in_print,
                    None,
                )?;
                Ok(())
            }
            r => r,
        };
        *self.next_lane_load.borrow_mut() = None;
        self.function.log_toolhead_pos(&format!(
            "Final Change Tool: Error State: {}, Is Paused {}, Position_saved {}, in toolchange: {}, POS: ",
            py_bool(self.error_state.get()),
            py_bool(self.function.is_paused()?),
            py_bool(self.position_saved.get()),
            py_bool(self.in_toolchange.get())
        ))?;
        res
    }

    fn change_tool_inner(&self, cur_lane: &Rc<AfcLane>, purge_length: Option<&PyVal>, restore_pos: bool, new_extruder_temp: Option<f64>) -> KResult<()> {
        let c = &AFC_AFC_CHANGE_TOOL;
        self.delta_time.set_start_time();
        if self.check_bypass(false, c) {
            return Ok(());
        }
        *self.next_lane_load.borrow_mut() = Some(cur_lane.name.clone());
        let next_extruder = cur_lane.extruder_obj()?.th_extruder_name.clone();
        let infinite_runout = cur_lane.status() == LaneState::InfiniteRunout;
        let adjusting_temperature = new_extruder_temp.is_some() || (infinite_runout && self.function.get_current_extruder()? != Some(next_extruder.clone()));

        let mut last_lane: Option<Rc<AfcLane>> = None;
        let mut next_heat: Option<(Rc<AfcExtruder>, f64)> = None;
        if adjusting_temperature {
            let next_temp = if infinite_runout { None } else { new_extruder_temp };
            let Some(result) = self.heat_next_extruder(false, next_temp)? else {
                let next = self.next_lane_load.borrow().clone();
                self.error.fix(Some("Failed to select or heat next extruder"), next.map(error::LaneRef::Name))?;
                return Ok(());
            };
            if infinite_runout {
                cur_lane.set_status(LaneState::Loaded);
            }
            if let Some(cur) = self.current() {
                last_lane = self.lane(&cur);
                if let Some(l) = &last_lane {
                    if !infinite_runout && l.extruder_obj()?.th_extruder_name != next_extruder {
                        let e = l.extruder_obj()?;
                        self.cooldown_last_extruder(&e, infinite_runout)?;
                    }
                }
            }
            next_heat = Some(result);
        }

        if Some(cur_lane.name.clone()) != self.current() {
            self.save_pos()?;
            self.in_toolchange.set(true);
            let mut current_lane_name = None;
            let mut unload_lane = None;
            if cur_lane.afc_prep_done.get() {
                self.logger.info(c, &format!("Tool Change - {} -> {}", py_opt_str(self.current().as_deref()), cur_lane.name));
                if !self.error_state.get() && self.number_of_toolchanges.get() != 0 && self.current_toolchange.get() != self.number_of_toolchanges.get() {
                    self.current_toolchange.set(self.current_toolchange.get() + 1);
                    self.logger.raw(c, &format!("//      Change {} out of {}", self.current_toolchange.get(), self.number_of_toolchanges.get()));
                }
                current_lane_name = self.current();
                if let Some(name) = &current_lane_name {
                    let Some(ul) = self.lane(name) else {
                        self.error.afc_error(c, &format!("{} Unknown", name), true, None)?;
                        return Ok(());
                    };
                    let force_unload = infinite_runout && !ul.extruder_obj()?.is_standalone();
                    if !self.tool_unload(&ul, false, force_unload)? {
                        self.error.fix(Some(" UNLOAD ERROR NOT CLEARED"), Some(error::LaneRef::Lane(ul.clone())))?;
                        self.afc_stats()?.increase_unload_error_count(self)?;
                        return Ok(());
                    }
                    if force_unload && !ul.is_direct_hub() {
                        self.lane_unload(&ul)?;
                    }
                    unload_lane = Some(ul);
                }
            }
            if let Some((next_extruder_obj, target_temp)) = &next_heat {
                if let Some(l) = &last_lane {
                    if infinite_runout && l.extruder_obj()?.th_extruder_name != next_extruder {
                        let e = l.extruder_obj()?;
                        self.cooldown_last_extruder(&e, infinite_runout)?;
                    }
                }
                self.logger.info(
                    c,
                    &format!("Heating and waiting for {} for {}", next_extruder_obj.name, if infinite_runout { "infinite runout" } else { "tool change." }),
                );
                let unbound = || {
                    KError::Py("UnboundLocalError".into(), "cannot access local variable 'unload_lane' where it is not associated with a value".into())
                };
                if current_lane_name.is_some() && infinite_runout && self.park.get() && self.park_cmd.is_some() {
                    self.logger.info(c, "Parking while waiting for extruder to heat.");
                    let ul = unload_lane.clone().ok_or_else(unbound)?;
                    self.gcode.run_script_from_command(&format!("{} EXTRUDER={}", py_opt_str(self.park_cmd.as_deref()), ul.extruder_obj()?.name))?;
                }
                let next_heater = next_extruder_obj.get_heater()?;
                self.wait_for_temp_within_tolerance(&next_heater, *target_temp, Some(next_extruder_obj.deadband))?;
                self.logger.info(c, &format!("{} heated and ready to print", next_extruder_obj.name));
                if current_lane_name.is_some() && infinite_runout && self.wipe.get() && self.wipe_cmd.is_some() {
                    self.logger.info(c, "Wiping ooze...");
                    let ul = unload_lane.clone().ok_or_else(unbound)?;
                    self.gcode.run_script_from_command(&format!("{} EXTRUDER={}", py_opt_str(self.wipe_cmd.as_deref()), ul.extruder_obj()?.name))?;
                }
            }
            // Load the new lane and restore the toolhead position if successful.
            if self.tool_load(cur_lane, purge_length, false)? && !self.error_state.get() {
                if restore_pos {
                    self.restore_pos(true)?;
                }
                let total_time = self.delta_time.log_total_time(c, "Total change time:");
                self.afc_stats()?.average_toolchange_time.average_time(total_time);
                self.in_toolchange.set(false);
                cur_lane.extruder_obj()?.estats.increase_toolcount_change(self)?;
            } else {
                self.afc_stats()?.increase_load_error_count(self)?;
            }
        } else {
            // KTC SELECT_TOOL may have changed the tool: make sure lanes are synced
            self.function.handle_activate_extruder()?;
            self.logger.info(c, &format!("{} already loaded", cur_lane.name));
            if !self.error_state.get() && self.current_toolchange.get() == -1 {
                self.current_toolchange.set(self.current_toolchange.get() + 1);
            }
        }
        Ok(())
    }

    /// _get_message(clear): the first message of the queue
    fn get_message(&self, clear: bool) -> PyDict {
        let mut q = self.logger.message_queue.borrow_mut();
        let (message, kind) = match q.first() {
            Some((m, k)) => (m.clone(), k.to_string()),
            None => (String::new(), String::new()),
        };
        if clear && !q.is_empty() {
            q.remove(0);
        }
        kaliru::pydict! { "message" => PyVal::Str(message), "type" => PyVal::Str(kind) }
    }

    /// get_status()
    pub(super) fn status(&self) -> KResult<PyVal> {
        let opt = |v: Option<String>| v.map(PyVal::Str).unwrap_or(PyVal::None);
        let mut s = PyDict::default();
        s.insert("version".into(), PyVal::from(AFC_VERSION));
        s.insert("current_load".into(), opt(self.current()));
        s.insert("current_lane".into(), opt(self.current_loading.borrow().clone()));
        s.insert("next_lane".into(), opt(self.next_lane_load.borrow().clone()));
        s.insert("current_state".into(), PyVal::from(self.current_state.get().as_str()));
        s.insert("current_toolchange".into(), PyVal::Int(self.current_toolchange.get().max(0)));
        s.insert("number_of_toolchanges".into(), PyVal::Int(self.number_of_toolchanges.get()));
        s.insert("spoolman".into(), opt(self.spoolman.borrow().clone()));
        s.insert("td1_present".into(), PyVal::Bool(self.td1_present()?));
        s.insert("lane_data_enabled".into(), PyVal::Bool(self.lane_data_enabled.get()));
        s.insert("error_state".into(), PyVal::Bool(self.error_state.get()));
        s.insert("bypass_state".into(), PyVal::Bool(self.get_bypass_state()));
        s.insert("quiet_mode".into(), PyVal::Bool(self.get_quiet_mode()));
        s.insert("position_saved".into(), PyVal::Bool(self.position_saved.get()));
        let mut unitdisplay = vec![];
        for unit in self.units.borrow().values() {
            let t = unit.type_name().replace(' ', "_");
            if !unit.lanes.borrow().is_empty() {
                unitdisplay.push(PyVal::Str(format!("{} {}", t.replace('\'', ""), unit.name)));
            }
        }
        s.insert("units".into(), PyVal::List(unitdisplay));
        s.insert("lanes".into(), PyVal::List(self.lanes.borrow().keys().map(|k| PyVal::Str(k.clone())).collect()));
        s.insert("maps".into(), PyVal::List(self.tool_cmds.borrow().keys().map(|k| PyVal::Str(k.clone())).collect()));
        s.insert("extruders".into(), PyVal::List(self.tools.borrow().values().map(|e| PyVal::Str(e.name.clone())).collect()));
        s.insert("hubs".into(), PyVal::List(self.hubs.borrow().keys().map(|k| PyVal::Str(k.clone())).collect()));
        s.insert("buffers".into(), PyVal::List(self.buffers.borrow().keys().map(|k| PyVal::Str(k.clone())).collect()));
        s.insert("message".into(), PyVal::Dict(self.get_message(false)));
        s.insert("led_state".into(), PyVal::Bool(self.led_state.get()));
        s.insert("multiple_tool_mapping".into(), PyVal::Bool(self.enable_multiple_mapping.get()));
        Ok(PyVal::Dict(s))
    }

    /// <ip>/printer/afc/status
    pub(super) fn webhooks_status(&self, web_request: &WebRequest) -> KResult<()> {
        let mut s = PyDict::default();
        let mut numoflanes = 0;
        for unit in self.units.borrow().values() {
            let mut u = kaliru::pydict! { "system" => PyVal::dict() };
            for lane in unit.lanes.borrow().values() {
                u.insert(lane.name.clone().into(), lane.get_status(false)?);
                numoflanes += 1;
            }
            let system = kaliru::pydict! {
                "type" => PyVal::Str(unit.type_name()),
                "hub_loaded" => unit.hub_obj().map(|h| PyVal::Bool(h.state())).unwrap_or(PyVal::None),
            };
            u.insert("system".into(), PyVal::Dict(system));
            s.insert(unit.name.clone().into(), PyVal::Dict(u));
        }
        let mut system = PyDict::default();
        system.insert("version".into(), PyVal::from(AFC_VERSION));
        system.insert("current_load".into(), self.current().map(PyVal::Str).unwrap_or(PyVal::None));
        system.insert("num_units".into(), PyVal::Int(self.units.borrow().len() as i64));
        system.insert("num_lanes".into(), PyVal::Int(numoflanes));
        system.insert("num_extruders".into(), PyVal::Int(self.tools.borrow().len() as i64));
        system.insert("spoolman".into(), self.spoolman.borrow().clone().map(PyVal::Str).unwrap_or(PyVal::None));
        system.insert("td1_present".into(), PyVal::Bool(self.td1_present()?));
        system.insert("lane_data_enabled".into(), PyVal::Bool(self.lane_data_enabled.get()));
        system.insert("current_toolchange".into(), PyVal::Int(self.current_toolchange.get()));
        system.insert("number_of_toolchanges".into(), PyVal::Int(self.number_of_toolchanges.get()));
        let mut extruders = PyDict::default();
        for e in self.tools.borrow().values() {
            extruders.insert(e.name.clone().into(), e.get_status()?);
        }
        let mut hubs = PyDict::default();
        for h in self.hubs.borrow().values() {
            hubs.insert(h.name.clone().into(), h.get_status());
        }
        system.insert("extruders".into(), PyVal::Dict(extruders));
        system.insert("hubs".into(), PyVal::Dict(hubs));
        let mut buffers = PyDict::default();
        for b in self.buffers.borrow().values() {
            buffers.insert(b.name.clone().into(), b.status()?);
        }
        system.insert("buffers".into(), PyVal::Dict(buffers));
        system.insert("led_state".into(), PyVal::Bool(self.led_state.get()));
        s.insert("system".into(), PyVal::Dict(system));
        web_request.send(PyVal::Dict(kaliru::pydict! { "status:" => PyVal::Dict(kaliru::pydict! { "AFC" => PyVal::Dict(s) }) }))
    }

    pub(super) fn cmd_afc_m104(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        self.afc_m109(gcmd, false)
    }

    pub(super) fn cmd_afc_m109(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        self.afc_m109(gcmd, true)
    }

    /// M104/M109 with T (tool) and D (deadband) parameters
    fn afc_m109(&self, gcmd: &GCodeCommand, wait: bool) -> KResult<()> {
        let c = &AFC_AFC_CMD_AFC_M109;
        self.logger.debug(c, &format!("AFC_M104/M109 raw cmd: {}", gcmd.get_commandline()));
        let toolnum = gcmd.get_int_opt("T", Lim::new().min(0))?;
        let temp = gcmd.get_float("S", Some(0.0), Lim::new())?;
        let deadband = gcmd.get_float_opt("D", Lim::new())?;
        let _snapmaker_param_a = gcmd.get_int_opt("A", Lim::new())?;
        let curr_extruder = self.function.get_current_extruder_obj()?;
        let extruder: Rc<dyn kaliru::toolhead::ExtraAxis> = match toolnum {
            Some(toolnum) => {
                let map = format!("T{}", toolnum);
                match self.function.get_lane_by_map(&map) {
                    Some(lane) => {
                        let extruder = lane.extruder_obj()?;
                        if !self.disable_ooze_check {
                            if let Some(curr) = curr_extruder.as_ref().filter(|e| e.lane_loaded.borrow().is_some()) {
                                let lanes: Vec<String> = curr.lanes.borrow().keys().cloned().collect();
                                for curr_extr_lane in lanes {
                                    if let Some(lane_obj) = self.lane(&curr_extr_lane) {
                                        let mapped = lane_obj.map.borrow().contains(&map);
                                        if curr.lane_loaded.borrow().as_deref() == Some(lane_obj.name.as_str()) && mapped {
                                            break;
                                        } else if mapped {
                                            self.logger.raw(
                                                c,
                                                &format!(
                                                    "<span class=warning--text>WARNING: Not setting temperature for {} since another lane is loaded for {}</span>",
                                                    map, curr.name
                                                ),
                                            );
                                            return Ok(());
                                        }
                                    }
                                }
                            }
                        }
                        self.logger.debug(c, &format!("Setting temperature for {} to {}", extruder.name, py_float_repr(temp)));
                        extruder.toolhead_extruder()?
                    }
                    None => {
                        self.logger.error(c, &format!("extruder not configured for T{}", toolnum));
                        return Ok(());
                    }
                }
            }
            None => self.toolhead()?.get_extruder(),
        };
        let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
        let heater = extruder.get_heater()?;
        pheaters.set_temperature(&heater, temp, false)?;
        if let (true, Some(deadband), true) = (wait, deadband, temp > 0.0) {
            self.wait_for_temp_within_tolerance(&heater, temp, Some(deadband))?;
            return Ok(());
        }
        let current_temp = heater.get_temp(self.reactor.monotonic()).0;
        let should_wait = wait && (current_temp - temp).abs() > self.temp_wait_tolerance;
        pheaters.set_temperature(&heater, temp, should_wait)?;
        self.logger.debug(c, "Done setting temp");
        Ok(())
    }

    /// Heats the next lane's extruder; None when there is no next lane
    fn heat_next_extruder(&self, wait: bool, next_temp: Option<f64>) -> KResult<Option<(Rc<AfcExtruder>, f64)>> {
        let c = &AFC_AFC__HEAT_NEXT_EXTRUDER;
        let next = self.next_lane_load.borrow().clone();
        let next_extruder = match next {
            Some(n) => self.lane(&n).ok_or_else(|| KError::Py("KeyError".into(), py_repr(&n)))?.extruder_obj()?,
            None => {
                self.error.afc_error(c, "Next lane load is None, cannot proceed with tool change", self.function.in_print()?, None)?;
                return Ok(None);
            }
        };
        let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
        let set_temp = match next_temp {
            Some(t) => t,
            None => self.toolhead()?.get_extruder().get_heater()?.get_temp(self.reactor.monotonic()).1,
        };
        let next_heater = next_extruder.get_heater()?;
        pheaters.set_temperature(&next_heater, set_temp, false)?;
        self.logger.info(c, &format!("Heating next extruder: {} to {}", next_extruder.name, py_float_repr(set_temp)));
        if wait && self.function.get_current_extruder()? != Some(next_extruder.name.clone()) {
            self.wait_for_temp_within_tolerance(&next_heater, set_temp, Some(next_extruder.deadband))?;
        }
        Ok(Some((next_extruder, set_temp)))
    }

    fn cooldown_last_extruder(&self, last_extruder: &AfcExtruder, is_infinite_runout: bool) -> KResult<()> {
        let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
        let last_heater = last_extruder.get_heater()?;
        let temperature = if is_infinite_runout {
            PyVal::Int(0)
        } else {
            PyVal::Float(kaliru::pyfmt::py_max(0.0, last_heater.target_temp.get() - last_extruder.toolchange_temp_drop))
        };
        self.logger.info(&AFC_AFC__COOLDOWN_LAST_EXTRUDER, &format!("Cooling down last extruder: {} to {}", last_extruder.name, temperature.str()));
        pheaters.set_temperature(&last_heater, temperature.as_f64().unwrap_or(0.0), false)
    }

    /// Waits until the heater is within tolerance/2 of the target
    pub fn wait_for_temp_within_tolerance(&self, heater: &Heater, target_temp: f64, tolerance: Option<f64>) -> KResult<()> {
        let Some(tolerance) = tolerance else { return Ok(()) };
        if target_temp <= 0.0 {
            return Ok(());
        }
        let min_temp = target_temp - (tolerance / 2.0);
        let max_temp = target_temp + (tolerance / 2.0);
        let mut eventtime = self.reactor.monotonic();
        while !self.printer.is_shutdown() {
            let (cur_temp, _) = heater.get_temp(eventtime);
            if min_temp <= cur_temp && cur_temp <= max_temp {
                return Ok(());
            }
            self.logger.debug(
                &AFC_AFC__WAIT_FOR_TEMP_WITHIN_TOLERANCE,
                &format!("{} temp: {:.2}C (waiting for {:.1}..{:.1})", heater.get_name(), cur_temp, min_temp, max_temp),
            );
            eventtime = self.reactor.pause(eventtime + 1.0)?;
        }
        Ok(())
    }

    pub(super) fn cmd_turn_off_afc_led(self: &Rc<Self>, _gcmd: &GCodeCommand) -> KResult<()> {
        self.led_state.set(false);
        let leds: Vec<_> = self.led_obj.borrow().values().cloned().collect();
        for led in leds {
            led.turn_off_leds()?;
        }
        Ok(())
    }

    pub(super) fn cmd_turn_on_afc_led(self: &Rc<Self>, _gcmd: &GCodeCommand) -> KResult<()> {
        self.led_state.set(true);
        let leds: Vec<_> = self.led_obj.borrow().values().cloned().collect();
        for led in leds {
            led.turn_on_leds()?;
        }
        Ok(())
    }

    pub(super) fn cmd_afc_stats(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let short = gcmd.get_int("SHORT", Some(self.short_stats as i64), Lim::new())? != 0;
        self.afc_stats()?.print_stats(self, short)
    }

    pub(super) fn cmd_afc_change_blade(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_AFC_CMD_AFC_CHANGE_BLADE;
        let extruder = gcmd.get_or("EXTRUDER", "extruder");
        let ext = self.tools.borrow().get(&extruder).cloned();
        match ext {
            Some(e) => {
                e.estats.last_blade_changed().set_current_time()?;
                e.estats.cut_total_since_changed().reset_count()?;
                self.logger.info(c, &format!("Cutter blade stats reset for {}", extruder));
            }
            None => self.logger.error(c, &format!("{} is not a valid extruder name", extruder)),
        }
        Ok(())
    }

    pub(super) fn cmd_afc_reset_stats(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_AFC_CMD_AFC_RESET_STATS;
        let extruder = gcmd.get_opt("EXTRUDER");
        let lane = gcmd.get_opt("LANE");
        let reset_lane = |l: &AfcLane| -> KResult<()> {
            l.lane_load_count()?.reset_count()?;
            self.logger.info(&AFC_AFC_CMD_AFC_RESET_STATS_RESET_LANE, &format!("Lane stats reset for {}", l.name));
            Ok(())
        };
        let reset_extruder = |e: &AfcExtruder| -> KResult<()> {
            e.estats.reset_stats()?;
            self.logger.info(&AFC_AFC_CMD_AFC_RESET_STATS_RESET_EXTRUDER, &format!("Extruder stats reset for {}", e.name));
            Ok(())
        };
        let given = |v: &Option<String>| v.as_deref().is_some_and(|s| !s.is_empty());
        if (given(&extruder) || given(&lane))
            && !self.db_backup.get() {
                let moonraker = self.moonraker.borrow().clone().ok_or_else(|| no_attr("NoneType", "trigger_db_backup"))?;
                if moonraker.trigger_db_backup() {
                    return Ok(());
                }
                self.db_backup.set(true);
            }
        if let Some(extruder) = extruder.as_deref().filter(|s| !s.is_empty()) {
            if extruder.to_lowercase().contains("all") {
                let tools: Vec<Rc<AfcExtruder>> = self.tools.borrow().values().cloned().collect();
                for e in tools {
                    reset_extruder(&e)?;
                }
                let stats = self.afc_stats()?;
                stats.average_tool_load_time.reset_count()?;
                stats.reset_average_times();
            } else {
                let e = self.tools.borrow().get(extruder).cloned();
                match e {
                    Some(e) => reset_extruder(&e)?,
                    None => self.logger.error(c, &format!("{} is not a valid extruder name", extruder)),
                }
            }
        }
        if let Some(lane) = lane.as_deref().filter(|s| !s.is_empty()) {
            if lane.to_lowercase().contains("all") {
                let lanes: Vec<Rc<AfcLane>> = self.lanes.borrow().values().cloned().collect();
                for l in lanes {
                    reset_lane(&l)?;
                }
            } else {
                match self.lane(lane) {
                    Some(l) => reset_lane(&l)?,
                    None => self.logger.error(c, &format!("{} is not a valid lane name", lane)),
                }
            }
        }
        if extruder.is_none() && lane.is_none() {
            self.logger.info(c, "A valid EXTRUDER or LANE needs to be specified to reset");
        }
        Ok(())
    }

    pub(super) fn cmd_afc_clear_message(self: &Rc<Self>, _gcmd: &GCodeCommand) -> KResult<()> {
        self.get_message(true);
        Ok(())
    }

    pub(super) fn cmd_afc_test_messages(self: &Rc<Self>, _gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_AFC_CMD__AFC_TEST_MESSAGES;
        self.logger.error(c, "Test Message 1");
        self.logger.error(c, "Test Message 2");
        self.logger.error(c, "Test Message 3");
        Ok(())
    }
}
