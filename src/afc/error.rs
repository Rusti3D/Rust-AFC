// AFC error handling: reporting, pausing and resuming (port of AFC_error.py)
//
// Copyright (C) 2024-2026 Armored Turtle
// Copyright (C) 2026 AFCProject
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::lane::{AfcLane, LaneState, MoveDirection, SpeedMode};
use super::logger::Code;
use super::{py_bool, Afc, State};
use kaliru::configfile::ConfigWrapper;
use kaliru::error::KResult;
use kaliru::extras::idle_timeout::IdleTimeout;
use kaliru::extras::pause_resume::PauseResume;
use kaliru::gcode::GCodeCommand;
use kaliru::logging;
use kaliru::printer::{PObj, Printer, PrinterObject};
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub const BASE_RESUME_NAME: &str = "RESUME";
pub const AFC_RENAME_RESUME_NAME: &str = "_AFC_RENAMED_RESUME_";
pub const BASE_PAUSE_NAME: &str = "PAUSE";
pub const AFC_RENAME_PAUSE_NAME: &str = "_AFC_RENAMED_PAUSE_";

pub const CMD_RESET_FAILURE_HELP: &str = "CLEAR STATUS ERROR";
pub const CMD_AFC_RESUME_HELP: &str = "Clear error state and restores position before resuming the print";

/// fix()'s lane: the object, or a lane name to look up
pub enum LaneRef {
    Lane(Rc<AfcLane>),
    Name(String),
}

/// [AFC_error]
pub struct AfcError {
    me: Weak<AfcError>,
    printer: Rc<Printer>,
    afc: RefCell<Weak<Afc>>,
    pause_resume: RefCell<Option<Rc<PauseResume>>>,
    pub pause: Cell<bool>,
    error_timeout: Cell<f64>,
    idle_timeout_val: Cell<f64>,
}

impl AfcError {
    fn new(config: &ConfigWrapper) -> Rc<AfcError> {
        let printer = config.get_printer();
        let e = Rc::new_cyclic(|me| AfcError {
            me: me.clone(),
            printer: printer.clone(),
            afc: RefCell::new(Weak::new()),
            pause_resume: RefCell::new(None),
            pause: Cell::new(false),
            error_timeout: Cell::new(0.0),
            idle_timeout_val: Cell::new(0.0),
        });
        let ee = e.clone();
        kaliru::events::KlippyConnect::on(&printer, move || ee.handle_connect());
        e
    }

    fn afc(&self) -> Rc<Afc> {
        self.afc.borrow().upgrade().expect("AFC")
    }

    pub fn pause_resume(&self) -> KResult<Rc<PauseResume>> {
        self.pause_resume.borrow().clone().ok_or_else(|| super::no_attr("afcError", "pause_resume"))
    }

    fn handle_connect(&self) -> KResult<()> {
        let afc = self.printer.lookup::<Afc>("AFC")?;
        *self.afc.borrow_mut() = Rc::downgrade(&afc);
        *self.pause_resume.borrow_mut() = Some(self.printer.lookup::<PauseResume>("pause_resume")?);
        self.error_timeout.set(afc.error_timeout);
        let idle = self.printer.lookup::<IdleTimeout>("idle_timeout")?;
        self.idle_timeout_val.set(idle.idle_timeout.get());
        let me = self.me.upgrade().expect("AFC_error");
        let m = me.clone();
        afc.gcode.register_command("RESET_FAILURE", Some(Rc::new(move |_| m.reset_failure_cmd())), false, Some(CMD_RESET_FAILURE_HELP))?;
        let m = me.clone();
        afc.gcode.register_command("AFC_RESUME", Some(Rc::new(move |g| m.cmd_afc_resume(g))), false, Some(CMD_AFC_RESUME_HELP))?;
        Ok(())
    }

    /// Tries to resolve a reported problem; pauses for the user when it can't
    pub fn fix(&self, problem: Option<&str>, lane: Option<LaneRef>) -> KResult<bool> {
        self.pause.set(true);
        let mut error_handled = false;
        let lane = match lane {
            Some(LaneRef::Name(name)) => match self.afc().lane(&name) {
                Some(l) => Some(l),
                None => {
                    self.pause_user_intervention(&format!("Unknown lane '{}' reported for problem: {}", name, super::py_opt_str(problem)))?;
                    return Ok(error_handled);
                }
            },
            Some(LaneRef::Lane(l)) => Some(l),
            None => None,
        };
        match (problem, &lane) {
            (None, _) => self.pause_user_intervention("Paused for unknown error")?,
            (Some("toolhead"), Some(l)) => error_handled = self.tool_head_fix(l)?,
            (Some(p), _) => self.pause_user_intervention(p)?,
        }
        if !error_handled {
            if let Some(l) = &lane {
                l.unit_obj()?.lane_fault(l)?;
            }
        }
        Ok(error_handled)
    }

    /// Retracts or reloads a lane to its load sensor after a toolhead fault
    fn tool_head_fix(&self, cur_lane: &Rc<AfcLane>) -> KResult<bool> {
        let afc = self.afc();
        if cur_lane.get_toolhead_pre_sensor_state()? {
            if cur_lane.extruder_obj()?.lane_loaded.borrow().as_deref() == Some(cur_lane.name.as_str()) {
                if !cur_lane.raw_load_state() {
                    self.pause_user_intervention("Filament not loaded in Lane")?;
                } else {
                    self.pause_user_intervention("no error detected")?;
                }
            } else {
                self.pause_user_intervention("laneloaded does not match extruder")?;
            }
        } else {
            let failed_to_retract_msg = format!("Failed to retract {} to load sensor", cur_lane.name);
            if cur_lane.raw_load_state() && !cur_lane.is_direct_hub() && cur_lane.extruder_obj()?.tool_start.as_deref() != Some("buffer") {
                afc.logger.info(&AFC_ERROR_AFCERROR_TOOLHEADFIX, &format!("Retracting {} back to load switch", cur_lane.name));
                if afc.homing_enabled {
                    let mut num_tries = 0;
                    while cur_lane.raw_load_state() {
                        let mut total_move_dist = cur_lane.dist_hub.get() + 500.0;
                        if let Some(h) = cur_lane.hub_obj_opt() {
                            total_move_dist += h.afc_bowden_length.get();
                        }
                        cur_lane.unit_obj()?.move_to_load(cur_lane, total_move_dist, MoveDirection::Neg, true, SpeedMode::Short)?;
                        num_tries += 1;
                        if num_tries >= 5 {
                            self.pause_user_intervention(&failed_to_retract_msg)?;
                            return Ok(false);
                        }
                    }
                } else {
                    let mut max_length = 5000;
                    while cur_lane.raw_load_state() {
                        cur_lane.move_lane(-5.0, afc.short_moves_speed, afc.short_moves_accel, true)?;
                        if max_length > 0 {
                            max_length -= 5;
                        } else {
                            self.pause_user_intervention(&failed_to_retract_msg)?;
                            return Ok(false);
                        }
                    }
                    max_length = 1000;
                    while !cur_lane.raw_load_state() {
                        cur_lane.move_lane(5.0, afc.short_moves_speed, afc.short_moves_accel, true)?;
                        if max_length > 0 {
                            max_length -= 5;
                        } else {
                            self.pause_user_intervention(&format!("Failed to move back {} to load sensor", cur_lane.name))?;
                            return Ok(false);
                        }
                    }
                }
                cur_lane.set_tool_unloaded(false)?;
                cur_lane.loaded_to_hub.set(false);
                cur_lane.unit_obj()?.prep_load(cur_lane)?;
                cur_lane.unit_obj()?.prep_post_load(cur_lane)?;
                afc.save_vars()?;
                self.pause.set(false);
                afc.logger.info(&AFC_ERROR_AFCERROR_TOOLHEADFIX, &format!("Done resetting {}", cur_lane.name));
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Logs the error; pauses when homed, not paused and a pause is wanted
    pub fn pause_user_intervention(&self, message: &str) -> KResult<()> {
        let afc = self.afc();
        afc.logger.error(&AFC_ERROR_AFCERROR_PAUSEUSERINTERVENTION, message);
        if afc.function.is_homed(true)? && !afc.function.is_paused()? {
            afc.save_pos()?;
            if self.pause.get() {
                self.pause_print()?;
            }
        }
        Ok(())
    }

    /// Sets the error state and runs PAUSE
    pub fn pause_print(&self) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_ERROR_AFCERROR_PAUSE_PRINT;
        self.set_error_state(true)?;
        afc.logger.info(c, "PAUSING");
        afc.gcode.run_script_from_command("PAUSE")?;
        afc.logger.debug(c, "After User Pause");
        afc.function.log_toolhead_pos("")
    }

    /// The error state (the position is saved on the first error)
    pub fn set_error_state(&self, state: bool) -> KResult<()> {
        let afc = self.afc();
        logging::warning(&format!("AFC debug: setting error state {}", py_bool(state)));
        if state && !afc.error_state.get() {
            afc.save_pos()?;
        }
        afc.error_state.set(state);
        afc.current_state.set(if state { State::Error } else { State::Idle });
        Ok(())
    }

    /// AFC_error(msg, pause, stack_name): stack_name defaults to the caller's
    /// function name
    pub fn afc_error(&self, caller: &Code, msg: &str, pause: bool, stack_name: Option<&str>) -> KResult<()> {
        let afc = self.afc();
        // Print to logger since respond_raw does not write to logger
        logging::warning(msg);
        let stack_name = stack_name.unwrap_or(caller.func);
        afc.logger.error_opt(&AFC_ERROR_AFCERROR_AFC_ERROR, msg, None, stack_name);
        if pause {
            self.pause_print()?;
        }
        Ok(())
    }

    fn reset_failure_cmd(&self) -> KResult<()> {
        self.reset_failure()
    }

    /// error_state, pause, position_saved and in_toolchange back to False
    pub fn reset_failure(&self) -> KResult<()> {
        let afc = self.afc();
        afc.logger.debug(&AFC_ERROR_AFCERROR_RESET_FAILURE, "Resetting failures");
        self.set_error_state(false)?;
        self.pause.set(false);
        afc.position_saved.set(false);
        afc.in_toolchange.set(false);
        Ok(())
    }

    fn state_debug(&self, what: &str) -> KResult<String> {
        let afc = self.afc();
        Ok(format!(
            "{}-Error State: {}, Is Paused {}, Position_saved {}, in toolchange: {}",
            what,
            py_bool(afc.error_state.get()),
            py_bool(afc.function.is_paused()?),
            py_bool(afc.position_saved.get()),
            py_bool(afc.in_toolchange.get())
        ))
    }

    /// AFC_RESUME (RESUME after PREP renamed it): clears the error state,
    /// runs the user's RESUME and restores the saved position
    pub fn cmd_afc_resume(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_ERROR_AFCERROR_CMD_AFC_RESUME;
        afc.in_toolchange.set(false);
        if !afc.function.is_paused()? {
            afc.logger.debug(c, "AFC_RESUME: Printer not paused, not executing resume code");
            return Ok(());
        }
        let temp_is_paused = afc.function.is_paused()?;
        afc.function.check_absolute_mode("AFC_RESUME")?;
        let move_z_pos = afc.last_gcode_position.borrow()[2] + afc.z_hop;
        let last_z = afc.gcode_move()?.last_position.borrow()[2];
        if afc.function.is_homed(true)? && last_z <= move_z_pos {
            afc.move_z_pos(move_z_pos, "AFC_RESUME", false)?;
        } else {
            afc.logger.debug(
                c,
                &format!("AFC_RESUME: not moving in z cur_pos:{} move_z_pos:{}", afc.gcode_move_last_position_repr()?, kaliru::pyfmt::py_float_repr(move_z_pos)),
            );
        }
        afc.logger.debug(c, "AFC_RESUME: Before User Restore");
        afc.function.log_toolhead_pos("")?;
        afc.gcode.run_script_from_command(&format!("{} {}", AFC_RENAME_RESUME_NAME, gcmd.get_raw_command_parameters()))?;
        if afc.error_state.get() || temp_is_paused || afc.position_saved.get() {
            self.set_error_state(false)?;
            if afc.function.is_homed(true)? {
                afc.restore_pos(false)?;
            }
            self.pause.set(false);
        }
        afc.logger.debug(c, &self.state_debug("RESUME")?);
        Ok(())
    }

    /// AFC_PAUSE (PAUSE after PREP renamed it): pauses, lifts z by z_hop,
    /// then runs the user's PAUSE
    pub fn cmd_afc_pause(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_ERROR_AFCERROR_CMD_AFC_PAUSE;
        if !afc.function.is_paused()? {
            afc.logger.debug(c, "AFC_PAUSE: Pausing");
            afc.save_pos()?;
            self.pause_resume()?.send_pause_command()?;
            afc.function.check_absolute_mode("AFC_PAUSE")?;
            let move_z_pos = afc.last_gcode_position.borrow()[2] + afc.z_hop;
            let last_z = afc.gcode_move()?.last_position.borrow()[2];
            if afc.function.is_homed(true)? && last_z <= move_z_pos {
                afc.move_z_pos(move_z_pos, "AFC_PAUSE", false)?;
            } else {
                afc.logger.debug(
                    c,
                    &format!("AFC_PAUSE: not moving in z cur_pos:{} move_z_pos:{}", afc.gcode_move_last_position_repr()?, kaliru::pyfmt::py_float_repr(move_z_pos)),
                );
            }
            afc.gcode.run_script_from_command(&format!("{} {}", AFC_RENAME_PAUSE_NAME, gcmd.get_raw_command_parameters()))?;
            let timeout_to_use = kaliru::pyfmt::py_max(self.error_timeout.get(), self.idle_timeout_val.get());
            afc.gcode.run_script_from_command(&format!("SET_IDLE_TIMEOUT TIMEOUT={}", kaliru::pyfmt::py_float_repr(timeout_to_use)))?;
        } else {
            afc.logger.debug(c, "AFC_PAUSE: Not Pausing");
        }
        afc.logger.debug(c, &self.state_debug("PAUSE")?);
        Ok(())
    }

    /// Stops the lane, marks it errored and reports (pausing when asked)
    pub fn handle_lane_failure(&self, caller: &Code, cur_lane: &Rc<AfcLane>, message: &str, pause: bool) -> KResult<()> {
        cur_lane.do_enable(false)?;
        cur_lane.set_status(LaneState::Error);
        let msg = format!("{} {}", cur_lane.name, message);
        self.afc_error(&AFC_ERROR_AFCERROR_HANDLE_LANE_FAILURE, &msg, pause, Some(caller.func))?;
        cur_lane.unit_obj()?.lane_fault(cur_lane)
    }
}

impl PrinterObject for AfcError {}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(AfcError::new(config))
}

