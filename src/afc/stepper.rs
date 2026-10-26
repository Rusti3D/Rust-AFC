// An AFC lane with its own stepper (port of AFC_stepper.py: AFCExtruderStepper,
// the lane part; the AFCLane part is lane.rs)
//
// Copyright (C) 2024-2026 Armored Turtle
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::lane::{AfcLane, Endstop, HOMING_SELECTOR, VALID_DIRECT_HUB};
use kaliru::chelper::itersolve::SkRef;
use kaliru::chelper::kin_cartesian::cartesian_stepper_alloc;
use kaliru::chelper::trapq::TrapQRef;
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::extras::force_move::calc_move_time;
use kaliru::extras::homing::{HomingToolhead, PrinterHoming};
use kaliru::extras::motion_queuing::PrinterMotionQueuing;
use kaliru::extras::query_endstops::QueryEndstops;
use kaliru::extras::stepper_enable::PrinterStepperEnable;
use kaliru::gcode::GCodeCommand;
use kaliru::kinematics::extruder::ExtruderStepper;
use kaliru::mcu::EndstopRef;
use kaliru::pins::PinsObject;
use kaliru::pyfmt::{py_float_repr, py_max};
use kaliru::pyval::PyVal;
use kaliru::reactor::ReactorCompletion;
use kaliru::stepper::McuStepper;
use kaliru::toolhead::ToolHead;
use kaliru::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const CMD_AFC_STEPPER_HOME_HELP: &str = "Command a manually home stepper to specified endstop";

/// The stepper part of an [AFC_stepper] lane (AFCExtruderStepper's own state)
pub struct StepperLane {
    pub extruder_stepper: Rc<ExtruderStepper>,
    /// the lane whose assist_move runs while it drives (python swaps the
    /// drive stepper's assist_move for the lane's around a homing move)
    assist_override: RefCell<Option<Weak<AfcLane>>>,
    stepper_enable: Rc<PrinterStepperEnable>,
    synced_to_extruder: Cell<bool>,
    print_current_set: Cell<bool>,
    default_homing_endstop: String,
    homing_accel: Cell<f64>,
    manual_axis_pos: Cell<f64>,
    /// _endstops: endstop key -> (MCU endstop, registered name)
    pub endstops: RefCell<IndexMap<String, (EndstopRef, String)>>,
    motion_queuing: Rc<PrinterMotionQueuing>,
    next_cmd_time: Cell<f64>,
    stepper_kinematics: SkRef,
    trapq: TrapQRef,
    tmc_print_current: Option<f64>,
    tmc_load_current: Option<f64>,
    base_rotation_dist: f64,
}

fn stepper_of(lane: &AfcLane) -> KResult<Rc<StepperLane>> {
    lane.stepper.borrow().clone().ok_or_else(|| KError::Other(format!("'AFCLane' object has no attribute 'extruder_stepper' ({})", lane.name)))
}

fn toolhead(lane: &AfcLane) -> KResult<Rc<ToolHead>> {
    lane.printer.lookup::<ToolHead>("toolhead")
}

/// [AFC_stepper <name>]: an AFC lane (AFCLane.__init__) and its stepper
pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<kaliru::printer::PObj> {
    let lane = AfcLane::new(config, false)?;
    let st = StepperLane::new(&lane, config)?;
    *lane.stepper.borrow_mut() = Some(st);
    Ok(lane)
}

impl StepperLane {
    /// AFCExtruderStepper.__init__ after AFCLane.__init__
    fn new(lane: &Rc<AfcLane>, config: &ConfigWrapper) -> KResult<Rc<StepperLane>> {
        let printer = config.get_printer();
        let extruder_stepper = ExtruderStepper::new(config)?;
        let stepper_enable = printer.lookup::<PrinterStepperEnable>("stepper_enable")?;
        // Homing-related configuration
        let default_homing_endstop = config.get("default_homing_endstop", "load")?;
        // Optional explicit hub endstop pin override in stepper section
        let hub_endstop = config.get_opt("hub_endstop")?;
        // Additional homing endstops to add to stepper, comma-seperated list (a set)
        let mut extra_homing_pins: Vec<String> = vec![];
        for p in config.getlist("homing_endstop_pins", "", ",", None)?.unwrap_or_default() {
            if !extra_homing_pins.contains(&p) {
                extra_homing_pins.push(p);
            }
        }
        // Pre-create MCU endstops now so MCU config callbacks run before ready
        let endstops = init_endstops(lane, config, &extruder_stepper.stepper, hub_endstop.as_deref(), &extra_homing_pins)?;

        // Register AFC_HOME mux command for this lane name
        let stepper_name = config.get_name().split_whitespace().last().unwrap_or("").to_string();
        let l = lane.clone();
        lane.afc.function.register_mux_command(
            lane.afc.show_macros,
            "AFC_STEPPER_HOME",
            "STEPPER",
            &stepper_name,
            Rc::new(move |g| cmd_afc_stepper_home(&l, g)),
            CMD_AFC_STEPPER_HOME_HELP,
            Some(&[
                ("STEPPER", "string", "lane1"),
                ("ENABLE", "int", "1"),
                ("SPEED", "float", "100"),
                ("ACCEL", "float", "100"),
                ("STOP_ON_ENDSTOP", "int", "1"),
                ("DIST", "float", "100"),
                ("ENDSTOP", "string", "load"),
                ("SYNC", "int", "1"),
            ]),
        )?;

        // Check for Klipper new motion queuing update
        let motion_queuing = printer.load_as::<PrinterMotionQueuing>(config, "motion_queuing")?;
        let stepper_kinematics = cartesian_stepper_alloc('x').expect("axis");
        let trapq = motion_queuing.allocate_trapq();

        // Current to use while printing, set to a lower current to reduce
        // stepper heat when printing. Defaults to global_print_current, if not
        // specified current is not changed.
        let tmc_print_current = match lane.afc.global_print_current {
            Some(d) => Some(config.getfloat("print_current", d, Lim::new())?),
            None => config.getfloat_opt("print_current", Lim::new())?,
        };
        let mut tmc_load_current = None;
        if tmc_print_current.is_some() {
            tmc_load_current = get_tmc_values(lane, config)?;
        }
        // Get and save base rotation dist
        let base_rotation_dist = extruder_stepper.stepper.get_rotation_distance().0;
        Ok(Rc::new(StepperLane {
            assist_override: RefCell::new(None),
            extruder_stepper,
            stepper_enable,
            synced_to_extruder: Cell::new(false),
            print_current_set: Cell::new(false),
            default_homing_endstop,
            homing_accel: Cell::new(0.0),
            manual_axis_pos: Cell::new(0.0),
            endstops: RefCell::new(endstops),
            motion_queuing,
            next_cmd_time: Cell::new(0.0),
            stepper_kinematics,
            trapq,
            tmc_print_current,
            tmc_load_current,
            base_rotation_dist,
        }))
    }

    pub fn stepper(&self) -> &Rc<McuStepper> {
        &self.extruder_stepper.stepper
    }
}

/// _get_tmc_values: the run current of the lane's TMC driver section
fn get_tmc_values(lane: &AfcLane, config: &ConfigWrapper) -> KResult<Option<f64>> {
    let name = config.get_name().to_string();
    let sections = config.fileconfig.borrow().sections();
    let Some(section) = sections.iter().find(|s| s.contains("tmc") && s.contains(&name)) else {
        let mut msg = format!("Could not find TMC for stepper {},", lane.name);
        msg += "\nplease add TMC section or disable 'print_current' from config files";
        return Err(KError::command(msg));
    };
    Ok(Some(config.getsection(section).getfloat("run_current", kaliru::configfile::Dflt::Required, Lim::new())?))
}

/// _inherit_from_unit: a key of the lane's unit section
fn inherit_from_unit(lane: &AfcLane, config: &ConfigWrapper, target_key: &str) -> Option<String> {
    let unit = &lane.unit;
    if unit.is_empty() {
        return None;
    }
    const IGNORE_PREFIXES: [&str; 7] = ["AFC_hub ", "AFC_extruder ", "AFC_buffer ", "AFC_stepper ", "AFC_functions ", "AFC_stats ", "AFC_respond "];
    let sections = config.fileconfig.borrow().sections();
    for sec in &sections {
        if !sec.starts_with("AFC_") || IGNORE_PREFIXES.iter().any(|p| sec.starts_with(p)) {
            continue;
        }
        // Match on final token == unit name
        if sec.split_whitespace().last() == Some(unit.as_str()) {
            let value = match config.getsection(sec).get_opt(target_key) {
                Ok(v) => v,
                Err(e) => {
                    lane.afc.logger.debug(&AFC_STEPPER_AFCEXTRUDERSTEPPER__INHERIT_FROM_UNIT, &format!("_inherit_from_unit({}) failed: {}", target_key, e.message));
                    return None;
                }
            };
            if let Some(v) = &value {
                lane.afc.logger.debug(
                    &AFC_STEPPER_AFCEXTRUDERSTEPPER__INHERIT_FROM_UNIT,
                    &format!("Inherited '{}'='{}' from section '{}' for unit '{}' {}", target_key, v, sec, unit, lane.name),
                );
            }
            return value;
        }
    }
    None
}

/// _get_section_value: a key of '<prefix> <name>', or None
fn get_section_value(lane: &AfcLane, config: &ConfigWrapper, prefix: &str, name: Option<&str>, key: &str) -> Option<String> {
    let name = name.filter(|n| !n.is_empty())?;
    let section = format!("{} {}", prefix, name);
    match config.getsection(&section).get_opt(key) {
        Ok(v) => v,
        Err(e) => {
            lane.afc.logger.debug(&AFC_STEPPER_AFCEXTRUDERSTEPPER__GET_SECTION_VALUE, &format!("Missing or invalid section '{}': {}", section, e.message));
            None
        }
    }
}

/// _init_endstops: the stepper's endstops, created at config time
fn init_endstops(lane: &Rc<AfcLane>, config: &ConfigWrapper, stepper: &Rc<McuStepper>, hub_endstop: Option<&str>, extra_pins: &[String]) -> KResult<IndexMap<String, (EndstopRef, String)>> {
    let printer = &lane.printer;
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let qes = printer.lookup_or_load::<QueryEndstops>(config, "query_endstops")?;
    let mut endstops = IndexMap::default();
    let mut add = |key: &str, pin: Option<&str>, suffix: &str| add_endstop(lane, &ppins, &qes, stepper, &mut endstops, key, pin, suffix);

    // Built-ins from AFCLane config
    add("load", lane.load.as_deref(), "load");
    // Extra raw pins declared in config
    for raw_pin in extra_pins {
        let r = raw_pin.trim();
        if r.is_empty() {
            continue;
        }
        // Derive a suffix from the pin token for display; keep key as exact raw string
        let token = r.split(':').next_back().unwrap_or(r);
        let suffix = token.replace('/', "_").replace(['^', '!'], "");
        add(r, Some(r), &suffix);
    }

    // Hub endstop (register at config time so MCU setup is complete)
    let hub_pin = match hub_endstop {
        Some(p) => Some(p.to_string()),
        None => {
            let mut hub_name = lane.hub.borrow().clone();
            if hub_name.as_deref().is_none_or(|h| h.is_empty() || VALID_DIRECT_HUB.contains(&h)) {
                hub_name = inherit_from_unit(lane, config, "hub");
            }
            get_section_value(lane, config, "AFC_hub", hub_name.as_deref(), "switch_pin")
        }
    };

    // Toolhead endstops from AFC_extruder (tool_start/tool_end)
    let afc_extruder_name = lane.afc_extruder_name.borrow().clone().filter(|n| !n.is_empty()).or_else(|| inherit_from_unit(lane, config, "extruder"));
    let tool_start_pin = get_section_value(lane, config, "AFC_extruder", afc_extruder_name.as_deref(), "pin_tool_start");
    let tool_end_pin = get_section_value(lane, config, "AFC_extruder", afc_extruder_name.as_deref(), "pin_tool_end");

    // Buffer endstops from AFC_buffer (advance/trailing), inherit from extruder or unit if needed
    let mut buffer_name = lane.buffer_name.borrow().clone().filter(|n| !n.is_empty());
    if buffer_name.is_none() {
        buffer_name = get_section_value(lane, config, "AFC_extruder", afc_extruder_name.as_deref(), "buffer")
            .filter(|n| !n.is_empty())
            .or_else(|| inherit_from_unit(lane, config, "buffer"));
    }
    let buffer_adv_pin = get_section_value(lane, config, "AFC_buffer", buffer_name.as_deref(), "advance_pin");
    let buffer_trail_pin = get_section_value(lane, config, "AFC_buffer", buffer_name.as_deref(), "trailing_pin");
    let fps_psf_buffer = get_section_value(lane, config, "AFC_buffer", buffer_name.as_deref(), "type");
    let is_fps_psf_buffer = fps_psf_buffer.as_deref() == Some("FPS_PSF");

    // Check to verify that hub is not a virtual sensor
    if let Some(hub_pin) = hub_pin.as_deref().filter(|p| !p.is_empty() && p.to_lowercase() != "virtual") {
        add("hub", Some(hub_pin), "hub");
    }
    if tool_start_pin.as_deref().is_some_and(|p| p.to_lowercase() == "virtual") {
        // Virtual toolhead sensor: no hardware pin to register an endstop for
    } else if tool_start_pin.as_deref() != Some("buffer") {
        add("tool_start", tool_start_pin.as_deref(), "tool_start");
    } else if buffer_adv_pin.is_some() {
        add("tool_start", buffer_adv_pin.as_deref(), "tool_start");
    } else if is_fps_psf_buffer {
    } else {
        let mut error_string = format!("Error: buffer set as pin_tool_start in [AFC_extruder {}] config section,", afc_extruder_name.as_deref().unwrap_or("None"));
        error_string += &format!(" but [AFC_buffer {}] is not found in config. Please make sure config ", buffer_name.as_deref().unwrap_or("None"));
        error_string += &format!("section exists or remove `buffer: {}` variable from your extruder config section.", buffer_name.as_deref().unwrap_or("None"));
        return Err(KError::config(error_string));
    }
    add("tool_end", tool_end_pin.as_deref(), "tool_end");
    add("buffer_advance", buffer_adv_pin.as_deref(), "buffer_adv");
    add("buffer_trailing", buffer_trail_pin.as_deref(), "buffer_trailing");
    Ok(endstops)
}

/// _add_endstop: the pin as an endstop on the lane's stepper
#[allow(clippy::too_many_arguments)]
fn add_endstop(
    lane: &AfcLane,
    ppins: &PinsObject,
    qes: &QueryEndstops,
    stepper: &Rc<McuStepper>,
    endstops: &mut IndexMap<String, (EndstopRef, String)>,
    key: &str,
    pin: Option<&str>,
    suffix: &str,
) {
    let c = &AFC_STEPPER_AFCEXTRUDERSTEPPER__ADD_ENDSTOP;
    let logger = &lane.afc.logger;
    let Some(pin) = pin else {
        logger.info(c, &format!("Pin for {} is none for {}", key, lane.name));
        return;
    };
    // Normalize and create endstop
    let made = (|| -> KResult<EndstopRef> {
        let pin_params = {
            let mut pp = ppins.0.borrow_mut();
            pp.allow_multi_use_pin(pin.trim_matches(|c| c == '!' || c == '^')).map_err(|e| KError::Pins(e.0))?;
            pp.parse_pin(pin, true, true).map_err(|e| KError::Pins(e.0))?;
            pp.lookup_pin(pin, true, true, None).map_err(|e| KError::Pins(e.0))?
        };
        let chip = pin_params.borrow().chip.clone();
        chip.setup_pin("endstop", &pin_params)
            .map_err(KError::Pins)
            .and_then(|o| kaliru::mcu::any_to_endstop(o).ok_or_else(|| KError::Other(format!("{} is not an endstop", pin))))
    })();
    let mcu_endstop = match made {
        Ok(e) => e,
        Err(e) => {
            logger.info(c, &format!("Error parsing pin for {} is none for {}", key, lane.name));
            logger.info(c, e.message());
            return;
        }
    };
    let name = if ["hub", "tool_start", "tool_end", "buffer_advance", "buffer_trailing"].contains(&key) {
        suffix.to_string()
    } else {
        format!("{}_{}", lane.name, suffix)
    };
    qes.register_endstop(mcu_endstop.clone(), &name);
    if mcu_endstop.add_stepper(stepper).is_err() {
        logger.info(c, &format!("Error when registering stepper {}", lane.name));
    }
    logger.debug(c, &format!("{} adding endstop {}:{}:{}", lane.name, key, name, pin));
    endstops.insert(key.to_string(), (mcu_endstop, name));
}

/// _get_steppers for a lane with its own stepper: the lane's endstops also
/// home its stepper
pub fn add_lane_endstops(lane: &AfcLane) -> KResult<()> {
    let st = stepper_of(lane)?;
    for es in lane.endstops.borrow().values() {
        if es.endstop_name.contains(HOMING_SELECTOR) {
            continue;
        }
        es.endstop.add_stepper(st.stepper())?;
        st.endstops.borrow_mut().insert(es.endstop_name.clone(), (es.endstop.clone(), es.endstop_name.clone()));
    }
    Ok(())
}

/// _get_steppers for a lane driven by its unit's drive stepper: the lane's
/// endstops (but its selector) home the drive stepper
pub fn add_endstops_to_drive(drive: &AfcLane, lane: &AfcLane) -> KResult<()> {
    let st = stepper_of(drive)?;
    for es in lane.endstops.borrow().values() {
        if es.endstop_name.contains(HOMING_SELECTOR) {
            continue;
        }
        es.endstop.add_stepper(st.stepper())?;
        st.endstops.borrow_mut().insert(es.endstop_name.clone(), (es.endstop.clone(), es.endstop_name.clone()));
    }
    Ok(())
}

/// the lane's selector endstop homes the unit's selector stepper
pub fn add_selector_endstop(selector: &AfcLane, lane: &AfcLane) -> KResult<()> {
    let st = stepper_of(selector)?;
    if let Some(es) = lane.endstops.borrow().get(HOMING_SELECTOR) {
        es.endstop.add_stepper(st.stepper())?;
        st.endstops.borrow_mut().insert(es.endstop_name.clone(), (es.endstop.clone(), es.endstop_name.clone()));
    }
    Ok(())
}

/// _add_endstop with a pre-built endstop (an FPS buffer's software one)
pub fn add_prebuilt_endstop(lane: &AfcLane, key: &str, suffix: &str, endstop: EndstopRef) -> KResult<()> {
    let st = stepper_of(lane)?;
    let name = if ["hub", "tool_start", "tool_end", "buffer_advance", "buffer_trailing"].contains(&key) { suffix.to_string() } else { format!("{}_{}", lane.name, suffix) };
    let qes = lane.printer.lookup::<QueryEndstops>("query_endstops")?;
    qes.register_endstop(endstop.clone(), &name);
    if endstop.add_stepper(st.stepper()).is_err() {
        lane.afc.logger.info(&AFC_STEPPER_AFCEXTRUDERSTEPPER__ADD_ENDSTOP, &format!("Error when registering stepper {}", lane.name));
    }
    lane.afc.logger.debug(&AFC_STEPPER_AFCEXTRUDERSTEPPER__ADD_ENDSTOP, &format!("{} adding endstop {}:{}:None", lane.name, key, name));
    st.endstops.borrow_mut().insert(key.to_string(), (endstop, name));
    Ok(())
}

/// an endstop of the unit's (HTLF's home switch) on its selector stepper
pub fn add_named_endstop(selector: &AfcLane, endstop: &EndstopRef, name: &str) -> KResult<()> {
    let st = stepper_of(selector)?;
    endstop.add_stepper(st.stepper())?;
    st.endstops.borrow_mut().insert(name.to_string(), (endstop.clone(), name.to_string()));
    Ok(())
}

/// the lane whose assist_move runs for this stepper's moves
fn assist_lane(lane: &AfcLane) -> Option<Rc<AfcLane>> {
    lane.stepper.borrow().as_ref().and_then(|st| st.assist_override.borrow().as_ref().and_then(|w| w.upgrade()))
}

/// home_to on a drive stepper for a lane it drives: the lane's assist runs
#[allow(clippy::too_many_arguments)]
pub fn home_to_for(drive: &Rc<AfcLane>, lane: &AfcLane, endstop: &Endstop, distance: f64, speed: &PyVal, accel: &PyVal, triggered: bool, check_trigger: bool, assist_active: bool) -> KResult<(bool, f64, bool)> {
    let st = stepper_of(drive)?;
    *st.assist_override.borrow_mut() = Some(lane.me_weak());
    let r = home_to(drive, endstop, distance, speed, accel, triggered, check_trigger, assist_active);
    *st.assist_override.borrow_mut() = None;
    r
}

/// do_homing_move, called by a unit (ViViD's selector)
#[allow(clippy::too_many_arguments)]
pub fn do_homing_move_pub(lane: &Rc<AfcLane>, movepos: f64, speed: &PyVal, accel: &PyVal, endstop_spec: &str, triggered: bool, check_trigger: bool, assist_active: bool) -> KResult<(bool, f64)> {
    do_homing_move(lane, movepos, speed, accel, endstop_spec, triggered, check_trigger, assist_active)
}

/// _submit_move: the move from position 0 onto the lane's trapq; its duration
fn submit_move(st: &StepperLane, movetime: f64, distance: f64, speed: f64, accel: f64) -> f64 {
    st.stepper().set_position(&[0.0, 0.0, 0.0]);
    let (axis_r, accel_t, cruise_t, cruise_v) = calc_move_time(distance, speed, accel);
    st.trapq.borrow_mut().append(movetime, accel_t, cruise_t, accel_t, 0.0, 0.0, 0.0, axis_r, 0.0, 0.0, 0.0, cruise_v, accel);
    accel_t + cruise_t + accel_t
}

/// _move: one move of the lane stepper (force_move.manual_move's way)
fn do_move(lane: &AfcLane, distance: f64, speed: f64, accel: f64, assist_active: bool, sync: bool) -> KResult<()> {
    let st = stepper_of(lane)?;
    if sync {
        sync_print_time(lane)?;
    }
    let assist = assist_lane(lane);
    let assist = assist.as_deref().unwrap_or(lane);
    assist.assist_move(speed, distance < 0.0, assist_active, || {
        // Code based off force_move.py manual_move function
        let toolhead = toolhead(lane)?;
        toolhead.flush_step_generation()?;
        let prev_sk = st.stepper().set_stepper_kinematics(Some(st.stepper_kinematics.clone()));
        let prev_trapq = st.stepper().set_trapq(Some(st.trapq.clone()));
        let mut print_time = toolhead.get_last_move_time()?;

        let dwell_time = submit_move(&st, print_time, distance, speed, accel);
        print_time += dwell_time;
        st.motion_queuing.note_mcu_movequeue_activity(print_time, true);
        if sync {
            toolhead.dwell(dwell_time)?;
            toolhead.flush_step_generation()?;
        }
        st.stepper().set_trapq(prev_trapq);
        st.stepper().set_stepper_kinematics(prev_sk);
        st.motion_queuing.wipe_trapq(&st.trapq);
        if sync {
            toolhead.wait_moves()?;
        }
        Ok(())
    })
}

/// move: the lane stepper by distance, in max_move_dis pieces
pub fn lane_move(lane: &AfcLane, distance: f64, mut speed: f64, accel: f64, assist_active: bool) -> KResult<()> {
    let direction = if distance > 0.0 { 1.0 } else { -1.0 };
    let mut move_total = distance.abs();
    let (rev_factor, max_move_dis) = {
        let s = lane.settings();
        (s.rev_long_moves_speed_factor.unwrap_or(0.0), s.max_move_dis.unwrap_or(0.0))
    };
    if direction == -1.0 {
        speed *= rev_factor;
    }
    // Breaks up move length to help with TTC errors
    while move_total > 0.0 {
        let move_value = if move_total > max_move_dis { max_move_dis } else { move_total };
        move_total -= move_value;
        do_move(lane, move_value * direction, speed, accel, assist_active, true)?;
    }
    Ok(())
}

/// do_enable: the lane stepper's motor on or off
pub fn lane_do_enable(lane: &AfcLane, enable: bool) -> KResult<()> {
    let st = stepper_of(lane)?;
    let stepper_name = format!("AFC_stepper {}", lane.name);
    let Ok(enabled) = st.stepper_enable.lookup_enable(&stepper_name) else {
        return Ok(());
    };
    if enabled.is_motor_enabled() == enable {
        return Ok(());
    }
    st.stepper_enable.set_motors_enable(&[stepper_name], enable)?;
    Ok(())
}

/// sync_print_time: the lane's command time caught up with the toolhead's
fn sync_print_time(lane: &AfcLane) -> KResult<()> {
    let st = stepper_of(lane)?;
    let toolhead = toolhead(lane)?;
    let print_time = toolhead.get_last_move_time()?;
    if st.next_cmd_time.get() > print_time {
        toolhead.dwell(st.next_cmd_time.get() - print_time)?;
    } else {
        st.next_cmd_time.set(print_time);
    }
    Ok(())
}

pub fn lane_sync_to_extruder(lane: &AfcLane, update_current: bool, th_extruder_name: Option<&str>) -> KResult<()> {
    let st = stepper_of(lane)?;
    let th_extruder_name = match th_extruder_name {
        Some(n) => n.to_string(),
        None => lane.extruder_obj()?.th_extruder_name.clone(),
    };
    if !st.synced_to_extruder.get() {
        st.extruder_stepper.sync_to_extruder(&th_extruder_name)?;
        st.synced_to_extruder.set(true);
    }
    if update_current {
        set_print_current(lane)?;
    }
    Ok(())
}

pub fn lane_unsync_to_extruder(lane: &AfcLane, update_current: bool) -> KResult<()> {
    let st = stepper_of(lane)?;
    if st.synced_to_extruder.get() {
        st.extruder_stepper.sync_to_extruder("")?;
        st.synced_to_extruder.set(false);
    }
    if update_current {
        set_load_current(lane)?;
    }
    Ok(())
}

/// _set_current: SET_TMC_CURRENT for the lane stepper
fn set_current(lane: &AfcLane, current: Option<f64>) -> KResult<()> {
    let st = stepper_of(lane)?;
    if let (Some(_), Some(current)) = (st.tmc_print_current, current) {
        let command = format!("SET_TMC_CURRENT STEPPER='{}' CURRENT={}", lane.name, py_float_repr(current));
        lane.afc.logger.debug(&AFC_STEPPER_AFCEXTRUDERSTEPPER__SET_CURRENT, &format!("Running macro: {}", command));
        lane.afc.gcode.run_script_from_command(&command)?;
    }
    Ok(())
}

pub fn set_load_current(lane: &AfcLane) -> KResult<()> {
    let st = stepper_of(lane)?;
    if st.print_current_set.get() {
        set_current(lane, st.tmc_load_current)?;
        st.print_current_set.set(false);
    }
    Ok(())
}

pub fn set_print_current(lane: &AfcLane) -> KResult<()> {
    let st = stepper_of(lane)?;
    if !st.print_current_set.get() {
        set_current(lane, st.tmc_print_current)?;
        st.print_current_set.set(true);
    }
    Ok(())
}

/// update_rotation_distance: base rotation distance / multiplier
pub fn update_rotation_distance(lane: &AfcLane, multiplier: f64) -> KResult<()> {
    let st = stepper_of(lane)?;
    st.extruder_stepper.stepper.set_rotation_distance(st.base_rotation_dist / multiplier);
    Ok(())
}

/// The lane as homing's toolhead (ManualStepper compatibility shims)
struct HomingLane(Rc<AfcLane>);

impl HomingToolhead for HomingLane {
    /// Only syncs print time, is needed for homing flow
    fn flush_step_generation(&self) -> KResult<()> {
        sync_print_time(&self.0)
    }
    fn get_position(&self) -> Vec<f64> {
        let pos = stepper_of(&self.0).map(|st| st.stepper().get_commanded_position()).unwrap_or(0.0);
        vec![pos, 0.0, 0.0, 0.0]
    }
    /// always sets to zero regardless of whats passed in
    fn set_position(&self, _newpos: &[f64], _homing_axes: &str) -> KResult<()> {
        stepper_of(&self.0)?.manual_axis_pos.set(0.0);
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        sync_print_time(&self.0)?;
        Ok(stepper_of(&self.0)?.next_cmd_time.get())
    }
    fn dwell(&self, delay: f64) -> KResult<()> {
        let st = stepper_of(&self.0)?;
        st.next_cmd_time.set(st.next_cmd_time.get() + py_max(0.0, delay));
        Ok(())
    }
    fn drip_move(&self, newpos: &[f64], speed: f64, drip_completion: &Rc<ReactorCompletion<bool>>) -> KResult<()> {
        let lane = &self.0;
        let st = stepper_of(lane)?;
        // Queue the homing move quickly; do not block or dwell here
        sync_print_time(lane)?;
        let target = newpos[0];
        let delta = target - st.manual_axis_pos.get();
        let start_time = st.next_cmd_time.get();
        let prev_sk = st.stepper().set_stepper_kinematics(Some(st.stepper_kinematics.clone()));
        let prev_trapq = st.stepper().set_trapq(Some(st.trapq.clone()));
        let dwell_time = submit_move(&st, start_time, delta, speed, st.homing_accel.get());
        let max_time = start_time + dwell_time;
        st.motion_queuing.drip_update_time(start_time, max_time, drip_completion)?;
        st.stepper().set_trapq(prev_trapq);
        st.stepper().set_stepper_kinematics(prev_sk);
        st.motion_queuing.wipe_trapq(&st.trapq);
        sync_print_time(lane)
    }
    fn kin_steppers(&self) -> Vec<Rc<McuStepper>> {
        stepper_of(&self.0).map(|st| vec![st.stepper().clone()]).unwrap_or_default()
    }
    fn kin_calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> Vec<f64> {
        let name = stepper_of(&self.0).map(|st| st.stepper().get_name(false)).unwrap_or_default();
        vec![stepper_positions.get(&name).copied().unwrap_or(0.0), 0.0, 0.0]
    }
}

/// _resolve_endstop_pin: a pre-created endstop by key (load, hub, tool,
/// buffer, a lane endstop name or a raw pin)
fn resolve_endstop_pin(lane: &AfcLane, endstop_spec: &str) -> KResult<(EndstopRef, String)> {
    let st = stepper_of(lane)?;
    let key = match endstop_spec {
        "tool" => "tool_start",
        "buffer" => "buffer_advance",
        k => k,
    };
    let found = st.endstops.borrow().get(key).cloned();
    match found {
        Some(e) => Ok(e),
        None => Err(KError::config(format!(
            "Unknown ENDSTOP '{}' for lane '{}'. Declare it in [AFC_stepper {}] using 'homing_endstop_pins: {}' or use ENDSTOP=load/hub/tool/buffer",
            endstop_spec, lane.name, lane.name, endstop_spec
        ))),
    }
}

fn endstop_str(endstop: &Endstop) -> String {
    match endstop {
        Endstop::None => "None".into(),
        Endstop::Name(n) => n.clone(),
    }
}

/// do_homing_move: (homed, distance moved); not homed is (False, movepos)
#[allow(clippy::too_many_arguments)]
fn do_homing_move(lane: &Rc<AfcLane>, movepos: f64, speed_pv: &PyVal, accel_pv: &PyVal, endstop_spec: &str, triggered: bool, check_trigger: bool, assist_active: bool) -> KResult<(bool, f64)> {
    let (speed, accel) = (speed_pv.as_f64().unwrap_or(0.0), accel_pv.as_f64().unwrap_or(0.0));
    let c = &AFC_STEPPER_AFCEXTRUDERSTEPPER_DO_HOMING_MOVE;
    let st = stepper_of(lane)?;
    let reactor = lane.printer.get_reactor();
    let start_ts = reactor.monotonic();
    let logger = &lane.afc.logger;
    logger.debug(
        c,
        &format!("[AFC_stepper:{}] Homing start endstop={} movepos={} speed={} accel={}", lane.name, endstop_spec, py_float_repr(movepos), speed_pv.str(), accel_pv.str()),
    );
    // Setting private variable so accel can be used in drip_move callback
    st.homing_accel.set(accel);
    // Avoid zero-distance drip sequences which can confuse stepcompress
    if movepos == st.manual_axis_pos.get() {
        logger.debug(c, &format!("AFC stepper '{}' homing target equals current position; skipping move", lane.name));
        return Ok((false, 0.0));
    }
    let pos = [movepos, 0.0, 0.0, 0.0];
    let phoming = lane.printer.lookup::<PrinterHoming>("homing")?;
    // Try to get a real MCU-backed endstop
    let endstop = match resolve_endstop_pin(lane, endstop_spec) {
        Ok(e) => e,
        Err(e) => {
            let raise_string = format!("ENDSTOP '{}' could not be resolved ({})", endstop_spec, e.message());
            logger.debug(c, &raise_string);
            return Err(KError::command(raise_string));
        }
    };
    // Start/stop assist exactly with homing lifetime
    let rewind = movepos < st.manual_axis_pos.get();
    let start_mcu_pos = st.stepper().get_mcu_position(None);
    let toolhead: Rc<dyn HomingToolhead> = Rc::new(HomingLane(lane.clone()));
    let assist = assist_lane(lane);
    let assist = assist.as_deref().unwrap_or(lane);
    let homed = assist.assist_move(speed, rewind, assist_active, || phoming.manual_home(&toolhead, vec![endstop], &pos, speed, triggered, check_trigger));
    match homed {
        Ok(()) => {
            let end_ts = reactor.monotonic();
            // Log distance at trigger using homing trigger positions
            let trig_mcu_pos = st.stepper().get_mcu_position(None);
            let step_dist = st.stepper().get_step_dist();
            let steps_moved = (trig_mcu_pos - start_mcu_pos).abs();
            let dist_mm = steps_moved as f64 * step_dist;
            logger.debug(c, &format!("Homed lane {} to ENDSTOP={} trigger after {:.3}mm (steps={} dt={:.3}s)", lane.name, endstop_spec, dist_mm, steps_moved, end_ts - start_ts));
            Ok((true, dist_mm))
        }
        Err(e) => {
            let msg = e.message().to_lowercase();
            if msg.contains("no trigger on") {
                if !lane.printer.is_shutdown() {
                    let pos = kaliru::chelper::itersolve::get_commanded_pos(&st.stepper_kinematics.borrow());
                    logger.debug(c, &format!("[{}] Homing: {}; continuing because homing_ignore_no_trigger is enabled {}", lane.name, e.message(), py_float_repr(pos)));
                    return Ok((false, movepos));
                }
                return Err(KError::command(e.message()));
            }
            if msg.contains("communication timeout during homing") || (msg.contains("endstop") && msg.contains("still triggered")) {
                logger.info(c, e.message());
                logger.debug(c, &kaliru::logging::format_traceback(&e));
                return Err(KError::command(e.message()));
            }
            // Other MCU homing issues: surface message and re-raise as gcode error
            logger.debug(c, &format!("MCU homing issue ({}); not proceeding with homing move", e.message()));
            Err(KError::command(e.message()))
        }
    }
}

/// home_to: (homed, distance moved, error)
#[allow(clippy::too_many_arguments)]
pub fn home_to(lane: &Rc<AfcLane>, endstop: &Endstop, distance: f64, speed: &PyVal, accel: &PyVal, triggered: bool, check_trigger: bool, assist_active: bool) -> KResult<(bool, f64, bool)> {
    let c = &AFC_STEPPER_AFCEXTRUDERSTEPPER_HOME_TO;
    let st = stepper_of(lane)?;
    let endstop_spec = endstop_str(endstop);
    // Compute an absolute target along our 1D axis
    let target = st.manual_axis_pos.get() + distance;
    let mut error = false;
    let mut homed = false;
    let mut move_distance = 0.0;
    match do_homing_move(lane, target, speed, accel, &endstop_spec, triggered, check_trigger, assist_active) {
        Ok((h, d)) => {
            homed = h;
            move_distance = d;
        }
        Err(e) => {
            error = true;
            let msg = format!("Error occurred when trying to home to {}, PAUSING!", endstop_spec);
            lane.afc.error.afc_error(c, &msg, lane.afc.function.in_print()?, None)?;
            lane.afc.logger.debug(c, &kaliru::logging::format_traceback(&e));
        }
    }
    sync_print_time(lane)?;
    Ok((homed, move_distance, error))
}

/// AFC_STEPPER_HOME STEPPER= [ENABLE=] [SPEED=] [ACCEL=] [STOP_ON_ENDSTOP=] [DIST=] [ENDSTOP=] [SYNC=]
fn cmd_afc_stepper_home(lane: &Rc<AfcLane>, gcmd: &GCodeCommand) -> KResult<()> {
    let st = stepper_of(lane)?;
    if let Some(enable) = gcmd.get_int_opt("ENABLE", Lim::new())? {
        lane_do_enable(lane, enable != 0)?;
    }
    // (self.short_moves_speed or 5.: the lane's value, an int when it is
    // AFC's default, else 5.0; a parameter given is a float)
    let (short_speed, short_accel) = {
        let s = lane.settings();
        let mi = s.move_ints;
        let speed = match s.short_moves_speed.filter(|v| *v != 0.0) {
            Some(v) if mi.short_speed => PyVal::Int(v as i64),
            Some(v) => PyVal::Float(v),
            None => PyVal::Float(5.0),
        };
        let accel = match s.short_moves_accel.filter(|v| *v != 0.0) {
            Some(v) if mi.short_accel => PyVal::Int(v as i64),
            Some(v) => PyVal::Float(v),
            None => PyVal::Float(0.0),
        };
        (speed, accel)
    };
    let speed_v = gcmd.get_float("SPEED", Some(short_speed.as_f64().unwrap_or(0.0)), Lim::new().above(0.0))?;
    let accel_v = gcmd.get_float("ACCEL", Some(short_accel.as_f64().unwrap_or(0.0)), Lim::new().min(0.0))?;
    let speed = if gcmd.get_opt("SPEED").is_some() { PyVal::Float(speed_v) } else { short_speed };
    let accel = if gcmd.get_opt("ACCEL").is_some() { PyVal::Float(accel_v) } else { short_accel };
    let homing_move = gcmd.get_int("STOP_ON_ENDSTOP", Some(0), Lim::new())?;
    if homing_move != 0 {
        let movepos = gcmd.get_float("DIST", None, Lim::new())?;
        let endstop_spec = gcmd.get_or("ENDSTOP", st.default_homing_endstop.as_str());
        do_homing_move(lane, movepos, &speed, &accel, &endstop_spec, homing_move > 0, homing_move.abs() == 1, true)?;
    } else if gcmd.get_float_opt("DIST", Lim::new())?.is_some() {
        let movepos = gcmd.get_float("DIST", None, Lim::new())?;
        let sync = gcmd.get_int("SYNC", Some(1), Lim::new())?;
        // Implement simple absolute-position move using our pipeline
        let delta = movepos - st.manual_axis_pos.get();
        if delta != 0.0 {
            do_move(lane, delta, speed_v, accel_v, true, true)?;
            st.manual_axis_pos.set(movepos);
        }
        if sync != 0 {
            sync_print_time(lane)?;
        }
    } else if gcmd.get_int("SYNC", Some(0), Lim::new())? != 0 {
        sync_print_time(lane)?;
    }
    Ok(())
}

