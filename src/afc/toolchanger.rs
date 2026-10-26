// AFC_Toolchanger: a toolchanger's tools as AFC's extruders, each tool its
// own lane when no unit feeds it (port of AFC_Toolchanger.py; tool changes go
// through klipper-toolchanger's SELECT_TOOL / UNSELECT_TOOL, or a tool's
// custom macros)
//
// Copyright (C) 2025 Armored Turtle
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::lane::{AfcLane, MoveWarning};
use super::unit::{AfcUnit, UnitKind};
use super::State;
use kaliru::configfile::ConfigWrapper;
use kaliru::error::KResult;
use kaliru::gcode::GCodeCommand;
use kaliru::printer::PObj;
use kaliru::configfile::Lim;
use std::rc::Rc;

const CMD_AFC_SELECT_TOOL_HELP: &str = "Select specified tool";
const CMD_AFC_UNSELECT_TOOL_HELP: &str = "Unselects and docks current tool on shuttle";
const CMD_AFC_SET_TOOLHEAD_LED_HELP: &str = "Turns on leds for toolhead specified by mapping, does not affect status led if status_led_idx variable is provided";

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let unit = AfcUnit::new(config, |config, _| Ok((UnitKind::Toolchanger, config.get("type", "Toolchanger")?)))?;
    *unit.logo.borrow_mut() = "<span class=success--text>Toolchanger Ready\n</span>".into();
    *unit.logo_error.borrow_mut() = "<span class=error--text>Toolchanger Not Ready</span>\n".into();
    let f = &unit.afc.function;
    let show = unit.afc.show_macros;
    let u = unit.clone();
    f.register_commands(show, "AFC_SELECT_TOOL", Rc::new(move |g| cmd_afc_select_tool(&u, g)), CMD_AFC_SELECT_TOOL_HELP, Some(&[("TOOL", "string", "extruder")]))?;
    let u = unit.clone();
    f.register_commands(show, "AFC_UNSELECT_TOOL", Rc::new(move |g| cmd_afc_unselect_tool(&u, g)), CMD_AFC_UNSELECT_TOOL_HELP, None)?;
    let u = unit.clone();
    f.register_commands(
        show,
        "AFC_SET_TOOLHEAD_LED",
        Rc::new(move |g| cmd_afc_set_toolhead_led(&u, g)),
        CMD_AFC_SET_TOOLHEAD_LED_HELP,
        Some(&[("MAP", "string", "T0"), ("TURN_ON", "int", "1")]),
    )?;
    Ok(unit)
}

/// system_Test: the T command, and whether the tool holds the lane
pub fn system_test(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, assign_tcmd: bool) -> KResult<bool> {
    let afc = &unit.afc;
    if assign_tcmd {
        afc.function.tcmd_assign(cur_lane)?;
    }
    // Now that a T command is assigned, send lane data to moonraker
    cur_lane.send_lane_data()?;
    let mut msg = String::new();
    let ext = cur_lane.extruder_obj()?;
    if cur_lane.prep_state.get() && cur_lane.load_state() && ext.lane_loaded.borrow().as_deref() == Some(cur_lane.name.as_str()) {
        msg = "<span class=success--text>LOADED</span>".into();
        msg += &ext.prep_on_shuttle_check();
    }
    afc.logger.raw(&AFC_TOOLCHANGER_AFCTOOLCHANGER_SYSTEM_TEST, &format!("{} tool cmd: {:<3} {}", cur_lane.name, cur_lane.map_to_string(), msg));
    cur_lane.set_afc_prep_done();
    Ok(true)
}

/// move_to_hub / move_to_load / load_then_home: a tool has nothing to move
pub fn no_move() -> (bool, f64, MoveWarning) {
    (true, 0.0, MoveWarning::None)
}

/// _increase_unselect: the current tool's unselected count
fn increase_unselect(unit: &AfcUnit) -> KResult<()> {
    if let Some(e) = unit.afc.function.get_current_extruder_obj()? {
        e.estats.tool_unselected().increase_count()?;
    }
    Ok(())
}

fn cmd_afc_select_tool(unit: &Rc<AfcUnit>, gcmd: &GCodeCommand) -> KResult<()> {
    let c = &AFC_TOOLCHANGER_AFCTOOLCHANGER_CMD_AFC_SELECT_TOOL;
    let tool_key = gcmd.get("TOOL")?;
    // First try by dictionary key, then by AFC_extruder name
    let tool = {
        let tools = unit.afc.tools.borrow();
        tools.get(&tool_key).cloned().or_else(|| tools.values().find(|e| e.name == tool_key).cloned())
    };
    match tool {
        Some(tool) => match tool.tc_lane.borrow().clone() {
            Some(lane) => tool_swap(unit, &lane, true)?,
            None => unit.afc.logger.error(c, &format!("Tool '{}' does not have a valid 'tc_lane' attribute.", tool_key)),
        },
        None => unit.afc.logger.error(c, &format!("Key:{} invalid for TOOL", tool_key)),
    }
    Ok(())
}

fn cmd_afc_unselect_tool(unit: &Rc<AfcUnit>, _gcmd: &GCodeCommand) -> KResult<()> {
    let afc = &unit.afc;
    increase_unselect(unit)?;
    let current_extruder = afc.function.get_current_extruder_obj()?;
    afc.current_state.set(State::ToolDock);
    if let Some(e) = &current_extruder {
        e.status.set(State::ToolDock);
    }
    match current_extruder.as_ref().and_then(|e| e.custom_unselect.clone()) {
        Some(cmd) => {
            afc.logger.info(&AFC_TOOLCHANGER_AFCTOOLCHANGER_CMD_AFC_UNSELECT_TOOL, &format!("Running custom unselect: {}", cmd));
            afc.gcode.run_script_from_command(&cmd)?;
        }
        None => afc.gcode.run_script_from_command("UNSELECT_TOOL")?,
    }
    afc.function.handle_activate_extruder()?;
    if let Some(lane_obj) = afc.function.get_current_lane_obj() {
        if lane_obj.prep_state.get() && lane_obj.load_state() {
            if lane_obj.tool_loaded.get() {
                lane_obj.unit_obj()?.lane_tool_loaded_idle(&lane_obj)?;
            } else {
                lane_obj.unit_obj()?.lane_tool_unloaded(&lane_obj)?;
            }
        } else {
            lane_obj.extruder_obj()?.set_status_led(&lane_obj.led_tool_unloaded())?;
        }
    }
    afc.spool.set_active_spool(&kaliru::pyval::PyVal::from(""))?;
    afc.current_state.set(State::Idle);
    if let Some(e) = &current_extruder {
        e.status.set(State::Idle);
    }
    Ok(())
}

/// tool_swap: pick up the lane's tool (SELECT_TOOL, or its custom macro),
/// keeping the gcode position through the tool's offsets
pub fn tool_swap(unit: &AfcUnit, lane: &Rc<AfcLane>, set_start_time: bool) -> KResult<()> {
    let afc = &unit.afc;
    let c = &AFC_TOOLCHANGER_AFCTOOLCHANGER_TOOL_SWAP;
    if set_start_time {
        afc.delta_time.set_start_time();
    }
    let current_extruder = afc.function.get_current_extruder_obj()?;
    if let Some(e) = &current_extruder {
        e.status.set(State::ToolDock);
    }
    afc.current_state.set(State::ToolSwap);
    let ext = lane.extruder_obj()?;
    ext.status.set(State::ToolPickup);
    ext.next_pickup.set(true);
    increase_unselect(unit)?;
    afc.function.log_toolhead_pos("Before toolswap: ")?;
    // Save the current position before switching tools and subtract offsets
    let gm = afc.gcode_move()?;
    {
        let base = gm.base_position.borrow().clone();
        let mut last = afc.last_gcode_position.borrow_mut();
        for i in 0..3 {
            last[i] -= base[i];
        }
    }
    // Perform a tool swap by selecting the appropriate extruder based on the lane's extruder object.
    afc.delta_time.log_with_time(c, "Performing tool swap", true);
    let name = &ext.th_extruder_name;
    let tool_index: i64 = if name == "extruder" { 0 } else { super::utils::py_int_of(&kaliru::pyval::PyVal::from(name.replace("extruder", "").as_str()))? };
    match &ext.custom_tool_swap {
        Some(cmd) => {
            afc.logger.info(c, &format!("Running custom select: {}", cmd));
            afc.gcode.run_script_from_command(cmd)?;
        }
        None => afc.gcode.run_script_from_command(&format!("SELECT_TOOL T={}", tool_index))?,
    }
    ext.next_pickup.set(false);
    if let Some(e) = &current_extruder {
        e.status.set(State::Idle);
    }
    // Switching toolhead extruders, this is mainly for setups with multiple extruders
    lane.activate_toolhead_extruder()?;
    // Need to call again since KTC activate callback happens before switching to new extruder
    afc.function.handle_activate_extruder()?;
    afc.toolhead()?.wait_moves()?;
    afc.delta_time.log_with_time(c, "Tool swap done", false);
    if let Some(s) = &afc.afc_stats()?.average_tool_swap_time {
        s.average_time(afc.delta_time.delta());
    }
    ext.status.set(State::Idle);
    afc.current_state.set(State::Idle);
    // Update the base position and homing position after the tool swap.
    *afc.base_position.borrow_mut() = gm.base_position.borrow().clone();
    *afc.homing_position.borrow_mut() = gm.homing_position.borrow().clone();
    // Update the last_gcode_position to reflect the new base position after the tool swap.
    {
        let base = gm.base_position.borrow().clone();
        let mut last = afc.last_gcode_position.borrow_mut();
        for i in 0..3 {
            last[i] += base[i];
        }
    }
    afc.function.log_toolhead_pos("After toolswap: ")?;
    ext.estats.tool_selected().increase_count()
}

fn cmd_afc_set_toolhead_led(unit: &Rc<AfcUnit>, gcmd: &GCodeCommand) -> KResult<()> {
    let mapping = gcmd.get("MAP")?;
    let state = gcmd.get_int("TURN_ON", Some(1), Lim::new())?;
    let Some(lane_obj) = unit.afc.function.get_lane_by_map(&mapping) else {
        let error_string = format!("{} mapping not found when trying to set toolhead led", mapping);
        unit.afc.logger.error(&AFC_TOOLCHANGER_AFCTOOLCHANGER_CMD_AFC_SET_TOOLHEAD_LED, &error_string);
        return Err(gcmd.error(error_string));
    };
    let (success, error_string) = lane_obj.extruder_obj()?.set_print_leds(state, false)?;
    if !success {
        return Err(gcmd.error(error_string));
    }
    Ok(())
}
