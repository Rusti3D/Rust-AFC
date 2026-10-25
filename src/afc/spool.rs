// AFC spool data: colors, materials, weights, spoolman, T(n) mappings (port
// of AFC_spool.py)
//
// Copyright (C) 2024-2026 Armored Turtle
// Copyright (C) 2026 AFCProject
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::lane::AfcLane;
use super::{no_attr, Afc};
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::gcode::GCodeCommand;
use kaliru::printer::{PObj, Printer, PrinterObject};
use kaliru::pyval::PyVal;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

pub const SPOOLMAN_REMOTE_METHOD: &str = "spoolman_set_active_spool";

pub const CMD_AFC_SET_SPOOL_TEMP_HELP: &str = "Set spool temperatures for a lane";
pub const CMD_SET_MAP_HELP: &str = "Changes T(n) mapping for a lane";
pub const CMD_AFC_SWAP_MAPPING_HELP: &str = "Swaps mapping between two lanes as provided with the FROM/TO variables";
pub const CMD_AFC_ADD_MAPPING_HELP: &str =
    "Adds a new T(n) macro that currently does not exist to a lane. Mapping can be passed in a comma separated list to add multiple to a single lane";
pub const CMD_AFC_REMOVE_MAPPING_HELP: &str =
    "Removes T(n) macro from specified lane, mapping can be passed in as comma separated list to remove multiple mappings from a lane";
pub const CMD_AFC_ENABLE_MULTIPLE_MAPPING_HELP: &str =
    "Enable ability to have multiple T(n) macros mapped to a single lane and enables use of virtual tools";
pub const CMD_SET_COLOR_HELP: &str = "Set filaments color for a lane";
pub const CMD_SET_WEIGHT_HELP: &str = "Sets filaments weight for a lane";
pub const CMD_SET_MATERIAL_HELP: &str = "Sets filaments material for a lane";
pub const CMD_SET_SPOOL_ID_HELP: &str = "Set lanes spoolman ID";
pub const CMD_SET_RUNOUT_HELP: &str = "Set runout lane";
pub const CMD_AFC_RESET_MAPPING_HELP: &str = "Resets all lane mapping in AFC";
pub const CMD_SET_NEXT_SPOOL_ID_HELP: &str = "Set the spool id to be loaded next into AFC";

pub type OnDone = Box<dyn FnOnce() -> KResult<()>>;

/// [AFC_spool]
pub struct AfcSpool {
    me: Weak<AfcSpool>,
    printer: Rc<Printer>,
    afc: RefCell<Weak<Afc>>,
    pub next_spool_id: RefCell<PyVal>,
}

type SpoolCmd = fn(&Rc<AfcSpool>, &GCodeCommand) -> KResult<()>;

impl AfcSpool {
    fn new(config: &ConfigWrapper) -> Rc<AfcSpool> {
        let printer = config.get_printer();
        let s = Rc::new_cyclic(|me| AfcSpool { me: me.clone(), printer: printer.clone(), afc: RefCell::new(Weak::new()), next_spool_id: RefCell::new(PyVal::None) });
        let ss = s.clone();
        kaliru::events::KlippyConnect::on(&printer, move || ss.handle_connect());
        s
    }

    fn afc(&self) -> Rc<Afc> {
        self.afc.borrow().upgrade().expect("AFC")
    }

    fn rc(&self) -> Rc<AfcSpool> {
        self.me.upgrade().expect("AFC_spool")
    }

    /// register_commands(afc): called from AFC's __init__
    pub fn register_commands(&self, afc: &Rc<Afc>) -> KResult<()> {
        *self.afc.borrow_mut() = Rc::downgrade(afc);
        let me = self.rc();
        let reg = |show: bool, name: &str, f: SpoolCmd, help: &str, options: Option<&[super::functions::MacroOption]>| -> KResult<()> {
            let s = me.clone();
            afc.function.register_commands(show, name, Rc::new(move |g| f(&s, g)), help, options)
        };
        reg(afc.show_macros, "AFC_ADD_MAPPING", AfcSpool::cmd_afc_add_mapping, CMD_AFC_ADD_MAPPING_HELP, Some(&[("LANE", "string", "lane1"), ("MAPPING", "string", "T0")]))?;
        reg(afc.show_macros, "AFC_REMOVE_MAPPING", AfcSpool::cmd_afc_remove_mapping, CMD_AFC_REMOVE_MAPPING_HELP, Some(&[("MAPPING", "string", "T0")]))?;
        reg(afc.show_macros, "AFC_ENABLE_MULTIPLE_MAPPING", AfcSpool::cmd_afc_enable_multiple_mapping, CMD_AFC_ENABLE_MULTIPLE_MAPPING_HELP, Some(&[("ENABLE", "int", "0")]))?;
        reg(false, "AFC_SWAP_MAPPING", AfcSpool::cmd_afc_swap_mapping, CMD_AFC_SWAP_MAPPING_HELP, None)
    }

    fn handle_connect(&self) -> KResult<()> {
        let me = self.rc();
        let s = me.clone();
        self.printer.register_event_handler(
            super::lane::REGISTER_MACROS_EVENT,
            Rc::new(move |args| {
                let lane: Rc<AfcLane> = kaliru::events::arg(super::lane::REGISTER_MACROS_EVENT, args, 0)?;
                s.register_lane_macros(&lane)
            }),
        );
        let gcode = self.afc().gcode.clone();
        let s = me.clone();
        gcode.register_command("AFC_RESET_MAPPING", Some(Rc::new(move |g| s.cmd_afc_reset_mapping(g))), false, Some(CMD_AFC_RESET_MAPPING_HELP))?;
        let s = me.clone();
        gcode.register_command("SET_NEXT_SPOOL_ID", Some(Rc::new(move |g| s.cmd_set_next_spool_id(g))), false, Some(CMD_SET_NEXT_SPOOL_ID_HELP))?;
        Ok(())
    }

    pub fn register_lane_macros(&self, lane_obj: &AfcLane) -> KResult<()> {
        let me = self.rc();
        let gcode = self.afc().gcode.clone();
        let cmds: [(&str, SpoolCmd, &str); 7] = [
            ("SET_COLOR", AfcSpool::cmd_set_color, CMD_SET_COLOR_HELP),
            ("SET_WEIGHT", AfcSpool::cmd_set_weight, CMD_SET_WEIGHT_HELP),
            ("SET_MATERIAL", AfcSpool::cmd_set_material, CMD_SET_MATERIAL_HELP),
            ("SET_SPOOL_ID", AfcSpool::cmd_set_spool_id, CMD_SET_SPOOL_ID_HELP),
            ("SET_RUNOUT", AfcSpool::cmd_set_runout, CMD_SET_RUNOUT_HELP),
            ("SET_MAP", AfcSpool::cmd_set_map, CMD_SET_MAP_HELP),
            ("AFC_SET_SPOOL_TEMP", AfcSpool::cmd_afc_set_spool_temp, CMD_AFC_SET_SPOOL_TEMP_HELP),
        ];
        for (cmd, f, help) in cmds {
            let s = me.clone();
            gcode.register_mux_command(cmd, "LANE", Some(&lane_obj.name), Rc::new(move |g| f(&s, g)), Some(help))?;
        }
        Ok(())
    }

    /// (snapmaker printers only: never on kalico)
    pub fn set_snapmaker_filament_params(&self, _lane: &AfcLane) {}

    fn lane_arg(&self, gcmd: &GCodeCommand, code: &super::logger::Code, missing: &str) -> Option<Rc<AfcLane>> {
        let afc = self.afc();
        let Some(lane) = gcmd.get_opt("LANE") else {
            afc.logger.info(code, missing);
            return None;
        };
        match afc.lane(&lane) {
            Some(l) => Some(l),
            None => {
                afc.logger.info(code, &format!("{} Unknown", lane));
                None
            }
        }
    }

    fn cmd_afc_set_spool_temp(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let afc = self.afc();
        let Some(cur_lane) = self.lane_arg(gcmd, &AFC_SPOOL_AFCSPOOL_CMD_AFC_SET_SPOOL_TEMP, "No LANE parameter provided, please specify a valid LANE parameter.") else {
            return Ok(());
        };
        let get = |name: &str, cur: &PyVal| -> KResult<PyVal> {
            Ok(match gcmd.get_opt(name) {
                Some(_) => PyVal::Int(gcmd.get_int(name, None, Lim::new().min(0))?),
                None => cur.clone(),
            })
        };
        let bed = get("BED_TEMP", &cur_lane.bed_temp.borrow().clone())?;
        *cur_lane.bed_temp.borrow_mut() = bed;
        let ext = get("EXTRUDER_TEMP", &cur_lane.extruder_temp.borrow().clone())?;
        *cur_lane.extruder_temp.borrow_mut() = ext;
        cur_lane.send_lane_data()?;
        afc.save_vars()
    }

    fn cmd_set_map(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_SPOOL_AFCSPOOL_CMD_SET_MAP;
        let lane = gcmd.get("LANE")?;
        let map_cmd = gcmd.get("MAP")?.to_uppercase();
        let Some(lane_switch) = afc.tool_cmds.borrow().get(&map_cmd).cloned() else {
            afc.logger.error(c, &format!("Invalid map command: {}", map_cmd));
            return Ok(());
        };
        afc.logger.debug(c, &format!("lane to switch is {}", lane_switch));
        let Some(cur_lane) = afc.lane(&lane) else {
            afc.logger.info(c, &format!("{} Unknown", lane));
            return Ok(());
        };
        let Some(sw_lane) = afc.lane(&lane_switch) else {
            afc.logger.info(c, &format!("{} is not found when swapping mapping.", lane_switch));
            return Ok(());
        };
        if Rc::ptr_eq(&sw_lane, &cur_lane) {
            afc.logger.info(c, &format!("{} is already mapped to {}", map_cmd, lane));
            return Ok(());
        }
        if afc.enable_multiple_mapping.get() {
            let m: Vec<String> = sw_lane.map.borrow().iter().filter(|c| **c != map_cmd).cloned().collect();
            *sw_lane.map.borrow_mut() = m;
            if *sw_lane.current_map.borrow() == map_cmd {
                *sw_lane.current_map.borrow_mut() = String::new();
            }
            let mut m = cur_lane.map.borrow().clone();
            m.push(map_cmd.clone());
            *cur_lane.map.borrow_mut() = m.into_iter().filter(|m| m != "NONE").collect();
        } else {
            let m = cur_lane.map.borrow().clone();
            *sw_lane.current_map.borrow_mut() = m.first().cloned().unwrap_or_default();
            *sw_lane.map.borrow_mut() = m;
            *cur_lane.map.borrow_mut() = vec![map_cmd.clone()];
            *cur_lane.current_map.borrow_mut() = map_cmd;
        }
        for m in sw_lane.map.borrow().iter() {
            if m == "NONE" {
                continue;
            }
            afc.tool_cmds.borrow_mut().insert(m.clone(), sw_lane.name.clone());
        }
        for m in cur_lane.map.borrow().iter() {
            afc.tool_cmds.borrow_mut().insert(m.clone(), cur_lane.name.clone());
        }
        sw_lane.send_lane_data()?;
        cur_lane.send_lane_data()?;
        afc.save_vars()
    }

    fn cmd_afc_swap_mapping(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let afc = self.afc();
        let lane_from = gcmd.get("FROM")?;
        let lane_to = gcmd.get("TO")?;
        if lane_from.to_lowercase() == lane_to.to_lowercase() {
            return Err(gcmd.error(format!("FROM({}) TO({}) values are the same, not swapping.", lane_from, lane_to)));
        }
        let from = afc.lane(&lane_from).ok_or_else(|| gcmd.error(format!("Swapping from {} not found", lane_from)))?;
        let to = afc.lane(&lane_to).ok_or_else(|| gcmd.error(format!("Swapping to {} not found", lane_to)))?;
        let from_mapping = from.map.borrow().clone();
        let to_mapping = to.map.borrow().clone();
        let from_current = from.current_map.borrow().clone();
        let to_current = to.current_map.borrow().clone();
        *from.current_map.borrow_mut() = to_current;
        *from.map.borrow_mut() = to_mapping;
        *to.current_map.borrow_mut() = from_current;
        *to.map.borrow_mut() = from_mapping;
        for (lane, name) in [(&from, &from.name), (&to, &to.name)] {
            for m in lane.map.borrow().iter() {
                if m != "NONE" {
                    afc.tool_cmds.borrow_mut().insert(m.clone(), name.clone());
                }
            }
        }
        from.send_lane_data()?;
        to.send_lane_data()?;
        afc.save_vars()?;
        afc.logger.info(&AFC_SPOOL_AFCSPOOL_CMD_AFC_SWAP_MAPPING, &format!("Swapping mapping from({}) to({}) successful.", lane_from, lane_to));
        Ok(())
    }

    fn cmd_afc_add_mapping(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_SPOOL_AFCSPOOL_CMD_AFC_ADD_MAPPING;
        if !afc.enable_multiple_mapping.get() {
            return Err(gcmd.error("Enable multiple mapping needs to be enabled to add mappings, enable by running AFC_ENABLE_MULTIPLE_MAPPING ENABLE=1"));
        }
        let lane = gcmd.get("LANE")?;
        let mapping = gcmd.get("MAPPING")?;
        let lane_obj = afc.lane(&lane).ok_or_else(|| gcmd.error(format!("{} is not a valid lane", lane)))?;
        for m in mapping.to_uppercase().split(',') {
            let valid = m.len() > 1 && m.starts_with('T') && m[1..].chars().all(|c| c.is_ascii_digit());
            if !valid {
                afc.error.afc_error(c, &format!("'{}' is not a valid mapping, expect T(n) format.", m), false, None)?;
                continue;
            }
            if afc.tool_cmds.borrow().contains_key(m) {
                afc.error.afc_error(c, &format!("Cannot map {} to {}, as mapping already exists.", m, lane), false, None)?;
                continue;
            }
            afc.logger.info(c, &format!("Adding {} to {}", m, lane));
            afc.tool_cmds.borrow_mut().insert(m.to_string(), lane.clone());
            let mut map = lane_obj.map.borrow().clone();
            map.push(m.to_string());
            *lane_obj.map.borrow_mut() = map;
            afc.function.register_tool_macro(&lane, m, "")?;
        }
        lane_obj.send_lane_data()?;
        afc.save_vars()
    }

    fn cmd_afc_remove_mapping(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_SPOOL_AFCSPOOL_CMD_AFC_REMOVE_MAPPING;
        if !afc.enable_multiple_mapping.get() {
            return Err(gcmd.error("Enable multiple mapping needs to be enabled to remove mappings, enable by running AFC_ENABLE_MULTIPLE_MAPPING ENABLE=1"));
        }
        let mapping = gcmd.get("MAPPING")?;
        for m in mapping.to_uppercase().split(',') {
            let lane = afc.tool_cmds.borrow_mut().shift_remove(m);
            let Some(lane) = lane else {
                afc.error.afc_error(c, &format!("Mapping {} does not exist", m), false, None)?;
                continue;
            };
            if let Some(lane_obj) = afc.lane(&lane) {
                afc.logger.info(c, &format!("Removing {} macro from {}", m, lane));
                let map: Vec<String> = lane_obj.map.borrow().iter().filter(|x| *x != m).cloned().collect();
                *lane_obj.map.borrow_mut() = map;
                if *lane_obj.current_map.borrow() == m {
                    let first = lane_obj.map.borrow().first().cloned().unwrap_or_default();
                    *lane_obj.current_map.borrow_mut() = first;
                }
                lane_obj.send_lane_data()?;
            }
            afc.gcode.register_command(m, None, false, None)?;
        }
        afc.save_vars()
    }

    fn cmd_afc_enable_multiple_mapping(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_SPOOL_AFCSPOOL_CMD_AFC_ENABLE_MULTIPLE_MAPPING;
        let enable = gcmd.get_int("ENABLE", Some(0), Lim::new())? != 0;
        if enable {
            afc.enable_multiple_mapping.set(true);
            afc.logger.info(c, "Multiple T(n) mapping per lane has been enabled along with ability to add virtual tools with AFC_ADD_MAPPING macro");
        } else {
            afc.enable_multiple_mapping.set(false);
            afc.logger.info(c, "Multiple T(n) mapping per lane and virtual tools has been disabled");
            self.reset_mapping("no")?;
        }
        afc.function.config_rewrite("AFC", "enable_multiple_mapping", &PyVal::Bool(afc.enable_multiple_mapping.get()), "")
    }

    fn cmd_set_color(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_SPOOL_AFCSPOOL_CMD_SET_COLOR;
        let Some(lane) = gcmd.get_opt("LANE") else {
            afc.logger.info(c, "No LANE Defined");
            return Ok(());
        };
        let color = gcmd.get_or("COLOR", "#000000");
        let Some(cur_lane) = afc.lane(&lane) else {
            afc.logger.info(c, &format!("{} Unknown", lane));
            return Ok(());
        };
        *cur_lane.color.borrow_mut() = PyVal::Str(format!("#{}", color.replace('#', "")));
        *cur_lane.multi_color.borrow_mut() = PyVal::List(vec![]);
        cur_lane.send_lane_data()?;
        if cur_lane.load_state() && afc.units.borrow().contains_key(&cur_lane.unit) {
            cur_lane.unit_obj()?.lane_loaded(&cur_lane, true)?;
        }
        afc.save_vars()?;
        self.set_snapmaker_filament_params(&cur_lane);
        Ok(())
    }

    fn cmd_set_weight(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_SPOOL_AFCSPOOL_CMD_SET_WEIGHT;
        let Some(lane) = gcmd.get_opt("LANE") else {
            afc.logger.info(c, "No LANE Defined");
            return Ok(());
        };
        let weight = gcmd.get_float("WEIGHT", Some(1000.0), Lim::new())?;
        let Some(cur_lane) = afc.lane(&lane) else {
            afc.logger.info(c, &format!("{} Unknown", lane));
            return Ok(());
        };
        *cur_lane.weight.borrow_mut() = PyVal::Float(weight);
        cur_lane.send_lane_data()?;
        afc.save_vars()
    }

    fn cmd_set_material(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_SPOOL_AFCSPOOL_CMD_SET_MATERIAL;
        let Some(lane) = gcmd.get_opt("LANE") else {
            afc.logger.info(c, "No LANE Defined");
            return Ok(());
        };
        let Some(cur_lane) = afc.lane(&lane) else {
            afc.logger.info(c, &format!("{} Unknown", lane));
            return Ok(());
        };
        let density = gcmd.get_float_opt("DENSITY", Lim::new())?;
        cur_lane.set_material(PyVal::Str(gcmd.get("MATERIAL")?))?;
        let d = gcmd.get_float_opt("DIAMETER", Lim::new())?;
        if let Some(d) = d {
            *cur_lane.filament_diameter.borrow_mut() = PyVal::Float(d);
        }
        let e = gcmd.get_float_opt("EMPTY_SPOOL_WEIGHT", Lim::new())?;
        if let Some(e) = e {
            *cur_lane.empty_spool_weight.borrow_mut() = PyVal::Float(e);
        }
        if let Some(d) = density {
            *cur_lane.filament_density.borrow_mut() = PyVal::Float(d);
        }
        cur_lane.send_lane_data()?;
        afc.save_vars()?;
        self.set_snapmaker_filament_params(&cur_lane);
        Ok(())
    }

    /// Tells spoolman the active spool (None clears it)
    pub fn set_active_spool(&self, id: &PyVal) -> KResult<()> {
        let afc = self.afc();
        if afc.spoolman.borrow().is_none() {
            return Ok(());
        }
        let id = if id.truthy() { PyVal::Int(super::utils::py_int_of(id)?) } else { PyVal::None };
        let args = kaliru::pydict! { "spool_id" => id };
        match afc.webhooks.call_remote_method(SPOOLMAN_REMOTE_METHOD, args) {
            Err(e) if e.is_command_error() => {
                afc.logger.error(&AFC_SPOOL_AFCSPOOL_SET_ACTIVE_SPOOL, &format!("Error trying to set active spool \n{}", e.message()));
                Ok(())
            }
            r => r,
        }
    }

    fn cmd_set_spool_id(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_SPOOL_AFCSPOOL_CMD_SET_SPOOL_ID;
        if afc.spoolman.borrow().is_none() {
            return Ok(());
        }
        let Some(lane) = gcmd.get_opt("LANE") else {
            afc.logger.info(c, "No LANE Defined");
            return Ok(());
        };
        let spool = gcmd.get_or("SPOOL_ID", "");
        let Some(cur_lane) = afc.lane(&lane) else {
            afc.logger.info(c, &format!("{} Unknown", lane));
            return Ok(());
        };
        let mut spool_id = PyVal::Str(spool.clone());
        if !spool.is_empty() {
            match super::utils::py_int_of(&PyVal::Str(spool.clone())) {
                Ok(i) => spool_id = PyVal::Int(i),
                Err(_) => {
                    afc.logger.error(c, &format!("Invalid spool ID: {}", spool));
                    return Ok(());
                }
            }
            let cur = cur_lane.spool_id.borrow().clone();
            if !cur.py_eq(&spool_id) && afc.lanes.borrow().values().any(|l| spool_id.py_eq(&l.spool_id.borrow())) {
                afc.logger.error(c, &format!("SpoolId {} already assigned to a lane, cannot assign to {}.", spool_id.str(), lane));
                return Ok(());
            }
        }
        let l = cur_lane.clone();
        let s = self.clone();
        self.set_spool_id(
            &cur_lane,
            spool_id,
            true,
            Some(Box::new(move || {
                if s.afc().current().as_deref() == Some(l.name.as_str()) {
                    s.set_active_spool(&l.spool_id.borrow().clone())?;
                }
                Ok(())
            })),
        )
    }

    /// _set_values: defaults for an unknown spool, then the next spool id
    pub fn set_values(&self, cur_lane: &Rc<AfcLane>, on_done: Option<OnDone>) -> KResult<()> {
        let afc = self.afc();
        cur_lane.auto_switch_triggered.set(false);
        if !cur_lane.remember_spool.borrow().unwrap_or(false) && matches!(*cur_lane.spool_id.borrow(), PyVal::None) {
            cur_lane.set_material(afc.default_material_type.clone().map(PyVal::Str).unwrap_or(PyVal::None))?;
            *cur_lane.color.borrow_mut() = PyVal::Str("#000000".into());
            *cur_lane.weight.borrow_mut() = PyVal::Int(1000);
        }
        let next = self.next_spool_id.borrow().clone();
        if afc.spoolman.borrow().is_some() && !matches!(next, PyVal::None) {
            *self.next_spool_id.borrow_mut() = PyVal::None;
            self.set_spool_id(cur_lane, next, true, on_done)
        } else if let Some(f) = on_done {
            f()
        } else {
            Ok(())
        }
    }

    /// clear_values: the lane's spool data
    pub fn clear_values(&self, cur_lane: &AfcLane) -> KResult<()> {
        *cur_lane.spool_id.borrow_mut() = PyVal::None;
        cur_lane.set_material(PyVal::Str(String::new()))?;
        *cur_lane.color.borrow_mut() = PyVal::Str(String::new());
        *cur_lane.multi_color.borrow_mut() = PyVal::List(vec![]);
        *cur_lane.weight.borrow_mut() = PyVal::Int(0);
        cur_lane.auto_switch_triggered.set(false);
        *cur_lane.extruder_temp.borrow_mut() = PyVal::None;
        *cur_lane.bed_temp.borrow_mut() = PyVal::None;
        cur_lane.clear_lane_data()?;
        *cur_lane.spool_vendor.borrow_mut() = PyVal::Str(String::new());
        *cur_lane.filament_name.borrow_mut() = PyVal::Str(String::new());
        Ok(())
    }

    /// set_spoolID: spoolman's data for the spool, or clears the lane
    pub fn set_spool_id(&self, cur_lane: &Rc<AfcLane>, spool_id: PyVal, save_vars: bool, on_done: Option<OnDone>) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_SPOOL_AFCSPOOL_SET_SPOOLID;
        let remember = cur_lane.remember_spool.borrow().unwrap_or(false);
        if afc.spoolman.borrow().is_some() && afc.moonraker.borrow().is_some() {
            if !matches!(spool_id, PyVal::None) && !spool_id.py_eq(&PyVal::Str(String::new())) {
                match super::utils::py_int_of(&spool_id) {
                    Ok(id) => {
                        let moonraker = afc.moonraker.borrow().clone().ok_or_else(|| no_attr("NoneType", "get_spool"))?;
                        let (s, l) = (self.rc(), cur_lane.clone());
                        moonraker.get_spool(id, Box::new(move |result| s.apply_spool_data(&l, spool_id.clone(), result, save_vars, on_done)));
                        return Ok(());
                    }
                    Err(e) => {
                        afc.error.afc_error(c, &format!("Error when trying to get Spoolman data for ID:{}, Error: {}", spool_id.str(), e.message()), false, None)?;
                    }
                }
            } else if !remember {
                self.clear_values(cur_lane)?;
            }
        } else if !remember {
            self.clear_values(cur_lane)?;
        }
        self.set_snapmaker_filament_params(cur_lane);
        if save_vars {
            afc.save_vars()?;
        }
        if let Some(f) = on_done {
            f()?;
        }
        Ok(())
    }

    /// _apply_spool_data: spoolman's answer applied to the lane
    fn apply_spool_data(&self, cur_lane: &Rc<AfcLane>, spool_id: PyVal, result: Option<PyVal>, save_vars: bool, on_done: Option<OnDone>) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_SPOOL_AFCSPOOL__APPLY_SPOOL_DATA;
        let res = (|| -> KResult<()> {
            let result = result.ok_or_else(|| KError::Py("ValueError".into(), format!("No spool data returned for SpoolID {}", spool_id.str())))?;
            let get = |d: &PyVal, k: &str| -> KResult<PyVal> {
                match d.get() {
                    PyVal::Dict(m) => m.get(k).cloned().ok_or_else(|| KError::Py("KeyError".into(), super::py_repr(k))),
                    _ => Err(KError::Py("TypeError".into(), "'NoneType' object is not subscriptable".into())),
                }
            };
            let opt = |d: &PyVal, k: &str, default: PyVal| -> PyVal {
                match d.get() {
                    PyVal::Dict(m) => m.get(k).cloned().unwrap_or(default),
                    _ => default,
                }
            };
            *cur_lane.spool_id.borrow_mut() = spool_id.clone();
            cur_lane.auto_switch_triggered.set(false);
            let filament = get(&result, "filament")?;
            cur_lane.set_material(opt(&filament, "material", PyVal::None))?;
            if !afc.ignore_spoolman_material_temps {
                *cur_lane.extruder_temp.borrow_mut() = opt(&filament, "settings_extruder_temp", PyVal::None);
            }
            *cur_lane.bed_temp.borrow_mut() = opt(&filament, "settings_bed_temp", PyVal::None);
            *cur_lane.filament_density.borrow_mut() = opt(&filament, "density", PyVal::None);
            *cur_lane.filament_diameter.borrow_mut() = opt(&filament, "diameter", PyVal::None);
            *cur_lane.empty_spool_weight.borrow_mut() = opt(&result, "spool_weight", PyVal::Int(190));
            *cur_lane.weight.borrow_mut() = opt(&result, "remaining_weight", PyVal::None);
            cur_lane.espooler.set_full_weight(opt(&result, "initial_weight", PyVal::Int(1000)));
            *cur_lane.filament_name.borrow_mut() = opt(&filament, "name", PyVal::Str(String::new()));
            let vendor = opt(&filament, "vendor", PyVal::None);
            *cur_lane.spool_vendor.borrow_mut() = PyVal::Str(String::new());
            if vendor.truthy() {
                *cur_lane.spool_vendor.borrow_mut() = opt(&vendor, "name", PyVal::None);
            }
            afc.logger.debug(c, &format!("Weight remaining for SpoolID {}: {}", spool_id.str(), cur_lane.weight.borrow().str()));
            if !afc.disable_weight_check {
                let w = cur_lane.weight.borrow().clone();
                if matches!(w, PyVal::None) || w.as_f64().is_some_and(|v| v <= 0.0) {
                    let error_str = format!("Invalid weight for spoolID: {}. Please check remaining weight before assigning.", spool_id.str());
                    afc.error.afc_error(c, &error_str, false, None)?;
                    self.clear_values(cur_lane)?;
                    return Ok(());
                }
            }
            let multi = opt(&filament, "multi_color_hexes", PyVal::None);
            if multi.truthy() {
                let parts: Vec<PyVal> = multi.str().split(',').map(|s| PyVal::Str(s.to_string())).collect();
                *cur_lane.color.borrow_mut() = PyVal::Str(format!("#{}", parts[0].str()));
                *cur_lane.multi_color.borrow_mut() = PyVal::List(parts);
            } else {
                *cur_lane.color.borrow_mut() = PyVal::Str(format!("#{}", opt(&filament, "color_hex", PyVal::None).str()));
                *cur_lane.multi_color.borrow_mut() = PyVal::List(vec![]);
            }
            cur_lane.send_lane_data()?;
            if cur_lane.load_state() && afc.units.borrow().contains_key(&cur_lane.unit) {
                cur_lane.unit_obj()?.lane_loaded(cur_lane, true)?;
            }
            Ok(())
        })();
        if let Err(e) = res {
            afc.error.afc_error(c, &format!("Error when trying to get Spoolman data for ID:{}, Error: {}", spool_id.str(), e.message()), false, None)?;
        }
        self.set_snapmaker_filament_params(cur_lane);
        if save_vars {
            afc.save_vars()?;
        }
        if let Some(f) = on_done {
            f()?;
        }
        Ok(())
    }

    fn cmd_set_runout(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_SPOOL_AFCSPOOL_CMD_SET_RUNOUT;
        let Some(lane) = gcmd.get_opt("LANE") else {
            afc.logger.info(c, "No LANE Defined");
            return Ok(());
        };
        let runout = gcmd.get_or("RUNOUT", "NONE");
        let is_none = runout.to_uppercase() == "NONE";
        if lane == runout {
            afc.logger.error(c, &format!("Lane({}) and runout({}) cannot be the same", lane, runout));
            return Ok(());
        }
        let Some(cur_lane) = afc.lane(&lane) else {
            afc.logger.error(c, &format!("Unknown lane: {}", lane));
            return Ok(());
        };
        if !is_none && afc.lane(&runout).is_none() {
            afc.logger.error(c, &format!("Unknown runout lane: {}", runout));
            return Ok(());
        }
        *cur_lane.runout_lane.borrow_mut() = if is_none { None } else { Some(runout) };
        afc.save_vars()
    }

    fn cmd_afc_reset_mapping(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let runout_opt = gcmd.get_or("RUNOUT", "yes").to_lowercase();
        self.reset_mapping(&runout_opt)
    }

    /// _reset_mapping: 1:1 T(n) maps again, numbered without gaps
    fn reset_mapping(&self, runout_opt: &str) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_SPOOL_AFCSPOOL__RESET_MAPPING;
        let lanes: Vec<Rc<AfcLane>> = afc.lanes.borrow().values().cloned().collect();
        let previously_used: std::collections::BTreeSet<String> = lanes.iter().flat_map(|l| l.map.borrow().clone()).filter(|c| c != "NONE").collect();
        let manually_assigned: std::collections::BTreeSet<String> = lanes.iter().flat_map(|l| l.config_map.clone()).collect();
        let total_lanes: usize = afc.units.borrow().values().map(|u| u.lanes.borrow().len()).sum();
        let mut auto_cmds: Vec<String> =
            (0..total_lanes + manually_assigned.len()).map(|i| format!("T{}", i)).filter(|t| !manually_assigned.contains(t)).take(total_lanes).collect();
        let units: Vec<Rc<super::unit::AfcUnit>> = afc.units.borrow().values().cloned().collect();
        for unit in &units {
            let ulanes: Vec<Rc<AfcLane>> = unit.lanes.borrow().values().cloned().collect();
            for lane in ulanes {
                lane.map.borrow_mut().clear();
                let map_cmd = match lane.config_map.first() {
                    Some(m) => m.clone(),
                    None => {
                        if auto_cmds.is_empty() {
                            return Err(KError::Py("IndexError".into(), "pop from empty list".into()));
                        }
                        auto_cmds.remove(0)
                    }
                };
                if !previously_used.contains(&map_cmd) {
                    afc.function.register_tool_macro(&lane.name, &map_cmd, "")?;
                }
                afc.tool_cmds.borrow_mut().insert(map_cmd.clone(), lane.name.clone());
                *lane.current_map.borrow_mut() = map_cmd.clone();
                *lane.map.borrow_mut() = vec![map_cmd];
            }
        }
        let new_cmds: std::collections::BTreeSet<String> =
            units.iter().flat_map(|u| u.lanes.borrow().values().map(|l| l.map.borrow()[0].clone()).collect::<Vec<_>>()).collect();
        let mut stale: Vec<String> = previously_used.difference(&new_cmds).cloned().collect();
        stale.sort_by_key(|a| super::utils::natural_sort_key(a));
        if !stale.is_empty() {
            afc.logger.info(c, &format!("{} remain, removing these mappings", stale.join(", ")));
            for cmd in &stale {
                afc.tool_cmds.borrow_mut().shift_remove(cmd);
                afc.gcode.register_command(cmd, None, false, None)?;
            }
        }
        for unit in &units {
            let ulanes: Vec<Rc<AfcLane>> = unit.lanes.borrow().values().cloned().collect();
            for lane in ulanes {
                lane.send_lane_data()?;
            }
        }
        if runout_opt != "no" {
            for lane in &lanes {
                *lane.runout_lane.borrow_mut() = None;
            }
        }
        afc.save_vars()?;
        afc.logger.info(c, &format!("Tool mappings reset{}", if runout_opt == "no" { "" } else { " and runout lanes reset" }));
        Ok(())
    }

    fn cmd_set_next_spool_id(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_SPOOL_AFCSPOOL_CMD_SET_NEXT_SPOOL_ID;
        let spool = gcmd.get_or("SPOOL_ID", "");
        let previous_id = self.next_spool_id.borrow().clone();
        if !spool.is_empty() {
            match super::utils::py_int_of(&PyVal::Str(spool.clone())) {
                Ok(i) => *self.next_spool_id.borrow_mut() = PyVal::Int(i),
                Err(_) => {
                    afc.logger.error(c, &format!("Invalid spool ID: {}", spool));
                    *self.next_spool_id.borrow_mut() = PyVal::None;
                }
            }
        } else {
            *self.next_spool_id.borrow_mut() = PyVal::None;
        }
        let next = self.next_spool_id.borrow().str();
        if previous_id.truthy() {
            afc.logger.info(c, &format!("Spool ID '{}' being overwritten for next load: '{}'", previous_id.str(), next));
        } else {
            afc.logger.info(c, &format!("Spool ID set for next load: '{}'", next));
        }
        Ok(())
    }
}

impl PrinterObject for AfcSpool {}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(AfcSpool::new(config))
}

