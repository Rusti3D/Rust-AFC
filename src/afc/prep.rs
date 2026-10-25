// AFC's PREP: restores the saved lane state and checks every lane at
// startup (port of AFC_prep.py)
//
// Copyright (C) 2024-2026 Armored Turtle
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::{error, py_repr, Afc};
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::gcode::GCodeCommand;
use kaliru::printer::{PObj, Printer, PrinterObject};
use kaliru::pyval::PyVal;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

/// [AFC_prep]
pub struct AfcPrep {
    me: Weak<AfcPrep>,
    printer: Rc<Printer>,
    afc: RefCell<Weak<Afc>>,
    delay: f64,
    enable: bool,
    dis_pause_resume_macro: bool,
    dis_unload_macro: bool,
    get_td1_data: bool,
    rename_occurred: Cell<bool>,
    assign_tcmd: Cell<bool>,
}

impl AfcPrep {
    fn new(config: &ConfigWrapper) -> KResult<Rc<AfcPrep>> {
        let printer = config.get_printer();
        let delay = config.getfloat("delay_time", 0.1, Lim::new().min(0.0))?;
        let enable = config.getboolean("enable", false)?;
        let dis_pause_resume_macro = config.getboolean("disable_pause_resume_remapping", false)?;
        let dis_unload_macro = config.getboolean("disable_unload_filament_remapping", false)?;
        let get_td1_data = config.getboolean("capture_td1_data", false)?;
        let p = Rc::new_cyclic(|me| AfcPrep {
            me: me.clone(),
            printer: printer.clone(),
            afc: RefCell::new(Weak::new()),
            delay,
            enable,
            dis_pause_resume_macro,
            dis_unload_macro,
            get_td1_data,
            rename_occurred: Cell::new(false),
            assign_tcmd: Cell::new(true),
        });
        let pp = p.clone();
        kaliru::events::KlippyConnect::on(&printer, move || pp.handle_connect());
        Ok(p)
    }

    fn afc(&self) -> Rc<Afc> {
        self.afc.borrow().upgrade().expect("AFC")
    }

    fn handle_connect(&self) -> KResult<()> {
        let afc = self.printer.lookup::<Afc>("AFC")?;
        *self.afc.borrow_mut() = Rc::downgrade(&afc);
        let me = self.me.upgrade().expect("AFC_prep");
        afc.gcode.register_command("PREP", Some(Rc::new(move |g| me.prep(g))), false, None)?;
        Ok(())
    }

    /// RESUME / PAUSE -> AFC_RESUME / AFC_PAUSE, UNLOAD_FILAMENT -> TOOL_UNLOAD
    fn rename_macros(&self) -> KResult<()> {
        if self.rename_occurred.replace(true) {
            return Ok(());
        }
        let afc = self.afc();
        if !self.dis_pause_resume_macro {
            let e = afc.error.clone();
            afc.function.rename(error::BASE_RESUME_NAME, error::AFC_RENAME_RESUME_NAME, Rc::new(move |g| e.cmd_afc_resume(g)), error::CMD_AFC_RESUME_HELP)?;
            let e = afc.error.clone();
            // (python passes the RESUME help for PAUSE too)
            afc.function.rename(error::BASE_PAUSE_NAME, error::AFC_RENAME_PAUSE_NAME, Rc::new(move |g| e.cmd_afc_pause(g)), error::CMD_AFC_RESUME_HELP)?;
        }
        if !self.dis_unload_macro {
            let a = afc.clone();
            afc.function.rename(super::BASE_UNLOAD_FILAMENT, super::RENAMED_UNLOAD_FILAMENT, Rc::new(move |g| a.cmd_tool_unload(g)), super::CMD_TOOL_UNLOAD_HELP)?;
        }
        Ok(())
    }

    fn td1_prep(&self, overall_status: bool) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_PREP_AFCPREP__TD1_PREP;
        let td1_present = afc.td1_present()?;
        let capture_td1_data = self.get_td1_data && td1_present;
        let mut any_td1_error = false;
        if td1_present {
            afc.logger.info(c, "Found TD-1 device connected to printer");
            any_td1_error = afc.function.check_for_td1_error(None, true)?.0;
        }
        if let Some(cur) = afc.function.get_current_lane_obj() {
            cur.unit_obj()?.select_lane(&cur, false)?;
            if capture_td1_data {
                afc.logger.info(c, "Cannot capture TD-1 data during PREP since toolhead is loaded");
            }
        } else if capture_td1_data {
            if !overall_status {
                afc.logger.info(c, "Cannot capture TD-1 data, not all of PREP succeeded");
            } else if any_td1_error {
                afc.logger.error(c, "Error with a TD-1 device, not collecting data during prep");
            } else {
                afc.logger.info(c, "Capturing TD-1 data for all loaded lanes");
                let lanes: Vec<_> = afc.lanes.borrow().values().cloned().collect();
                for lane in lanes {
                    if lane.td1_device_id.borrow().is_some() && lane.load_state() && lane.prep_state.get() && !lane.get_td1_data()?.0 {
                        break;
                    }
                }
                afc.logger.info(c, "Done capturing TD-1 data");
            }
        }
        Ok(())
    }

    /// PREP
    fn prep(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let afc = self.afc();
        let c = &AFC_PREP_AFCPREP_PREP;
        let mut overall_status = true;
        while self.printer.state_message() != "Printer is ready" {
            afc.reactor.pause(afc.reactor.monotonic() + 1.0)?;
        }
        self.rename_macros()?;
        afc.handle_moonraker_connect()?;

        // load Unit stored variables
        let path = format!("{}.unit", afc.var_file);
        let mut units = PyVal::dict();
        let size = std::fs::metadata(&path).map(|m| m.len()).ok();
        if size.is_some_and(|s| s > 0) {
            let text = std::fs::read_to_string(&path).map_err(|e| KError::Py(kaliru::pyfmt::py_oserror_class(&e).into(), kaliru::pyfmt::py_oserror(&e, &path)))?;
            match PyVal::json_loads(&text) {
                Ok(v) => units = v,
                Err(e) => {
                    afc.error.afc_error(
                        c,
                        &format!("Error when trying to open and decode {}.unit file.\nPlease fix file or delete if file is empty, then restart klipper.", afc.var_file),
                        false,
                        None,
                    )?;
                    afc.logger.error_opt(c, "", Some(&e), "");
                    return Ok(());
                }
            }
        } else {
            let mut error_string = format!("Error: {}.unit file not found. Please check the path in the ", afc.var_file);
            error_string += "AFC.cfg file and make sure the file and path exists.";
            afc.error.afc_error(c, &error_string, false, None)?;
        }
        let key = |d: &PyVal, k: &str| -> Option<PyVal> {
            match d.get() {
                PyVal::Dict(m) => m.get(k).cloned(),
                _ => None,
            }
        };
        let system = key(&units, "system");
        let extruders_saved = system.as_ref().and_then(|s| key(s, "extruders"));

        let tools: Vec<_> = afc.tools.borrow().values().cloned().collect();
        for extruder_obj in &tools {
            extruder_obj.set_status_led(&afc.led_tool_unloaded)?;
            if extruder_obj.on_shuttle() && afc.function.get_current_extruder()?.as_deref() != Some(extruder_obj.th_extruder_name.as_str()) {
                afc.gcode.run_script_from_command(&format!("ACTIVATE_EXTRUDER EXTRUDER={}", extruder_obj.th_extruder_name))?;
            }
            if let Some(saved) = extruders_saved.as_ref().and_then(|e| key(e, &extruder_obj.name)) {
                if let Some(ll) = key(&saved, "lane_loaded").filter(|v| v.truthy()) {
                    *extruder_obj.lane_loaded.borrow_mut() = Some(ll.str());
                }
            }
        }
        afc.print_version(true);

        let lanes: Vec<_> = afc.lanes.borrow().values().cloned().collect();
        for cur_lane in &lanes {
            let unit = afc.units.borrow().get(&cur_lane.unit).cloned().ok_or_else(|| KError::Py("KeyError".into(), py_repr(&cur_lane.unit)))?;
            if !unit.lanes.borrow().contains_key(&cur_lane.name) {
                return Err(super::no_attr("dict", "append"));
            }
            let Some(saved) = key(&units, &cur_lane.unit).and_then(|u| key(&u, &cur_lane.name)) else { continue };
            let get = |k: &str| key(&saved, k);
            if let Some(v) = get("spool_id") {
                *cur_lane.spool_id.borrow_mut() = v;
            }
            if afc.spoolman.borrow().is_some() && cur_lane.spool_id.borrow().truthy() {
                let id = cur_lane.spool_id.borrow().clone();
                afc.spool.set_spool_id(cur_lane, id, false, None)?;
            } else {
                if let Some(v) = get("material") {
                    cur_lane.set_material(v)?;
                }
                if let Some(v) = get("color") {
                    *cur_lane.color.borrow_mut() = v;
                }
                if let Some(v) = get("weight") {
                    *cur_lane.weight.borrow_mut() = v;
                }
                if let Some(v) = get("density") {
                    *cur_lane.filament_density.borrow_mut() = v;
                }
                if let Some(v) = get("diameter") {
                    *cur_lane.filament_diameter.borrow_mut() = v;
                }
                if let Some(v) = get("empty_spool_weight") {
                    *cur_lane.empty_spool_weight.borrow_mut() = v;
                }
                if let Some(v) = get("bed_temp") {
                    *cur_lane.bed_temp.borrow_mut() = v;
                }
                if let Some(v) = get("extruder_temp") {
                    *cur_lane.extruder_temp.borrow_mut() = v;
                }
                cur_lane.need_purge.set(get("need_purge").map(|v| v.truthy()).unwrap_or(false));
                for slot in [&cur_lane.bed_temp, &cur_lane.extruder_temp, &cur_lane.weight] {
                    let value = slot.borrow().clone();
                    let new = match &value {
                        PyVal::Str(s) if s.is_empty() || s == "NONE" => PyVal::None,
                        PyVal::Float(_) => value.clone(),
                        v if v.truthy() => match super::utils::py_float_of(v) {
                            Ok(f) => PyVal::Float(f),
                            Err(_) => PyVal::Int(0),
                        },
                        _ => PyVal::Int(0),
                    };
                    *slot.borrow_mut() = new;
                }
            }
            if let Some(v) = get("runout_lane") {
                *cur_lane.runout_lane.borrow_mut() = if matches!(v, PyVal::None) { None } else { Some(v.str()) };
            }
            if cur_lane.runout_lane.borrow().as_deref().is_some_and(|r| r.is_empty() || r == "NONE") {
                *cur_lane.runout_lane.borrow_mut() = None;
            }
            if let Some(temp_map) = get("map").filter(|v| v.truthy()) {
                let m: Vec<String> = match &temp_map {
                    PyVal::Str(s) => s.replace(' ', "").split(',').map(|s| s.to_string()).collect(),
                    v => v.items().unwrap_or_default().iter().map(|x| x.str()).collect(),
                };
                *cur_lane.map.borrow_mut() = m;
            }
            *cur_lane.current_map.borrow_mut() = get("current_map").map(|v| v.str()).unwrap_or_default();
            for map_str in cur_lane.map.borrow().iter() {
                if map_str != "NONE" {
                    afc.tool_cmds.borrow_mut().insert(map_str.clone(), cur_lane.name.clone());
                }
            }
            if get("hub_loaded").is_some() {
                return Err(super::no_attr("str", "loaded_to_hub"));
            }
            if let Some(v) = get("loaded_to_hub") {
                cur_lane.loaded_to_hub.set(v.truthy());
            }
            if let Some(v) = get("tool_loaded") {
                cur_lane.tool_loaded.set(v.truthy());
            }
            if let Some(v) = get("td1_data") {
                *cur_lane.td1_data.borrow_mut() = v;
            }
        }

        let units_list: Vec<(String, _)> = afc.units.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect();
        for (name, cur_unit) in units_list {
            afc.logger.info(c, &format!("{} {} Prepping lanes", cur_unit.type_name(), name));
            let mut lane_check = true;
            let ulanes: Vec<_> = cur_unit.lanes.borrow().values().cloned().collect();
            for lane in ulanes {
                if !cur_unit.system_test(&lane, self.delay, self.assign_tcmd.get(), self.enable)? {
                    lane_check = false;
                }
            }
            if lane_check {
                afc.logger.raw(c, &cur_unit.logo.borrow());
            } else {
                afc.logger.raw(c, &cur_unit.logo_error.borrow());
            }
            overall_status = overall_status && lane_check;
        }
        if afc.get_bypass_state() {
            afc.logger.info(c, "Filament loaded in bypass, toolchanges deactivated");
        }
        self.td1_prep(overall_status)?;
        if let Some(cur) = afc.function.get_current_lane_obj() {
            cur.unit_obj()?.select_lane(&cur, true)?;
            cur.sync_to_extruder(true)?;
        }
        // Restore the virtual bypass state
        let bypass = afc.bypass.borrow().clone().ok_or_else(|| super::no_attr("NoneType", "name"))?;
        let mut bypass_name = "Bypass";
        if bypass.name.contains("virtual") {
            bypass_name = "Virtual bypass";
            let enabled = system.as_ref().and_then(|s| key(s, "bypass")).map(|b| key(&b, "enabled").map(|v| v.truthy()).ok_or_else(|| KError::Py("KeyError".into(), "'enabled'".into())));
            let state = match enabled {
                Some(r) => r?,
                None => false,
            };
            bypass.filament_present.set(state);
            bypass.sensor_enabled.set(state);
        }
        if bypass.filament_present.get() {
            afc.logger.raw(c, &format!("<span class=warning--text>{} enabled</span>", bypass_name));
        }
        for extruder in &tools {
            extruder.estats.check_cut_threshold(&afc)?;
        }
        if afc.current().is_none() {
            afc.spool.set_active_spool(&PyVal::None)?;
        }
        self.assign_tcmd.set(false);
        afc.prep_done.set(true);
        afc.save_vars()?;
        for (buffer_name, buffer_obj) in afc.buffers.borrow().iter() {
            if buffer_obj.advance_state.get() && buffer_obj.trailing_state.get() {
                afc.logger.raw(
                    c,
                    &format!(
                        "<span class=warning--text>Warning: Both advance and trailing switches are triggered on Buffer {}. Please check your buffer switches or configuration.</span>",
                        buffer_name
                    ),
                );
            }
        }
        let error_str = afc.verify_macro_positions();
        if !error_str.is_empty() {
            afc.logger.error(c, &error_str);
        }
        Ok(())
    }
}

impl PrinterObject for AfcPrep {}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(AfcPrep::new(config)?)
}
