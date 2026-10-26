// AFC statistics kept in moonraker's database (port of AFC_stats.py; a stat
// without moonraker is AFC_extruder's _LocalAFCStat)
//
// Copyright (C) 2024-2026 Armored Turtle
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::logger::AfcLogger;
use super::utils::{self, Moonraker};
use super::Afc;
use kaliru::error::KResult;
use kaliru::pyval::PyVal;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

/// One stat value (an int, a float or a date string)
pub struct StatVar {
    parent_name: RefCell<String>,
    name: String,
    moonraker: Option<Rc<Moonraker>>,
    pub new_average: Cell<bool>,
    value: RefCell<PyVal>,
}

/// python's value of a stat read back from the database: int(), float(), as is
fn coerce(value: PyVal) -> PyVal {
    if let Ok(i) = utils::py_int_of(&value) {
        return PyVal::Int(i);
    }
    if let Ok(f) = utils::py_float_of(&value) {
        return PyVal::Float(f);
    }
    value
}

impl StatVar {
    /// _LocalAFCStat(value)
    pub fn local(value: PyVal) -> StatVar {
        StatVar { parent_name: RefCell::new(String::new()), name: String::new(), moonraker: None, new_average: Cell::new(false), value: RefCell::new(value) }
    }

    /// AFCStats_var(parent_name, name, data, moonraker, new_parent_name, new_average)
    pub fn new(parent_name: &str, name: &str, data: Option<&PyVal>, moonraker: Rc<Moonraker>, new_parent_name: &str, new_average: bool) -> StatVar {
        let s = StatVar {
            parent_name: RefCell::new(parent_name.to_string()),
            name: name.to_string(),
            moonraker: Some(moonraker.clone()),
            new_average: Cell::new(new_average),
            value: RefCell::new(PyVal::Int(0)),
        };
        let multi_parent: Vec<&str> = parent_name.split('.').collect();
        let new_multi_parent: Vec<&str> = new_parent_name.split('.').collect();
        let has = |k: &str| matches!(data.map(|d| d.get()), Some(PyVal::Dict(d)) if d.contains_key(k));
        if let Some(data) = data {
            if has(new_multi_parent[0]) {
                s.get_value(data, &new_multi_parent);
            } else if has(multi_parent[0]) {
                s.get_value(data, &multi_parent);
            } else {
                moonraker.logger.debug(
                    &AFC_STATS_AFCSTATS_VAR___INIT__,
                    &format!("No data in database for {}.{}:{}", parent_name, name, super::py_bool(true)),
                );
                *s.value.borrow_mut() = PyVal::Int(0);
            }
        }
        if !new_parent_name.is_empty() {
            *s.parent_name.borrow_mut() = new_parent_name.to_string();
            if has(multi_parent[0]) {
                s.update_database();
                let mut key: Vec<&str> = multi_parent.clone();
                key.push(name);
                moonraker.remove_database_entry(utils::AFC_STATS_KEY, &key.join("."));
            }
        }
        s
    }

    fn get_value(&self, data: &PyVal, parent_name: &[&str]) {
        let logger = &self.moonraker.as_ref().expect("moonraker").logger;
        let child = |d: &PyVal, k: &str| -> Option<PyVal> {
            match d.get() {
                PyVal::Dict(m) => m.get(k).cloned(),
                _ => None,
            }
        };
        let check_and_return = |d: &PyVal| child(d, &self.name).unwrap_or(PyVal::Str("0".into()));
        let value = match parent_name.len() {
            1 => check_and_return(&child(data, parent_name[0]).unwrap_or(PyVal::None)),
            2 => {
                let p = child(data, parent_name[0]).unwrap_or(PyVal::None);
                match child(&p, parent_name[1]) {
                    Some(pp) => check_and_return(&pp),
                    None => {
                        logger.debug(
                            &AFC_STATS_AFCSTATS_VAR__GET_VALUE,
                            &format!("No data in database for {}.{}:{}", parent_name.join("."), self.name, super::py_bool(true)),
                        );
                        PyVal::Int(0)
                    }
                }
            }
            _ => {
                logger.error(&AFC_STATS_AFCSTATS_VAR__GET_VALUE, "Cannot have more than two parent names for stats");
                PyVal::Int(0)
            }
        };
        *self.value.borrow_mut() = coerce(value);
    }

    pub fn value(&self) -> PyVal {
        self.value.borrow().clone()
    }

    pub fn set_value(&self, v: PyVal) {
        *self.value.borrow_mut() = v;
    }

    /// average_time(value): a running sum (new calculation) or halving average
    pub fn average_time(&self, value: f64) {
        let cur = self.value();
        let new = if cur.as_f64().is_some_and(|v| v > 0.0) {
            let sum = utils::py_add(&cur, &PyVal::Float(value)).unwrap_or(PyVal::Float(value));
            if !self.new_average.get() {
                PyVal::Float(sum.as_f64().unwrap_or(0.0) / 2.0)
            } else {
                sum
            }
        } else {
            PyVal::Float(value)
        };
        *self.value.borrow_mut() = new;
        self.update_database();
    }

    pub fn get_average(&self, total: f64) -> f64 {
        let v = self.value().as_f64().unwrap_or(0.0);
        if self.new_average.get() && total > 0.0 {
            v / total
        } else {
            v
        }
    }

    pub fn increase_count(&self) -> KResult<()> {
        let v = utils::py_add(&self.value(), &PyVal::Int(1))?;
        *self.value.borrow_mut() = v;
        self.update_database();
        Ok(())
    }

    pub fn reset_count(&self) -> KResult<()> {
        *self.value.borrow_mut() = PyVal::Int(0);
        self.update_database();
        self.new_average.set(true);
        Ok(())
    }

    pub fn update_database(&self) {
        if let Some(m) = &self.moonraker {
            m.update_afc_stats(&format!("{}.{}", self.parent_name.borrow(), self.name), self.value());
        }
    }

    pub fn set_current_time(&self) -> KResult<()> {
        *self.value.borrow_mut() = PyVal::Str(utils::datetime_now().strftime("%Y-%m-%d %H:%M"));
        self.update_database();
        Ok(())
    }

    /// str(stat)
    pub fn str(&self) -> String {
        self.value().str()
    }
}

/// The AFC-wide statistics (toolchanges, errors, average times)
pub struct AfcStats {
    logger: Rc<AfcLogger>,
    multiple_tools: bool,
    tc_without_error: StatVar,
    tc_last_load_error: StatVar,
    total_load_errors: StatVar,
    total_unload_errors: StatVar,
    new_average_calc: StatVar,
    pub average_toolchange_time: StatVar,
    pub average_tool_unload_time: StatVar,
    pub average_tool_load_time: StatVar,
    pub average_tool_swap_time: Option<StatVar>,
}

impl AfcStats {
    pub fn new(moonraker: Rc<Moonraker>, logger: Rc<AfcLogger>, multiple_tools: bool) -> AfcStats {
        let values = moonraker.get_afc_stats();
        let v = values.as_ref();
        let m = || moonraker.clone();
        let tc_without_error = StatVar::new("toolchange_count", "changes_without_error", v, m(), "error_stats", false);
        let tc_last_load_error = StatVar::new("toolchange_count", "last_load_error", v, m(), "error_stats", false);
        let total_load_errors = StatVar::new("toolchange_count", "total_load_errors", v, m(), "error_stats", false);
        let total_unload_errors = StatVar::new("toolchange_count", "total_unload_errors", v, m(), "error_stats", false);
        let has_average = matches!(v.map(|d| d.get()), Some(PyVal::Dict(d)) if d.contains_key("average_time"));
        let new_average_calc_default = if has_average { 0 } else { 1 };
        let new_average_calc = StatVar::new("average_time", "new_average_calc", v, m(), "", false);
        if new_average_calc.value().py_eq(&PyVal::Int(0)) {
            new_average_calc.set_value(PyVal::Int(new_average_calc_default));
            new_average_calc.update_database();
        }
        let na = new_average_calc.value().truthy();
        let average_toolchange_time = StatVar::new("average_time", "tool_change", v, m(), "", na);
        let average_tool_unload_time = StatVar::new("average_time", "tool_unload", v, m(), "", na);
        let average_tool_load_time = StatVar::new("average_time", "tool_load", v, m(), "", na);
        if tc_last_load_error.value().py_eq(&PyVal::Int(0)) {
            tc_last_load_error.set_value(PyVal::Str("N/A".into()));
            tc_last_load_error.update_database();
        }
        let mut average_tool_swap_time = None;
        if multiple_tools {
            logger.debug(&AFC_STATS_AFCSTATS___INIT__, "Multiple tools detected for stats");
            average_tool_swap_time = Some(StatVar::new("average_time", "tool_swap", v, m(), "", na));
        }
        AfcStats {
            logger,
            multiple_tools,
            tc_without_error,
            tc_last_load_error,
            total_load_errors,
            total_unload_errors,
            new_average_calc,
            average_toolchange_time,
            average_tool_unload_time,
            average_tool_load_time,
            average_tool_swap_time,
        }
    }

    pub fn increase_toolchange_wo_error(&self) -> KResult<()> {
        self.tc_without_error.increase_count()
    }

    fn reset_toolchange_wo_error(&self) -> KResult<()> {
        self.tc_without_error.reset_count()?;
        self.tc_last_load_error.set_current_time()
    }

    pub fn increase_load_error_count(&self, afc: &Afc) -> KResult<()> {
        if afc.testing {
            return Ok(());
        }
        self.reset_toolchange_wo_error()?;
        self.total_load_errors.increase_count()
    }

    pub fn increase_unload_error_count(&self, afc: &Afc) -> KResult<()> {
        if afc.testing {
            return Ok(());
        }
        self.reset_toolchange_wo_error()?;
        self.total_unload_errors.increase_count()
    }

    pub fn reset_average_times(&self) {
        let _ = self.average_toolchange_time.reset_count();
        let _ = self.average_tool_unload_time.reset_count();
        let _ = self.average_tool_load_time.reset_count();
        if let Some(s) = &self.average_tool_swap_time {
            let _ = s.reset_count();
        }
        self.new_average_calc.set_value(PyVal::Int(1));
        self.new_average_calc.update_database();
    }

    /// AFC_STATS: the stats table to the console
    pub fn print_stats(&self, afc: &Afc, short: bool) -> KResult<()> {
        const SECTIONAL_LENGTH: usize = 42;
        let (max_width, max_span) = if short { (44usize, 42usize) } else { (87usize, SECTIONAL_LENGTH * 2 + 1) };
        let add_end = || if short { "|\n" } else { "" };
        let l = |s: &str, w: usize| format!("{:<w$}", s, w = w);
        let r = |s: &str, w: usize| format!("{:>w$}", s, w = w);
        let ctr = |s: &str, w: usize| utils::py_center(s, w);
        let fmt = |v: &PyVal| v.str();
        let mut total_selected = PyVal::Int(0);
        let mut total_unselected = PyVal::Int(0);
        let mut total_changes = PyVal::Int(0);
        let mut total_unload = PyVal::Int(0);
        let mut total_load = PyVal::Int(0);
        let max_lane_name_size = afc.lanes.borrow().keys().map(|k| k.chars().count()).max().unwrap_or(0);

        let mut overall_str = format!("{}\n", "-".repeat(max_width));
        overall_str += &format!("|{}|\n", ctr("Overall Stats", max_width - 2));
        overall_str += &format!("|{} : {}", r("Changes without error", 22), l(&fmt(&self.tc_without_error.value()), 17));
        overall_str += add_end();
        overall_str += &format!("|{} : {}|\n", r("Last error date", 22), l(&fmt(&self.tc_last_load_error.value()), 17));
        overall_str += &format!("|{} : {}", r("Total Load Errors", 22), l(&fmt(&self.total_load_errors.value()), 17));
        overall_str += add_end();
        overall_str += &format!("|{} : {}|\n", r("Total Unload Errors", 22), l(&fmt(&self.total_unload_errors.value()), 17));

        let dash = format!("|{}|\n", "-".repeat(max_width - 2));
        let mut print_str = dash.clone();
        if !short {
            print_str += &format!("|{}", ctr("Toolchanges", SECTIONAL_LENGTH));
            print_str += &format!("|{}|\n", ctr("Cut", SECTIONAL_LENGTH));
            print_str += &dash;
        }
        for extruder in afc.tools.borrow().values() {
            let stats = &extruder.estats;
            let mut cut_str = format!("|{}|\n", ctr(&format!("{} Cut", extruder.name), max_span));
            let cuts_since_change = format!("{}/{}", fmt(&stats.cut_total_since_changed().value()), stats.cut_threshold_for_warning);
            total_changes = utils::py_add(&total_changes, &stats.tc_total().value())?;
            total_unload = utils::py_add(&total_unload, &stats.tc_tool_unload().value())?;
            total_load = utils::py_add(&total_load, &stats.tc_tool_load().value())?;
            let extruder_lbl = if short { format!("{} Toolchanges", extruder.name) } else { extruder.name.clone() };
            print_str += &format!("|{}|\n", ctr(&extruder_lbl, max_span));
            print_str += &format!("|{} : {}", r("Successful Changes", 22), l(&fmt(&stats.tc_total().value()), 17));
            print_str += add_end();
            let s = format!("|{} : {}|\n", r("Total", 22), l(&fmt(&stats.cut_total().value()), 17));
            if short {
                cut_str += &s;
            } else {
                print_str += &s;
            }
            print_str += &format!("|{} : {}", r("Tool Unload", 22), l(&fmt(&stats.tc_tool_unload().value()), 17));
            print_str += add_end();
            let s = format!("|{} : {}|\n", r("Total since changed", 22), l(&cuts_since_change, 17));
            if short {
                cut_str += &s;
            } else {
                print_str += &s;
            }
            print_str += &format!("|{} : {}", r("Tool Load", 22), l(&fmt(&stats.tc_tool_load().value()), 17));
            print_str += add_end();
            let s = format!("|{} : {}|\n", r("Blade last changed", 22), l(&fmt(&stats.last_blade_changed().value()), 17));
            if short {
                cut_str += &s;
            } else {
                print_str += &s;
            }
            if self.multiple_tools {
                total_selected = utils::py_add(&total_selected, &stats.tool_selected().value())?;
                total_unselected = utils::py_add(&total_unselected, &stats.tool_unselected().value())?;
                print_str += &format!("|{} : {}", r("Tool Unselected", 22), l(&fmt(&stats.tool_unselected().value()), 17));
                if !short {
                    print_str += &format!("|{}|\n", ctr(" ", SECTIONAL_LENGTH));
                } else {
                    print_str += add_end();
                }
                print_str += &format!("|{} : {}", r("Tool Selected", 22), l(&fmt(&stats.tool_selected().value()), 17));
                if !short {
                    print_str += &format!("|{}|\n", ctr(" ", SECTIONAL_LENGTH));
                } else {
                    print_str += add_end();
                }
            }
            if short {
                print_str += &dash;
                print_str += &cut_str;
            }
            print_str += &dash;
        }
        let avg = |label: &str, v: f64| format!("{} : {:4.2}s", r(label, 22), v);
        let avg_tool_load = avg("Avg Tool Load", self.average_tool_load_time.get_average(total_load.as_f64().unwrap_or(0.0)));
        let avg_tool_unload = avg("Avg Tool Unload", self.average_tool_unload_time.get_average(total_unload.as_f64().unwrap_or(0.0)));
        let avg_tool_change = avg("Avg Tool Change", self.average_toolchange_time.get_average(total_changes.as_f64().unwrap_or(0.0)));
        let avg_tool_swap = match &self.average_tool_swap_time {
            Some(s) => avg("Avg Tool Swap", s.get_average(total_selected.as_f64().unwrap_or(0.0))),
            None => String::new(),
        };
        let mut avg_str = format!("|{}", l(&avg_tool_load, SECTIONAL_LENGTH));
        avg_str += add_end();
        avg_str += &format!("|{}|\n", l(&avg_tool_unload, SECTIONAL_LENGTH));
        avg_str += &format!("|{}", l(&avg_tool_change, SECTIONAL_LENGTH));
        avg_str += add_end();
        if !short || !avg_tool_swap.is_empty() {
            avg_str += &format!("|{}|\n", l(&avg_tool_swap, SECTIONAL_LENGTH));
        }
        if self.multiple_tools {
            avg_str += &format!("|{} : {}", r("Total Load", 22), l(&total_load.str(), 17));
            avg_str += add_end();
            avg_str += &format!("|{} : {}|\n", r("Total Unload", 22), l(&total_unload.str(), 17));
            avg_str += &format!("|{} : {}", r("Total Selected", 22), l(&total_selected.str(), 17));
            avg_str += add_end();
            avg_str += &format!("|{} : {}|\n", r("Total Unselected", 22), l(&total_unselected.str(), 17));
        }
        let mut print_str = overall_str + &avg_str + &print_str;

        let name_width = max_lane_name_size.max(9);
        let name_count_width = name_width + " : Lane change count: ".len() + 7;
        let long_format_pad = " ".repeat(SECTIONAL_LENGTH.saturating_sub(name_count_width) / 2);
        let mut strings = vec![];
        for unit in afc.units.borrow().values() {
            for lane in unit.lanes.borrow().values() {
                let espooler_stats = lane.espooler.get_spooler_stats(short)?;
                let count = lane.lane_load_count()?.value();
                let mut string = format!("{} : Lane change count: {}", r(&lane.name, name_width), r(&fmt(&count), 7));
                if short {
                    string = format!("|{}|\n", ctr(&string, max_span));
                    if !espooler_stats.is_empty() {
                        string += &format!("|{}|\n", ctr(&espooler_stats, max_width - 3));
                    }
                } else {
                    string = long_format_pad.clone() + &string;
                    if !espooler_stats.is_empty() {
                        string += &format!(" {}", espooler_stats);
                    }
                }
                strings.push(string);
            }
        }
        if short {
            print_str += &strings.concat();
        } else {
            let mut temp_str = String::new();
            let end_string = |print_str: &mut String, temp_str: &mut String| {
                *print_str += &format!("{:<86}|\n", temp_str);
                temp_str.clear();
            };
            for s in &strings {
                if temp_str.chars().count() > 60 {
                    end_string(&mut print_str, &mut temp_str);
                }
                if s.chars().count() > 60 {
                    if !temp_str.is_empty() {
                        end_string(&mut print_str, &mut temp_str);
                    }
                    print_str += &format!("|{}  |\n", l(s, max_width - 4));
                } else {
                    temp_str += &format!("|{}", l(s, SECTIONAL_LENGTH));
                }
            }
            if !temp_str.is_empty() {
                end_string(&mut print_str, &mut temp_str);
            }
        }
        print_str += &format!("{}\n", "-".repeat(max_width));
        self.logger.raw(&AFC_STATS_AFCSTATS_PRINT_STATS, &print_str);
        Ok(())
    }
}
