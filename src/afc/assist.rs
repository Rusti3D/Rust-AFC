// A lane's spooler assist (port of AFC_assist.py's Espooler for lanes
// without spooler motors: native_scope leaves afc_motor_* to the python AFC)
//
// Copyright (C) 2024-2026 Armored Turtle
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::lane::AfcLane;
use super::stats::StatVar;
use super::{no_attr, Afc};
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::pyval::PyVal;
use std::cell::RefCell;
use std::rc::Rc;

/// AFCEspoolerStats: the N20 motors' run times
pub struct EspoolerStats {
    name: String,
    n20_runtime_fwd: RefCell<Rc<StatVar>>,
    n20_runtime_rwd: RefCell<Rc<StatVar>>,
}

impl EspoolerStats {
    /// update_database: only directions that ran (never, without motors)
    pub fn update_database(&self) {}

    fn handle_moonraker_stats(&self, afc: &Afc) -> KResult<()> {
        let moonraker = afc.moonraker.borrow().clone().ok_or_else(|| no_attr("NoneType", "get_afc_stats"))?;
        let values = moonraker.get_afc_stats();
        *self.n20_runtime_fwd.borrow_mut() = Rc::new(StatVar::new(&self.name, "n20_runtime_fwd", values.as_ref(), moonraker.clone(), "", false));
        *self.n20_runtime_rwd.borrow_mut() = Rc::new(StatVar::new(&self.name, "n20_runtime_rwd", values.as_ref(), moonraker, "", false));
        Ok(())
    }

    pub fn reset_runtimes(&self) -> KResult<()> {
        self.n20_runtime_fwd.borrow().reset_count()?;
        self.n20_runtime_rwd.borrow().reset_count()
    }
}

/// Espooler: the lane's spooler values; no motor pins in the native scope
pub struct Espooler {
    name: String,
    afc: Rc<Afc>,
    full_weight_cfg: Option<f64>,
    full_weight: RefCell<PyVal>,
    stats: RefCell<Option<Rc<EspoolerStats>>>,
}

impl Espooler {
    pub fn new(name: &str, config: &ConfigWrapper, afc: &Rc<Afc>) -> KResult<Rc<Espooler>> {
        for pin in ["afc_motor_rwd", "afc_motor_fwd", "afc_motor_enb"] {
            if config.get_opt(pin)?.is_some() {
                return Err(KError::config(format!("AFC lane {} is not native", pin)));
            }
        }
        let _rwd_speed_multi = config.getfloat("rwd_speed_multiplier", 0.5, Lim::new())?;
        let _fwd_speed_multi = config.getfloat("fwd_speed_multiplier", 0.5, Lim::new())?;
        let _n20_break_delay_time = config.getfloat_opt("n20_break_delay_time", Lim::new())?;
        let _timer_delay = config.getfloat_opt("timer_delay", Lim::new())?;
        let _enable_assist = config.getboolean_opt("enable_assist")?;
        let _enable_assist_weight = config.getfloat_opt("enable_assist_weight", Lim::new())?;
        let _debug = config.getboolean_opt("debug")?;
        let _enable_kick_start = config.getboolean_opt("enable_kick_start")?;
        // Espooler_values
        let _kick_start_time = config.getfloat_opt("kick_start_time", Lim::new())?;
        let _spool_outer_diameter = config.getfloat_opt("spool_outer_diameter", Lim::new().min(100))?;
        let _spool_inner_diameter = config.getfloat_opt("spool_inner_diameter", Lim::new().min(1))?;
        let _max_motor_rpm = config.getfloat_opt("max_motor_rpm", Lim::new())?;
        let _delta_movement = config.getfloat_opt("delta_movement", Lim::new())?;
        let _scaling = config.getfloat_opt("spoolrate", Lim::new())?;
        let _spool_ratio = config.getfloat_opt("spool_ratio", Lim::new())?;
        let full_weight_cfg = config.getfloat_opt("full_weight", Lim::new().min(1))?;
        let _espool_rot_dist = config.getfloat_opt("espool_rot_dist", Lim::new())?;
        config.get_printer().lookup_or_load::<super::functions::AfcFunction>(config, "AFC_functions")?;
        Ok(Rc::new(Espooler {
            name: name.to_string(),
            afc: afc.clone(),
            full_weight_cfg,
            full_weight: RefCell::new(PyVal::None),
            stats: RefCell::new(None),
        }))
    }

    pub fn handle_ready(&self) {
        self.afc.logger.info(&AFC_ASSIST_ESPOOLER_HANDLE_READY, &format!("Not starting timer for {}", self.name));
    }

    /// The unit's values for those the lane does not set
    pub fn handle_connect(&self, lane: &AfcLane) -> KResult<()> {
        let unit = lane.unit_obj()?;
        *self.full_weight.borrow_mut() = PyVal::Float(self.full_weight_cfg.unwrap_or(unit.settings.full_weight));
        let z = || RefCell::new(Rc::new(StatVar::local(PyVal::Int(0))));
        *self.stats.borrow_mut() = Some(Rc::new(EspoolerStats { name: self.name.clone(), n20_runtime_fwd: z(), n20_runtime_rwd: z() }));
        Ok(())
    }

    pub fn handle_moonraker_connect(&self) -> KResult<()> {
        self.stats()?.handle_moonraker_stats(&self.afc)
    }

    pub fn stats(&self) -> KResult<Rc<EspoolerStats>> {
        self.stats.borrow().clone().ok_or_else(|| no_attr("NoneType", "update_database"))
    }

    /// espooler_values.full_weight
    pub fn full_weight(&self) -> PyVal {
        self.full_weight.borrow().clone()
    }

    pub fn set_full_weight(&self, v: PyVal) {
        *self.full_weight.borrow_mut() = v;
    }

    pub fn has_rwd(&self) -> bool {
        false
    }

    pub fn assist(&self, _value: f64) -> KResult<()> {
        Ok(())
    }

    pub fn do_assist_move(&self, _print_time: Option<f64>) -> KResult<()> {
        Ok(())
    }

    pub fn enable_timer(&self) {}

    pub fn disable_timer(&self) {}

    /// "" without motors
    pub fn get_spooler_stats(&self, _short: bool) -> String {
        String::new()
    }
}
