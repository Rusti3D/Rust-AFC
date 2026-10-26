// AFC_poop: purge settings and AFC's purge sequence (port of AFC_poop.py;
// AFC 1.2.7 itself purges through the poop_cmd macro and never calls poop())
//
// Copyright (C) 2024-2026 Armored Turtle
// Copyright (C) 2026 AFCProject
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::{utils, Afc};
use kaliru::configfile::{ConfigWrapper, Dflt, Lim};
use kaliru::error::KResult;
use kaliru::printer::{PObj, PrinterObject};
use kaliru::pyval::PyVal;
use std::rc::Rc;

pub struct Poop {
    afc: Rc<Afc>,
    verbose: bool,
    purge_loc_xy: String,
    purge_start: f64,
    purge_spd: f64,
    fast_z: f64,
    z_lift: f64,
    full_fan: bool,
    purge_length: f64,
    max_iteration_length: f64,
    iteration_z_raise: f64,
    iteration_z_change: f64,
}

impl PrinterObject for Poop {}

impl Poop {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Poop>> {
        let printer = config.get_printer();
        let afc = printer.lookup_or_load::<Afc>(config, "AFC")?;
        let f = |name: &str, default: f64| config.getfloat(name, default, Lim::new());
        let _verbose = config.getboolean("verbose", false)?;
        let purge_loc_xy = config.get("purge_loc_xy", Dflt::Required)?;
        let _purge_start = f("purge_start", 0.0)?;
        let purge_spd = f("purge_spd", 6.5)?;
        let fast_z = f("fast_z", 200.0)?;
        let z_lift = f("z_lift", 20.0)?;
        let _restore_position = config.getboolean("restore_position", false)?;
        let purge_start = f("purge_start", 20.0)?;
        let full_fan = config.getboolean("full_fan", false)?;
        let purge_length = f("purge_length", 70.111)?;
        let _purge_length_min = f("purge_length_min", 60.999)?;
        let max_iteration_length = f("max_iteration_length", 40.0)?;
        let iteration_z_raise = f("iteration_z_raise", 6.0)?;
        let iteration_z_change = f("iteration_z_change", 0.6)?;
        // (python reads 'verbose' first, then overwrites it with 'comment')
        let verbose = config.getboolean("comment", false)?;
        Ok(Rc::new(Poop {
            afc,
            verbose,
            purge_loc_xy,
            purge_start,
            purge_spd,
            fast_z,
            z_lift,
            full_fan,
            purge_length,
            max_iteration_length,
            iteration_z_raise,
            iteration_z_change,
        }))
    }

    /// poop: to the purge location, purge in rising iterations, fast lift
    pub fn poop(&self) -> KResult<()> {
        let c = &AFC_POOP_AFC_POOP_POOP;
        let afc = &self.afc;
        let Ok(gm) = afc.gcode_move() else { return Ok(()) };
        let info = |msg: String| {
            if self.verbose {
                afc.logger.info(c, &msg);
            }
        };
        // pooppos is gcode_move's last_position itself, edited in place
        let edit = |f: &dyn Fn(&mut Vec<f64>)| -> Vec<f64> {
            let mut lp = gm.last_position.borrow_mut();
            f(&mut lp);
            lp.clone()
        };
        let mut step = 1;
        info(format!("AFC_Poop: {} Move To Purge Location", step));
        let xy: Vec<&str> = self.purge_loc_xy.split(',').collect();
        let coord = |i: usize| -> KResult<f64> {
            let s = xy.get(i).ok_or_else(|| kaliru::error::KError::Py("IndexError".into(), "list index out of range".into()))?;
            utils::py_float_of(&PyVal::Str(s.to_string()))
        };
        let (x, y) = (coord(0)?, coord(1)?);
        let pooppos = edit(&|p| {
            p[0] = x;
            p[1] = y;
        });
        gm.move_with_transform(&pooppos, 100.0)?;
        let pooppos = edit(&|p| p[2] = self.purge_start);
        gm.move_with_transform(&pooppos, 100.0)?;
        step += 1;
        if self.full_fan {
            info(format!("AFC_Poop: {} Set Cooling Fan to Full Speed", step));
            afc.gcode.run_script_from_command("M106 S255")?;
            step += 1;
        }
        let iterations = (self.purge_length / self.max_iteration_length).trunc() as i64;
        for iteration in 0..iterations {
            info(format!("AFC_Poop: {} Purge Iteration {}", step, iteration));
            let purge_amount_left = self.purge_length - (self.max_iteration_length * iteration as f64);
            let extrude_amount = purge_amount_left / self.max_iteration_length;
            let extrude_ratio = extrude_amount / self.max_iteration_length;
            let step_triangular = (iteration * (iteration + 1)) as f64 / 2.0;
            let z_raise_substract = if iteration == 0 { self.purge_start } else { step_triangular * self.iteration_z_change };
            let raise_z = (self.iteration_z_raise - z_raise_substract) * extrude_ratio;
            let duration = extrude_amount / self.purge_spd;
            let speed = raise_z / duration;
            let pooppos = edit(&|p| {
                p[2] += raise_z;
                p[3] += extrude_amount;
            });
            gm.move_with_transform(&pooppos, speed)?;
        }
        step += 1;
        info(format!("AFC_Poop: {} Fast Z Lift to keep poop from sticking", step));
        let pooppos = edit(&|p| p[2] = self.z_lift);
        gm.move_with_transform(&pooppos, self.fast_z)?;
        step += 1;
        if self.full_fan {
            info(format!("AFC_Poop: {} Restore fan speed and feedrate", step));
            afc.gcode.run_script_from_command("M106 S0")?;
        }
        Ok(())
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(Poop::new(config)?)
}
