// AFC_form_tip: AFC's own tip forming (port of AFC_form_tip.py), run on a
// tool unload when form_tip_cmd is "AFC"
//
// Copyright (C) 2024-2026 Armored Turtle
// Copyright (C) 2026 AFCProject
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::{utils, Afc};
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::KResult;
use kaliru::extras::heaters::PrinterHeaters;
use kaliru::gcode::GCodeCommand;
use kaliru::printer::{PObj, Printer, PrinterObject};
use kaliru::pyval::PyVal;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

const CMD_TEST_AFC_TIP_FORMING_HELP: &str = "Gives ability to test AFC tip forming without doing a tool change";
const CMD_GET_TIP_FORMING_HELP: &str = "Shows the tip forming configuration";
const CMD_SET_TIP_FORMING_HELP: &str = "Sets tip forming configuration";

/// The float settings in their order, cooling_moves and use_skinnydip read
/// after the first 8 (python's int defaults stay ints)
const FLOATS: [(&str, i64); 13] = [
    ("ramming_volume", 0),
    ("toolchange_temp", 0),
    ("unloading_speed_start", 80),
    ("unloading_speed", 18),
    ("cooling_tube_position", 35),
    ("cooling_tube_length", 10),
    ("initial_cooling_speed", 10),
    ("final_cooling_speed", 50),
    ("skinnydip_distance", 4),
    ("dip_insertion_speed", 4),
    ("dip_extraction_speed", 4),
    ("melt_zone_pause", 4),
    ("cooling_zone_pause", 4),
];

pub struct FormTip {
    printer: Rc<Printer>,
    afc: Rc<Afc>,
    /// FLOATS' values by name
    values: RefCell<kaliru::IndexMap<&'static str, PyVal>>,
    cooling_moves: Cell<i64>,
    use_skinnydip: Cell<bool>,
}

impl PrinterObject for FormTip {}

impl FormTip {
    fn new(config: &ConfigWrapper) -> KResult<Rc<FormTip>> {
        let printer = config.get_printer();
        let afc = printer.lookup_or_load::<Afc>(config, "AFC")?;
        let mut values = kaliru::IndexMap::default();
        let getfloat = |name: &str, default: i64| super::getfloat_pv(config, name, PyVal::Int(default));
        for (name, default) in &FLOATS[..8] {
            values.insert(*name, getfloat(name, *default)?);
        }
        let cooling_moves = config.getint("cooling_moves", 4, Lim::new())?;
        let use_skinnydip = config.getboolean("use_skinnydip", false)?;
        for (name, default) in &FLOATS[8..] {
            values.insert(*name, getfloat(name, *default)?);
        }
        let tip = Rc::new(FormTip { printer, afc: afc.clone(), values: RefCell::new(values), cooling_moves: Cell::new(cooling_moves), use_skinnydip: Cell::new(use_skinnydip) });
        type Cmd = fn(&FormTip, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, Cmd, &str); 3] = [
            ("TEST_AFC_TIP_FORMING", FormTip::cmd_test_afc_tip_forming, CMD_TEST_AFC_TIP_FORMING_HELP),
            ("GET_TIP_FORMING", FormTip::cmd_get_tip_forming, CMD_GET_TIP_FORMING_HELP),
            ("SET_TIP_FORMING", FormTip::cmd_set_tip_forming, CMD_SET_TIP_FORMING_HELP),
        ];
        for (cmd, f, help) in cmds {
            let t = tip.clone();
            afc.gcode.register_command(cmd, Some(Rc::new(move |g| f(&t, g))), false, Some(help))?;
        }
        Ok(tip)
    }

    fn v(&self, name: &str) -> PyVal {
        self.values.borrow().get(name).cloned().unwrap_or(PyVal::Int(0))
    }

    fn f(&self, name: &str) -> KResult<f64> {
        utils::py_float_of(&self.v(name))
    }

    fn afc_extrude(&self, distance: f64, speed: f64) -> KResult<()> {
        self.afc.move_e_pos(distance, speed, "form tip", false)
    }

    fn cmd_test_afc_tip_forming(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.tip_form()
    }

    fn cmd_get_tip_forming(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let mut status_msg = String::from("Tip Forming Configuration:\n");
        let line = |name: &str, v: String| format!("{:<23}{}\n", format!("{}:", name), v);
        for (name, _) in &FLOATS[..8] {
            status_msg += &line(name, self.v(name).str());
        }
        status_msg += &line("cooling_moves", self.cooling_moves.get().to_string());
        status_msg += &line("use_skinnydip", super::py_bool(self.use_skinnydip.get()).to_string());
        for (name, _) in &FLOATS[8..] {
            status_msg += &line(name, self.v(name).str());
        }
        self.afc.logger.raw(&AFC_FORM_TIP_AFC_TIP_FORM_CMD_GET_TIP_FORMING, &status_msg);
        Ok(())
    }

    /// SET_TIP_FORMING: the given values (the config file stays as it is)
    fn cmd_set_tip_forming(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let get = |name: &str| -> KResult<PyVal> {
            let key = name.to_uppercase();
            Ok(match gcmd.get_opt(&key) {
                Some(_) => PyVal::Float(gcmd.get_float(&key, None, Lim::new())?),
                None => self.v(name),
            })
        };
        for (name, _) in &FLOATS[..8] {
            let v = get(name)?;
            self.values.borrow_mut().insert(name, v);
        }
        self.cooling_moves.set(gcmd.get_int("COOLING_MOVES", Some(self.cooling_moves.get()), Lim::new())?);
        let skinnydip = gcmd.get_or("USE_SKINNYDIP", super::py_bool(self.use_skinnydip.get()));
        self.use_skinnydip.set(skinnydip.to_lowercase() == "true");
        for (name, _) in &FLOATS[8..] {
            let v = get(name)?;
            self.values.borrow_mut().insert(name, v);
        }
        Ok(())
    }

    /// tip_form: ramming, retraction and nozzle separation, cooling moves,
    /// an optional skinny dip; the toolchange temperature meanwhile
    pub fn tip_form(&self) -> KResult<()> {
        let c = &AFC_FORM_TIP_AFC_TIP_FORM_TIP_FORM;
        let logger = &self.afc.logger;
        let reactor = &self.afc.reactor;
        let mut step = 1;
        let extruder = self.afc.toolhead()?.get_extruder();
        let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
        // Saving current temp so it can be set back when done if toolchange_temp is not zero
        let current_temp = extruder.get_heater()?.target_temp.get();
        let ramming_volume = self.f("ramming_volume")?;
        if ramming_volume > 0.0 {
            logger.info(c, &format!("AFC-TIP-FORM: Step {}: Ramming", step));
            let ratio = ramming_volume / 23.0;
            for (d, s) in [
                (0.5784, 299.0),
                (0.5834, 302.0),
                (0.5918, 306.0),
                (0.6169, 319.0),
                (0.3393, 350.0),
                (0.3363, 350.0),
                (0.7577, 392.0),
                (0.8382, 434.0),
                (0.7776, 469.0),
                (0.1293, 469.0),
                (0.9673, 501.0),
                (1.0176, 527.0),
                (0.5956, 544.0),
                (1.0662, 552.0),
            ] {
                self.afc_extrude(d * ratio, s / 60.0)?;
            }
            step += 1;
        }
        logger.info(c, &format!("AFC-TIP-FORM: Step {}: Retraction & Nozzle Separation", step));
        let cooling_tube_length = self.f("cooling_tube_length")?;
        let unloading_speed = self.f("unloading_speed")?;
        let total_retraction_distance = self.f("cooling_tube_position")? + cooling_tube_length - 15.0;
        self.afc_extrude(-15.0, self.f("unloading_speed_start")?)?;
        if total_retraction_distance > 0.0 {
            self.afc_extrude(-0.7 * total_retraction_distance, 1.0 * unloading_speed)?;
            self.afc_extrude(-0.2 * total_retraction_distance, 0.5 * unloading_speed)?;
            self.afc_extrude(-0.1 * total_retraction_distance, 0.3 * unloading_speed)?;
        }
        let toolchange_temp = self.v("toolchange_temp");
        if utils::py_float_of(&toolchange_temp)? > 0.0 {
            let wait = !self.use_skinnydip.get();
            logger.info(c, &format!("AFC-TIP-FORM: Waiting for temperature to get to {}", toolchange_temp.str()));
            pheaters.set_temperature(&extruder.get_heater()?, utils::py_float_of(&toolchange_temp)?, wait)?;
        }
        step += 1;
        logger.info(c, &format!("AFC-TIP-FORM: Step {}: Cooling Moves", step));
        let initial_cooling_speed = self.f("initial_cooling_speed")?;
        let cooling_moves = self.cooling_moves.get();
        let speed_inc = (self.f("final_cooling_speed")? - initial_cooling_speed) / (2 * cooling_moves - 1) as f64;
        for m in 0..cooling_moves {
            let speed = initial_cooling_speed + speed_inc * m as f64 * 2.0;
            self.afc_extrude(cooling_tube_length, speed)?;
            self.afc_extrude(-cooling_tube_length, speed + speed_inc)?;
        }
        step += 1;
        if self.use_skinnydip.get() {
            logger.info(c, &format!("AFC-TIP-FORM: Step {}: Skinny Dipping", step));
            let skinnydip_distance = self.f("skinnydip_distance")?;
            self.afc_extrude(skinnydip_distance, self.f("dip_insertion_speed")?)?;
            reactor.pause(reactor.monotonic() + self.f("melt_zone_pause")?)?;
            self.afc_extrude(-skinnydip_distance, self.f("dip_extraction_speed")?)?;
            reactor.pause(reactor.monotonic() + self.f("cooling_zone_pause")?)?;
        }
        let heater = extruder.get_heater()?;
        if heater.target_temp.get() != current_temp {
            logger.info(c, &format!("AFC-TIP-FORM: Setting temperature back to {}", kaliru::pyfmt::py_float_repr(current_temp)));
            pheaters.set_temperature(&heater, current_temp, false)?;
        }
        logger.info(c, "AFC-TIP-FORM: Done");
        Ok(())
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(FormTip::new(config)?)
}
