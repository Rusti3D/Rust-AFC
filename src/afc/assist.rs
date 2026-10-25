// A lane's spooler assist: port of AFC_assist.py (AFCassistMotor, the N20
// motor pins; AFCEspoolerStats, their run times; Espooler_values and
// Espooler, the assist while loading, unloading and printing)
//
// Copyright (C) 2024-2026 Armored Turtle
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::lane::AfcLane;
use super::stats::StatVar;
use super::{no_attr, utils, Afc};
use kaliru::configfile::{ConfigWrapper, Dflt, Lim};
use kaliru::error::{KError, KResult};
use kaliru::gcode::GCodeCommand;
use kaliru::mcu::{DigitalOutPin, Mcu, PwmPin};
use kaliru::pyval::PyVal;
use kaliru::reactor::{Reactor, Timer, NEVER, NOW};
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const PIN_MIN_TIME: f64 = 0.100;
const RESEND_HOST_TIME: f64 = 0.300 + PIN_MIN_TIME;
const MAX_SCHEDULE_TIME: f64 = 5.0;
const RPM_TO_RPS: f64 = 60.0;

/// python's float division: ZeroDivisionError where rust gives inf/NaN
fn fdiv(a: f64, b: f64) -> KResult<f64> {
    if b == 0.0 {
        return Err(KError::Py("ZeroDivisionError".into(), "float division by zero".into()));
    }
    Ok(a / b)
}

fn num(v: &PyVal) -> KResult<f64> {
    utils::py_float_of(v)
}

enum MotorPin {
    Pwm(Rc<dyn PwmPin>),
    Digital(Rc<dyn DigitalOutPin>),
}

/// AFCassistMotor: one of the spooler's motor pins (an output_pin of its own)
pub struct AssistMotor {
    reactor: Rc<Reactor>,
    mcu_pin: MotorPin,
    pub is_pwm: bool,
    pub scale: f64,
    last_value: Cell<f64>,
    last_print_time: Cell<f64>,
    shutdown_value: f64,
    resend_interval: f64,
    resend_timer: RefCell<Option<Timer>>,
    me: Weak<AssistMotor>,
}

impl AssistMotor {
    fn new(config: &ConfigWrapper, kind: &str) -> KResult<Rc<AssistMotor>> {
        let printer = config.get_printer();
        // Determine pin type
        let pwm = config.getboolean("pwm", false)?;
        let pin = config.get(&format!("afc_motor_{}", kind), Dflt::Required)?;
        let (mcu_pin, is_pwm, scale) = if pwm && kind != "enb" {
            let p = kaliru::pins::setup_pwm_pin(&printer, &pin)?;
            let cycle_time = config.getfloat("cycle_time", 0.100, Lim::new().above(0.0).max(MAX_SCHEDULE_TIME))?;
            let hardware_pwm = config.getboolean("hardware_pwm", false)?;
            p.setup_cycle_time(cycle_time, hardware_pwm)?;
            let scale = config.getfloat("scale", 1.0, Lim::new().above(0.0))?;
            (MotorPin::Pwm(p), true, scale)
        } else {
            (MotorPin::Digital(kaliru::pins::setup_digital_out_pin(&printer, &pin)?), false, 1.0)
        };
        // Support mcu checking for maximum duration
        let max_mcu_duration = config.getfloat("maximum_mcu_duration", 0.0, Lim::new().min(0.500).max(MAX_SCHEDULE_TIME))?;
        match &mcu_pin {
            MotorPin::Pwm(p) => p.setup_max_duration(max_mcu_duration),
            MotorPin::Digital(p) => p.setup_max_duration(max_mcu_duration),
        }
        let mut resend_interval = 0.0;
        if max_mcu_duration != 0.0 {
            config.deprecate("maximum_mcu_duration", None);
            resend_interval = max_mcu_duration - RESEND_HOST_TIME;
        }
        // Determine start and shutdown values
        let static_value = config.getfloat_opt("static_value", Lim::new().min(0.0).max(scale))?;
        let (last_value, shutdown_value) = match static_value {
            Some(v) => {
                config.deprecate("static_value", None);
                (v / scale, v / scale)
            }
            None => (
                config.getfloat("value", 0.0, Lim::new().min(0.0).max(scale))? / scale,
                config.getfloat("shutdown_value", 0.0, Lim::new().min(0.0).max(scale))? / scale,
            ),
        };
        match &mcu_pin {
            MotorPin::Pwm(p) => p.setup_start_value(last_value, shutdown_value),
            MotorPin::Digital(p) => p.setup_start_value(last_value != 0.0, shutdown_value != 0.0),
        }
        Ok(Rc::new_cyclic(|me| AssistMotor {
            reactor: printer.get_reactor(),
            mcu_pin,
            is_pwm,
            scale,
            last_value: Cell::new(last_value),
            last_print_time: Cell::new(0.0),
            shutdown_value,
            resend_interval,
            resend_timer: RefCell::new(None),
            me: me.clone(),
        }))
    }

    fn get_mcu(&self) -> Rc<Mcu> {
        match &self.mcu_pin {
            MotorPin::Pwm(p) => p.get_mcu(),
            MotorPin::Digital(p) => p.get_mcu(),
        }
    }

    pub fn set_pin(&self, print_time: f64, value: f64) -> KResult<()> {
        self.set_pin_resend(print_time, value, false)
    }

    fn set_pin_resend(&self, print_time: f64, value: f64, is_resend: bool) -> KResult<()> {
        if value == self.last_value.get() && !is_resend {
            return Ok(());
        }
        let print_time = kaliru::pyfmt::py_max(print_time, self.last_print_time.get() + PIN_MIN_TIME);
        match &self.mcu_pin {
            MotorPin::Pwm(p) => p.set_pwm(print_time, value)?,
            MotorPin::Digital(p) => p.set_digital(print_time, value != 0.0)?,
        }
        self.last_value.set(value);
        self.last_print_time.set(print_time);
        if self.resend_interval != 0.0 && self.resend_timer.borrow().is_none() {
            let w = self.me.clone();
            let t = self.reactor.register_timer(
                Box::new(move |et| match w.upgrade() {
                    Some(m) => m.resend_current_val(et).unwrap_or_else(|e| {
                        kaliru::logging::exception_err("AFC assist motor resend", &e);
                        NEVER
                    }),
                    None => NEVER,
                }),
                NOW,
            );
            *self.resend_timer.borrow_mut() = Some(t);
        }
        Ok(())
    }

    fn resend_current_val(&self, _eventtime: f64) -> KResult<f64> {
        if self.last_value.get() == self.shutdown_value {
            if let Some(t) = self.resend_timer.borrow_mut().take() {
                self.reactor.unregister_timer(&t);
            }
            return Ok(NEVER);
        }
        let systime = self.reactor.monotonic();
        let print_time = self.get_mcu().estimated_print_time(systime);
        let time_diff = (self.last_print_time.get() + self.resend_interval) - print_time;
        if time_diff > 0.0 {
            // Reschedule for resend time
            return Ok(systime + time_diff);
        }
        self.set_pin_resend(print_time + PIN_MIN_TIME, self.last_value.get(), true)?;
        Ok(systime + self.resend_interval)
    }
}

#[derive(Clone, Copy, PartialEq)]
enum EspoolerDir {
    Fwd,
    Rwd,
}

/// AFCEspoolerStats: the N20 motors' run times
pub struct EspoolerStats {
    name: String,
    n20_runtime_fwd: RefCell<Rc<StatVar>>,
    n20_runtime_rwd: RefCell<Rc<StatVar>>,
    fwd_updated: Cell<bool>,
    rwd_updated: Cell<bool>,
    direction: Cell<Option<EspoolerDir>>,
    direction_start: Cell<Option<f64>>,
    direction_end: Cell<Option<f64>>,
}

impl EspoolerStats {
    fn new(name: &str) -> EspoolerStats {
        let z = || RefCell::new(Rc::new(StatVar::local(PyVal::Int(0))));
        EspoolerStats {
            name: name.to_string(),
            n20_runtime_fwd: z(),
            n20_runtime_rwd: z(),
            fwd_updated: Cell::new(false),
            rwd_updated: Cell::new(false),
            direction: Cell::new(None),
            direction_start: Cell::new(None),
            direction_end: Cell::new(None),
        }
    }

    fn handle_moonraker_stats(&self, afc: &Afc) -> KResult<()> {
        let moonraker = afc.moonraker.borrow().clone().ok_or_else(|| no_attr("NoneType", "get_afc_stats"))?;
        let values = moonraker.get_afc_stats();
        *self.n20_runtime_fwd.borrow_mut() = Rc::new(StatVar::new(&self.name, "n20_runtime_fwd", values.as_ref(), moonraker.clone(), "", false));
        *self.n20_runtime_rwd.borrow_mut() = Rc::new(StatVar::new(&self.name, "n20_runtime_rwd", values.as_ref(), moonraker, "", false));
        Ok(())
    }

    /// _convert_value: seconds as "<value><s|m|h>"
    fn convert_value(value: &PyVal) -> KResult<String> {
        let mut value = num(value)?;
        let mut unit = 's';
        if value > 9999.0 {
            value /= 60.0;
            unit = 'm';
        }
        if value > 9999.0 {
            value /= 60.0;
            unit = 'h';
        }
        Ok(format!("{:.2}{}", value, unit))
    }

    fn n20_runtime_fwd(&self) -> KResult<String> {
        Self::convert_value(&self.n20_runtime_fwd.borrow().value())
    }

    fn n20_runtime_rwd(&self) -> KResult<String> {
        Self::convert_value(&self.n20_runtime_rwd.borrow().value())
    }

    /// the direction setter: only while none is set
    fn set_direction(&self, value: EspoolerDir) {
        if self.direction.get().is_none() {
            self.direction.set(Some(value));
        }
    }

    /// the start_time setter: only while none is set
    fn set_start_time(&self, eventtime: f64) {
        if self.direction_start.get().is_none() {
            self.direction_start.set(Some(eventtime));
        }
    }

    /// the end_time setter: adds end - start to the direction that ran
    fn set_end_time(&self, eventtime: f64) -> KResult<()> {
        if self.direction.get().is_none() && self.direction_end.get().is_none() && self.direction_start.get().is_none() {
            return Ok(());
        }
        self.direction_end.set(Some(eventtime));
        let Some(start) = self.direction_start.get() else {
            return Err(KError::Py("TypeError".into(), "'>' not supported between instances of 'float' and 'NoneType'".into()));
        };
        if eventtime > start {
            let delta = eventtime - start;
            if self.direction.get() == Some(EspoolerDir::Fwd) {
                let v = self.n20_runtime_fwd.borrow().clone();
                v.set_value(utils::py_add(&v.value(), &PyVal::Float(delta))?);
                self.fwd_updated.set(true);
            } else {
                let v = self.n20_runtime_rwd.borrow().clone();
                v.set_value(utils::py_add(&v.value(), &PyVal::Float(delta))?);
                self.rwd_updated.set(true);
            }
        }
        self.direction.set(None);
        self.direction_start.set(None);
        self.direction_end.set(None);
        Ok(())
    }

    pub fn reset_runtimes(&self) -> KResult<()> {
        self.n20_runtime_fwd.borrow().reset_count()?;
        self.n20_runtime_rwd.borrow().reset_count()
    }

    /// update_database: the directions that ran since the last update
    pub fn update_database(&self) {
        if self.fwd_updated.get() {
            self.n20_runtime_fwd.borrow().update_database();
            self.fwd_updated.set(false);
        }
        if self.rwd_updated.get() {
            self.n20_runtime_rwd.borrow().update_database();
            self.rwd_updated.set(false);
        }
    }
}

/// Espooler_values: the assist's values, set per lane or else the unit's
/// (kick_start_time and delta_movement read back scaled by spoolrate)
struct EspoolerValues {
    kick_start_time: RefCell<PyVal>,
    spool_outer_diameter: RefCell<PyVal>,
    spool_inner_diameter: RefCell<PyVal>,
    max_motor_rpm: RefCell<PyVal>,
    delta_movement: RefCell<PyVal>,
    scaling: RefCell<PyVal>,
    spool_ratio: RefCell<PyVal>,
    full_weight: RefCell<PyVal>,
    espool_rot_dist: RefCell<PyVal>,
    cruise_time: Cell<f64>,
}

impl EspoolerValues {
    fn new(config: &ConfigWrapper) -> KResult<EspoolerValues> {
        let opt = |name: &str, lim: Lim| -> KResult<RefCell<PyVal>> { Ok(RefCell::new(config.getfloat_opt(name, lim)?.map_or(PyVal::None, PyVal::Float))) };
        Ok(EspoolerValues {
            kick_start_time: opt("kick_start_time", Lim::new())?,
            spool_outer_diameter: opt("spool_outer_diameter", Lim::new().min(100))?,
            spool_inner_diameter: opt("spool_inner_diameter", Lim::new().min(1))?,
            max_motor_rpm: opt("max_motor_rpm", Lim::new())?,
            delta_movement: opt("delta_movement", Lim::new())?,
            scaling: opt("spoolrate", Lim::new())?,
            spool_ratio: opt("spool_ratio", Lim::new())?,
            full_weight: opt("full_weight", Lim::new().min(1))?,
            espool_rot_dist: opt("espool_rot_dist", Lim::new())?,
            cruise_time: Cell::new(0.0),
        })
    }

    fn kick_start_time(&self) -> KResult<f64> {
        Ok(num(&self.kick_start_time.borrow())? * num(&self.scaling.borrow())?)
    }

    fn delta_movement(&self) -> KResult<f64> {
        Ok(num(&self.delta_movement.borrow())? * num(&self.scaling.borrow())?)
    }

    fn outer_circ(&self) -> KResult<f64> {
        Ok(num(&self.spool_outer_diameter.borrow())? * std::f64::consts::PI)
    }

    fn delta_circ(&self) -> KResult<f64> {
        Ok((num(&self.spool_outer_diameter.borrow())? - num(&self.spool_inner_diameter.borrow())?) * std::f64::consts::PI)
    }

    /// calculate_cruise_time: how long the motor runs for one assist
    fn calculate_cruise_time(&self, weight: &PyVal) -> KResult<f64> {
        let rps = num(&self.max_motor_rpm.borrow())? / RPM_TO_RPS;
        let spool_rot_s = fdiv(num(&self.espool_rot_dist.borrow())? * fdiv(rps, num(&self.spool_ratio.borrow())?)?, self.outer_circ()?)?;
        let w_r = (fdiv(num(weight)?, num(&self.full_weight.borrow())?)? + 1.0) * self.delta_circ()?;
        let cruise_time = fdiv(fdiv(self.delta_movement()?, w_r)?, spool_rot_s)?;
        self.cruise_time.set(cruise_time);
        Ok(cruise_time)
    }

    fn handle_connect(&self, lane: &AfcLane) -> KResult<()> {
        let unit = lane.unit_obj()?;
        let s = &unit.settings;
        let fill = |v: &RefCell<PyVal>, d: PyVal| {
            if matches!(*v.borrow(), PyVal::None) {
                *v.borrow_mut() = d;
            }
        };
        fill(&self.kick_start_time, PyVal::Float(s.kick_start_time));
        fill(&self.spool_outer_diameter, PyVal::Float(lane.outer_diameter));
        fill(&self.spool_inner_diameter, PyVal::Float(lane.inner_diameter));
        fill(&self.max_motor_rpm, lane.max_motor_rpm.clone());
        fill(&self.espool_rot_dist, PyVal::Float(s.espool_rot_dist));
        fill(&self.delta_movement, s.delta_movement.clone());
        fill(&self.scaling, PyVal::Float(s.scaling));
        fill(&self.spool_ratio, s.spool_ratio.clone());
        fill(&self.full_weight, s.full_weight.clone());
        let full_weight = self.full_weight.borrow().clone();
        let cruise_time = self.calculate_cruise_time(&full_weight)?;
        self.cruise_time.set(cruise_time);
        Ok(())
    }
}

/// Espooler: drives a lane's spooler motors while loading and unloading,
/// and assists the spool while printing
pub struct Espooler {
    name: String,
    afc: Rc<Afc>,
    reactor: Rc<Reactor>,
    callback_timer: Timer,
    stats_timer: Timer,
    lane_obj: RefCell<Weak<AfcLane>>,
    mcu: Rc<Mcu>,
    afc_motor_rwd: Option<Rc<AssistMotor>>,
    afc_motor_fwd: Option<Rc<AssistMotor>>,
    afc_motor_enb: Option<Rc<AssistMotor>>,
    n20_break_delay_time: RefCell<PyVal>,
    timer_delay: RefCell<PyVal>,
    enable_assist: RefCell<PyVal>,
    enable_assist_weight: RefCell<PyVal>,
    debug: RefCell<PyVal>,
    enable_kick_start: RefCell<PyVal>,
    past_extruder_position: Cell<f64>,
    values: EspoolerValues,
    stats: RefCell<Option<Rc<EspoolerStats>>>,
}

const CMD_TEST_ESPOOLER_ASSIST_HELP: &str = "Test espooler print assist for a specified lane";
const CMD_ENABLE_ESPOOLER_ASSIST_HELP: &str = "Enabled espooler print assist for a specified lane, this can be used while printing";
const CMD_DISABLE_ESPOOLER_ASSIST_HELP: &str = "Disables espooler print assist for a specified lane, this can be used while printing";
const CMD_SET_ESPOOLER_VALUES_HELP: &str = "Macro for setting/updating espooler values without having to restart klipper.";
const CMD_AFC_RESET_MOTOR_TIME_HELP: &str = "Resets N20 active time, useful for resetting time for N20 if one was replaced in a lane";

impl Espooler {
    pub fn new(name: &str, config: &ConfigWrapper, afc: &Rc<Afc>) -> KResult<Rc<Espooler>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let mcu = printer.lookup::<Mcu>("mcu")?;
        let afc_motor_rwd = config.get_opt("afc_motor_rwd")?;
        let afc_motor_fwd = config.get_opt("afc_motor_fwd")?;
        let afc_motor_enb = config.get_opt("afc_motor_enb")?;
        // Forward / reverse multipliers to apply to rpm (the lane's own copies
        // are what assist_move uses)
        let _rwd_speed_multi = config.getfloat("rwd_speed_multiplier", 0.5, Lim::new())?;
        let _fwd_speed_multi = config.getfloat("fwd_speed_multiplier", 0.5, Lim::new())?;
        let opt_f = |name: &str| -> KResult<PyVal> { Ok(config.getfloat_opt(name, Lim::new())?.map_or(PyVal::None, PyVal::Float)) };
        let opt_b = |name: &str| -> KResult<PyVal> { Ok(config.getboolean_opt(name)?.map_or(PyVal::None, PyVal::Bool)) };
        let n20_break_delay_time = opt_f("n20_break_delay_time")?;
        let timer_delay = opt_f("timer_delay")?;
        let enable_assist = opt_b("enable_assist")?;
        let enable_assist_weight = opt_f("enable_assist_weight")?;
        let debug = opt_b("debug")?;
        let enable_kick_start = opt_b("enable_kick_start")?;
        let values = EspoolerValues::new(config)?;
        let function = printer.lookup_or_load::<super::functions::AfcFunction>(config, "AFC_functions")?;
        let afc_motor_rwd = afc_motor_rwd.map(|_| AssistMotor::new(config, "rwd")).transpose()?;
        let afc_motor_fwd = afc_motor_fwd.map(|_| AssistMotor::new(config, "fwd")).transpose()?;
        // (python sets up the enable pin after registering the forward pin's
        // commands; registering allocates nothing, so the pins' order is the same)
        let afc_motor_enb = afc_motor_enb.map(|_| AssistMotor::new(config, "enb")).transpose()?;
        let esp = Rc::new_cyclic(|me: &Weak<Espooler>| {
            let w = me.clone();
            let callback_timer = reactor.register_timer(
                Box::new(move |et| match w.upgrade() {
                    Some(e) => e.timer_callback(et).unwrap_or_else(|err| {
                        kaliru::logging::exception_err("AFC espooler timer", &err);
                        NEVER
                    }),
                    None => NEVER,
                }),
                NEVER,
            );
            let w = me.clone();
            let stats_timer = reactor.register_timer(
                Box::new(move |et| match w.upgrade() {
                    Some(e) => e.timer_stats_callback(et).unwrap_or_else(|err| {
                        kaliru::logging::exception_err("AFC espooler stats timer", &err);
                        NEVER
                    }),
                    None => NEVER,
                }),
                NEVER,
            );
            Espooler {
                name: name.to_string(),
                afc: afc.clone(),
                reactor: reactor.clone(),
                callback_timer,
                stats_timer,
                lane_obj: RefCell::new(Weak::new()),
                mcu,
                afc_motor_rwd,
                afc_motor_fwd,
                afc_motor_enb,
                n20_break_delay_time: RefCell::new(n20_break_delay_time),
                timer_delay: RefCell::new(timer_delay),
                enable_assist: RefCell::new(enable_assist),
                enable_assist_weight: RefCell::new(enable_assist_weight),
                debug: RefCell::new(debug),
                enable_kick_start: RefCell::new(enable_kick_start),
                past_extruder_position: Cell::new(-1.0),
                values,
                stats: RefCell::new(None),
            }
        });
        if esp.afc_motor_fwd.is_some() {
            type Cmd = fn(&Espooler, &GCodeCommand) -> KResult<()>;
            let cmds: [(&str, Cmd, &str); 4] = [
                ("SET_ESPOOLER_VALUES", Espooler::cmd_set_espooler_values, CMD_SET_ESPOOLER_VALUES_HELP),
                ("TEST_ESPOOLER_ASSIST", Espooler::cmd_test_espooler_assist, CMD_TEST_ESPOOLER_ASSIST_HELP),
                ("ENABLE_ESPOOLER_ASSIST", Espooler::cmd_enable_espooler_assist, CMD_ENABLE_ESPOOLER_ASSIST_HELP),
                ("DISABLE_ESPOOLER_ASSIST", Espooler::cmd_disable_espooler_assist, CMD_DISABLE_ESPOOLER_ASSIST_HELP),
            ];
            for (cmd, f, help) in cmds {
                let e = esp.clone();
                afc.gcode.register_mux_command(cmd, "LANE", Some(name), Rc::new(move |g| f(&e, g)), Some(help))?;
            }
        }
        // Only register macro if fwd or rwd pins are defined
        if esp.afc_motor_fwd.is_some() || esp.afc_motor_rwd.is_some() {
            let e = esp.clone();
            function.register_mux_command(
                afc.show_macros,
                "AFC_RESET_MOTOR_TIME",
                "LANE",
                name,
                Rc::new(move |g| e.cmd_afc_reset_motor_time(g)),
                CMD_AFC_RESET_MOTOR_TIME_HELP,
                Some(&[("LANE", "string", "lane1")]),
            )?;
        }
        Ok(esp)
    }
}

impl Espooler {
    fn lane(&self) -> KResult<Rc<AfcLane>> {
        self.lane_obj.borrow().upgrade().ok_or_else(|| no_attr("NoneType", "weight"))
    }

    pub fn handle_ready(&self) {
        if self.afc_motor_fwd.is_some() || self.afc_motor_rwd.is_some() {
            self.reactor.update_timer(&self.stats_timer, self.reactor.monotonic() + 30.0);
        } else {
            self.afc.logger.info(&AFC_ASSIST_ESPOOLER_HANDLE_READY, &format!("Not starting timer for {}", self.name));
        }
    }

    /// The unit's values for those the lane does not set
    pub fn handle_connect(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        self.values.handle_connect(lane)?;
        *self.stats.borrow_mut() = Some(Rc::new(EspoolerStats::new(&self.name)));
        *self.lane_obj.borrow_mut() = Rc::downgrade(lane);
        let unit = lane.unit_obj()?;
        let s = &unit.settings;
        let fill = |v: &RefCell<PyVal>, d: PyVal| {
            if matches!(*v.borrow(), PyVal::None) {
                *v.borrow_mut() = d;
            }
        };
        fill(&self.n20_break_delay_time, PyVal::Float(s.n20_break_delay_time));
        fill(&self.timer_delay, s.timer_delay.clone());
        fill(&self.enable_assist, PyVal::Bool(s.enable_assist));
        fill(&self.enable_assist_weight, PyVal::Float(s.enable_assist_weight));
        fill(&self.debug, PyVal::Bool(s.debug));
        fill(&self.enable_kick_start, PyVal::Bool(s.enable_kick_start));
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
        self.values.full_weight.borrow().clone()
    }

    pub fn set_full_weight(&self, v: PyVal) {
        *self.values.full_weight.borrow_mut() = v;
    }

    /// afc_motor_rwd is not None
    pub fn has_rwd(&self) -> bool {
        self.afc_motor_rwd.is_some()
    }

    /// afc_motor_rwd.is_pwm
    pub fn rwd_is_pwm(&self) -> bool {
        self.afc_motor_rwd.as_ref().is_some_and(|m| m.is_pwm)
    }

    /// Every 30 s: the run times to moonraker, unless printing
    fn timer_stats_callback(&self, _eventtime: f64) -> KResult<f64> {
        if !self.afc.function.is_printing(true)? {
            self.stats()?.update_database();
        }
        Ok(self.reactor.monotonic() + 30.0)
    }

    /// The print assist: a spool assist each delta_movement of extrusion
    fn timer_callback(&self, eventtime: f64) -> KResult<f64> {
        if self.enable_assist.borrow().truthy() && self.afc.function.in_print()? && !self.afc.function.is_paused()? && !self.afc.in_toolchange.get() {
            let past = self.past_extruder_position.get();
            let extruder_pos = self.afc.function.get_extruder_pos(Some(eventtime), Some(past), None)?;
            let delta_length = extruder_pos - past;
            if past == -1.0 {
                self.past_extruder_position.set(extruder_pos);
            } else if delta_length > self.values.delta_movement()? {
                self.past_extruder_position.set(extruder_pos);
                self.do_assist_move(Some(self.get_print_time(Some(eventtime))))?;
            }
            if self.debug.borrow().truthy() {
                self.afc.logger.info(
                    &AFC_ASSIST_ESPOOLER_TIMER_CALLBACK,
                    &format!("Timer Callback {:.3} e:{:.3} d:{:.3} p:{:.3}", eventtime, extruder_pos, delta_length, self.past_extruder_position.get()),
                );
            }
        }
        Ok(self.reactor.monotonic() + num(&self.timer_delay.borrow())?)
    }

    /// _get_print_time: a reactor time as print time, at least PIN_MIN_TIME ahead
    fn get_print_time(&self, systime: Option<f64>) -> f64 {
        let now = self.reactor.monotonic();
        let target_time = systime.unwrap_or(now);
        self.mcu.estimated_print_time(kaliru::pyfmt::py_max(target_time, now + PIN_MIN_TIME))
    }

    /// _kick_start: full speed for kick_start_time, to get the spool moving
    fn kick_start(&self, print_time: f64, reverse: bool) -> KResult<f64> {
        if reverse {
            self.move_reverse(print_time, 1.0)?;
        } else {
            self.move_forwards(print_time, 1.0)?;
        }
        Ok(print_time + self.values.kick_start_time()?)
    }

    fn set_enable_pin(&self, print_time: f64, value: f64) -> KResult<()> {
        if let Some(enb) = &self.afc_motor_enb {
            enb.set_pin(print_time, value)?;
        }
        if value == 0.0 {
            self.stats()?.set_end_time(print_time)
        } else {
            self.stats()?.set_start_time(print_time);
            Ok(())
        }
    }

    /// One print assist: kick start then cruise, below enable_assist_weight
    pub fn do_assist_move(&self, print_time: Option<f64>) -> KResult<()> {
        let Some(fwd) = &self.afc_motor_fwd else { return Ok(()) };
        let time = print_time.unwrap_or_else(|| self.get_print_time(None));
        let mut print_time = time;
        let lane = self.lane()?;
        let weight = lane.weight.borrow().clone();
        if num(&weight)? < num(&self.enable_assist_weight.borrow())? {
            print_time = self.kick_start(print_time, false)?;
            self.move_forwards(print_time, 1.0)?;
            let cruise_time = self.values.calculate_cruise_time(&weight)?;
            self.values.cruise_time.set(cruise_time);
            print_time += cruise_time;
            fwd.set_pin(print_time, 0.0)?;
            self.set_enable_pin(print_time, 0.0)?;
        }
        if self.debug.borrow().truthy() {
            self.afc.logger.debug(
                &AFC_ASSIST_ESPOOLER_DO_ASSIST_MOVE,
                &format!(
                    "Cruise time: {:.3} {:.3} {:.3}, Weight: {}, Enable weight: {}",
                    self.values.cruise_time.get(),
                    time,
                    print_time,
                    weight.str(),
                    self.enable_assist_weight.borrow().str()
                ),
            );
        }
        Ok(())
    }

    fn move_forwards(&self, print_time: f64, value: f64) -> KResult<()> {
        self.stats()?.set_direction(EspoolerDir::Fwd);
        self.set_enable_pin(print_time, 1.0)?;
        self.afc_motor_fwd.as_ref().ok_or_else(|| no_attr("NoneType", "_set_pin"))?.set_pin(print_time, value)
    }

    fn move_reverse(&self, print_time: f64, value: f64) -> KResult<()> {
        self.stats()?.set_direction(EspoolerDir::Rwd);
        self.set_enable_pin(print_time, 1.0)?;
        self.afc_motor_rwd.as_ref().ok_or_else(|| no_attr("NoneType", "_set_pin"))?.set_pin(print_time, value)
    }

    /// The spooler on: value < 0 reverse, > 0 forwards, 0 off (braked)
    pub fn assist(&self, value: f64) -> KResult<()> {
        let mut value = value;
        let mut reverse = false;
        let mut print_time = self.get_print_time(None);
        // Return if both pins are not defined
        if self.afc_motor_rwd.is_none() && self.afc_motor_fwd.is_none() {
            return Ok(());
        }
        let assist_motor = if value < 0.0 {
            let Some(m) = &self.afc_motor_rwd else { return Ok(()) };
            value *= -1.0;
            reverse = true;
            let stats = self.stats()?;
            stats.set_direction(EspoolerDir::Rwd);
            stats.set_start_time(print_time);
            m
        } else if value > 0.0 {
            let Some(m) = &self.afc_motor_fwd else { return Ok(()) };
            let stats = self.stats()?;
            stats.set_direction(EspoolerDir::Fwd);
            stats.set_start_time(print_time);
            m
        } else {
            self.break_espooler()?;
            return self.stats()?.set_end_time(print_time);
        };
        value /= assist_motor.scale;
        if !assist_motor.is_pwm && value != 0.0 && value != 1.0 && value > 0.0 {
            value = 1.0;
        }
        if let Some(enb) = &self.afc_motor_enb {
            let enable = if value != 0.0 { 1.0 } else { 0.0 };
            enb.set_pin(print_time, enable)?;
        }
        if self.enable_kick_start.borrow().truthy() {
            print_time = self.kick_start(print_time, reverse)?;
        }
        assist_motor.set_pin(print_time, value)
    }

    /// break_espooler: brake the N20 motors (all pins on), then let them coast
    fn break_espooler(&self) -> KResult<()> {
        let mut print_time = self.get_print_time(None);
        if let (Some(fwd), Some(rwd), Some(_)) = (&self.afc_motor_fwd, &self.afc_motor_rwd, &self.afc_motor_enb) {
            rwd.set_pin(print_time, 1.0)?;
            self.set_enable_pin(print_time, 1.0)?;
            fwd.set_pin(print_time, 1.0)?;
            // Forward predict delay time instead of adding reactor pause in code
            print_time += num(&self.n20_break_delay_time.borrow())?;
            self.set_enable_pin(print_time, 0.0)?;
        } else if self.afc_motor_enb.is_some() {
            self.set_enable_pin(print_time, 0.0)?;
        }
        if let Some(rwd) = &self.afc_motor_rwd {
            rwd.set_pin(print_time, 0.0)?;
        }
        if let Some(fwd) = &self.afc_motor_fwd {
            fwd.set_pin(print_time, 0.0)?;
        }
        Ok(())
    }

    /// The print assist's timer on (after a load into the toolhead)
    pub fn enable_timer(&self) -> KResult<()> {
        if self.afc_motor_fwd.is_none() {
            return Ok(());
        }
        self.past_extruder_position.set(-1.0);
        if self.enable_assist.borrow().truthy() {
            if self.debug.borrow().truthy() {
                self.afc.logger.info(&AFC_ASSIST_ESPOOLER_ENABLE_TIMER, &format!("{} espooler timer enabled", self.name));
            }
            self.reactor.update_timer(&self.callback_timer, self.reactor.monotonic() + num(&self.timer_delay.borrow())?);
        }
        Ok(())
    }

    /// The print assist's timer off (before an unload)
    pub fn disable_timer(&self) {
        if self.afc_motor_fwd.is_none() {
            return;
        }
        self.past_extruder_position.set(-1.0);
        self.reactor.update_timer(&self.callback_timer, NEVER);
    }

    /// The N20 run times for the console ("" without motors)
    pub fn get_spooler_stats(&self, short: bool) -> KResult<String> {
        let mut ret_str = String::from("N20 active time:");
        let (fwd, rwd) = (self.afc_motor_fwd.is_some(), self.afc_motor_rwd.is_some());
        if !fwd && !rwd {
            ret_str = String::new();
        }
        if fwd {
            let v = self.stats()?.n20_runtime_fwd()?;
            if short {
                ret_str += " fwd:";
                ret_str = format!("{:>32}{:>8}  ", ret_str, v);
                if rwd {
                    ret_str += "|\n";
                }
            } else {
                ret_str += &format!(" fwd:{:>8}", v);
            }
        }
        if rwd {
            let v = self.stats()?.n20_runtime_rwd()?;
            if short {
                if !fwd {
                    ret_str += " rwd:";
                    ret_str = format!("{:>32}{:>8}  ", ret_str, v);
                } else {
                    ret_str += &format!("|{:>32}{:>8}  ", "rwd:", v);
                }
            } else {
                if !fwd {
                    // Pad out the " fwd:########" slot that's skipped here so
                    // "rwd:" lands in the same column it would if fwd were
                    // also present.
                    ret_str += &format!("{:>13}", "");
                }
                ret_str += &format!(" rwd:{:>8}", v);
            }
        }
        Ok(ret_str)
    }

    fn cmd_test_espooler_assist(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.do_assist_move(Some(self.get_print_time(None)))
    }

    fn cmd_enable_espooler_assist(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        *self.enable_assist.borrow_mut() = PyVal::Bool(true);
        if self.afc.function.get_current_lane().as_deref() == Some(self.name.as_str()) {
            self.enable_timer()?;
            self.afc.logger.info(&AFC_ASSIST_ESPOOLER_CMD_ENABLE_ESPOOLER_ASSIST, &format!("Espooler assist enabled for {}", self.name));
        } else {
            self.afc.logger.info(&AFC_ASSIST_ESPOOLER_CMD_ENABLE_ESPOOLER_ASSIST, &format!("{} currently not loaded only enabling assist, not enabling timer", self.name));
        }
        Ok(())
    }

    fn cmd_disable_espooler_assist(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.disable_timer();
        *self.enable_assist.borrow_mut() = PyVal::Bool(false);
        self.afc.logger.info(&AFC_ASSIST_ESPOOLER_CMD_DISABLE_ESPOOLER_ASSIST, &format!("Espooler assist disabled for {}", self.name));
        Ok(())
    }

    fn cmd_set_espooler_values(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let f = &self.afc.function;
        let section = self.lane()?.fullname.clone();
        let get = |kind: &str, var: PyVal, name: &str, key: Option<&str>, cast: bool| f.gcode_get_value(gcmd, kind, &var, name, &section, key, cast);
        let set = |cell: &RefCell<PyVal>, v: PyVal| *cell.borrow_mut() = v;
        let cur = |cell: &RefCell<PyVal>| cell.borrow().clone();
        let v = &self.values;
        set(&self.n20_break_delay_time, get("get_float", cur(&self.n20_break_delay_time), "BREAK_DELAY", Some("n20_break_delay_time"), false)?);
        set(&v.kick_start_time, get("get_float", PyVal::Float(v.kick_start_time()?), "KICK_START_TIME", None, false)?);
        set(&v.spool_outer_diameter, get("get_float", cur(&v.spool_outer_diameter), "SPOOL_OUTER_DIAMETER", None, false)?);
        set(&v.spool_inner_diameter, get("get_float", cur(&v.spool_inner_diameter), "SPOOL_INNER_DIAMETER", None, false)?);
        set(&v.full_weight, get("get_float", cur(&v.full_weight), "FULL_WEIGHT", None, false)?);
        set(&v.spool_ratio, get("get_float", cur(&v.spool_ratio), "SPOOL_RATIO", None, false)?);
        set(&v.max_motor_rpm, get("get_float", cur(&v.max_motor_rpm), "MAX_MOTOR_RPM", None, false)?);
        set(&v.espool_rot_dist, get("get_float", cur(&v.espool_rot_dist), "ESPOOL_ROT_DIST", None, false)?);
        set(&v.delta_movement, get("get_float", PyVal::Float(v.delta_movement()?), "DELTA_MOVEMENT", None, false)?);
        set(&v.scaling, get("get_float", cur(&v.scaling), "SPOOLRATE", None, false)?);
        set(&self.timer_delay, get("get_float", cur(&self.timer_delay), "TIMER_DELAY", None, false)?);
        set(&self.enable_assist_weight, get("get_float", cur(&self.enable_assist_weight), "ASSIST_WEIGHT", Some("enable_assist_weight"), false)?);
        set(&self.enable_assist, get("get_int", cur(&self.enable_assist), "ENABLE_ASSIST", None, true)?);
        set(&self.debug, get("get_int", cur(&self.debug), "DEBUG", None, true)?);
        set(&self.enable_kick_start, get("get_int", cur(&self.enable_kick_start), "ENABLE_KICK_START", None, true)?);
        // update cruise time with new values
        let full_weight = cur(&v.full_weight);
        let cruise_time = v.calculate_cruise_time(&full_weight)?;
        v.cruise_time.set(cruise_time);
        Ok(())
    }

    fn cmd_afc_reset_motor_time(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.stats()?.reset_runtimes()?;
        self.afc.logger.info(&AFC_ASSIST_ESPOOLER_CMD_AFC_RESET_MOTOR_TIME, &format!("N20 active time has been reset for {}", self.name));
        Ok(())
    }
}
