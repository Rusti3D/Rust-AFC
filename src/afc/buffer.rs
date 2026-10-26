// AFC_buffer: a switched (TurtleNeck style) buffer, its advance and trailing
// switches setting the active lane's rotation distance (port of
// AFC_buffer.py's AFCBuffer; native_scope leaves FPS_PSF buffers to the
// python AFC)
//
// Copyright (C) 2024-2026 AFCProject
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::lane::AfcLane;
use super::utils::{add_filament_switch, SwitchOptions};
use super::{no_attr, Afc};
use kaliru::configfile::{ConfigWrapper, Dflt, Lim};
use kaliru::error::{KError, KResult};
use kaliru::extras::buttons::PrinterButtons;
use kaliru::gcode::GCodeCommand;
use kaliru::printer::{PObj, Printer, PrinterObject};
use kaliru::pyval::{PyDict, PyVal};
use kaliru::reactor::{Reactor, Timer, NEVER, NOW};
use kaliru::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const TRAILING_STATE_NAME: &str = "Trailing";
const ADVANCING_STATE_NAME: &str = "Advancing";
const CHECK_RUNOUT_TIMEOUT: f64 = 0.5;

const CMD_AFC_SET_ERROR_SENSITIVITY_HELP: &str = "Set filament error sensitivity (0-10, 0=disabled)";
const CMD_SET_BUFFER_MULTIPLIER_HELP: &str = "live adjust buffer high and low multiplier";
const CMD_LANE_ROT_FACTOR_HELP: &str = "change rotation distance by factor specified";
const CMD_QUERY_BUFFER_HELP: &str = "Report Buffer sensor state";

pub struct AfcBuffer {
    me: Weak<AfcBuffer>,
    printer: Rc<Printer>,
    pub afc: Rc<Afc>,
    pub reactor: Rc<Reactor>,
    pub name: String,
    pub kind: String,
    pub debug: bool,
    /// type FPS_PSF: the analog sensor's part (fps.rs)
    pub fps: Option<super::fps::Fps>,
    pub lanes: RefCell<IndexMap<String, Rc<AfcLane>>>,
    last_state: RefCell<String>,
    enable: Cell<bool>,
    current_lane: RefCell<Option<Rc<AfcLane>>>,
    pub advance_state: Cell<bool>,
    pub trailing_state: Cell<bool>,
    min_event_systime: Cell<f64>,
    error_sensitivity: RefCell<PyVal>,
    fault_sensitivity: RefCell<PyVal>,
    filament_error_pos: Cell<Option<f64>>,
    past_extruder_position: Cell<Option<f64>>,
    extruder_pos_timer: RefCell<Option<Timer>>,
    fault_timer: Cell<Option<&'static str>>,
    led: bool,
    led_index: Option<String>,
    led_advancing: String,
    led_trailing: String,
    led_neutral: String,
    led_buffer_disabled: String,
    pub advance_pin: Option<String>,
    multiplier_high: RefCell<PyVal>,
    multiplier_low: RefCell<PyVal>,
    last_multiplier: RefCell<PyVal>,
}

fn num(v: &PyVal) -> f64 {
    v.as_f64().unwrap_or(0.0)
}

/// get_fault_sensitivity: the extrusion distance before a fault, 0 disables
fn get_fault_sensitivity(sensitivity: &PyVal) -> PyVal {
    if num(sensitivity) > 0.0 {
        PyVal::Float((11.0 - num(sensitivity)) * 10.0)
    } else {
        PyVal::Int(0)
    }
}

impl AfcBuffer {
    fn new(config: &ConfigWrapper) -> KResult<Rc<AfcBuffer>> {
        let printer = config.get_printer();
        let afc = printer.lookup_or_load::<Afc>(config, "AFC")?;
        let reactor = afc.reactor.clone();
        let kind = config.get("type", "switched")?;
        let function = printer.lookup_or_load::<super::functions::AfcFunction>(config, "AFC_functions")?;
        let name = config.get_name().split(' ').next_back().unwrap_or("").to_string();
        let debug = config.getboolean("debug", false)?;
        let enable_sensors_in_gui = config.getboolean("enable_sensors_in_gui", afc.enable_sensors_in_gui)?;
        let buttons = printer.lookup_or_load::<PrinterButtons>(config, "buttons")?;
        // error sensitivity, 0 disables, 1 is the least, 10 is the most
        let error_sensitivity = config
            .getfloat_value("filament_error_sensitivity", &Dflt::Val(PyVal::Int(0)), &Lim::new().min(0.0).max(10.0))?
            .unwrap_or(PyVal::Int(0));
        let fault_sensitivity = get_fault_sensitivity(&error_sensitivity);
        // LED SETTINGS
        let led_index = config.get_opt("led_index")?;
        let led_advancing = config.get("led_buffer_advancing", afc.led_buffer_advancing.as_str())?;
        let led_trailing = config.get("led_buffer_trailing", afc.led_buffer_trailing.as_str())?;
        let led_neutral = config.get("led_buffer_neutral", afc.led_buffer_neutral.as_str())?;
        let led_buffer_disabled = config.get("led_buffer_disable", afc.led_buffer_disabled.as_str())?;
        let led = led_index.is_some();
        let _buffer_distance = config.getfloat_opt("distance", Lim::new())?;
        let multiplier_high = config.getfloat_value("multiplier_high", &Dflt::Val(PyVal::Float(1.1)), &Lim::new().min(1.0))?.unwrap_or(PyVal::Float(1.1));
        let multiplier_low = config.getfloat_value("multiplier_low", &Dflt::Val(PyVal::Float(0.9)), &Lim::new().min(0.0).max(1.0))?.unwrap_or(PyVal::Float(0.9));
        let (advance_pin, trailing_pin) = if kind == "switched" {
            (Some(config.get("advance_pin", Dflt::Required)?), Some(config.get("trailing_pin", Dflt::Required)?))
        } else {
            (None, None)
        };
        // (AFCFPSBuffer's own settings come after the base buffer's)
        let fps_cfg = if kind == "FPS_PSF" { Some(super::fps::FpsConfig::new(&printer, config, &name)?) } else { None };
        let (multiplier_high, multiplier_low, last_multiplier) = match &fps_cfg {
            Some(c) => {
                let (h, l) = super::fps::Fps::multipliers(c);
                (PyVal::Float(h), PyVal::Float(l), PyVal::Float(1.0))
            }
            None => (multiplier_high, multiplier_low, PyVal::Int(1)),
        };
        let reactor2 = reactor.clone();
        let buffer = Rc::new_cyclic(|me| AfcBuffer {
            me: me.clone(),
            printer: printer.clone(),
            afc: afc.clone(),
            reactor,
            name: name.clone(),
            kind: kind.clone(),
            debug,
            fps: fps_cfg.map(|c| super::fps::Fps::new(c, &reactor2, me)),
            lanes: RefCell::new(IndexMap::default()),
            last_state: RefCell::new("Unknown".into()),
            enable: Cell::new(false),
            current_lane: RefCell::new(None),
            advance_state: Cell::new(false),
            trailing_state: Cell::new(false),
            min_event_systime: Cell::new(NEVER),
            error_sensitivity: RefCell::new(error_sensitivity),
            fault_sensitivity: RefCell::new(fault_sensitivity),
            filament_error_pos: Cell::new(None),
            past_extruder_position: Cell::new(None),
            extruder_pos_timer: RefCell::new(None),
            fault_timer: Cell::new(None),
            led,
            led_index,
            led_advancing,
            led_trailing,
            led_neutral,
            led_buffer_disabled,
            advance_pin: advance_pin.clone(),
            multiplier_high: RefCell::new(multiplier_high),
            multiplier_low: RefCell::new(multiplier_low),
            last_multiplier: RefCell::new(last_multiplier),
        });
        let b = buffer.clone();
        kaliru::events::KlippyReady::on(&printer, move || b.handle_ready());
        if let (Some(advance_pin), Some(trailing_pin)) = (&advance_pin, &trailing_pin) {
            // Pull config for Turtleneck style buffer (advance and training switches)
            let opts = SwitchOptions { show_sensor: enable_sensors_in_gui, ..Default::default() };
            add_filament_switch(&format!("{}_expanded", name), advance_pin, &printer, &opts)?;
            add_filament_switch(&format!("{}_compressed", name), trailing_pin, &printer, &opts)?;
            // Turtleneck Buffer
            let w = buffer.me.clone();
            buttons.register_buttons(&[advance_pin.as_str()], Rc::new(move |et, state| w.upgrade().map_or(Ok(()), |b| b.advance_callback(et, state != 0))))?;
            let w = buffer.me.clone();
            buttons.register_buttons(&[trailing_pin.as_str()], Rc::new(move |et, state| w.upgrade().map_or(Ok(()), |b| b.trailing_callback(et, state != 0))))?;
        }
        // Register Macros
        let b = buffer.clone();
        let is_fps = kind == "FPS_PSF";
        function.register_mux_command(
            afc.show_macros,
            "QUERY_BUFFER",
            "BUFFER",
            &name,
            Rc::new(move |g| if b.fps.is_some() { super::fps::cmd_query_buffer(&b, g) } else { b.cmd_query_buffer(g) }),
            if is_fps { super::fps::CMD_QUERY_BUFFER_HELP } else { CMD_QUERY_BUFFER_HELP },
            Some(&[("BUFFER", "string", if is_fps { "" } else { "Turtle_1" })]),
        )?;
        type Cmd = fn(&AfcBuffer, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, Cmd, Option<&str>); 5] = [
            ("ENABLE_BUFFER", AfcBuffer::cmd_enable_buffer, None),
            ("DISABLE_BUFFER", AfcBuffer::cmd_disable_buffer, None),
            ("AFC_SET_ERROR_SENSITIVITY", AfcBuffer::cmd_afc_set_error_sensitivity, Some(CMD_AFC_SET_ERROR_SENSITIVITY_HELP)),
            ("SET_ROTATION_FACTOR", AfcBuffer::cmd_set_rotation_factor, Some(CMD_LANE_ROT_FACTOR_HELP)),
            ("SET_BUFFER_MULTIPLIER", AfcBuffer::cmd_set_buffer_multiplier, Some(CMD_SET_BUFFER_MULTIPLIER_HELP)),
        ];
        for (cmd, f, help) in cmds {
            let b = buffer.clone();
            afc.gcode.register_mux_command(cmd, "BUFFER", Some(&name), Rc::new(move |g| f(&b, g)), help)?;
        }
        afc.buffers.borrow_mut().insert(name, buffer.clone());
        if buffer.fps.is_some() {
            super::fps::finish_init(&buffer, &printer, super::fps::report_time(config)?, enable_sensors_in_gui)?;
        }
        Ok(buffer)
    }

    fn handle_ready(&self) -> KResult<()> {
        self.min_event_systime.set(self.reactor.monotonic() + 2.0);
        if num(&self.error_sensitivity.borrow()) > 0.0 {
            self.setup_fault_timer()?;
        }
        if let Some(idx) = &self.led_index {
            // Verify that LED config is found
            let (error_string, led) = self.afc.function.verify_led_object(idx);
            if led.is_none() {
                return Err(KError::config(error_string));
            }
        }
        Ok(())
    }

    pub fn disable_fault_sensitivity(&self) {
        *self.fault_sensitivity.borrow_mut() = PyVal::Int(0);
    }

    pub fn restore_fault_sensitivity(&self) {
        *self.fault_sensitivity.borrow_mut() = get_fault_sensitivity(&self.error_sensitivity.borrow());
    }

    fn setup_fault_timer(&self) -> KResult<()> {
        self.update_filament_error_pos()?;
        // register timer that will run to check buffer state changes
        if self.extruder_pos_timer.borrow().is_none() {
            let w = self.me.clone();
            let t = self.reactor.register_timer(
                Box::new(move |et| match w.upgrade() {
                    Some(b) => b.extruder_pos_update_event(et).unwrap_or_else(|e| {
                        kaliru::logging::exception_err("AFC buffer fault timer", &e);
                        NEVER
                    }),
                    None => NEVER,
                }),
                NEVER,
            );
            *self.extruder_pos_timer.borrow_mut() = Some(t);
        }
        Ok(())
    }

    fn start_fault_timer(&self) {
        if let Some(t) = self.extruder_pos_timer.borrow().as_ref() {
            self.fault_timer.set(Some("Running"));
            self.reactor.update_timer(t, NOW);
        }
    }

    pub fn stop_fault_timer(&self) {
        if let Some(t) = self.extruder_pos_timer.borrow().as_ref() {
            self.fault_timer.set(Some("Stopped"));
            self.reactor.update_timer(t, NEVER);
        }
    }

    pub fn fault_detection_enabled(&self) -> bool {
        num(&self.fault_sensitivity.borrow()) > 0.0
    }

    pub fn start_fault_detection(&self, multiplier: PyVal) -> KResult<()> {
        self.set_multiplier(multiplier)?;
        self.update_filament_error_pos()?;
        self.start_fault_timer();
        Ok(())
    }

    /// get_extruder_pos: the extruder's position, forward moves only
    fn get_extruder_pos(&self) -> KResult<f64> {
        let current_pos = self.afc.function.get_extruder_pos(None, self.past_extruder_position.get(), None)?;
        self.past_extruder_position.set(Some(current_pos));
        Ok(current_pos)
    }

    pub fn update_filament_error_pos(&self) -> KResult<()> {
        let current_pos = self.get_extruder_pos()?;
        self.filament_error_pos.set(Some(current_pos + num(&self.fault_sensitivity.borrow())));
        Ok(())
    }

    fn extruder_pos_update_event(&self, eventtime: f64) -> KResult<f64> {
        if self.fps.is_some() {
            return super::fps::extruder_pos_update_event(self, eventtime);
        }
        let extruder_pos = self.get_extruder_pos()?;
        // Check for filament problems (a stepperless_drive unit, OpenAMS, is
        // not native)
        if self.afc.function.is_printing(true)? {
            if let Some(error_pos) = self.filament_error_pos.get() {
                if extruder_pos > error_pos {
                    self.pause_on_error("AFC filament fault detected! Take necessary action.", true)?;
                    // Update error position after triggering to prevent repeated triggers
                    self.update_filament_error_pos()?;
                }
            }
        }
        Ok(eventtime + CHECK_RUNOUT_TIMEOUT)
    }

    fn pause_on_error(&self, msg: &str, pause: bool) -> KResult<()> {
        let eventtime = self.reactor.monotonic();
        if eventtime < self.min_event_systime.get() || !self.enable.get() || self.afc.function.is_paused()? {
            return Ok(());
        }
        if pause {
            let mut msg = msg.to_string();
            let last_state = self.last_state.borrow().clone();
            if last_state == TRAILING_STATE_NAME {
                msg += "\nCLOG DETECTED";
            }
            if last_state == ADVANCING_STATE_NAME {
                msg += "\nAFC NOT FEEDING";
            }
            self.afc.error.afc_error(&AFC_BUFFER_AFCBUFFER_PAUSE_ON_ERROR, &msg, true, None)?;
        }
        Ok(())
    }

    pub fn led(&self, color: &str) -> KResult<()> {
        if self.led {
            self.afc.function.afc_led(&PyVal::from(color), self.led_index.as_deref())?;
        }
        Ok(())
    }

    pub fn enable_buffer(&self, lane: &Rc<AfcLane>) -> KResult<()> {
        if self.fps.is_some() {
            let me = self.me.upgrade().ok_or_else(|| no_attr("AFCBuffer", "fps_value"))?;
            return super::fps::enable_buffer(&me, lane);
        }
        *self.current_lane.borrow_mut() = Some(lane.clone());
        self.led(&self.led_buffer_disabled)?;
        self.enable.set(true);
        let fault = self.fault_detection_enabled();
        let multiplier = if *self.last_state.borrow() == TRAILING_STATE_NAME {
            let low = num(&self.multiplier_low.borrow());
            if fault {
                PyVal::Float((low * 2.0) / 5.0)
            } else {
                self.multiplier_low.borrow().clone()
            }
        } else {
            let high = num(&self.multiplier_high.borrow());
            if fault {
                PyVal::Float(high * 1.5)
            } else {
                self.multiplier_high.borrow().clone()
            }
        };
        if fault {
            self.start_fault_detection(multiplier)?;
        } else {
            self.set_multiplier(multiplier)?;
        }
        self.afc.logger.debug(&AFC_BUFFER_AFCBUFFER_ENABLE_BUFFER, &format!("{} buffer enabled for {}", self.name, lane.name));
        Ok(())
    }

    pub fn disable_buffer(&self) -> KResult<()> {
        if self.fps.is_some() {
            return super::fps::disable_buffer(self);
        }
        self.enable.set(false);
        let Some(lane) = self.current_lane.borrow().clone() else { return Ok(()) };
        self.afc.logger.debug(&AFC_BUFFER_AFCBUFFER_DISABLE_BUFFER, &format!("{} buffer disabled for {}", self.name, lane.name));
        self.led(&self.led_buffer_disabled)?;
        self.reset_multiplier()?;
        if num(&self.error_sensitivity.borrow()) > 0.0 && self.extruder_pos_timer.borrow().is_some() {
            self.stop_fault_timer();
        }
        *self.current_lane.borrow_mut() = None;
        Ok(())
    }

    /// the lane's stepper's rotation distance; None for a lane without one
    pub fn rotation_distance(lane: &AfcLane) -> Option<f64> {
        if let Some(s) = lane.stepper.borrow().as_ref() {
            return Some(s.extruder_stepper.stepper.get_rotation_distance().0);
        }
        let drive = lane.drive_stepper.borrow().clone();
        drive.and_then(|d| d.stepper.borrow().as_ref().map(|s| s.extruder_stepper.stepper.get_rotation_distance().0))
    }

    fn set_multiplier(&self, multiplier: PyVal) -> KResult<()> {
        if self.fps.is_some() {
            return super::fps::set_multiplier(self, num(&multiplier));
        }
        if !self.enable.get() {
            return Ok(());
        }
        let Some(lane) = self.current_lane.borrow().clone() else { return Ok(()) };
        if lane.stepper.borrow().is_none() {
            return Ok(());
        }
        let m = num(&multiplier);
        lane.update_rotation_distance(m)?;
        *self.last_multiplier.borrow_mut() = multiplier;
        if m > 1.0 {
            *self.last_state.borrow_mut() = ADVANCING_STATE_NAME.into();
            self.led(&self.led_trailing)?;
        } else if m < 1.0 {
            *self.last_state.borrow_mut() = TRAILING_STATE_NAME.into();
            self.led(&self.led_advancing)?;
        }
        self.afc.logger.debug(
            &AFC_BUFFER_AFCBUFFER_SET_MULTIPLIER,
            &format!("New rotation distance for {} after applying factor: {:.4}", lane.name, Self::rotation_distance(&lane).unwrap_or(0.0)),
        );
        Ok(())
    }

    fn reset_multiplier(&self) -> KResult<()> {
        if self.fps.is_some() {
            return super::fps::reset_multiplier(self);
        }
        self.afc.logger.debug(&AFC_BUFFER_AFCBUFFER_RESET_MULTIPLIER, "Buffer multiplier reset");
        let Some(lane) = self.current_lane.borrow().clone() else { return Ok(()) };
        if lane.stepper.borrow().is_none() {
            return Ok(());
        }
        lane.update_rotation_distance(1.0)?;
        self.afc.logger.info(
            &AFC_BUFFER_AFCBUFFER_RESET_MULTIPLIER,
            &format!("Rotation distance reset for {} : {:.4}", lane.name, Self::rotation_distance(&lane).unwrap_or(0.0)),
        );
        Ok(())
    }

    fn advance_callback(&self, _eventtime: f64, state: bool) -> KResult<()> {
        let c = &AFC_BUFFER_AFCBUFFER_ADVANCE_CALLBACK;
        self.advance_state.set(state);
        if self.printer.state_message() == "Printer is ready" && self.enable.get() {
            if self.current_lane.borrow().is_some() && state {
                if self.fault_detection_enabled() {
                    let multiplier_extra_low = (num(&self.multiplier_low.borrow()) * 2.0) / 5.0;
                    self.start_fault_detection(PyVal::Float(multiplier_extra_low))?;
                } else {
                    self.set_multiplier(self.multiplier_low.borrow().clone())?;
                    self.afc.logger.debug(c, "Buffer Triggered State: Advanced");
                }
            } else if self.fault_detection_enabled() {
                self.stop_fault_timer();
                self.set_multiplier(self.multiplier_low.borrow().clone())?;
                self.afc.logger.debug(c, "Buffer Triggered State: Advanced");
            }
        }
        *self.last_state.borrow_mut() = TRAILING_STATE_NAME.into();
        Ok(())
    }

    fn trailing_callback(&self, _eventtime: f64, state: bool) -> KResult<()> {
        let c = &AFC_BUFFER_AFCBUFFER_TRAILING_CALLBACK;
        self.trailing_state.set(state);
        if self.printer.state_message() == "Printer is ready" && self.enable.get() {
            if self.current_lane.borrow().is_some() && state {
                if self.fault_detection_enabled() {
                    let multiplier_extra_high = num(&self.multiplier_high.borrow()) * 1.5;
                    self.start_fault_detection(PyVal::Float(multiplier_extra_high))?;
                } else {
                    self.set_multiplier(self.multiplier_high.borrow().clone())?;
                    self.afc.logger.debug(c, "Buffer Triggered State: Trailing");
                }
            } else if self.fault_detection_enabled() {
                self.stop_fault_timer();
                self.set_multiplier(self.multiplier_high.borrow().clone())?;
                self.afc.logger.debug(c, "Buffer Triggered State: Trailing");
            }
        }
        *self.last_state.borrow_mut() = ADVANCING_STATE_NAME.into();
        Ok(())
    }

    pub fn current_lane(&self) -> Option<Rc<AfcLane>> {
        self.current_lane.borrow().clone()
    }

    pub fn set_current_lane(&self, lane: Option<Rc<AfcLane>>) {
        *self.current_lane.borrow_mut() = lane;
    }

    pub fn enabled(&self) -> bool {
        self.enable.get()
    }

    pub fn set_enabled(&self, v: bool) {
        self.enable.set(v);
    }

    pub fn set_last_state(&self, s: &str) {
        *self.last_state.borrow_mut() = s.to_string();
    }

    /// (multiplier_high, multiplier_low)
    pub fn multiplier_values(&self) -> (f64, f64) {
        (num(&self.multiplier_high.borrow()), num(&self.multiplier_low.borrow()))
    }

    pub fn last_multiplier_f64(&self) -> f64 {
        num(&self.last_multiplier.borrow())
    }

    pub fn set_last_multiplier(&self, m: PyVal) {
        *self.last_multiplier.borrow_mut() = m;
    }

    pub fn led_advancing(&self) -> String {
        self.led_advancing.clone()
    }

    pub fn led_trailing(&self) -> String {
        self.led_trailing.clone()
    }

    pub fn led_neutral(&self) -> String {
        self.led_neutral.clone()
    }

    pub fn led_buffer_disabled(&self) -> String {
        self.led_buffer_disabled.clone()
    }

    pub fn error_sensitivity_f64(&self) -> f64 {
        num(&self.error_sensitivity.borrow())
    }

    pub fn error_sensitivity_str(&self) -> String {
        self.error_sensitivity.borrow().str()
    }

    pub fn has_fault_timer(&self) -> bool {
        self.extruder_pos_timer.borrow().is_some()
    }

    pub fn filament_error_pos(&self) -> Option<f64> {
        self.filament_error_pos.get()
    }

    pub fn get_extruder_pos_pub(&self) -> KResult<f64> {
        self.get_extruder_pos()
    }

    pub fn pause_on_error_pub(&self, msg: &str, pause: bool) -> KResult<()> {
        self.pause_on_error(msg, pause)
    }

    pub fn buffer_status(&self) -> String {
        self.last_state.borrow().clone()
    }

    fn cmd_afc_set_error_sensitivity(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let sensitivity = gcmd.get_float("SENSITIVITY", None, Lim::new().min(0.0).max(10.0))?;
        let old_sensitivity = num(&self.error_sensitivity.borrow());
        *self.error_sensitivity.borrow_mut() = PyVal::Float(sensitivity);
        *self.fault_sensitivity.borrow_mut() = get_fault_sensitivity(&PyVal::Float(sensitivity));
        // Update fault detection state based on new sensitivity
        if old_sensitivity == 0.0 && sensitivity > 0.0 {
            // Fault detection was disabled, now enabling
            if self.fault_detection_enabled() {
                self.setup_fault_timer()?;
                if self.kind == "switched" {
                    let multiplier = if *self.last_state.borrow() == TRAILING_STATE_NAME {
                        self.multiplier_low.borrow().clone()
                    } else {
                        self.multiplier_high.borrow().clone()
                    };
                    self.start_fault_detection(multiplier)?;
                } else {
                    self.start_fault_detection(PyVal::Float(1.0))?;
                }
            }
        } else if old_sensitivity > 0.0 && sensitivity == 0.0 {
            // Fault detection was enabled, now disabling
            self.stop_fault_timer();
        } else if sensitivity > 0.0 {
            // Update the error position with new sensitivity
            self.update_filament_error_pos()?;
        }
        self.afc.logger.info(
            &AFC_BUFFER_AFCBUFFER_CMD_AFC_SET_ERROR_SENSITIVITY,
            &format!("Error sensitivity set to {} (fault sensitivity: {})", self.error_sensitivity.borrow().str(), self.fault_sensitivity.borrow().str()),
        );
        Ok(())
    }

    fn cmd_set_buffer_multiplier(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_BUFFER_AFCBUFFER_CMD_SET_BUFFER_MULTIPLIER;
        if self.current_lane.borrow().is_none() || !self.enable.get() {
            return Ok(());
        }
        let Some(chg_multiplier) = gcmd.get_opt("MULTIPLIER") else {
            self.afc.logger.info(c, "Multiplier must be provided, HIGH or LOW");
            return Ok(());
        };
        let chg_multiplier = chg_multiplier.to_uppercase();
        let chg_factor = gcmd.get_float("FACTOR", None, Lim::new())?;
        if chg_factor <= 0.0 {
            self.afc.logger.info(c, "FACTOR must be greater than 0");
            return Ok(());
        }
        let f = kaliru::pyfmt::py_float_repr(chg_factor);
        if chg_multiplier == "HIGH" && chg_factor > 1.0 {
            *self.multiplier_high.borrow_mut() = PyVal::Float(chg_factor);
            if self.kind == "switched" {
                self.set_multiplier(PyVal::Float(chg_factor))?;
            }
            self.afc.logger.info(c, &format!("multiplier_high set to {}", f));
            self.afc.logger.info(c, &format!("multiplier_high: {} MUST be updated under buffer config for value to be saved", f));
        } else if chg_multiplier == "LOW" && chg_factor < 1.0 {
            *self.multiplier_low.borrow_mut() = PyVal::Float(chg_factor);
            if self.kind == "switched" {
                self.set_multiplier(PyVal::Float(chg_factor))?;
            }
            self.afc.logger.info(c, &format!("multiplier_low set to {}", f));
            self.afc.logger.info(c, &format!("multiplier_low: {} MUST be updated under buffer config for value to be saved", f));
        } else {
            self.afc.logger.info(c, "multiplier_high must be greater than 1, multiplier_low must be less than 1");
        }
        Ok(())
    }

    fn cmd_set_rotation_factor(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_BUFFER_AFCBUFFER_CMD_SET_ROTATION_FACTOR;
        if self.current_lane.borrow().is_some() && self.enable.get() {
            let change_factor = gcmd.get_float("FACTOR", Some(1.0), Lim::new())?;
            if change_factor <= 0.0 {
                self.afc.logger.info(c, "FACTOR must be greater than 0");
            } else if change_factor == 1.0 {
                self.set_multiplier(PyVal::Int(1))?;
                self.afc.logger.info(c, "Rotation distance reset to base value");
            } else {
                self.set_multiplier(PyVal::Float(change_factor))?;
            }
        } else {
            self.afc.logger.info(c, &format!("BUFFER {} NOT ENABLED", self.name));
        }
        Ok(())
    }

    fn cmd_query_buffer(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let buffer_status = self.buffer_status();
        let suffix = match buffer_status.as_str() {
            TRAILING_STATE_NAME => " (buffer is compressing)",
            ADVANCING_STATE_NAME => " (buffer is expanding)",
            _ => "",
        };
        let mut state_info = format!("{}{}", buffer_status, suffix);
        if self.enable.get() {
            if let Some(lane) = self.current_lane.borrow().clone() {
                let rotation_dist = Self::rotation_distance(&lane).ok_or_else(|| no_attr("NoneType", "stepper"))?;
                state_info += &format!("\n{} Rotation distance: {:.4}", lane.name, rotation_dist);
            }
            if num(&self.error_sensitivity.borrow()) > 0.0 {
                state_info += &format!("\nFault detection enabled, sensitivity {}", self.error_sensitivity.borrow().str());
            }
        }
        self.afc.logger.info(&AFC_BUFFER_AFCBUFFER_CMD_QUERY_BUFFER, &format!("{} : {}", self.name, state_info));
        Ok(())
    }

    fn cmd_enable_buffer(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let lane = gcmd.get("LANE")?;
        let lane_obj = self.lanes.borrow().get(&lane).cloned();
        let Some(lane_obj) = lane_obj else {
            return Err(gcmd.error(format!("{} not assigned to {} buffer", lane, self.name)));
        };
        self.enable_buffer(&lane_obj)
    }

    fn cmd_disable_buffer(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.disable_buffer()
    }

    pub fn status(&self) -> KResult<PyVal> {
        let mut r = PyDict::default();
        r.insert("state".into(), PyVal::Str(self.last_state.borrow().clone()));
        r.insert("lanes".into(), PyVal::List(self.lanes.borrow().values().map(|l| PyVal::Str(l.name.clone())).collect()));
        r.insert("enabled".into(), PyVal::Bool(self.enable.get()));
        // Add current rotation distance if buffer is enabled and lane is loaded
        let lane = self.current_lane.borrow().clone();
        match lane.filter(|_| self.enable.get()) {
            Some(lane) => {
                if let Some(rd) = Self::rotation_distance(&lane) {
                    r.insert("rotation_distance".into(), PyVal::Float(rd));
                }
                r.insert("active_lane".into(), PyVal::Str(lane.name.clone()));
            }
            None => {
                r.insert("rotation_distance".into(), PyVal::None);
                r.insert("active_lane".into(), PyVal::None);
            }
        }
        // Add in multiplier information for automated testing
        r.insert("multiplier_high".into(), self.multiplier_high.borrow().clone());
        r.insert("multiplier_low".into(), self.multiplier_low.borrow().clone());
        r.insert("multiplier".into(), self.last_multiplier.borrow().clone());
        // Add fault detection information
        let sensitivity = self.error_sensitivity.borrow().clone();
        r.insert("fault_detection_enabled".into(), PyVal::Bool(num(&sensitivity) > 0.0));
        r.insert("error_sensitivity".into(), sensitivity.clone());
        r.insert("fault_timer".into(), self.fault_timer.get().map_or(PyVal::None, PyVal::from));
        let r = {
            let mut r = r;
        match self.filament_error_pos.get().filter(|_| num(&sensitivity) > 0.0) {
            Some(error_pos) => {
                let current_pos = self.get_extruder_pos()?;
                r.insert("distance_to_fault".into(), PyVal::Float(error_pos - current_pos));
                r.insert("filament_error_pos".into(), PyVal::Float(error_pos));
                r.insert("current_pos".into(), PyVal::Float(current_pos));
            }
            None => {
                r.insert("distance_to_fault".into(), PyVal::None);
            }
        }
            if let Some(f) = &self.fps {
                super::fps::add_status(f, &mut r);
            }
            r
        };
        Ok(PyVal::Dict(r))
    }
}

impl PrinterObject for AfcBuffer {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        match self.status() {
            Ok(s) => Some(s),
            Err(e) => {
                kaliru::logging::exception_err("AFC_buffer get_status", &e);
                None
            }
        }
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let buffer_type = config.get("type", "switched")?;
    if buffer_type == "switched" {
        return Ok(AfcBuffer::new(config)?);
    }
    if buffer_type == "FPS_PSF" {
        return Ok(AfcBuffer::new(config)?);
    }
    Err(KError::config(format!("{} not valid, only switched(turtleneck style) or FPS_PSF are valid options", buffer_type)))
}
