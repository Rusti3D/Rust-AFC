// AFC_buffer type FPS_PSF: an analog filament pressure sensor (OpenAMS FPS,
// PSF) whose reading proportionally sets the lane's rotation distance, and
// software endstops at its thresholds (port of AFC_buffer.py's AFCFPSBuffer
// and FPSEndstopWrapper)
//
// Copyright (C) 2024-2026 AFCProject
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::buffer::AfcBuffer;
use super::codes::*;
use super::lane::AfcLane;
use super::utils::VirtualSensor;
use kaliru::configfile::{ConfigWrapper, Dflt, Lim};
use kaliru::error::{KError, KResult};
use kaliru::gcode::GCodeCommand;
use kaliru::mcu::{AdcPin, Endstop, Mcu, StepperRef};
use kaliru::printer::Printer;
use kaliru::pyval::PyVal;
use kaliru::reactor::{Reactor, ReactorCompletion, Timer, NEVER, NOW};
use kaliru::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub const ADVANCING_STATE_NAME: &str = "Advancing";
pub const TRAILING_STATE_NAME: &str = "Trailing";
pub const NEUTRAL_STATE_NAME: &str = "Neutral";
const FPS_ENDSTOP_POLL_TIME: f64 = 0.01; // 10ms poll interval for software endstop
pub const CMD_QUERY_BUFFER_HELP: &str = "Report FPS buffer sensor state";
const CMD_AFC_SET_FPS_SET_POINT_HELP: &str = "Live adjust FPS buffer set point target";

/// The FPS settings, read before the buffer exists
pub struct FpsConfig {
    adc: Rc<dyn AdcPin>,
    reversed: bool,
    set_point: f64,
    low_point: f64,
    high_point: f64,
    homing_high_point: f64,
    multiplier_high: f64,
    multiplier_low: f64,
    trailing_min_multiplier: f64,
    multiplier_hysteresis: f64,
    deadband: f64,
    smoothing: f64,
    update_interval: f64,
    enable_integral_correction: bool,
    integral_gain: f64,
    integral_extrusion_threshold: f64,
}

/// _check_deadband: the error when the neutral window reaches a threshold
fn check_deadband(name: &str, set_point: f64, deadband: f64, low_point: f64, high_point: f64) -> String {
    let half_db = deadband / 2.0;
    let deadband_low = set_point - half_db;
    let deadband_high = set_point + half_db;
    let f = kaliru::pyfmt::py_float_repr;
    let mut error_msg = String::new();
    if deadband_low <= low_point {
        error_msg = format!("AFC_FPS {}: deadband ({}) too wide - neutral_low ({:.3}) must be above low_point ({})", name, f(deadband), deadband_low, f(low_point));
    }
    if deadband_high >= high_point {
        if !error_msg.is_empty() {
            error_msg += "\n";
        }
        error_msg += &format!("AFC_FPS {}: deadband ({}) too wide - neutral_high ({:.3}) must be below high_point ({})", name, f(deadband), deadband_high, f(high_point));
    }
    error_msg
}

impl FpsConfig {
    pub fn new(printer: &Printer, config: &ConfigWrapper, name: &str) -> KResult<FpsConfig> {
        // ---- ADC / FPS sensor configuration ----
        let adc_pin = config.get("adc_pin", Dflt::Required)?;
        let adc = kaliru::pins::setup_adc_pin(printer, &adc_pin)?;
        let sample_count = config.getint("sample_count", 5, Lim::new())?;
        let sample_time = config.getfloat("sample_time", 0.005, Lim::new())?;
        let report_time = config.getfloat("report_time", 0.100, Lim::new())?;
        let reversed = config.getboolean("reversed", false)?;
        // (Kalico: setup_adc_sample without report_time, setup_adc_callback with it)
        adc.setup_adc_sample(sample_time, sample_count, 0.0, 1.0, 0);
        let lim = |lo: f64, hi: f64| Lim::new().min(lo).max(hi);
        let sp_default = config.getfloat("neutral_point", 0.5, lim(0.1, 0.9))?;
        let set_point = config.getfloat("set_point", sp_default, lim(0.1, 0.9))?;
        let lp_default = config.getfloat("max_tension", 0.1, lim(0.0, 0.5))?;
        let low_point = config.getfloat("low_point", lp_default, lim(0.0, 0.5))?;
        let hp_default = config.getfloat("max_compression", 0.9, lim(0.5, 1.0))?;
        let high_point = config.getfloat("high_point", hp_default, lim(0.5, 1.0))?;
        let homing_high_point = config.getfloat("homing_high_point", 0.7, lim(0.6, 1.0))?;
        // Multiplier range — how aggressively the buffer corrects
        let multiplier_high = config.getfloat("multiplier_high", 1.15, Lim::new().min(1.0))?;
        let multiplier_low = config.getfloat("multiplier_low", 0.85, lim(0.0, 1.0))?;
        let trailing_min_multiplier = config.getfloat("trailing_min_multiplier", 1.00, Lim::new().min(1.0))?;
        let multiplier_hysteresis = config.getfloat("multiplier_hysteresis", 0.003, lim(0.0, 1.0))?;
        let deadband = config.getfloat("deadband", 0.30, lim(0.0, 0.6))?;
        let f = kaliru::pyfmt::py_float_repr;
        if low_point >= set_point {
            return Err(KError::config(format!("AFC_FPS {}: low_point ({}) must be less than set_point ({})", name, f(low_point), f(set_point))));
        }
        if high_point <= set_point {
            return Err(KError::config(format!("AFC_FPS {}: high_point ({}) must be greater than set_point ({})", name, f(high_point), f(set_point))));
        }
        let error_msg = check_deadband(name, set_point, deadband, low_point, high_point);
        if !error_msg.is_empty() {
            return Err(KError::config(error_msg));
        }
        let smoothing = config.getfloat("smoothing", 0.3, lim(0.0, 0.95))?;
        let update_interval = config.getfloat("update_interval", 0.25, Lim::new().min(0.05))?;
        let enable_integral_correction = config.getboolean("enable_integral_correction", false)?;
        let integral_gain = config.getfloat("integral_gain", 0.004, lim(0.0, 0.2))?;
        let integral_extrusion_threshold = config.getfloat("integral_extrusion_threshold", 0.02, Lim::new().min(0.0))?;
        let _ = report_time;
        Ok(FpsConfig {
            adc,
            reversed,
            set_point,
            low_point,
            high_point,
            homing_high_point,
            multiplier_high,
            multiplier_low,
            trailing_min_multiplier,
            multiplier_hysteresis,
            deadband,
            smoothing,
            update_interval,
            enable_integral_correction,
            integral_gain,
            integral_extrusion_threshold,
        })
    }
}

/// The FPS buffer's state next to the base buffer's
pub struct Fps {
    adc: Rc<dyn AdcPin>,
    reversed: bool,
    pub fps_value: Cell<f64>,
    pub smoothed_fps: Cell<f64>,
    pub set_point: Cell<f64>,
    pub low_point: f64,
    pub high_point: f64,
    homing_high_point: f64,
    trailing_min_multiplier: f64,
    multiplier_hysteresis: f64,
    deadband: Cell<f64>,
    smoothing: f64,
    update_interval: f64,
    enable_integral_correction: bool,
    integral_gain: f64,
    integral_terms: RefCell<IndexMap<String, f64>>,
    last_logged_integral: Cell<f64>,
    integral_extrusion_threshold: f64,
    integral_last_extruder_pos: Cell<Option<f64>>,
    advance_latched: Cell<bool>,
    latch_enabled: Cell<bool>,
    fila_adv: RefCell<Option<Rc<VirtualSensor>>>,
    fila_trail: RefCell<Option<Rc<VirtualSensor>>>,
    correction_timer: Timer,
    correction_running: Cell<bool>,
    saved_multipliers: RefCell<IndexMap<(String, String), (String, f64)>>,
    pub fps_endstop: Rc<FpsEndstop>,
    pub fps_trailing_endstop: Rc<FpsEndstop>,
}

impl Fps {
    /// the FPS part of a buffer being made (me: the buffer)
    pub fn new(cfg: FpsConfig, reactor: &Rc<Reactor>, me: &Weak<AfcBuffer>) -> Fps {
        let w = me.clone();
        let correction_timer = reactor.register_timer(
            Box::new(move |et| match w.upgrade() {
                Some(b) => correction_event(&b, et).unwrap_or_else(|e| {
                    kaliru::logging::exception_err("AFC FPS correction", &e);
                    NEVER
                }),
                None => NEVER,
            }),
            NEVER,
        );
        let endstop = |trailing: bool| {
            Rc::new(FpsEndstop {
                buffer: me.clone(),
                trailing,
                reactor: reactor.clone(),
                steppers: RefCell::new(vec![]),
                trigger_time: Cell::new(0.0),
                completion: RefCell::new(None),
                poll_timer: RefCell::new(None),
                me: RefCell::new(Weak::new()),
            })
        };
        let fps_endstop = endstop(false);
        *fps_endstop.me.borrow_mut() = Rc::downgrade(&fps_endstop);
        let fps_trailing_endstop = endstop(true);
        *fps_trailing_endstop.me.borrow_mut() = Rc::downgrade(&fps_trailing_endstop);
        Fps {
            adc: cfg.adc,
            reversed: cfg.reversed,
            fps_value: Cell::new(0.0),
            smoothed_fps: Cell::new(0.5),
            set_point: Cell::new(cfg.set_point),
            low_point: cfg.low_point,
            high_point: cfg.high_point,
            homing_high_point: cfg.homing_high_point,
            trailing_min_multiplier: cfg.trailing_min_multiplier,
            multiplier_hysteresis: cfg.multiplier_hysteresis,
            deadband: Cell::new(cfg.deadband),
            smoothing: cfg.smoothing,
            update_interval: cfg.update_interval,
            enable_integral_correction: cfg.enable_integral_correction,
            integral_gain: cfg.integral_gain,
            integral_terms: RefCell::new(IndexMap::default()),
            last_logged_integral: Cell::new(0.0),
            integral_extrusion_threshold: cfg.integral_extrusion_threshold,
            integral_last_extruder_pos: Cell::new(None),
            advance_latched: Cell::new(false),
            latch_enabled: Cell::new(false),
            fila_adv: RefCell::new(None),
            fila_trail: RefCell::new(None),
            correction_timer,
            correction_running: Cell::new(false),
            saved_multipliers: RefCell::new(IndexMap::default()),
            fps_endstop,
            fps_trailing_endstop,
        }
    }

    /// the multipliers the FPS settings give the buffer
    pub fn multipliers(cfg: &FpsConfig) -> (f64, f64) {
        (cfg.multiplier_high, cfg.multiplier_low)
    }

    /// True when FPS reading indicates the buffer is compressed (at high_point)
    pub fn buffer_triggered(&self) -> bool {
        self.smoothed_fps.get() >= self.homing_high_point
    }

    /// True when FPS reading indicates the buffer is stretched (at low_point)
    pub fn buffer_trailing_triggered(&self) -> bool {
        self.smoothed_fps.get() <= self.low_point
    }

    pub fn enable_advance_latch(&self) {
        self.latch_enabled.set(true);
        self.advance_latched.set(false);
    }

    pub fn clear_advance_latch(&self) {
        self.latch_enabled.set(false);
        self.advance_latched.set(false);
    }
}

/// the rest of the FPS buffer's making, once the buffer exists: the ADC
/// callback, the gui's virtual sensors, AFC_SET_FPS_SET_POINT
pub fn finish_init(buffer: &Rc<AfcBuffer>, printer: &Rc<Printer>, report_time: f64, enable_sensors_in_gui: bool) -> KResult<()> {
    let Some(fps) = buffer.fps.as_ref() else { return Ok(()) };
    let w = Rc::downgrade(buffer);
    fps.adc.setup_adc_callback(report_time, Rc::new(move |read_time, read_value| w.upgrade().map_or(Ok(()), |b| adc_callback(&b, read_time, read_value))));
    let none = || -> Rc<dyn Fn(bool) -> KResult<()>> { Rc::new(|_| Ok(())) };
    *fps.fila_adv.borrow_mut() = Some(VirtualSensor::new_opt(printer, &format!("{}_expanded", buffer.name), enable_sensors_in_gui, None)?);
    *fps.fila_trail.borrow_mut() = Some(VirtualSensor::new_opt(printer, &format!("{}_compressed", buffer.name), enable_sensors_in_gui, None)?);
    let _ = none;
    let b = buffer.clone();
    buffer.afc.gcode.register_mux_command(
        "AFC_SET_FPS_SET_POINT",
        "BUFFER",
        Some(&buffer.name),
        Rc::new(move |g| cmd_afc_set_fps_set_point(&b, g)),
        Some(CMD_AFC_SET_FPS_SET_POINT_HELP),
    )
}

/// the report_time setting (read again for the ADC callback)
pub fn report_time(config: &ConfigWrapper) -> KResult<f64> {
    Ok(config.getfloat("report_time", 0.100, Lim::new())?)
}

/// _lane_has_rotation_control: the lane has a stepper (its own or a drive)
pub fn lane_has_rotation_control(lane: Option<&Rc<AfcLane>>) -> bool {
    lane.is_some_and(|l| l.stepper.borrow().is_some() || l.drive_stepper.borrow().is_some())
}

/// _adc_callback: a new reading, smoothed; the buffer state while no
/// correction runs
fn adc_callback(b: &Rc<AfcBuffer>, read_time: f64, read_value: f64) -> KResult<()> {
    let Some(fps) = b.fps.as_ref() else { return Ok(()) };
    let read_value = if fps.reversed { 1.0 - read_value } else { read_value };
    fps.fps_value.set(read_value);
    // Apply exponential moving average
    fps.smoothed_fps.set(fps.smoothing * fps.smoothed_fps.get() + (1.0 - fps.smoothing) * read_value);
    let lane = b.current_lane();
    let has_stepper = lane_has_rotation_control(lane.as_ref());
    // If buffer was enabled before lane stepper wiring was ready,
    // lazily start correction once the lane exposes rotation control.
    if b.enabled() && has_stepper && !fps.correction_running.get() {
        b.reactor.update_timer(&fps.correction_timer, NOW);
        fps.correction_running.set(true);
    }
    let correction_active = b.enabled() && has_stepper && fps.correction_running.get();
    if !correction_active {
        let half_db = fps.deadband.get() / 2.0;
        let s = fps.smoothed_fps.get();
        let sp = fps.set_point.get();
        if s > sp + half_db {
            b.set_last_state(TRAILING_STATE_NAME);
            b.advance_state.set(true);
            if fps.latch_enabled.get() {
                fps.advance_latched.set(true);
            }
            b.trailing_state.set(false);
        } else if fps.advance_latched.get() {
            // Latched during load: keep advance_state True even if
            // pressure drops briefly between motor pulses.
            b.advance_state.set(true);
        } else if s < sp - half_db {
            b.set_last_state(ADVANCING_STATE_NAME);
            b.advance_state.set(false);
            b.trailing_state.set(true);
        } else {
            b.set_last_state(NEUTRAL_STATE_NAME);
            b.advance_state.set(false);
            b.trailing_state.set(false);
        }
    }
    let _ = read_time;
    update_virtual_sensors(b);
    Ok(())
}

/// _update_virtual_sensors: the buffer state on the gui's sensors
fn update_virtual_sensors(b: &AfcBuffer) {
    let Some(fps) = b.fps.as_ref() else { return };
    if let Some(s) = fps.fila_adv.borrow().as_ref() {
        s.note_filament_present(b.advance_state.get());
    }
    if let Some(s) = fps.fila_trail.borrow().as_ref() {
        s.note_filament_present(b.trailing_state.get());
    }
}

/// _correction_event: the proportional (and optional integral) correction
fn correction_event(b: &Rc<AfcBuffer>, eventtime: f64) -> KResult<f64> {
    let Some(fps) = b.fps.as_ref() else { return Ok(NEVER) };
    let lane = b.current_lane();
    let Some(lane) = lane.filter(|_| b.enabled()) else {
        fps.correction_running.set(false);
        return Ok(NEVER);
    };
    // Non-stepper lanes (ACE/OpenAMS) don't need rotation correction
    if !lane_has_rotation_control(Some(&lane)) {
        fps.correction_running.set(false);
        return Ok(NEVER);
    }
    let reading = fps.smoothed_fps.get();
    let set_point = fps.set_point.get();
    let deviation = reading - set_point;
    let half_db = fps.deadband.get() / 2.0;
    let (multiplier_high, multiplier_low) = b.multiplier_values();
    // Proportional multiplier across the full range
    let mut multiplier = if deviation > 0.0 {
        let range_size = kaliru::pyfmt::py_max(fps.high_point - set_point, 0.01);
        let fraction = kaliru::pyfmt::py_min(deviation / range_size, 1.0);
        1.0 - fraction * (1.0 - multiplier_low)
    } else if deviation < 0.0 {
        let range_size = kaliru::pyfmt::py_max(set_point - fps.low_point, 0.01);
        let fraction = kaliru::pyfmt::py_min(-deviation / range_size, 1.0);
        let m = 1.0 + fraction * (multiplier_high - 1.0);
        let trailing_floor = kaliru::pyfmt::py_min(fps.trailing_min_multiplier, multiplier_high);
        kaliru::pyfmt::py_max(m, trailing_floor)
    } else {
        1.0
    };
    // Determine state for LED indication and fault reporting
    let target_direction = if reading < set_point - half_db {
        ADVANCING_STATE_NAME
    } else if reading > set_point + half_db {
        TRAILING_STATE_NAME
    } else {
        NEUTRAL_STATE_NAME
    };
    let mut integral = fps.integral_terms.borrow().get(&lane.name).copied().unwrap_or(0.0);
    if fps.enable_integral_correction {
        if is_extruding(b, fps) {
            integral -= fps.integral_gain * deviation * fps.update_interval;
            // Anti-windup: clamp so a stuck sensor or real clog can't wind it up forever.
            integral = kaliru::pyfmt::py_max(multiplier_low - 1.0, kaliru::pyfmt::py_min(multiplier_high - 1.0, integral));
            fps.integral_terms.borrow_mut().insert(lane.name.clone(), integral);
        }
        multiplier += integral;
        // Clamp the combined P+I output to the configured band.
        multiplier = kaliru::pyfmt::py_max(multiplier_low, kaliru::pyfmt::py_min(multiplier_high, multiplier));
    }
    // Apply multiplier, gated by hysteresis when set
    let mut log_event = false;
    let last_applied = b.last_multiplier_f64();
    if fps.multiplier_hysteresis <= 0.0 {
        set_multiplier(b, multiplier)?;
        if (last_applied - multiplier).abs() > 0.001 {
            log_event = true;
        }
        if integral != fps.last_logged_integral.get() {
            log_event = true;
        }
    } else if (multiplier - last_applied).abs() > fps.multiplier_hysteresis {
        set_multiplier(b, multiplier)?;
        log_event = true;
    }
    // Update state flags
    let last_state = b.buffer_status();
    let (led_color, adv, trail) = match target_direction {
        ADVANCING_STATE_NAME => (b.led_advancing(), false, true),
        TRAILING_STATE_NAME => (b.led_trailing(), true, false),
        _ => (b.led_neutral(), false, false),
    };
    if last_state != target_direction {
        log_event = true;
        b.led(&led_color)?;
    }
    b.set_last_state(target_direction);
    b.advance_state.set(adv);
    b.trailing_state.set(trail);
    if log_event && b.debug {
        b.afc.logger.debug(
            &AFC_BUFFER_AFCFPSBUFFER__CORRECTION_EVENT,
            &format!(
                "FPS_buffer {}: fps={:.3} smoothed={:.3} integral={:+.4} multiplier={:.4} state={}",
                b.name,
                fps.fps_value.get(),
                fps.smoothed_fps.get(),
                integral,
                b.last_multiplier_f64(),
                b.buffer_status()
            ),
        );
        fps.last_logged_integral.set(integral);
    }
    // Update error position while correction is active; stop only when
    // stuck at an extreme despite correction (real clog/feed failure).
    if b.fault_detection_enabled() && reading < fps.high_point && reading > fps.low_point {
        b.update_filament_error_pos()?;
    }
    update_virtual_sensors(b);
    Ok(eventtime + fps.update_interval)
}

/// _is_extruding: the extruder moved by the threshold since the last tick
fn is_extruding(b: &AfcBuffer, fps: &Fps) -> bool {
    let Ok(current_pos) = b.afc.function.get_extruder_pos(None, None, None) else { return false };
    let previous = fps.integral_last_extruder_pos.replace(Some(current_pos));
    match previous {
        None => false,
        Some(p) => (current_pos - p).abs() >= fps.integral_extrusion_threshold,
    }
}

/// enable_buffer: the buffer on for the lane; with a stepper the correction
/// runs (with the lane's saved multiplier back)
pub fn enable_buffer(b: &Rc<AfcBuffer>, lane: &Rc<AfcLane>) -> KResult<()> {
    let fps = b.fps.as_ref().ok_or_else(|| super::no_attr("AFCBuffer", "fps_value"))?;
    b.set_current_lane(Some(lane.clone()));
    b.set_enabled(true);
    fps.latch_enabled.set(false);
    fps.advance_latched.set(false);
    let has_stepper = lane_has_rotation_control(Some(lane));
    b.led(&b.led_neutral())?;
    // Reset smoothed value to current reading
    fps.smoothed_fps.set(fps.fps_value.get());
    if has_stepper {
        // Restore the last multiplier if the same lane is coming back;
        // a different lane on this extruder means a new spool, start at 1.0.
        let extruder_name = lane.extruder_obj_opt().map(|e| e.th_extruder_name.clone()).unwrap_or_else(|| "None".into());
        let saved = fps.saved_multipliers.borrow().get(&(extruder_name.clone(), lane.name.clone())).cloned();
        match saved.filter(|(n, m)| *n == lane.name && *m != 1.0) {
            Some((_, m)) => {
                set_multiplier(b, m)?;
                b.afc.logger.debug(&AFC_BUFFER_AFCFPSBUFFER_ENABLE_BUFFER, &format!("{} restored multiplier {:.4} for {} on {}", b.name, m, lane.name, extruder_name));
            }
            None => b.set_last_multiplier(PyVal::Float(1.0)),
        }
        // Start the proportional correction timer
        b.reactor.update_timer(&fps.correction_timer, NOW);
        fps.correction_running.set(true);
        if b.fault_detection_enabled() {
            b.start_fault_detection(PyVal::Float(1.0))?;
        }
    } else {
        fps.correction_running.set(false);
    }
    b.afc.logger.debug(
        &AFC_BUFFER_AFCFPSBUFFER_ENABLE_BUFFER,
        &format!("{} FPS buffer enabled for {} (correction={})", b.name, lane.name, if has_stepper { "active" } else { "off/adc-only" }),
    );
    Ok(())
}

/// disable_buffer: the buffer off, the lane's multiplier saved, timers off
pub fn disable_buffer(b: &AfcBuffer) -> KResult<()> {
    let fps = b.fps.as_ref().ok_or_else(|| super::no_attr("AFCBuffer", "fps_value"))?;
    b.set_enabled(false);
    fps.latch_enabled.set(false);
    fps.advance_latched.set(false);
    let Some(lane) = b.current_lane() else { return Ok(()) };
    b.afc.logger.debug(&AFC_BUFFER_AFCFPSBUFFER_DISABLE_BUFFER, &format!("{} {} buffer disabled for {}", b.name, b.kind, lane.name));
    b.led(&b.led_buffer_disabled())?;
    // Save the last multiplier for this extruder/lane so it can be
    // restored on the next tool change back to this same lane.
    if lane_has_rotation_control(Some(&lane)) {
        if let Some(extruder_name) = lane.extruder_obj_opt().map(|e| e.th_extruder_name.clone()) {
            let m = b.last_multiplier_f64();
            fps.saved_multipliers.borrow_mut().insert((extruder_name.clone(), lane.name.clone()), (lane.name.clone(), m));
            b.afc.logger.debug(&AFC_BUFFER_AFCFPSBUFFER_DISABLE_BUFFER, &format!("{} saved multiplier {:.4} for {} on {}", b.name, m, lane.name, extruder_name));
        }
    }
    reset_multiplier(b)?;
    // Stop correction timer
    b.reactor.update_timer(&fps.correction_timer, NEVER);
    fps.correction_running.set(false);
    // Stop fault detection
    if b.error_sensitivity_f64() > 0.0 && b.has_fault_timer() {
        b.stop_fault_timer();
    }
    b.set_current_lane(None);
    Ok(())
}

/// set_multiplier: the lane's rotation distance by the multiplier
pub fn set_multiplier(b: &AfcBuffer, multiplier: f64) -> KResult<()> {
    if !b.enabled() {
        return Ok(());
    }
    let Some(lane) = b.current_lane() else { return Ok(()) };
    if !lane_has_rotation_control(Some(&lane)) {
        return Ok(());
    }
    b.set_last_multiplier(PyVal::Float(multiplier));
    lane.update_rotation_distance(multiplier)
}

/// reset_multiplier: the lane's rotation distance back to its base
pub fn reset_multiplier(b: &AfcBuffer) -> KResult<()> {
    let Some(lane) = b.current_lane() else { return Ok(()) };
    if !lane_has_rotation_control(Some(&lane)) {
        return Ok(());
    }
    b.set_last_multiplier(PyVal::Float(1.0));
    lane.update_rotation_distance(1.0)?;
    b.afc.logger.debug(&AFC_BUFFER_AFCFPSBUFFER_RESET_MULTIPLIER, &format!("FPS buffer multiplier reset for {}", lane.name));
    Ok(())
}

/// extruder_pos_update_event: a feed fault, unless the reading can still correct
pub fn extruder_pos_update_event(b: &AfcBuffer, eventtime: f64) -> KResult<f64> {
    let fps = b.fps.as_ref().ok_or_else(|| super::no_attr("AFCBuffer", "fps_value"))?;
    const CHECK_RUNOUT_TIMEOUT: f64 = 0.5;
    if let Some(cur_lane) = b.current_lane() {
        // This may be needed to be updated for IDEX machines
        let active_extruder = b.afc.toolhead()?.get_extruder();
        if let Some(name) = cur_lane.extruder_obj_opt().map(|e| e.th_extruder_name.clone()) {
            if active_extruder.get_name() != name {
                return Ok(eventtime + CHECK_RUNOUT_TIMEOUT);
            }
        }
    }
    let extruder_pos = b.get_extruder_pos_pub()?;
    if b.afc.function.is_printing(true)? {
        if let Some(error_pos) = b.filament_error_pos() {
            // Keep updating while FPS is in the correctable range; only let
            // it expire when stuck at an extreme.
            let s = fps.smoothed_fps.get();
            if fps.low_point <= s && s <= fps.high_point {
                b.update_filament_error_pos()?;
                return Ok(eventtime + CHECK_RUNOUT_TIMEOUT);
            }
            if extruder_pos > error_pos {
                b.pause_on_error_pub("AFC FPS buffer filament fault detected! Take necessary action.", true)?;
                b.update_filament_error_pos()?;
            }
        }
    }
    Ok(eventtime + CHECK_RUNOUT_TIMEOUT)
}

/// QUERY_BUFFER for an FPS buffer
pub fn cmd_query_buffer(b: &AfcBuffer, _gcmd: &GCodeCommand) -> KResult<()> {
    let fps = b.fps.as_ref().ok_or_else(|| super::no_attr("AFCBuffer", "fps_value"))?;
    let buffer_status = b.buffer_status();
    let suffix = match buffer_status.as_str() {
        ADVANCING_STATE_NAME => " (buffer is compressed - increasing feed)",
        TRAILING_STATE_NAME => " (buffer is expanded - reducing feed)",
        NEUTRAL_STATE_NAME => " (buffer is centered)",
        _ => "",
    };
    let mut state_info = format!("{}{}", buffer_status, suffix);
    state_info += &format!("\n{} raw: {:.3}  smoothed: {:.3}  set_point: {:.2}", b.kind, fps.fps_value.get(), fps.smoothed_fps.get(), fps.set_point.get());
    if b.enabled() {
        if let Some(lane) = b.current_lane() {
            if let Some(rd) = AfcBuffer::rotation_distance(&lane) {
                state_info += &format!("\n{} Rotation distance: {:.4}", lane.name, rd);
            }
        }
    }
    if b.error_sensitivity_f64() > 0.0 {
        state_info += &format!("\nFault detection enabled, sensitivity {}", b.error_sensitivity_str());
    }
    b.afc.logger.info(&AFC_BUFFER_AFCFPSBUFFER_CMD_QUERY_BUFFER, &format!("{} : {}", b.name, state_info));
    Ok(())
}

fn cmd_afc_set_fps_set_point(b: &AfcBuffer, gcmd: &GCodeCommand) -> KResult<()> {
    let fps = b.fps.as_ref().ok_or_else(|| super::no_attr("AFCBuffer", "fps_value"))?;
    let new_set_point = gcmd.get_float("SET_POINT", Some(fps.set_point.get()), Lim::new().min(0.1).max(0.9))?;
    let new_deadband = gcmd.get_float("DEADBAND", Some(fps.deadband.get()), Lim::new().min(0.0).max(0.6))?;
    let half_db = new_deadband / 2.0;
    let f = kaliru::pyfmt::py_float_repr;
    if new_set_point <= fps.low_point || new_set_point >= fps.high_point {
        return Err(gcmd.error(format!("SET_POINT must be between low_point({}) and high_point({})", f(fps.low_point), f(fps.high_point))));
    }
    let error_msg = check_deadband(&b.name, new_set_point, new_deadband, fps.low_point, fps.high_point);
    if !error_msg.is_empty() {
        return Err(gcmd.error(error_msg));
    }
    fps.set_point.set(new_set_point);
    fps.deadband.set(new_deadband);
    b.afc.logger.info(
        &AFC_BUFFER_AFCFPSBUFFER_CMD_AFC_SET_FPS_SET_POINT,
        &format!("FPS set_point={:.2} deadband={:.2} (neutral window {:.2}-{:.2})", new_set_point, new_deadband, new_set_point - half_db, new_set_point + half_db),
    );
    Ok(())
}

/// get_status: the FPS readings on top of the buffer's
pub fn add_status(fps: &Fps, r: &mut kaliru::pyval::PyDict) {
    r.insert("fps_value".into(), PyVal::Float(kaliru::pyfmt::py_round(fps.fps_value.get(), 3)));
    r.insert("smoothed_fps".into(), PyVal::Float(kaliru::pyfmt::py_round(fps.smoothed_fps.get(), 3)));
    r.insert("set_point".into(), PyVal::Float(fps.set_point.get()));
}

/// FPSEndstopWrapper: a software endstop at an FPS threshold (advance:
/// compressed at homing_high_point, trailing: stretched at low_point)
pub struct FpsEndstop {
    buffer: Weak<AfcBuffer>,
    trailing: bool,
    reactor: Rc<Reactor>,
    steppers: RefCell<Vec<StepperRef>>,
    trigger_time: Cell<f64>,
    completion: RefCell<Option<Rc<ReactorCompletion<bool>>>>,
    poll_timer: RefCell<Option<Timer>>,
    me: RefCell<Weak<FpsEndstop>>,
}

impl FpsEndstop {
    fn triggered(&self) -> bool {
        self.buffer.upgrade().and_then(|b| b.fps.as_ref().map(|f| if self.trailing { f.buffer_trailing_triggered() } else { f.buffer_triggered() })).unwrap_or(false)
    }

    fn mcu(&self) -> Option<Rc<Mcu>> {
        self.buffer.upgrade().and_then(|b| b.fps.as_ref().map(|f| f.adc.get_mcu()))
    }

    fn poll_fps(&self, eventtime: f64) -> f64 {
        if self.triggered() {
            if let Some(mcu) = self.mcu() {
                self.trigger_time.set(mcu.estimated_print_time(eventtime));
            }
            if let Some(c) = self.completion.borrow().as_ref() {
                c.complete(true);
            }
            return NEVER;
        }
        eventtime + FPS_ENDSTOP_POLL_TIME
    }
}

impl Endstop for FpsEndstop {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu().expect("FPS buffer")
    }

    fn add_stepper(&self, stepper: &StepperRef) -> KResult<()> {
        if !self.steppers.borrow().iter().any(|s| Rc::ptr_eq(s, stepper)) {
            self.steppers.borrow_mut().push(stepper.clone());
        }
        Ok(())
    }

    fn get_steppers(&self) -> Vec<StepperRef> {
        self.steppers.borrow().clone()
    }

    fn home_start(&self, _print_time: f64, _sample_time: f64, _sample_count: i64, _rest_time: f64, triggered: bool) -> KResult<Rc<ReactorCompletion<bool>>> {
        self.trigger_time.set(0.0);
        let completion = self.reactor.completion::<bool>();
        *self.completion.borrow_mut() = Some(completion.clone());
        // Check if already triggered before starting poll timer
        if self.triggered() == triggered {
            self.trigger_time.set(self.get_mcu().estimated_print_time(self.reactor.monotonic()));
            completion.complete(true);
        } else {
            let w = self.me.borrow().clone();
            let t = self.reactor.register_timer(Box::new(move |et| w.upgrade().map_or(NEVER, |e| e.poll_fps(et))), NOW);
            *self.poll_timer.borrow_mut() = Some(t);
        }
        Ok(completion)
    }

    fn home_wait(&self, _home_end_time: f64) -> KResult<f64> {
        if let Some(t) = self.poll_timer.borrow_mut().take() {
            self.reactor.unregister_timer(&t);
        }
        Ok(self.trigger_time.get())
    }

    fn query_endstop(&self, _print_time: f64) -> KResult<i64> {
        Ok(if self.triggered() { 1 } else { 0 })
    }
}
