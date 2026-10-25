// The Elegoo Canvas: AFC unit and lanes (port of AFC_canvas.py and
// AFC_canvas_lane.py). The lanes have no stepper: each is a drv8833 DC motor
// with a hall odometer, two LED pins, and the shared toolhead sensor.
use super::codes::*;
use super::lane::{AfcLane, Endstop, MoveDirection, MoveWarning, SpeedMode, HOMING_BUFFER, HOMING_BUFFER_TRAIL, HOMING_HUB, HOMING_LOAD, HOMING_PREP, HOMING_TOOL, HOMING_TOOL_START};
use super::respond::AfcPrompt;
use super::unit::{AfcUnit, UnitKind, UnitSettings};
use super::{no_attr, Afc};
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::extras::buttons::PrinterButtons;
use kaliru::gcode::GCodeCommand;
use kaliru::mcu::{McuDigitalOut, McuPwm};
use kaliru::printer::PObj;
use kaliru::pyval::{PyDict, PyVal};
use std::cell::Cell;
use std::rc::Rc;

pub const CALIBRATION_UNSUPPORTED: &str = "Canvas does not support calibration.";
const DEFAULT_ODOMETER_POLL_INTERVAL: f64 = 0.05;
const DEFAULT_ODOMETER_LOAD_THRESHOLD: i64 = 3;
const DEFAULT_EXTRUDER_FEED_CHUNK: f64 = 1.0;
const DEFAULT_LOAD_TO_TOOLHEAD_TIMEOUT: f64 = 30.0;
const DEFAULT_EXTRUDER_FEED_TIMEOUT: f64 = 10.0;
const DEFAULT_LOAD_ATTEMPTS: i64 = 3;
const DEFAULT_LOAD_RECOVERY_RETRACT_DISTANCE: f64 = 30.0;
const DEFAULT_CUTTER_WAIT_TIME: f64 = 3.0;
const LED_PWM_CYCLE_TIME: f64 = 0.01;

pub const CMD_AFC_CANVAS_TOOL_LOAD_HELP: &str = "CANVAS-specific tool load for a lane";
pub const CMD_AFC_CANVAS_TOOL_UNLOAD_HELP: &str = "CANVAS-specific tool unload for a lane";

/// The Canvas unit's part: its feed / eject settings, power pins and cutter
pub struct CanvasUnit {
    prep_distance: f64,
    prep_speed: f64,
    eject_distance: f64,
    eject_speed: f64,
    _enable_9v: Option<Rc<McuDigitalOut>>,
    _enable_24v: Option<Rc<McuDigitalOut>>,
    pub cutter_sensor_state: Cell<bool>,
}

/// A Canvas lane's motor: COSMOS's drv8833 printer object (a klippy module
/// of COSMOS, not of Kalico: the distribution provides it), driven through
/// its methods as the python AFC drives it
pub struct Drv8833(PObj);

impl Drv8833 {
    fn call(&self, name: &str, args: &[PyVal]) -> KResult<PyVal> {
        self.0.call_method(name, args).unwrap_or_else(|| Err(no_attr("PrinterDrv8833", name)))
    }

    pub fn drv8833_move(&self, speed: f64, distance: f64, wait_for_completion: bool) -> KResult<()> {
        self.call("drv8833_move", &[PyVal::Float(speed), PyVal::Float(distance), PyVal::Bool(wait_for_completion)]).map(|_| ())
    }

    pub fn drv8833_set_speed(&self, speed: f64) -> KResult<()> {
        self.call("drv8833_set_speed", &[PyVal::Float(speed)]).map(|_| ())
    }

    /// .active: a move is running
    pub fn is_active(&self) -> KResult<bool> {
        self.call("active", &[]).map(|v| v.truthy())
    }
}

/// An LED channel: its pin and the time it was last set
struct LedPin {
    pin: Rc<McuPwm>,
    last_set_time: Cell<f64>,
}

/// The Canvas lane's part (AFCCanvasLane)
pub struct CanvasLane {
    pub drv8833_object_name: String,
    canvas_motor: Rc<Drv8833>,
    red_led_pin: Option<LedPin>,
    white_led_pin: Option<LedPin>,
    odometer_mm_per_pulse: f64,
    odometer_poll_interval: f64,
    odometer_load_threshold: i64,
    load_to_toolhead_timeout: f64,
    extruder_feed_timeout: f64,
    load_attempts: i64,
    load_recovery_retract_distance: f64,
    pub odometer_count: Cell<i64>,
    last_odometer_eventtime: Cell<Option<f64>>,
    last_odometer_state: Cell<bool>,
    disengage_distance: f64,
}

/// The digital_out pin for a power enable, started on
fn setup_output_pin(config: &ConfigWrapper, pin_name: Option<String>) -> KResult<Option<Rc<McuDigitalOut>>> {
    let Some(pin_name) = pin_name else { return Ok(None) };
    let printer = config.get_printer();
    let pin = kaliru::pins::setup_pin_as::<McuDigitalOut>(&printer, "digital_out", &pin_name)?;
    pin.setup_start_value(true, true);
    Ok(Some(pin))
}

/// [AFC_canvas <name>]
pub fn load_config_prefix_unit(config: &ConfigWrapper) -> KResult<PObj> {
    let unit = AfcUnit::new(config, |config, settings: &UnitSettings| {
        let type_name = config.get("type", "canvas")?;
        let prep_distance = config.getfloat("prep_distance", settings.short_move_dis, Lim::new())?;
        let prep_speed = config.getfloat("prep_speed", settings.short_moves_speed, Lim::new())?;
        let eject_distance = config.getfloat("eject_distance", settings.short_move_dis, Lim::new().above(0.0))?;
        let eject_speed = config.getfloat("eject_speed", settings.long_moves_speed, Lim::new().above(0.0))?;
        let cutter_pin = config.get_opt("cutter_pin")?;
        let _cutter_sensor_pin = match config.get_opt("cutter_sensor_pin")? {
            Some(p) => Some(p),
            None => cutter_pin,
        };
        let enable_9v = setup_output_pin(config, config.get_opt("enable_9v_pin")?)?;
        let enable_24v = setup_output_pin(config, config.get_opt("enable_24v_pin")?)?;
        let kind = CanvasUnit {
            prep_distance,
            prep_speed,
            eject_distance,
            eject_speed,
            _enable_9v: enable_9v,
            _enable_24v: enable_24v,
            cutter_sensor_state: Cell::new(false),
        };
        Ok((UnitKind::Canvas(kind), type_name))
    })?;
    // (the cutter sensor's buttons are registered once the unit exists)
    let config_cutter = match config.get_opt("cutter_sensor_pin")? {
        Some(p) => Some(p),
        None => config.get_opt("cutter_pin")?,
    };
    if let Some(pin) = config_cutter {
        let buttons = config.get_printer().lookup_or_load::<PrinterButtons>(config, "buttons")?;
        let u = Rc::downgrade(&unit);
        buttons.register_buttons(
            &[pin.as_str()],
            Rc::new(move |_et, state| {
                if let Some(u) = u.upgrade() {
                    u.canvas().cutter_sensor_state.set(state != 0);
                }
                Ok(())
            }),
        )?;
    }
    Ok(unit)
}

/// [AFC_canvas_lane <name>]
pub fn load_config_prefix_lane(config: &ConfigWrapper) -> KResult<PObj> {
    let lane = AfcLane::new(config, true)?;
    let printer = config.get_printer();
    let Some(drv8833_object_name) = config.get_opt("drv8833")? else {
        return Err(KError::config(format!("drv8833_object must be configured in [{}]", config.get_name())));
    };
    let canvas_motor = match printer.lookup_object_opt(&format!("drv8833 {}", drv8833_object_name)) {
        Some(m) => Rc::new(Drv8833(m)),
        None => return Err(KError::config(format!("Unable to find CANVAS motor object '{}' for lane {}", drv8833_object_name, lane.name))),
    };
    let setup_led_pin = |pin_name: Option<String>| -> KResult<Option<LedPin>> {
        let Some(pin_name) = pin_name else { return Ok(None) };
        let pin = kaliru::pins::setup_pin_as::<McuPwm>(&printer, "pwm", &pin_name)?;
        pin.setup_cycle_time(LED_PWM_CYCLE_TIME, false);
        pin.setup_start_value(0.0, 0.0);
        pin.setup_max_duration(0.0);
        Ok(Some(LedPin { pin, last_set_time: Cell::new(0.0) }))
    };
    let red_led_pin = setup_led_pin(config.get_opt("led_red_pin")?)?;
    let white_led_pin = setup_led_pin(config.get_opt("led_white_pin")?)?;
    let odometer_pin = config.get_opt("odometer_pin")?.ok_or_else(|| KError::config(format!("Option 'odometer_pin' in section '{}' must be specified", config.get_name())))?;
    let odometer_mm_per_pulse = config.getfloat("odometer_resolution", kaliru::configfile::Dflt::Required, Lim::new().min(0.0))?;
    let odometer_poll_interval = config.getfloat("odometer_poll_interval", DEFAULT_ODOMETER_POLL_INTERVAL, Lim::new().min(0.01))?;
    let odometer_load_threshold = config.getint("odometer_load_threshold", DEFAULT_ODOMETER_LOAD_THRESHOLD, Lim::new().min(0))?;
    let load_to_toolhead_timeout = config.getfloat("load_to_toolhead_timeout", DEFAULT_LOAD_TO_TOOLHEAD_TIMEOUT, Lim::new().min(0.1))?;
    let extruder_feed_timeout = config.getfloat("extruder_feed_timeout", DEFAULT_EXTRUDER_FEED_TIMEOUT, Lim::new().min(0.1))?;
    let load_attempts = config.getint("load_attempts", DEFAULT_LOAD_ATTEMPTS, Lim::new().min(1))?;
    let load_recovery_retract_distance = config.getfloat("load_recovery_retract_distance", DEFAULT_LOAD_RECOVERY_RETRACT_DISTANCE, Lim::new().min(0.0))?;
    let buttons = printer.lookup_or_load::<PrinterButtons>(config, "buttons")?;
    let l = Rc::downgrade(&lane);
    buttons.register_buttons(
        &[odometer_pin.as_str()],
        Rc::new(move |et, state| {
            if let Some(l) = l.upgrade() {
                odometer_callback(&l, et, state != 0);
            }
            Ok(())
        }),
    )?;
    let disengage_distance = config.getfloat("disengage_distance", 1.0, Lim::new())?;
    *lane.canvas.borrow_mut() = Some(CanvasLane {
        drv8833_object_name,
        canvas_motor,
        red_led_pin,
        white_led_pin,
        odometer_mm_per_pulse,
        odometer_poll_interval,
        odometer_load_threshold,
        load_to_toolhead_timeout,
        extruder_feed_timeout,
        load_attempts,
        load_recovery_retract_distance,
        odometer_count: Cell::new(0),
        last_odometer_eventtime: Cell::new(None),
        last_odometer_state: Cell::new(false),
        disengage_distance,
    });
    if lane.custom_load_cmd.borrow().is_none() {
        *lane.custom_load_cmd.borrow_mut() = Some(format!("AFC_CANVAS_TOOL_LOAD LANE={}", lane.name));
    }
    if lane.custom_unload_cmd.borrow().is_none() {
        *lane.custom_unload_cmd.borrow_mut() = Some(format!("AFC_CANVAS_TOOL_UNLOAD LANE={}", lane.name));
    }
    let l = lane.clone();
    lane.afc.gcode.register_mux_command("AFC_CANVAS_TOOL_LOAD", "LANE", Some(&lane.name), Rc::new(move |g| cmd_afc_canvas_tool_load(&l, g)), Some(CMD_AFC_CANVAS_TOOL_LOAD_HELP))?;
    let l = lane.clone();
    lane.afc.gcode.register_mux_command("AFC_CANVAS_TOOL_UNLOAD", "LANE", Some(&lane.name), Rc::new(move |g| cmd_afc_canvas_tool_unload(&l, g)), Some(CMD_AFC_CANVAS_TOOL_UNLOAD_HELP))?;
    // _register_frontend_compat_aliases: the lane under AFC_lane / AFC_stepper names
    for alias in [format!("AFC_lane {}", lane.name), format!("AFC_stepper {}", lane.name)] {
        if alias == lane.fullname {
            continue;
        }
        if let Some(existing) = printer.lookup_object_opt(&alias) {
            let same = kaliru::printer::downcast_rc::<AfcLane>(existing).is_some_and(|e| Rc::ptr_eq(&e, &lane));
            if !same {
                continue;
            }
        }
        let obj: PObj = lane.clone();
        if printer.add_object(&alias, obj.clone()).is_err() {
            printer.objects_set_default(&alias, obj);
        }
    }
    lane.load_state_raw.set(false);
    Ok(lane)
}

fn canvas_of(lane: &AfcLane) -> KResult<std::cell::Ref<'_, CanvasLane>> {
    std::cell::Ref::filter_map(lane.canvas.borrow(), |c| c.as_ref()).map_err(|_| no_attr("AFCLane", "canvas_motor"))
}

/// _set_gpio_pin: the LED's duty cycle, at least 0.2 s after the last
fn set_gpio_pin(lane: &AfcLane, pin: Option<&LedPin>, value: f64) -> KResult<()> {
    let Some(p) = pin else { return Ok(()) };
    let mcu = p.pin.get_mcu();
    let mut print_time = mcu.estimated_print_time(lane.afc.reactor.monotonic()) + mcu.min_schedule_time() + 0.1;
    print_time = kaliru::pyfmt::py_max(print_time, p.last_set_time.get() + 0.2);
    p.last_set_time.set(print_time);
    p.pin.set_pwm(print_time, value)
}

/// apply_canvas_led: an RGBW color's red and white as the two LED duty cycles
pub fn apply_canvas_led(lane: &AfcLane, color_string: &str) -> KResult<()> {
    let mut channels: Vec<String> = color_string.split(',').map(|s| kaliru::pyfmt::py_strip(s).to_string()).collect();
    while channels.len() < 4 {
        channels.push("0".into());
    }
    let f = |s: &str| super::utils::py_float_of(&PyVal::Str(s.to_string()));
    let red_value = kaliru::pyfmt::py_max(0.0, kaliru::pyfmt::py_min(f(&channels[0])?, 1.0));
    let white_value = kaliru::pyfmt::py_max(0.0, kaliru::pyfmt::py_min(f(&channels[3])?, 1.0));
    let c = canvas_of(lane)?;
    set_gpio_pin(lane, c.red_led_pin.as_ref(), red_value)?;
    set_gpio_pin(lane, c.white_led_pin.as_ref(), white_value)
}

/// move(): the motor by distance at speed (accel / assist unused)
pub fn lane_move(lane: &AfcLane, distance: f64, speed: f64, _accel: f64, wait_for_completion: bool) -> KResult<()> {
    lane.unit_obj()?.select_lane(&lane.rc(), false)?;
    if distance == 0.0 {
        return Ok(());
    }
    let motor = canvas_of(lane)?.canvas_motor.clone();
    motor.drv8833_move(speed, distance, wait_for_completion)
}

pub fn lane_do_enable(lane: &AfcLane, enable: bool) -> KResult<()> {
    if !enable {
        let motor = canvas_of(lane)?.canvas_motor.clone();
        motor.drv8833_set_speed(0.0)?;
    }
    Ok(())
}

/// _canvas_endstop_state: the sensor an endstop name stands for
fn canvas_endstop_state(lane: &AfcLane, endstop: &Endstop) -> KResult<bool> {
    let toolhead = [HOMING_HUB, HOMING_TOOL, HOMING_TOOL_START].iter().any(|n| endstop.is(n))
        || *endstop == lane.hub_endstop_name()
        || *endstop == lane.tool_endstop_name();
    if toolhead {
        return lane.get_toolhead_pre_sensor_state();
    }
    if endstop.is(HOMING_LOAD) || *endstop == lane.load_es() {
        return Ok(lane.raw_load_state());
    }
    // (no buffers in the native scope: getattr(None, ..., False))
    if endstop.is(HOMING_BUFFER) || endstop.is(HOMING_BUFFER_TRAIL) {
        return Ok(false);
    }
    Ok(false)
}

/// _canvas_move_until: chunks of short_move_dis until the endstop reads
/// the wanted state
fn canvas_move_until(lane: &AfcLane, endstop: &Endstop, distance: f64, speed: f64) -> KResult<(bool, f64, MoveWarning)> {
    let desired_state = distance >= 0.0;
    let mut remaining = distance.abs();
    let mut moved = 0.0;
    let chunk = kaliru::pyfmt::py_max(lane.short_move_dis(), 1.0);
    if canvas_endstop_state(lane, endstop)? == desired_state {
        return Ok((true, 0.0, MoveWarning::None));
    }
    while remaining > 0.0 {
        let step = kaliru::pyfmt::py_min(chunk, remaining);
        lane_move(lane, if distance >= 0.0 { step } else { -step }, speed, lane.short_moves_accel(), true)?;
        moved += step;
        remaining -= step;
        if canvas_endstop_state(lane, endstop)? == desired_state {
            return Ok((true, moved, MoveWarning::None));
        }
    }
    Ok((false, moved, MoveWarning::Warn))
}

pub fn lane_move_to(lane: &AfcLane, distance: f64, speed_mode: SpeedMode, endstop: &Endstop, use_homing: bool) -> KResult<(bool, f64, MoveWarning)> {
    if !use_homing {
        lane.move_advanced(distance, speed_mode, super::lane::AssistActive::No)?;
        return Ok((true, 0.0, MoveWarning::None));
    }
    let (speed, _) = lane.get_speed_accel(speed_mode);
    canvas_move_until(lane, endstop, distance, speed)
}

/// disengage_motors: a short move against the last direction, then off
fn disengage_motors(lane: &AfcLane, direction: f64) -> KResult<()> {
    let d = canvas_of(lane)?.disengage_distance;
    if d <= 0.0 {
        return lane_do_enable(lane, false);
    }
    lane.afc.reactor.pause(lane.afc.reactor.monotonic() + 0.5)?;
    let release_distance = if direction >= 0.0 { -d.abs() } else { d.abs() };
    lane_move(lane, release_distance, 30.0, lane.short_moves_accel(), true)?;
    lane_do_enable(lane, false)
}

fn odometer_callback(lane: &AfcLane, eventtime: f64, state: bool) {
    let Ok(c) = canvas_of(lane) else { return };
    if state && !c.last_odometer_state.get() {
        c.odometer_count.set(c.odometer_count.get() + 1);
        c.last_odometer_eventtime.set(Some(eventtime));
    }
    c.last_odometer_state.set(state);
}

fn reset_odometer(lane: &AfcLane) -> KResult<()> {
    let c = canvas_of(lane)?;
    c.odometer_count.set(0);
    c.last_odometer_eventtime.set(None);
    c.last_odometer_state.set(false);
    Ok(())
}

fn get_odometer_distance(lane: &AfcLane) -> KResult<f64> {
    let c = canvas_of(lane)?;
    Ok(c.odometer_count.get() as f64 * c.odometer_mm_per_pulse)
}

/// The TimeoutError move_with_odometer raises
fn timeout_error(msg: String) -> KError {
    KError::Py("TimeoutError".into(), msg)
}

fn is_timeout(e: &KError) -> bool {
    matches!(e.base(), KError::Py(cls, _) if cls == "TimeoutError")
}

/// move_with_odometer: runs the motor until the odometer counts the
/// distance or stop_condition holds; TimeoutError after 5x the move time
fn move_with_odometer(lane: &AfcLane, distance: f64, speed: f64, stop_condition: Option<&dyn Fn() -> bool>) -> KResult<f64> {
    let target_distance = distance.abs();
    if target_distance == 0.0 {
        return Ok(0.0);
    }
    let signed_speed = if distance >= 0.0 { speed.abs() } else { -speed.abs() };
    let mut moved = 0.0;
    let timeout = (target_distance / kaliru::pyfmt::py_max(speed.abs(), 1.0)) * 5.0 + 1.0;
    let poll_interval = kaliru::pyfmt::py_max(canvas_of(lane)?.odometer_poll_interval, 0.01);
    let reactor = lane.afc.reactor.clone();
    let deadline = reactor.monotonic() + timeout;
    reset_odometer(lane)?;
    lane.unit_obj()?.select_lane(&lane.rc(), false)?;
    let motor = canvas_of(lane)?.canvas_motor.clone();
    let res = (|| -> KResult<()> {
        motor.drv8833_set_speed(signed_speed)?;
        while moved < target_distance {
            if stop_condition.is_some_and(|f| f()) {
                break;
            }
            moved = get_odometer_distance(lane)?;
            if moved >= target_distance {
                break;
            }
            let now = reactor.monotonic();
            if now >= deadline {
                return Err(timeout_error(format!("Timed out waiting for odometer movement on {}", lane.name)));
            }
            reactor.pause(now + poll_interval)?;
            moved = get_odometer_distance(lane)?;
        }
        Ok(())
    })();
    motor.drv8833_set_speed(0.0)?;
    res?;
    Ok(kaliru::pyfmt::py_min(moved, target_distance))
}

fn run_cutter_macro(lane: &AfcLane) -> KResult<()> {
    let afc = &lane.afc;
    if afc.tool_cut.get() {
        let ext = lane.extruder_obj()?;
        ext.estats.increase_cut_total()?;
        afc.gcode.run_script_from_command(&format!("{} EXTRUDER={}", super::py_opt_str(afc.tool_cut_cmd.as_deref()), ext.name))?;
    }
    Ok(())
}

fn run_post_cut_macros(lane: &AfcLane) -> KResult<()> {
    let afc = &lane.afc;
    let ext = lane.extruder_obj()?;
    if afc.tool_cut.get() && afc.park.get() {
        afc.gcode.run_script_from_command(&format!("{} EXTRUDER={}", super::py_opt_str(afc.park_cmd.as_deref()), ext.name))?;
    }
    if afc.form_tip.get() {
        if afc.park.get() {
            afc.gcode.run_script_from_command(&format!("{} EXTRUDER={}", super::py_opt_str(afc.park_cmd.as_deref()), ext.name))?;
        }
        if afc.form_tip_cmd.as_deref() == Some("AFC") {
            return Err(no_attr("NoneType", "tip_form"));
        }
        afc.gcode.run_script_from_command(&super::py_opt_str(afc.form_tip_cmd.as_deref()))?;
    }
    Ok(())
}

/// _move_canvas_with_extruder_feed: the motor moves on its own while the
/// extruder feeds in chunks behind it
fn move_canvas_with_extruder_feed(lane: &AfcLane, distance: f64, canvas_speed: f64, extruder_speed: f64, label: &str) -> KResult<()> {
    if distance == 0.0 {
        return Ok(());
    }
    let feed_direction = if distance >= 0.0 { 1.0 } else { -1.0 };
    let feed_chunk = DEFAULT_EXTRUDER_FEED_CHUNK * feed_direction;
    let (motor, feed_timeout) = {
        let c = canvas_of(lane)?;
        (c.canvas_motor.clone(), c.extruder_feed_timeout)
    };
    let deadline = lane.afc.reactor.monotonic() + feed_timeout;
    let res = (|| -> KResult<()> {
        lane_move(lane, distance, canvas_speed, lane.short_moves_accel(), false)?;
        while motor.is_active()? {
            if lane.afc.reactor.monotonic() >= deadline {
                return Err(timeout_error(format!(
                    "CANVAS motor did not complete the assisted extruder move within {} seconds",
                    super::utils::py_format_g(feed_timeout)
                )));
            }
            lane.afc.move_e_pos(feed_chunk, extruder_speed, label, true)?;
        }
        Ok(())
    })();
    motor.drv8833_set_speed(0.0)?;
    res
}

fn cutter_sensor_engaged(lane: &AfcLane) -> KResult<bool> {
    Ok(lane.unit_obj()?.canvas().cutter_sensor_state.get())
}

/// AFC_CANVAS_TOOL_LOAD: to the toolhead sensor, then feed into the extruder
fn cmd_afc_canvas_tool_load(lane: &Rc<AfcLane>, _gcmd: &GCodeCommand) -> KResult<()> {
    let c = &AFC_CANVAS_LANE_AFCCANVASLANE_CMD_AFC_CANVAS_TOOL_LOAD;
    let afc = lane.afc.clone();
    lane.select_lane()?;
    afc.check_extruder_temp(lane, false)?;
    if cutter_sensor_engaged(lane)? {
        afc.error.handle_lane_failure(c, lane, "CANVAS tool load failed: cutter sensor is engaged.", true)?;
        return Ok(());
    }
    let ext = lane.extruder_obj()?;
    if afc.park.get() {
        afc.gcode.run_script_from_command(&format!("{} EXTRUDER={}", super::py_opt_str(afc.park_cmd.as_deref()), ext.name))?;
    }
    let (load_attempts, motor, load_timeout, recovery, threshold) = {
        let cl = canvas_of(lane)?;
        (cl.load_attempts, cl.canvas_motor.clone(), cl.load_to_toolhead_timeout, cl.load_recovery_retract_distance, cl.odometer_load_threshold)
    };
    let long_speed = lane.long_moves_speed();
    for load_attempt in 0..load_attempts {
        afc.logger.info(c, &format!("Attempting CANVAS tool load for {}, try {}/{}", lane.name, load_attempt + 1, load_attempts));
        if lane.get_toolhead_pre_sensor_state()? {
            return Ok(());
        }
        let start = afc.reactor.monotonic();
        motor.drv8833_set_speed(lane.short_moves_speed())?;
        let mut toolhead_sensor_load_fail = false;
        while !lane.get_toolhead_pre_sensor_state()? {
            let now = afc.reactor.monotonic();
            if now - start > load_timeout {
                motor.drv8833_set_speed(0.0)?;
                afc.logger.warning(c, "CANVAS tool load timed out while moving filament to the hub. Attempting recovery.");
                toolhead_sensor_load_fail = true;
                if move_with_odometer(lane, -recovery, long_speed, None).is_err() {
                    afc.logger.warning(c, "CANVAS tool load recovery failed while retracting filament to the hub using the odometer.");
                }
                break;
            }
            afc.reactor.pause(now + 0.005)?;
        }
        if toolhead_sensor_load_fail {
            continue;
        }
        motor.drv8833_set_speed(0.0)?;
        lane.loaded_to_hub.set(true);
        if let Some(hub) = lane.hub_obj_opt().filter(|h| h.afc_bowden_length.get() > 0.0) {
            let dist = hub.afc_bowden_length.get() + load_attempt as f64;
            match move_canvas_with_extruder_feed(lane, dist, lane.short_moves_speed(), ext.tool_load_speed.get(), "CANVAS tool load extra move") {
                Ok(()) => {}
                Err(e) if is_timeout(&e) => {
                    afc.logger.warning(c, "CANVAS tool load timed out while moving filament from the hub to the extruder. Attempting recovery.");
                    disengage_motors(lane, 1.0)?;
                    afc.move_e_pos(-dist, ext.tool_load_speed.get(), "CANVAS tool load recovery retract", true)?;
                    if move_with_odometer(lane, -recovery, long_speed, None).is_err() {
                        afc.logger.warning(c, "CANVAS tool load recovery failed while retracting filament to the hub using the odometer.");
                    }
                    continue;
                }
                Err(e) => return Err(e),
            }
        }
        reset_odometer(lane)?;
        if ext.tool_stn.get() > 0.0 {
            afc.move_e_pos(ext.tool_stn.get(), ext.tool_load_speed.get(), "CANVAS tool load", true)?;
        }
        let count = canvas_of(lane)?.odometer_count.get();
        if count <= threshold {
            afc.logger.warning(
                c,
                &format!("Odometer count after load is {}, which may mean the filament was not grabbed by the extruder. Attempting to unload.", count),
            );
            disengage_motors(lane, 1.0)?;
            if ext.tool_stn_unload.get() > 0.0 {
                afc.move_e_pos(-ext.tool_stn_unload.get(), ext.tool_unload_speed.get(), "CANVAS tool unload", true)?;
            }
            if let Some(hub) = lane.hub_obj_opt().filter(|h| h.afc_unload_bowden_length.get() > 0.0) {
                match move_with_odometer(lane, -hub.afc_unload_bowden_length.get(), long_speed, None) {
                    Ok(_) => {}
                    Err(e) if is_timeout(&e) => {
                        afc.logger.warning(c, "CANVAS tool load recovery failed while retracting filament to the hub using the odometer.");
                    }
                    Err(e) => return Err(e),
                }
            }
            disengage_motors(lane, -1.0)?;
            continue;
        }
        disengage_motors(lane, 1.0)?;
        return Ok(());
    }
    afc.error.handle_lane_failure(c, lane, &format!("CANVAS tool load failed after {} attempts.", load_attempts), true)
}

/// Retries the cutter until its sensor disengages
fn run_cutter_sequence(lane: &AfcLane) -> KResult<bool> {
    let c = &AFC_CANVAS_LANE_AFCCANVASLANE_RUN_CUTTER_SEQUENCE;
    let afc = &lane.afc;
    if !afc.tool_cut.get() {
        return Ok(true);
    }
    let load_attempts = canvas_of(lane)?.load_attempts;
    for attempt in 0..load_attempts {
        afc.logger.info(c, &format!("Attempting cutting filament, try {}/{}", attempt + 1, load_attempts));
        run_cutter_macro(lane)?;
        let start = afc.reactor.monotonic();
        let mut timed_out = false;
        while cutter_sensor_engaged(lane)? {
            let now = afc.reactor.monotonic();
            if now - start >= DEFAULT_CUTTER_WAIT_TIME {
                afc.logger.warning(c, "Cutter sensor did not disengage after cutting filament. Trying again.");
                timed_out = true;
                break;
            }
            afc.reactor.pause(now + 0.005)?;
        }
        if timed_out {
            continue;
        }
        return Ok(true);
    }
    Ok(false)
}

/// AFC_CANVAS_TOOL_UNLOAD: cut, retract from the extruder, retract to the hub
fn cmd_afc_canvas_tool_unload(lane: &Rc<AfcLane>, _gcmd: &GCodeCommand) -> KResult<()> {
    let c = &AFC_CANVAS_LANE_AFCCANVASLANE_CMD_AFC_CANVAS_TOOL_UNLOAD;
    let afc = lane.afc.clone();
    lane.select_lane()?;
    afc.check_extruder_temp(lane, false)?;
    lane.disable_buffer()?;
    lane.unit_obj()?.lane_unloading(lane)?;
    if cutter_sensor_engaged(lane)? {
        afc.error.handle_lane_failure(c, lane, "CANVAS tool unload failed: cutter sensor is engaged before cutting.", true)?;
        return Ok(());
    }
    if !run_cutter_sequence(lane)? {
        afc.error.handle_lane_failure(c, lane, "CANVAS tool unload failed: cutter sensor did not disengage after cutting.", true)?;
        return Ok(());
    }
    run_post_cut_macros(lane)?;
    let load_attempts = canvas_of(lane)?.load_attempts;
    let ext = lane.extruder_obj()?;
    for attempt in 0..load_attempts {
        afc.logger.info(c, &format!("Attempting CANVAS tool unload for {}, try {}/{}", lane.name, attempt + 1, load_attempts));
        let mut failed_operation = "toolhead gear retract";
        let r = (|| -> KResult<()> {
            if ext.tool_stn_unload.get() > 0.0 {
                afc.move_e_pos(-ext.tool_stn_unload.get(), ext.tool_unload_speed.get(), "CANVAS tool unload", true)?;
            }
            failed_operation = "Canvas-unit retract";
            if let Some(hub) = lane.hub_obj_opt().filter(|h| h.afc_unload_bowden_length.get() > 0.0) {
                move_with_odometer(lane, -hub.afc_unload_bowden_length.get(), lane.long_moves_speed(), None)?;
            }
            failed_operation = "check toolhead sensor";
            if lane.get_toolhead_pre_sensor_state()? {
                return Err(timeout_error("CANVAS tool unload failed to clear the shared toolhead sensor after extruder retract".into()));
            }
            lane.loaded_to_hub.set(false);
            disengage_motors(lane, -1.0)
        })();
        match r {
            Ok(()) => return Ok(()),
            Err(e) if is_timeout(&e) => afc.logger.warning(c, &format!("CANVAS tool unload failed during {}: {}.", failed_operation, e.message())),
            Err(e) => return Err(e),
        }
    }
    afc.error.handle_lane_failure(c, lane, &format!("CANVAS tool unload failed after {} attempts.", load_attempts), true)
}

/// get_status additions: the motor and its odometer
pub fn lane_status_extra(lane: &AfcLane, r: &mut PyDict) -> KResult<()> {
    let c = canvas_of(lane)?;
    r.insert("canvas_motor".into(), PyVal::Str(c.drv8833_object_name.clone()));
    r.insert("odometer_count".into(), PyVal::Int(c.odometer_count.get()));
    r.insert("odometer_distance".into(), PyVal::Float(c.odometer_count.get() as f64 * c.odometer_mm_per_pulse));
    Ok(())
}

// The Canvas unit

pub fn unit_handle_connect(unit: &AfcUnit) {
    *unit.logo.borrow_mut() = "<span class=success--text>CANVAS Ready</span>\n".into();
    *unit.logo_error.borrow_mut() = "<span class=error--text>CANVAS Not Ready</span>\n".into();
}

/// UNIT_CALIBRATION and friends: the Canvas cannot calibrate
pub fn unit_calibration_unsupported(unit: &AfcUnit, _gcmd: &GCodeCommand) -> KResult<()> {
    let prompt = AfcPrompt::new(&unit.afc.logger);
    prompt.create_custom_p(&format!("{} Calibration", unit.name), Some(CALIBRATION_UNSUPPORTED), None, true, None, None);
    Ok(())
}

/// The Canvas's lane LED functions: a debug line and the LED color
pub fn unit_lane_led(unit: &AfcUnit, lane: &Rc<AfcLane>, func: &str, color: &str) -> KResult<()> {
    let code = match func {
        "lane_loaded" => &AFC_CANVAS_AFCCANVAS_LANE_LOADED,
        "lane_unloading" => &AFC_CANVAS_AFCCANVAS_LANE_UNLOADING,
        "lane_loading" => &AFC_CANVAS_AFCCANVAS_LANE_LOADING,
        "lane_fault" => &AFC_CANVAS_AFCCANVAS_LANE_FAULT,
        _ => &AFC_CANVAS_AFCCANVAS_LANE_ILLUMINATE_SPOOL,
    };
    unit.afc.logger.debug(code, &format!("{}: {}", func, lane.name));
    apply_canvas_led(lane, color)
}

pub fn unit_lane_not_ready(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<()> {
    unit.afc.logger.debug(&AFC_CANVAS_AFCCANVAS_LANE_NOT_READY, &format!("lane_not_ready: {}", lane.name));
    apply_canvas_led(lane, &unit.afc.led_off)?;
    lane.load_state_raw.set(false);
    Ok(())
}

pub fn unit_lane_unloaded(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<()> {
    unit.afc.logger.debug(&AFC_CANVAS_AFCCANVAS_LANE_UNLOADED, &format!("lane_unloaded: {}", lane.name));
    apply_canvas_led(lane, &unit.afc.led_off)?;
    lane.load_state_raw.set(false);
    Ok(())
}

/// lane_tool_loaded / _unloaded / _loaded_idle: lane LED and the
/// extruder's status LED
pub fn unit_lane_tool_led(unit: &AfcUnit, lane: &Rc<AfcLane>, func: &str, lane_color: &str, extruder_color: &str) -> KResult<()> {
    let code = match func {
        "lane_tool_loaded" => &AFC_CANVAS_AFCCANVAS_LANE_TOOL_LOADED,
        "lane_tool_unloaded" => &AFC_CANVAS_AFCCANVAS_LANE_TOOL_UNLOADED,
        _ => &AFC_CANVAS_AFCCANVAS_LANE_TOOL_LOADED_IDLE,
    };
    unit.afc.logger.debug(code, &format!("{}: {}", func, lane.name));
    apply_canvas_led(lane, lane_color)?;
    lane.extruder_obj()?.set_status_led(extruder_color)
}

pub fn unit_prep_load(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<()> {
    let c = &AFC_CANVAS_AFCCANVAS_PREP_LOAD;
    let cu = unit.canvas();
    unit.afc.logger.debug(c, &format!("prep_load: {}", lane.name));
    apply_canvas_led(lane, &unit.afc.led_loading)?;
    let l = lane.clone();
    match move_with_odometer(lane, cu.prep_distance, cu.prep_speed, Some(&move || !l.prep_state.get())) {
        Ok(_) => Ok(()),
        Err(e) if is_timeout(&e) => {
            unit.afc.logger.warning(c, &format!("prep_load: {} timed out", lane.name));
            Ok(())
        }
        Err(e) => Err(e),
    }
}

pub fn unit_prep_post_load(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<()> {
    lane.load_state_raw.set(lane.prep_state.get());
    unit.afc.logger.debug(&AFC_CANVAS_AFCCANVAS_PREP_POST_LOAD, &format!("prep_post_load: {}", lane.name));
    apply_canvas_led(lane, &unit.afc.led_ready)?;
    disengage_motors(lane, 1.0)
}

pub fn unit_eject_lane(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<()> {
    let cu = unit.canvas();
    let l = lane.clone();
    match move_with_odometer(lane, -cu.eject_distance, cu.eject_speed, Some(&move || !l.prep_state.get())) {
        Ok(_) => {}
        Err(e) if is_timeout(&e) => unit.afc.logger.warning(&AFC_CANVAS_AFCCANVAS_EJECT_LANE, &format!("eject_lane: {} timed out", lane.name)),
        Err(e) => return Err(e),
    }
    disengage_motors(lane, -1.0)
}

/// move_to_hub: plain moves, or chunks until the toolhead sensor changes
pub fn unit_move_to_hub(unit: &AfcUnit, lane: &Rc<AfcLane>, dist: f64, dir: MoveDirection, use_homing: bool, speed_mode: SpeedMode) -> KResult<(bool, f64, MoveWarning)> {
    let move_distance = dist.abs() * dir.value();
    if !use_homing {
        lane.move_advanced(move_distance, speed_mode, super::lane::AssistActive::No)?;
        return Ok((true, 0.0, MoveWarning::None));
    }
    let (speed, _) = lane.get_speed_accel(speed_mode);
    let sensor_active = lane.get_toolhead_pre_sensor_state()?;
    let target_state = move_distance >= 0.0;
    if sensor_active == target_state {
        return Ok((true, 0.0, MoveWarning::None));
    }
    let mut remaining = dist.abs();
    let mut moved = 0.0;
    let chunk = kaliru::pyfmt::py_max(lane.short_move_dis(), 1.0);
    let _ = unit;
    while remaining > 0.0 {
        let step = kaliru::pyfmt::py_min(chunk, remaining);
        lane.move_lane(step * dir.value(), speed, lane.short_moves_accel(), false)?;
        moved += step;
        remaining -= step;
        if lane.get_toolhead_pre_sensor_state()? == target_state {
            return Ok((true, moved, MoveWarning::None));
        }
    }
    Ok((false, moved, MoveWarning::Warn))
}

pub fn unit_load_then_home(_unit: &AfcUnit, lane: &Rc<AfcLane>, distance: f64, endstop: &Endstop) -> KResult<(bool, f64, MoveWarning)> {
    let (speed, _) = lane.get_speed_accel(SpeedMode::Long);
    canvas_move_until(lane, endstop, distance, speed)
}

/// _get_startup_prep_state: the prep endstop queried before PREP is done
fn get_startup_prep_state(unit: &AfcUnit, lane: &AfcLane) -> bool {
    let prep_state = lane.prep_state.get();
    if lane.afc_prep_done.get() {
        return prep_state;
    }
    let endstops = lane.endstops.borrow();
    let Some(es) = endstops.get(HOMING_PREP) else { return prep_state };
    let Some(toolhead) = unit.printer.lookup_object_opt("toolhead").and_then(kaliru::printer::downcast_rc::<kaliru::toolhead::ToolHead>) else {
        return prep_state;
    };
    match toolhead.get_last_move_time().and_then(|t| es.endstop.query_endstop(t)).map(|v| v != 0) {
        Ok(v) => v,
        Err(_) => prep_state,
    }
}

/// system_Test: restores the lane's state from its prep sensor during PREP
pub fn unit_system_test(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, _delay: f64, assign_tcmd: bool, _enable_movement: bool) -> KResult<bool> {
    let afc: &Afc = &unit.afc;
    let mut msg;
    let prep_state = get_startup_prep_state(unit, cur_lane);
    cur_lane.prep_state.set(prep_state);
    cur_lane.load_state_raw.set(prep_state);
    if !prep_state {
        unit.lane_unloaded(cur_lane)?;
        msg = "EMPTY READY FOR SPOOL".to_string();
    } else {
        unit.lane_loaded(cur_lane, false)?;
        msg = "FILAMENT PRESENT".to_string();
        if cur_lane.tool_loaded.get() && cur_lane.extruder_obj_opt().is_some_and(|e| e.lane_loaded.borrow().as_deref() == Some(cur_lane.name.as_str())) {
            cur_lane.sync_to_extruder(true)?;
            if afc.current().as_deref() == Some(cur_lane.name.as_str()) {
                unit.lane_tool_loaded(cur_lane)?;
            } else {
                unit.lane_tool_loaded_idle(cur_lane)?;
            }
            msg += " in ToolHead";
        }
    }
    if assign_tcmd {
        afc.function.tcmd_assign(cur_lane)?;
    }
    cur_lane.send_lane_data()?;
    cur_lane.do_enable(false)?;
    afc.logger.info(&AFC_CANVAS_AFCCANVAS_SYSTEM_TEST, &format!("{} tool cmd: {:<3} {}", cur_lane.name, cur_lane.map_to_string(), msg));
    cur_lane.set_afc_prep_done();
    Ok(true)
}
