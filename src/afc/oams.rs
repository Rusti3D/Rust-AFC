// [AFC_OAMS <name>]: an OpenAMS unit's MCU (port of AFC_OAMS.py). It
// configures the pressure (FPS) buffer, the first-stage (f1s) and hub Hall
// sensors and the PID loops, sends load / unload / follower / calibration
// commands and keeps the sensor and action status the firmware reports.
//
// Copyright (C) 2026 AFCProject
//
// This file include code inspired/modified from OpenAms Project. https://github.com/OpenAMSOrg/klipper_openams
// Originally authored by JR Lomas(aka KnightRadiant) and licensed under the MIT license
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::Afc;
use kaliru::configfile::{ConfigWrapper, Dflt, Lim};
use kaliru::error::{KError, KResult};
use kaliru::gcode::GCodeCommand;
use kaliru::mcu::{CommandQueryWrapper, CommandWrapper, Mcu};
use kaliru::msgproto::{Params, Val};
use kaliru::printer::{PObj, Printer, PrinterObject};
use kaliru::pyval::PyVal;
use kaliru::reactor::Reactor;
use std::cell::{Cell, RefCell};
use std::collections::{BTreeMap, HashMap};
use std::rc::{Rc, Weak};

/// OAMSStatus: the firmware's action codes
pub mod status {
    pub const LOADING: i64 = 0;
    pub const UNLOADING: i64 = 1;
    pub const FORWARD_FOLLOWING: i64 = 2;
    pub const REVERSE_FOLLOWING: i64 = 3;
    pub const COASTING: i64 = 4;
    pub const STOPPED: i64 = 5;
    pub const CALIBRATING: i64 = 6;
    pub const ERROR: i64 = 7;
    pub const NAMES: [&str; 8] = ["LOADING", "UNLOADING", "FORWARD_FOLLOWING", "REVERSE_FOLLOWING", "COASTING", "STOPPED", "CALIBRATING", "ERROR"];
}

/// OAMSOpCode: the result codes of the firmware's actions
pub mod opcode {
    pub const SUCCESS: i64 = 0;
    pub const ERROR_UNSPECIFIED: i64 = 1;
    pub const ERROR_BUSY: i64 = 2;
    pub const SPOOL_ALREADY_IN_BAY: i64 = 3;
    pub const NO_SPOOL_IN_BAY: i64 = 4;
    pub const ERROR_KLIPPER_CALL: i64 = 5;
    pub const CANCEL: i64 = 6;
    pub const NAMES: [&str; 7] = ["SUCCESS", "ERROR_UNSPECIFIED", "ERROR_BUSY", "SPOOL_ALREADY_IN_BAY", "NO_SPOOL_IN_BAY", "ERROR_KLIPPER_CALL", "CANCEL"];
}

/// _oams_enum_name: 'no spool in bay' for NO_SPOOL_IN_BAY, '<kind> <value>' when unknown
fn enum_name(names: &[&str], value: i64, kind: &str) -> String {
    match usize::try_from(value).ok().and_then(|i| names.get(i)) {
        Some(k) => k.replace('_', " ").to_lowercase(),
        None => format!("{} {}", kind, value),
    }
}

const CMD_OAMS_RETRY_STATUS_HELP: &str = "Display retry configuration and state";
const CMD_OAMS_RESET_RETRY_COUNTS_HELP: &str = "Reset retry counters";
const CMD_OAMS_CURRENT_PID_SET_HELP: &str = "Set the PID values for the current sensor";
const CMD_OAMS_PID_SET_HELP: &str = "Set the PID values for the OAMS";
const CMD_OAMS_PID_AUTOTUNE_HELP: &str = "Run PID autotune";
const CMD_OAMS_CALIBRATE_HUB_HES_HELP: &str = "Calibrate the range of a single hub HES";
const CMD_OAMS_CALIBRATE_PTFE_LENGTH_HELP: &str = "Calibrate the length of the PTFE tube";
const CMD_OAMS_LOAD_SPOOL_HELP: &str = "Load a new spool of filament";
const CMD_OAMS_UNLOAD_SPOOL_HELP: &str = "Unload a spool of filament";
const CMD_OAMS_ABORT_ACTION_HELP: &str = "Abort the current OAMS action";
const CMD_OAMS_FOLLOWER_HELP: &str = "Enable or disable follower and set its direction";
const CMD_AFC_STOP_OAMS_FOLLOWER_HELP: &str = "Stop the OAMS follower motor";
const CMD_AFC_START_OAMS_FOLLOWER_HELP: &str = "Start the OAMS follower motor";

/// RetryState: one spool's load-retry bookkeeping
#[derive(Default)]
struct RetryState {
    count: i64,
    last_attempt: Option<f64>,
    was_retry: bool,
}

/// The MCU commands, looked up at klippy:connect
#[derive(Default)]
struct Commands {
    load_spool: Option<CommandWrapper>,
    unload_spool: Option<CommandWrapper>,
    follower: Option<CommandWrapper>,
    calibrate_ptfe_length: Option<CommandWrapper>,
    calibrate_hub_hes: Option<CommandWrapper>,
    pid: Option<CommandWrapper>,
    set_led_error: Option<CommandWrapper>,
    /// Some(None): looked up, but the firmware lacks it
    load_spool_cancel: Option<Option<CommandWrapper>>,
    spool_query_spool: Option<CommandQueryWrapper>,
}

pub struct AfcOams {
    me: Weak<AfcOams>,
    pub printer: Rc<Printer>,
    pub afc: Rc<Afc>,
    pub mcu: Rc<Mcu>,
    reactor: Rc<Reactor>,
    /// the section's last word (the hardware service's name)
    section_name: String,
    /// full section name, e.g. "AFC_OAMS oams1"
    config_name: String,
    pub name: String,
    fps_upper_threshold: f64,
    fps_lower_threshold: f64,
    fps_is_reversed: bool,
    pub current_spool: Cell<Option<i64>>,
    pub encoder_clicks: Cell<i64>,
    i_value: Cell<f64>,
    f1s_hes_on: Vec<f64>,
    f1s_hes_is_above: bool,
    hub_hes_on: RefCell<Vec<f64>>,
    hub_hes_is_above: bool,
    filament_path_length: f64,
    pub oams_idx: i64,
    kd: Cell<f64>,
    ki: Cell<f64>,
    kp: Cell<f64>,
    current_kp: Cell<f64>,
    current_ki: Cell<f64>,
    current_kd: Cell<f64>,
    fps_target: Cell<f64>,
    current_target: Cell<f64>,
    /// an int 0 until the firmware reports a float
    pub fps_value: RefCell<PyVal>,
    pub f1s_hes_value: RefCell<[i64; 4]>,
    pub hub_hes_value: RefCell<[i64; 4]>,
    pub action_status: Cell<Option<i64>>,
    pub action_status_code: Cell<Option<i64>>,
    action_status_value: Cell<Option<i64>>,
    load_retry_max: i64,
    unload_retry_max: i64,
    retry_delay: f64,
    auto_unload_on_failed_load: bool,
    load_stall_grace: f64,
    load_stall_dwell: f64,
    load_retry_state: RefCell<BTreeMap<i64, RetryState>>,
    unload_retry_count: Cell<i64>,
    last_unload_attempt: Cell<f64>,
    last_successful_load: RefCell<HashMap<i64, f64>>,
    load_retry_failures: Cell<i64>,
    unload_retry_failures: Cell<i64>,
    last_load_failure_time: Cell<Option<f64>>,
    last_unload_failure_time: Cell<Option<f64>>,
    cmds: RefCell<Commands>,
}

/// float_to_u32: a float's IEEE-754 single bits
pub fn float_to_u32(f: f64) -> i64 {
    (f as f32).to_bits() as i64
}

/// u32_to_float: the float whose single bits these are
pub fn u32_to_float(i: i64) -> f64 {
    f32::from_bits(i as u32) as f64
}

fn float_list(config: &ConfigWrapper, option: &str) -> KResult<Vec<f64>> {
    config.get(option, Dflt::Required)?.split(',').map(|x| super::utils::py_float_of(&PyVal::Str(x.trim().to_string()))).collect()
}

fn param(params: &Params, key: &str) -> KResult<i64> {
    match params.get(key) {
        Some(Val::Int(i)) => Ok(*i),
        _ => Err(KError::Py("KeyError".into(), kaliru::pyfmt::py_str_repr(key))),
    }
}

fn py_opt_int(v: Option<i64>) -> String {
    v.map_or("None".into(), |v| v.to_string())
}

fn py_opt_float(v: Option<f64>) -> PyVal {
    v.map_or(PyVal::None, PyVal::Float)
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let printer = config.get_printer();
    let section_name = config.get_name().split_whitespace().last().unwrap_or("").to_string();
    let mcu = kaliru::mcu::get_printer_mcu(&printer, &config.get("mcu", "mcu")?)?;
    let reactor = printer.get_reactor();
    let afc = printer.lookup_or_load::<Afc>(config, "AFC")?;
    let fps_upper_threshold = config.getfloat("fps_upper_threshold", Dflt::Required, Lim::new())?;
    let fps_lower_threshold = config.getfloat("fps_lower_threshold", Dflt::Required, Lim::new())?;
    let fps_is_reversed = config.getboolean("fps_is_reversed", Dflt::Required)?;
    let f1s_hes_on = float_list(config, "f1s_hes_on")?;
    let f1s_hes_is_above = config.getboolean("f1s_hes_is_above", Dflt::Required)?;
    let hub_hes_on = float_list(config, "hub_hes_on")?;
    let hub_hes_is_above = config.getboolean("hub_hes_is_above", Dflt::Required)?;
    let filament_path_length = config.getfloat("ptfe_length", Dflt::Required, Lim::new())?;
    let oams_idx = config.getint("oams_idx", Dflt::Required, Lim::new())?;
    let kd = config.getfloat("kd", 0.0, Lim::new())?;
    let ki = config.getfloat("ki", 0.0, Lim::new())?;
    let kp = config.getfloat("kp", 6.0, Lim::new())?;
    let current_kp = config.getfloat("current_kp", 0.375, Lim::new())?;
    let current_ki = config.getfloat("current_ki", 0.0, Lim::new())?;
    let current_kd = config.getfloat("current_kd", 0.0, Lim::new())?;
    let fps_target = config.getfloat("fps_target", 0.5, Lim::new().min(0.0).max(1.0).above(fps_lower_threshold).below(fps_upper_threshold))?;
    let current_target = config.getfloat("current_target", 0.3, Lim::new().min(0.1).max(0.4))?;
    let config_name = config.get_name().to_string();
    let name = config_name.split_whitespace().last().unwrap_or("").to_string();
    // (read after the mcu callbacks and commands are registered in python;
    // the registrations do not depend on them)
    let load_retry_max = config.getint("load_retry_max", 3, Lim::new().min(1).max(5))?;
    let unload_retry_max = config.getint("unload_retry_max", 2, Lim::new().min(1).max(3))?;
    let retry_delay = config.getfloat("retry_delay", 3.0, Lim::new().above(0.0))?;
    let auto_unload_on_failed_load = config.getboolean("auto_unload_on_failed_load", true)?;
    let load_stall_grace = config.getfloat("load_stall_grace", 3.0, Lim::new().min(0.0))?;
    let load_stall_dwell = config.getfloat("load_stall_dwell", 5.0, Lim::new().min(0.0))?;
    let oams = Rc::new_cyclic(|me| AfcOams {
        me: me.clone(),
        printer: printer.clone(),
        afc,
        mcu,
        reactor,
        section_name,
        config_name,
        name,
        fps_upper_threshold,
        fps_lower_threshold,
        fps_is_reversed,
        current_spool: Cell::new(None),
        encoder_clicks: Cell::new(0),
        i_value: Cell::new(0.0),
        f1s_hes_on,
        f1s_hes_is_above,
        hub_hes_on: RefCell::new(hub_hes_on),
        hub_hes_is_above,
        filament_path_length,
        oams_idx,
        kd: Cell::new(kd),
        ki: Cell::new(ki),
        kp: Cell::new(kp),
        current_kp: Cell::new(current_kp),
        current_ki: Cell::new(current_ki),
        current_kd: Cell::new(current_kd),
        fps_target: Cell::new(fps_target),
        current_target: Cell::new(current_target),
        fps_value: RefCell::new(PyVal::Int(0)),
        f1s_hes_value: RefCell::new([0; 4]),
        hub_hes_value: RefCell::new([0; 4]),
        action_status: Cell::new(None),
        action_status_code: Cell::new(None),
        action_status_value: Cell::new(None),
        load_retry_max,
        unload_retry_max,
        retry_delay,
        auto_unload_on_failed_load,
        load_stall_grace,
        load_stall_dwell,
        load_retry_state: RefCell::new(BTreeMap::new()),
        unload_retry_count: Cell::new(0),
        last_unload_attempt: Cell::new(0.0),
        last_successful_load: RefCell::new(HashMap::new()),
        load_retry_failures: Cell::new(0),
        unload_retry_failures: Cell::new(0),
        last_load_failure_time: Cell::new(None),
        last_unload_failure_time: Cell::new(None),
        cmds: RefCell::new(Commands::default()),
    });
    // MCU communication
    let responses: [(&str, fn(&AfcOams, &Params) -> KResult<()>); 3] = [
        ("oams_action_status", AfcOams::oams_action_status),
        ("oams_cmd_stats", AfcOams::oams_cmd_stats),
        ("oams_cmd_current_status", AfcOams::oams_cmd_current_status),
    ];
    for (msg, f) in responses {
        let w = Rc::downgrade(&oams);
        oams.mcu.register_response(Some(Rc::new(move |p: &Params| w.upgrade().map_or(Ok(()), |o| f(&o, p)))), msg, None);
    }
    let w = Rc::downgrade(&oams);
    oams.mcu.register_config_callback(Rc::new(move || w.upgrade().map_or(Ok(()), |o| o.build_config())));
    oams.register_commands()?;
    let o = oams.clone();
    kaliru::events::KlippyConnect::on(&printer, move || o.handle_connect());
    let o = oams.clone();
    kaliru::events::KlippyReady::on(&printer, move || o.handle_ready());
    Ok(oams)
}

impl AfcOams {
    fn missing(&self, attr: &str) -> KError {
        KError::Py("AttributeError".into(), format!("'AFC_OAMS' object has no attribute '{}'", attr))
    }

    fn send(&self, pick: fn(&Commands) -> &Option<CommandWrapper>, attr: &str, data: &[i64]) -> KResult<()> {
        let cmds = self.cmds.borrow();
        let cmd = pick(&cmds).as_ref().ok_or_else(|| self.missing(attr))?;
        let data: Vec<Val> = data.iter().map(|v| Val::Int(*v)).collect();
        cmd.send(&data, 0, 0)
    }

    fn pause(&self, delay: f64) -> KResult<()> {
        self.reactor.pause(self.reactor.monotonic() + delay)?;
        Ok(())
    }

    /// _resolve_lane_name: the AFC lane on a spool bay
    fn resolve_lane_name(&self, spool_idx: i64) -> Option<String> {
        super::openams::resolve_lane_name_from_afc(&self.afc, &self.section_name, spool_idx)
    }

    fn lane_label(&self, spool_idx: i64) -> String {
        match self.resolve_lane_name(spool_idx) {
            Some(n) => format!("lane {}", n),
            None => format!("lane (spool {})", spool_idx),
        }
    }

    pub fn status(&self) -> PyVal {
        let ints = |v: &[i64; 4]| PyVal::List(v.iter().map(|i| PyVal::Int(*i)).collect());
        PyVal::Dict(kaliru::pydict! {
            "current_spool" => self.current_spool.get().map_or(PyVal::None, PyVal::Int),
            "f1s_hes_value" => ints(&self.f1s_hes_value.borrow()),
            "hub_hes_value" => ints(&self.hub_hes_value.borrow()),
            "fps_value" => self.fps_value.borrow().clone(),
            "load_retry_failures" => PyVal::Int(self.load_retry_failures.get()),
            "unload_retry_failures" => PyVal::Int(self.unload_retry_failures.get()),
            "last_load_failure_time" => py_opt_float(self.last_load_failure_time.get()),
            "last_unload_failure_time" => py_opt_float(self.last_unload_failure_time.get()),
        })
    }

    fn handle_connect(&self) -> KResult<()> {
        let c = &AFC_OAMS_AFC_OAMS_HANDLE_CONNECT;
        let r = (|| -> KResult<()> {
            let defs: [(&str, fn(&mut Commands) -> &mut Option<CommandWrapper>); 7] = [
                ("oams_cmd_load_spool spool=%c", |c| &mut c.load_spool),
                ("oams_cmd_unload_spool", |c| &mut c.unload_spool),
                ("oams_cmd_follower enable=%c direction=%c", |c| &mut c.follower),
                ("oams_cmd_calibrate_ptfe_length spool=%c", |c| &mut c.calibrate_ptfe_length),
                ("oams_cmd_calibrate_hub_hes spool=%c", |c| &mut c.calibrate_hub_hes),
                ("oams_cmd_pid kp=%u ki=%u kd=%u target=%u", |c| &mut c.pid),
                ("oams_set_led_error idx=%c value=%c", |c| &mut c.set_led_error),
            ];
            for (fmt, slot) in defs {
                let cmd = self.mcu.lookup_command(fmt, None)?;
                *slot(&mut self.cmds.borrow_mut()) = Some(cmd);
            }
            match self.mcu.lookup_command("oams_cmd_load_spool_cancel", None) {
                Ok(cmd) => self.cmds.borrow_mut().load_spool_cancel = Some(Some(cmd)),
                Err(e) => {
                    self.cmds.borrow_mut().load_spool_cancel = Some(None);
                    self.afc.logger.warning(
                        c,
                        &format!(
                            "Failed to initialize OAMS load filament cancel command: {}\nMost likely the firmware needs to be updated to support this command.",
                            e.message()
                        ),
                    );
                }
            }
            let cmd_queue = self.mcu.alloc_command_queue();
            let q = self.mcu.lookup_query_command("oams_cmd_query_spool", "oams_query_response_spool spool=%u", None, Some(cmd_queue), false)?;
            self.cmds.borrow_mut().spool_query_spool = Some(q);
            self.clear_errors()
        })();
        if let Err(e) = r {
            self.afc.logger.error(c, &format!("Failed to initialize OAMS commands: {}", e.message()));
        }
        Ok(())
    }

    fn handle_ready(&self) -> KResult<()> {
        self.action_status.set(None);
        self.action_status_code.set(None);
        self.action_status_value.set(None);
        self.afc.logger.info(&AFC_OAMS_AFC_OAMS_HANDLE_READY, &format!("OAMS[{}]: Cleared software error states on ready", self.oams_idx));
        Ok(())
    }

    /// clear_errors: error LEDs off, the action state cleared, current spool re-read
    pub fn clear_errors(&self) -> KResult<()> {
        let c = &AFC_OAMS_AFC_OAMS_CLEAR_ERRORS;
        for i in 0..4 {
            if let Err(e) = self.set_led_error(i, 0) {
                self.afc.logger.error(c, &format!("Failed to clear LED error for bay {} on {}: {}", i, self.name, e.message()));
            }
        }
        self.action_status.set(None);
        self.action_status_code.set(None);
        self.action_status_value.set(None);
        match self.determine_current_spool() {
            Ok(s) => self.current_spool.set(s),
            Err(e) => self.afc.logger.error(c, &format!("Failed to determine current spool during clear_errors on {}: {}", self.name, e.message())),
        }
        Ok(())
    }

    pub fn set_led_error(&self, idx: i64, value: i64) -> KResult<()> {
        self.afc.logger.debug(&AFC_OAMS_AFC_OAMS_SET_LED_ERROR, &format!("Setting LED {} to {}", idx, value));
        self.send(|c| &c.set_led_error, "oams_set_led_error_cmd", &[idx, value])
    }

    /// determine_current_spool: the firmware's loaded bay (None: none / no answer)
    pub fn determine_current_spool(&self) -> KResult<Option<i64>> {
        let c = &AFC_OAMS_AFC_OAMS_DETERMINE_CURRENT_SPOOL;
        let params = {
            let cmds = self.cmds.borrow();
            let q = cmds.spool_query_spool.as_ref().ok_or_else(|| self.missing("oams_spool_query_spool_cmd"))?;
            q.send(&[], 0, 0, true)?
        };
        let Some(spool_val) = params.get("spool") else {
            self.afc.logger.warning(c, &format!("OAMS[{}]: Spool query response missing 'spool' field", self.oams_idx));
            return Ok(None);
        };
        let spool_val = match spool_val {
            Val::Int(i) => *i,
            _ => 0,
        };
        if (0..=3).contains(&spool_val) {
            return Ok(Some(spool_val));
        }
        if spool_val != 255 {
            self.afc.logger.warning(
                c,
                &format!("OAMS[{}]: Unexpected spool index {} from hardware (expected 0-3 or 255); treating as no spool loaded", self.oams_idx, spool_val),
            );
        } else {
            self.afc.logger.debug(c, &format!("OAMS[{}]: No spool loaded (hardware returned 255)", self.oams_idx));
        }
        Ok(None)
    }

    fn register_commands(&self) -> KResult<()> {
        let oams_id = self.oams_idx.to_string();
        let gcode = &self.afc.gcode;
        let commands: [(&str, fn(&AfcOams, &GCodeCommand) -> KResult<()>, &str); 11] = [
            ("OAMS_LOAD_SPOOL", AfcOams::cmd_oams_load_spool, CMD_OAMS_LOAD_SPOOL_HELP),
            ("OAMS_UNLOAD_SPOOL", AfcOams::cmd_oams_unload_spool, CMD_OAMS_UNLOAD_SPOOL_HELP),
            ("OAMS_FOLLOWER", AfcOams::cmd_oams_follower, CMD_OAMS_FOLLOWER_HELP),
            ("OAMS_CALIBRATE_PTFE_LENGTH", AfcOams::cmd_oams_calibrate_ptfe_length, CMD_OAMS_CALIBRATE_PTFE_LENGTH_HELP),
            ("OAMS_CALIBRATE_HUB_HES", AfcOams::cmd_oams_calibrate_hub_hes, CMD_OAMS_CALIBRATE_HUB_HES_HELP),
            ("OAMS_PID_AUTOTUNE", AfcOams::cmd_oams_pid_autotune, CMD_OAMS_PID_AUTOTUNE_HELP),
            ("OAMS_PID_SET", AfcOams::cmd_oams_pid_set, CMD_OAMS_PID_SET_HELP),
            ("OAMS_CURRENT_PID_SET", AfcOams::cmd_oams_current_pid_set, CMD_OAMS_CURRENT_PID_SET_HELP),
            ("OAMS_ABORT_ACTION", AfcOams::cmd_oams_abort_action, CMD_OAMS_ABORT_ACTION_HELP),
            ("OAMS_RETRY_STATUS", AfcOams::cmd_oams_retry_status, CMD_OAMS_RETRY_STATUS_HELP),
            ("OAMS_RESET_RETRY_COUNTS", AfcOams::cmd_oams_reset_retry_counts, CMD_OAMS_RESET_RETRY_COUNTS_HELP),
        ];
        for (cmd, f, help) in commands {
            let w = self.me.clone();
            gcode.register_mux_command(cmd, "OAMS", Some(&oams_id), Rc::new(move |g| w.upgrade().map_or(Ok(()), |o| f(&o, g))), Some(help))?;
        }
        let show = self.afc.show_macros;
        let w = self.me.clone();
        self.afc.function.register_mux_command(
            show,
            "AFC_STOP_OAMS_FOLLOWER",
            "OAMS",
            &oams_id,
            Rc::new(move |g| w.upgrade().map_or(Ok(()), |o| o.cmd_afc_stop_oams_follower(g))),
            CMD_AFC_STOP_OAMS_FOLLOWER_HELP,
            None,
        )?;
        let w = self.me.clone();
        self.afc.function.register_mux_command(
            show,
            "AFC_START_OAMS_FOLLOWER",
            "OAMS",
            &oams_id,
            Rc::new(move |g| w.upgrade().map_or(Ok(()), |o| o.cmd_afc_start_oams_follower(g))),
            CMD_AFC_START_OAMS_FOLLOWER_HELP,
            None,
        )
    }

    fn cmd_oams_retry_status(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let mut msg_lines = vec![
            format!("OAMS[{}] Retry Status:", self.oams_idx),
            format!("  Load retry max: {}", self.load_retry_max),
            format!("  Unload retry max: {}", self.unload_retry_max),
            format!("  Retry delay: {:.1}s", self.retry_delay),
            format!("  Auto-unload on failed load: {}", if self.auto_unload_on_failed_load { "True" } else { "False" }),
            format!("  Current unload retry count: {}", self.unload_retry_count.get()),
        ];
        let state = self.load_retry_state.borrow();
        if !state.is_empty() {
            msg_lines.push("  Load retry counts:".into());
            for (spool_idx, retry) in state.iter() {
                msg_lines.push(format!("    Spool {}: {}/{}", spool_idx, retry.count, self.load_retry_max));
            }
        } else {
            msg_lines.push("  No active load retries".into());
        }
        gcmd.respond_info(&msg_lines.join("\n"), true);
        Ok(())
    }

    fn cmd_oams_reset_retry_counts(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.load_retry_state.borrow_mut().clear();
        self.unload_retry_count.set(0);
        self.last_unload_attempt.set(0.0);
        self.last_successful_load.borrow_mut().clear();
        gcmd.respond_info(&format!("OAMS[{}]: Reset all retry counters", self.oams_idx), true);
        Ok(())
    }

    /// load_spool_with_retry: (loaded, message)
    pub fn load_spool_with_retry(&self, spool_idx: i64, max_retries: Option<i64>) -> KResult<(bool, String)> {
        let c = &AFC_OAMS_AFC_OAMS_LOAD_SPOOL_WITH_RETRY;
        let mut retry_count = self.load_retry_state.borrow_mut().entry(spool_idx).or_default().count;
        let mut attempt_history = vec![];
        let retry_limit = match max_retries {
            Some(m) if m > 0 => m,
            _ => self.load_retry_max,
        };
        while retry_count < retry_limit {
            if retry_count > 0 {
                let delay = self.retry_delay;
                self.afc.logger.info(c, &format!("OAMS[{}]: Load retry {}/{} for {}, waiting {:.1}s", self.oams_idx, retry_count + 1, retry_limit, self.lane_label(spool_idx), delay));
                self.pause(delay)?;
                self.abort_current_action(opcode::ERROR_KLIPPER_CALL, true)?;
                self.pause(1.0)?;
            }
            {
                let mut state = self.load_retry_state.borrow_mut();
                let retry = state.entry(spool_idx).or_default();
                retry.count = retry_count + 1;
                retry.last_attempt = Some(self.reactor.monotonic());
            }
            let (code, message) = self.load_spool(spool_idx)?;
            if code == Some(opcode::SUCCESS) || code == Some(opcode::CANCEL) {
                self.last_successful_load.borrow_mut().insert(spool_idx, self.reactor.monotonic());
                if let Some(r) = self.load_retry_state.borrow_mut().get_mut(&spool_idx) {
                    r.was_retry = retry_count > 0;
                }
                self.load_retry_state.borrow_mut().remove(&spool_idx);
                self.afc.logger.info(c, &format!("OAMS[{}]: Successfully loaded {} on attempt {}", self.oams_idx, self.lane_label(spool_idx), retry_count + 1));
                return Ok((true, message));
            }
            attempt_history.push(format!("Attempt {}: {}", retry_count + 1, message));
            let lane_label = self.lane_label(spool_idx);
            if retry_count + 1 < retry_limit {
                self.afc.logger.warning(c, &format!("OAMS[{}]: Load failed for {}: {}. Attempt {}/{}", self.oams_idx, lane_label, message, retry_count + 1, retry_limit));
                if self.auto_unload_on_failed_load {
                    self.afc.logger.info(c, &format!("OAMS[{}]: Auto-unloading before retry", self.oams_idx));
                    let (unload_success, unload_msg) = self.unload_spool_with_retry(None)?;
                    if !unload_success {
                        self.afc.logger.error(c, &format!("OAMS[{}]: Failed to unload before retry: {}", self.oams_idx, unload_msg));
                        self.load_retry_state.borrow_mut().remove(&spool_idx);
                        self.load_retry_failures.set(self.load_retry_failures.get() + 1);
                        self.last_load_failure_time.set(Some(self.reactor.monotonic()));
                        return Ok((
                            false,
                            format!("Failed to unload {} back to AMS before retry. Load aborted after {} attempts. {}", lane_label, retry_count + 1, unload_msg),
                        ));
                    }
                }
                retry_count += 1;
            } else {
                break;
            }
        }
        self.load_retry_state.borrow_mut().remove(&spool_idx);
        self.load_retry_failures.set(self.load_retry_failures.get() + 1);
        self.last_load_failure_time.set(Some(self.reactor.monotonic()));
        Ok((false, format!("Failed to load {} after {} attempts. Attempt history: {}", self.lane_label(spool_idx), retry_limit, attempt_history.join("; "))))
    }

    /// unload_spool_with_retry: (unloaded, message)
    pub fn unload_spool_with_retry(&self, max_retries: Option<i64>) -> KResult<(bool, String)> {
        let c = &AFC_OAMS_AFC_OAMS_UNLOAD_SPOOL_WITH_RETRY;
        let mut attempt_history = vec![];
        let retry_limit = match max_retries {
            Some(m) if m > 0 => m,
            _ => self.unload_retry_max,
        };
        while self.unload_retry_count.get() < retry_limit {
            if self.unload_retry_count.get() > 0 {
                let delay = self.retry_delay;
                self.afc.logger.info(c, &format!("OAMS[{}]: Unload retry {}/{}, waiting {:.1}s", self.oams_idx, self.unload_retry_count.get() + 1, retry_limit, delay));
                self.pause(delay)?;
                self.abort_current_action(opcode::ERROR_KLIPPER_CALL, true)?;
                self.pause(1.0)?;
                let r = (|| -> KResult<()> {
                    for cmd in ["M83", "G92 E0", "G1 E-5.00 F1200", "M400"] {
                        self.afc.gcode.run_script_from_command(cmd)?;
                    }
                    Ok(())
                })();
                if let Err(e) = r {
                    self.afc.logger.warning(c, &format!("OAMS[{}]: Failed to retract extruder before unload retry: {}", self.oams_idx, e.message()));
                }
            }
            self.unload_retry_count.set(self.unload_retry_count.get() + 1);
            let attempt_number = self.unload_retry_count.get();
            self.last_unload_attempt.set(self.reactor.monotonic());
            let (success, message) = self.unload_spool()?;
            if success {
                self.reset_unload_retry_count();
                let lane_label = match self.current_spool.get().and_then(|s| self.resolve_lane_name(s)) {
                    Some(n) => format!("lane {}", n),
                    None => "filament".into(),
                };
                self.afc.logger.info(c, &format!("OAMS[{}]: Successfully unloaded {} on attempt {}", self.oams_idx, lane_label, attempt_number));
                return Ok((true, message));
            }
            attempt_history.push(format!("Attempt {}: {}", self.unload_retry_count.get(), message));
            if self.unload_retry_count.get() < retry_limit {
                self.afc.logger.warning(c, &format!("OAMS[{}]: Unload failed: {}. Attempt {}/{}", self.oams_idx, message, self.unload_retry_count.get(), retry_limit));
            } else {
                break;
            }
        }
        self.reset_unload_retry_count();
        self.unload_retry_failures.set(self.unload_retry_failures.get() + 1);
        self.last_unload_failure_time.set(Some(self.reactor.monotonic()));
        Ok((false, format!("Failed to unload after {} attempts. Attempt history: {}", retry_limit, attempt_history.join("; "))))
    }

    fn reset_unload_retry_count(&self) {
        self.unload_retry_count.set(0);
        self.last_unload_attempt.set(0.0);
    }

    /// load_spool_cancel: the firmware cancel, when it has one
    pub fn load_spool_cancel(&self) -> KResult<String> {
        let cmds = self.cmds.borrow();
        let cancel = cmds.load_spool_cancel.as_ref().ok_or_else(|| self.missing("oams_load_spool_cancel_cmd"))?;
        match cancel {
            Some(cmd) => {
                cmd.send(&[], 0, 0)?;
                Ok("OAMS load spool operation cancelled".into())
            }
            None => Ok("OAMS load spool cancel command not available on this firmware".into()),
        }
    }

    fn get_pid_args(gcmd: &GCodeCommand, default_target: f64) -> KResult<(f64, f64, f64, f64)> {
        let p = gcmd.get_float_opt("P", Lim::new())?;
        let i = gcmd.get_float_opt("I", Lim::new())?;
        let d = gcmd.get_float_opt("D", Lim::new())?;
        let t = gcmd.get_float_opt("TARGET", Lim::new())?;
        let p = p.ok_or_else(|| gcmd.error("P value is required"))?;
        let i = i.ok_or_else(|| gcmd.error("I value is required"))?;
        let d = d.ok_or_else(|| gcmd.error("D value is required"))?;
        Ok((p, i, d, t.unwrap_or(default_target)))
    }

    fn cmd_oams_current_pid_set(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let (p, i, d, t) = Self::get_pid_args(gcmd, self.current_target.get())?;
        self.send(|c| &c.pid, "oams_pid_cmd", &[float_to_u32(p), float_to_u32(i), float_to_u32(d), float_to_u32(t)])?;
        self.current_kp.set(p);
        self.current_ki.set(i);
        self.current_kd.set(d);
        self.current_target.set(t);
        gcmd.respond_info(&format!("Current PID values set to P={:.6} I={:.6} D={:.6} TARGET={:.6}", p, i, d, t), true);
        Ok(())
    }

    fn cmd_oams_pid_set(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let (p, i, d, t) = Self::get_pid_args(gcmd, self.fps_target.get())?;
        self.send(|c| &c.pid, "oams_pid_cmd", &[float_to_u32(p), float_to_u32(i), float_to_u32(d), float_to_u32(t)])?;
        self.kp.set(p);
        self.ki.set(i);
        self.kd.set(d);
        self.fps_target.set(t);
        gcmd.respond_info(&format!("PID values set to P={:.6} I={:.6} D={:.6} TARGET={:.6}", p, i, d, t), true);
        Ok(())
    }

    fn cmd_oams_pid_autotune(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let target_flow = gcmd.get_float_opt("TARGET_FLOW", Lim::new())?;
        let target_temp = gcmd.get_float_opt("TARGET_TEMP", Lim::new())?;
        let target_flow = target_flow.ok_or_else(|| gcmd.error("TARGET flowrate in mm^3/s is required"))?;
        let target_temp = target_temp.ok_or_else(|| gcmd.error("TARGET temperature in degrees C is required"))?;
        let extrusion_speed_per_min = 60.0 * target_flow / (std::f64::consts::PI * (1.75f64 / 2.0).powi(2));
        let extrusion_length = extrusion_speed_per_min / 60.0 * 30.0;
        self.afc.gcode.run_script_from_command(&format!("M104 S{:.6}", target_temp))?;
        self.afc.gcode.run_script_from_command(&format!("G1 E{:.6} F{:.6}", extrusion_length, extrusion_speed_per_min))
    }

    /// wait until the firmware reports the action done
    fn wait_action(&self) -> KResult<()> {
        while self.action_status.get().is_some() {
            self.pause(0.5)?;
        }
        Ok(())
    }

    fn cmd_oams_calibrate_hub_hes(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let spool_idx = gcmd.get_int_opt("SPOOL", Lim::new())?.ok_or_else(|| gcmd.error("SPOOL index is required"))?;
        if !(0..=3).contains(&spool_idx) {
            return Err(gcmd.error("Invalid SPOOL index"));
        }
        self.action_status.set(Some(status::CALIBRATING));
        self.send(|c| &c.calibrate_hub_hes, "oams_calibrate_hub_hes_cmd", &[spool_idx])?;
        self.wait_action()?;
        if self.action_status_code.get() == Some(opcode::SUCCESS) {
            let value = u32_to_float(self.action_status_value.get().unwrap_or(0));
            gcmd.respond_info(&format!("Calibrated HES {} to {:.6} threshold", spool_idx, value), true);
            self.hub_hes_on.borrow_mut()[spool_idx as usize] = value;
            let values = self.hub_hes_on.borrow().iter().map(|v| kaliru::pyfmt::py_float_repr(*v)).collect::<Vec<_>>().join(", ");
            let cal_msg = format!("\n{} hub_hes_on: {}", self.config_name, values);
            self.afc.function.config_rewrite(&self.config_name, "hub_hes_on", &PyVal::Str(values), &cal_msg)?;
            gcmd.respond_info(&format!("HES calibration complete: hub_hes_on index {} = {:.6} saved to config", spool_idx, value), true);
            Ok(())
        } else {
            Err(gcmd.error(format!("Calibration of HES {} failed", spool_idx)))
        }
    }

    fn cmd_oams_calibrate_ptfe_length(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.action_status.set(Some(status::CALIBRATING));
        let spool = gcmd.get_int_opt("SPOOL", Lim::new())?.ok_or_else(|| gcmd.error("SPOOL index is required"))?;
        self.send(|c| &c.calibrate_ptfe_length, "oams_calibrate_ptfe_length_cmd", &[spool])?;
        self.wait_action()?;
        if self.action_status_code.get() == Some(opcode::SUCCESS) {
            let ptfe_val = self.action_status_value.get().unwrap_or(0).to_string();
            gcmd.respond_info(&format!("Calibrated PTFE length to {}", ptfe_val), true);
            let cal_msg = format!("\n{} ptfe_length: {}", self.config_name, ptfe_val);
            self.afc.function.config_rewrite(&self.config_name, "ptfe_length", &PyVal::Str(ptfe_val.clone()), &cal_msg)?;
            gcmd.respond_info(&format!("PTFE calibration complete: ptfe_length {} saved to config", ptfe_val), true);
            Ok(())
        } else {
            Err(gcmd.error("Calibration of PTFE length failed"))
        }
    }

    /// load_spool: one load, waiting for the firmware's result (code, message)
    pub fn load_spool(&self, spool_idx: i64) -> KResult<(Option<i64>, String)> {
        let c = &AFC_OAMS_AFC_OAMS_LOAD_SPOOL;
        self.action_status.set(Some(status::LOADING));
        self.send(|c| &c.load_spool, "oams_load_spool_cmd", &[spool_idx])?;
        let start = self.reactor.monotonic();
        let timeout = start + 45.0;
        let stall_enabled = self.load_stall_dwell > 0.0;
        let mut last_clicks = self.encoder_clicks.get();
        let mut last_move_time = start;
        while self.action_status.get().is_some() {
            let now = self.reactor.monotonic();
            // Early stall detection: the encoder going flat past the spin-up
            // grace means a stuck spool
            if stall_enabled && self.encoder_clicks.get() != last_clicks {
                last_clicks = self.encoder_clicks.get();
                last_move_time = now;
            }
            if stall_enabled && now - start > self.load_stall_grace && now - last_move_time > self.load_stall_dwell {
                self.afc.logger.error(c, &format!("OAMS[{}]: Load stalled - encoder stopped advancing for {:.0}s (spool stuck)", self.oams_idx, self.load_stall_dwell));
                if let Err(e) = self.load_spool_cancel() {
                    self.afc.logger.warning(c, &format!("OAMS[{}]: Failed to cancel stalled load: {}", self.oams_idx, e.message()));
                }
                self.action_status.set(None);
                self.action_status_code.set(Some(opcode::ERROR_UNSPECIFIED));
                return Ok((Some(opcode::ERROR_UNSPECIFIED), "OAMS load stalled (spool stuck, no encoder movement)".into()));
            }
            if now > timeout {
                self.afc.logger.error(c, &format!("OAMS[{}]: Load operation timed out after 45 seconds", self.oams_idx));
                // The firmware is still in its load routine: cancel it, or it
                // rejects every later command as busy
                if let Err(e) = self.load_spool_cancel() {
                    self.afc.logger.warning(c, &format!("OAMS[{}]: Failed to cancel stuck load after timeout: {}", self.oams_idx, e.message()));
                }
                self.action_status.set(None);
                self.action_status_code.set(Some(opcode::ERROR_UNSPECIFIED));
                return Ok((Some(opcode::ERROR_UNSPECIFIED), "OAMS load operation timed out (MCU unresponsive)".into()));
            }
            self.pause(0.2)?;
        }
        let code = self.action_status_code.get();
        Ok(match code {
            Some(opcode::SUCCESS) => {
                self.current_spool.set(Some(spool_idx));
                (code, "Spool loaded successfully".into())
            }
            Some(opcode::ERROR_KLIPPER_CALL) => (code, "Spool loading stopped by klipper monitor".into()),
            Some(opcode::ERROR_BUSY) => (code, "OAMS is busy".into()),
            Some(opcode::CANCEL) => (code, "Spool loading cancelled".into()),
            Some(v) => (code, format!("Unknown error from OAMS with code {}", v)),
            None => return Err(KError::Py("TypeError".into(), "%d format: a real number is required, not NoneType".into())),
        })
    }

    fn cmd_oams_load_spool(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.action_status.set(Some(status::LOADING));
        let spool_idx = gcmd.get_int_opt("SPOOL", Lim::new())?.ok_or_else(|| gcmd.error("SPOOL index is required"))?;
        if !(0..=3).contains(&spool_idx) {
            return Err(gcmd.error("Invalid SPOOL index"));
        }
        let quiet = gcmd.get_int("QUIET", Some(0), Lim::new())?;
        let (success, message) = self.load_spool_with_retry(spool_idx, None)?;
        if success && quiet == 0 {
            gcmd.respond_info(&message, true);
        } else if !success {
            return Err(gcmd.error(message));
        }
        Ok(())
    }

    /// unload_spool: one unload, waiting for the firmware's result
    pub fn unload_spool(&self) -> KResult<(bool, String)> {
        let c = &AFC_OAMS_AFC_OAMS_UNLOAD_SPOOL;
        self.action_status.set(Some(status::UNLOADING));
        self.send(|c| &c.unload_spool, "oams_unload_spool_cmd", &[])?;
        let timeout = self.reactor.monotonic() + 40.0;
        while self.action_status.get().is_some() {
            if self.reactor.monotonic() > timeout {
                self.afc.logger.error(c, &format!("OAMS[{}]: Unload operation timed out after 40 seconds", self.oams_idx));
                self.action_status.set(None);
                self.action_status_code.set(Some(opcode::ERROR_UNSPECIFIED));
                return Ok((false, "OAMS unload operation timed out (MCU unresponsive)".into()));
            }
            self.pause(0.2)?;
        }
        Ok(match self.action_status_code.get() {
            Some(opcode::SUCCESS) => {
                self.current_spool.set(None);
                (true, "Spool unloaded successfully".into())
            }
            Some(opcode::ERROR_KLIPPER_CALL) => (false, "Spool unloading stopped by klipper monitor".into()),
            Some(opcode::ERROR_BUSY) => (false, "OAMS is busy".into()),
            Some(opcode::NO_SPOOL_IN_BAY) => {
                self.current_spool.set(None);
                (true, "Spool already unloaded (NO_SPOOL_IN_BAY)".into())
            }
            Some(opcode::CANCEL) => (false, "Unload was cancelled (stale cancel response interfered)".into()),
            v => (false, format!("Unknown error from OAMS (status_code={})", py_opt_int(v))),
        })
    }

    fn cmd_oams_unload_spool(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let max_retries = gcmd.get_int_opt("MAX_RETRIES", Lim::new())?;
        let (success, message) = self.unload_spool_with_retry(max_retries)?;
        if success {
            gcmd.respond_info(&message, true);
            Ok(())
        } else {
            Err(gcmd.error(message))
        }
    }

    fn cmd_oams_abort_action(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let code = gcmd.get_int("CODE", Some(opcode::ERROR_KLIPPER_CALL), Lim::new())?;
        let wait = gcmd.get_int("WAIT", Some(1), Lim::new())?;
        self.abort_current_action(code, wait != 0)
    }

    /// oams_load_spool_cmd.send([spool]): a load, not waited for
    pub fn send_load_spool(&self, spool_idx: i64) -> KResult<()> {
        self.send(|c| &c.load_spool, "oams_load_spool_cmd", &[spool_idx])
    }

    pub fn set_oams_follower(&self, enable: i64, direction: i64) -> KResult<()> {
        self.send(|c| &c.follower, "oams_follower_cmd", &[enable, direction])
    }

    /// abort_current_action: cancel in the firmware, then clear the action
    pub fn abort_current_action(&self, code: i64, wait: bool) -> KResult<()> {
        let c = &AFC_OAMS_AFC_OAMS_ABORT_CURRENT_ACTION;
        let Some(action) = self.action_status.get() else { return Ok(()) };
        if let Err(e) = self.load_spool_cancel() {
            self.afc.logger.warning(c, &format!("OAMS[{}]: Failed to send firmware cancel during abort: {}", self.oams_idx, e.message()));
        }
        if wait {
            self.afc.logger.debug(c, &format!("OAMS[{}]: Aborting current action {} with code {}", self.oams_idx, action, code));
            let timeout = self.reactor.monotonic() + 5.0;
            let mut pause_delay = 0.5;
            while self.action_status.get().is_some() {
                if self.reactor.monotonic() > timeout {
                    self.afc.logger.debug(c, &format!("OAMS[{}]: Abort timeout - forcing clear", self.oams_idx));
                    break;
                }
                self.pause(pause_delay)?;
                pause_delay = f64::min(pause_delay + 0.25, 1.5);
            }
            self.action_status_code.set(Some(code));
            self.action_status_value.set(None);
            self.action_status.set(None);
            self.afc.logger.info(c, &format!("OAMS[{}]: Abort complete", self.oams_idx));
        } else {
            self.action_status_code.set(Some(code));
            self.action_status_value.set(None);
            self.action_status.set(None);
            self.afc.logger.debug(c, &format!("OAMS[{}]: Abort without waiting - status cleared", self.oams_idx));
        }
        Ok(())
    }

    fn cmd_oams_follower(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let enable = gcmd.get_int_opt("ENABLE", Lim::new())?.ok_or_else(|| gcmd.error("ENABLE is required"))?;
        let direction = gcmd.get_int_opt("DIRECTION", Lim::new())?.ok_or_else(|| gcmd.error("DIRECTION is required"))?;
        self.set_oams_follower(enable, direction)?;
        if enable == 1 && direction == 0 {
            gcmd.respond_info("Follower enable in reverse direction", true);
        } else if enable == 1 && direction == 1 {
            gcmd.respond_info("Follower enable in forward direction", true);
        } else if enable == 0 {
            gcmd.respond_info("Follower disabled", true);
        }
        Ok(())
    }

    fn cmd_afc_stop_oams_follower(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.afc.gcode.run_script_from_command(&format!("OAMS_FOLLOWER OAMS={} ENABLE=0 DIRECTION=1", self.oams_idx))
    }

    fn cmd_afc_start_oams_follower(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.afc.gcode.run_script_from_command(&format!("OAMS_FOLLOWER OAMS={} ENABLE=1 DIRECTION=1", self.oams_idx))
    }

    fn oams_cmd_stats(&self, params: &Params) -> KResult<()> {
        *self.fps_value.borrow_mut() = PyVal::Float(u32_to_float(param(params, "fps_value")?));
        for i in 0..4 {
            self.f1s_hes_value.borrow_mut()[i] = param(params, &format!("f1s_hes_value_{}", i))?;
        }
        for i in 0..4 {
            self.hub_hes_value.borrow_mut()[i] = param(params, &format!("hub_hes_value_{}", i))?;
        }
        self.encoder_clicks.set(param(params, "encoder_clicks")?);
        Ok(())
    }

    fn oams_cmd_current_status(&self, params: &Params) -> KResult<()> {
        self.i_value.set(u32_to_float(param(params, "current_value")?));
        Ok(())
    }

    fn oams_action_status(&self, params: &Params) -> KResult<()> {
        let c = &AFC_OAMS_AFC_OAMS__OAMS_ACTION_STATUS;
        self.afc.logger.debug(c, "OAMS status received");
        let action = param(params, "action")?;
        let code = param(params, "code")?;
        let names = || format!("{} ({})", enum_name(&status::NAMES, action, "action"), enum_name(&opcode::NAMES, code, "code"));
        if [status::LOADING, status::UNLOADING, status::ERROR].contains(&action) {
            self.action_status.set(None);
            self.action_status_code.set(Some(code));
        } else if action == status::CALIBRATING {
            self.action_status.set(None);
            self.action_status_code.set(Some(code));
            self.action_status_value.set(Some(param(params, "value")?));
        } else if code == opcode::ERROR_KLIPPER_CALL {
            self.action_status.set(None);
            self.action_status_code.set(Some(code));
        } else if [status::FORWARD_FOLLOWING, status::REVERSE_FOLLOWING, status::COASTING, status::STOPPED].contains(&action) {
            self.afc.logger.debug(c, &format!("OAMS status update (non-action): {}", names()));
        } else {
            self.afc.logger.debug(c, &format!("OAMS status update (unhandled): {}", names()));
        }
        Ok(())
    }

    /// _build_config: the firmware's buffer, sensor, PID, PTFE and logger setup
    fn build_config(&self) -> KResult<()> {
        let u = float_to_u32;
        let b = |v: bool| v as i64;
        let on = |v: &[f64], i: usize| v.get(i).copied().ok_or_else(|| KError::Py("IndexError".into(), "list index out of range".into()));
        let hub = self.hub_hes_on.borrow().clone();
        let cmds = [
            format!("config_oams_buffer upper={} lower={} is_reversed={}", u(self.fps_upper_threshold), u(self.fps_lower_threshold), b(self.fps_is_reversed)),
            format!(
                "config_oams_f1s_hes on1={} on2={} on3={} on4={} is_above={}",
                u(on(&self.f1s_hes_on, 0)?),
                u(on(&self.f1s_hes_on, 1)?),
                u(on(&self.f1s_hes_on, 2)?),
                u(on(&self.f1s_hes_on, 3)?),
                b(self.f1s_hes_is_above)
            ),
            format!("config_oams_hub_hes on1={} on2={} on3={} on4={} is_above={}", u(on(&hub, 0)?), u(on(&hub, 1)?), u(on(&hub, 2)?), u(on(&hub, 3)?), b(self.hub_hes_is_above)),
            format!("config_oams_pid kp={} ki={} kd={} target={}", u(self.kp.get()), u(self.ki.get()), u(self.kd.get()), u(self.fps_target.get())),
            // ("%u" of the float length: its integer part)
            format!("config_oams_ptfe length={}", self.filament_path_length.trunc() as i64),
            format!("config_oams_current_pid kp={} ki={} kd={} target={}", u(self.current_kp.get()), u(self.current_ki.get()), u(self.current_kd.get()), u(self.current_target.get())),
            format!("config_oams_logger idx={}", self.oams_idx),
        ];
        for cmd in cmds {
            self.mcu.add_config_cmd(&cmd, false, false);
        }
        Ok(())
    }
}

impl PrinterObject for AfcOams {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        Some(self.status())
    }
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        let f1s = *self.f1s_hes_value.borrow();
        let hub = *self.hub_hes_value.borrow();
        Some((
            false,
            format!(
                "\nOAMS[{}]: current_spool={} fps_value={} f1s_hes_value_0={} f1s_hes_value_1={} f1s_hes_value_2={} f1s_hes_value_3={} hub_hes_value_0={} hub_hes_value_1={} hub_hes_value_2={} hub_hes_value_3={} kp={} ki={} kd={} encoder_clicks={} i_value={:.2}\n",
                self.oams_idx,
                py_opt_int(self.current_spool.get()),
                self.fps_value.borrow().str(),
                f1s[0],
                f1s[1],
                f1s[2],
                f1s[3],
                hub[0],
                hub[1],
                hub[2],
                hub[3],
                self.kp.get() as i64,
                self.ki.get() as i64,
                self.kd.get() as i64,
                self.encoder_clicks.get(),
                self.i_value.get()
            ),
        ))
    }
}

/// the AFC_OAMS object of a name, when it is one
pub fn lookup(printer: &Printer, name: &str) -> Option<Rc<AfcOams>> {
    printer.lookup_object_opt(&format!("AFC_OAMS {}", name)).and_then(kaliru::printer::downcast_rc::<AfcOams>)
}
