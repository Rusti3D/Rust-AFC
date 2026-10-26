// The OpenAMS as an AFC unit (port of AFC_OpenAMS.py: afcAMS, its follower
// motor controller and its stuck-spool / clog monitor). Stepperless: the
// [AFC_OAMS] controller (oams.rs) feeds and retracts, the unit polls its Hall
// sensors into the lanes' prep / hub state and verifies engagement on loads.
//
// AMSEventBus and LaneRegistry have no subscribers and no registered lanes in
// AFC, so their events change nothing: a spool's lane is found by scanning
// AFC's units (resolve_lane_name_from_afc), as the python falls back to.
//
// Copyright (C) 2026 AFCProject
//
// This file include code inspired/modified from OpenAms Project. https://github.com/OpenAMSOrg/klipper_openams
// Originally authored by JR Lomas(aka KnightRadiant) and licensed under the MIT license
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::buffer::AfcBuffer;
use super::codes::*;
use super::extruder::AfcExtruder;
use super::lane::{AfcLane, LaneState};
use super::oams::{self, AfcOams};
use super::unit::{AfcUnit, UnitKind, UnitSettings};
use super::utils::DateTime;
use super::Afc;
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::gcode::GCodeCommand;
use kaliru::printer::PObj;
use kaliru::pyval::PyVal;
use kaliru::reactor::{Reactor, Timer, NEVER, NOW};
use kaliru::IndexMap;
use std::cell::{Cell, RefCell};
use std::collections::{HashMap, HashSet};
use std::rc::{Rc, Weak};

pub const UNIT_TYPE: &str = "OpenAMS";

const CMD_AFC_OAMS_CALIBRATE_PTFE_HELP: &str = "Calibrate OpenAMS PTFE length";
const CMD_AFC_OAMS_CALIBRATE_HUB_HES_HELP: &str = "Calibrate OpenAMS hub HES for a spool";
const CMD_AFC_OAMS_CALIBRATE_HUB_HES_ALL_HELP: &str = "Calibrate all loaded OpenAMS hub HES sensors";
const CMD_AFC_OAMS_CLEAR_ERRORS_HELP: &str = "Clear OpenAMS errors and resync state";
const CMD_STUCK_RECOVERY_HELP: &str = "Internal: auto-recover from a stuck spool via unload+reload";

// ── monitor constants ────────────────────────────────────────────────
const MONITOR_INTERVAL: f64 = 2.0;
const MONITOR_INTERVAL_IDLE: f64 = 4.0;
const STUCK_PRESSURE_CLEAR: f64 = 0.12;
const STUCK_DWELL: f64 = 2.0;
const STUCK_MIN_ENCODER: i64 = 3;

/// CLOG_SENSITIVITY: None turns clog detection off
fn clog_sensitivity(name: &str) -> Option<f64> {
    match name {
        "off" => None,
        "low" => Some(1.5),
        "high" => Some(0.7),
        // ('medium', and .get()'s default for any other value)
        _ => Some(1.0),
    }
}

/// FPSLoadState
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum FpsLoadState {
    Unloaded,
    Loaded,
}

/// FPSState: one FPS channel's tracking
pub struct FpsState {
    pub state: FpsLoadState,
    pub current_lane: Option<String>,
    pub current_oams: Option<String>,
    pub current_spool_idx: Option<i64>,
    pub since: Option<f64>,
    pub toolhead_confirmed: bool,
    pub last_encoder: Option<i64>,
    pub stuck_active: bool,
    pub stuck_start_time: Option<f64>,
    pub clog_active: bool,
    pub clog_start_time: Option<f64>,
    pub engagement_in_progress: bool,
    pub engagement_checked_at: Option<f64>,
    pub last_lane_change_time: Option<f64>,
}

impl FpsState {
    fn new() -> FpsState {
        FpsState {
            state: FpsLoadState::Unloaded,
            current_lane: None,
            current_oams: None,
            current_spool_idx: None,
            since: None,
            toolhead_confirmed: false,
            last_encoder: None,
            stuck_active: false,
            stuck_start_time: None,
            clog_active: false,
            clog_start_time: None,
            engagement_in_progress: false,
            engagement_checked_at: None,
            last_lane_change_time: None,
        }
    }

    pub fn reset(&mut self) {
        self.state = FpsLoadState::Unloaded;
        self.current_lane = None;
        self.current_oams = None;
        self.current_spool_idx = None;
        self.since = None;
        self.toolhead_confirmed = false;
        self.stuck_active = false;
        self.stuck_start_time = None;
        self.clog_active = false;
        self.clog_start_time = None;
        self.engagement_in_progress = false;
    }
}

/// OAMSMonitor: stuck spool / clog detection for one FPS channel while printing
pub struct OamsMonitor {
    fps_name: String,
    fps: Rc<AfcBuffer>,
    unit: Weak<AfcUnit>,
    reactor: Rc<Reactor>,
    enable_clog: bool,
    pub state: RefCell<FpsState>,
    timer: RefCell<Option<Timer>>,
    running: Cell<bool>,
    oams: RefCell<Option<Rc<AfcOams>>>,
    stuck_pressure_low: f64,
    stuck_load_grace: f64,
}

impl OamsMonitor {
    fn logger(&self) -> Option<Rc<super::logger::AfcLogger>> {
        self.unit.upgrade().map(|u| u.afc.logger.clone())
    }

    pub fn start(self: &Rc<Self>, oams_obj: &Rc<AfcOams>) {
        *self.oams.borrow_mut() = Some(oams_obj.clone());
        self.running.set(true);
        {
            let mut st = self.state.borrow_mut();
            st.stuck_start_time = None;
            st.clog_start_time = None;
            st.last_encoder = None;
        }
        if self.timer.borrow().is_none() {
            let w = Rc::downgrade(self);
            let t = self.reactor.register_timer(Box::new(move |et| w.upgrade().map_or(NEVER, |m| m.monitor_tick(et))), NOW + MONITOR_INTERVAL);
            *self.timer.borrow_mut() = Some(t);
        }
        if let Some(l) = self.logger() {
            l.debug(&AFC_OPENAMS_OAMSMONITOR_START, &format!("Monitor started for {}", self.fps_name));
        }
    }

    pub fn stop(&self) {
        self.running.set(false);
        if let Some(t) = self.timer.borrow_mut().take() {
            self.reactor.unregister_timer(&t);
        }
        if let Some(l) = self.logger() {
            l.debug(&AFC_OPENAMS_OAMSMONITOR_STOP, &format!("Monitor stopped for {}", self.fps_name));
        }
    }

    pub fn notify_load_complete(&self, lane_name: &str, oams_name: &str, spool_idx: i64) {
        let now = self.reactor.monotonic();
        let mut st = self.state.borrow_mut();
        st.state = FpsLoadState::Loaded;
        st.current_lane = Some(lane_name.to_string());
        st.current_oams = Some(oams_name.to_string());
        st.current_spool_idx = Some(spool_idx);
        st.since = Some(now);
        st.last_lane_change_time = st.since;
        // Not confirmed at the toolhead until AFC's load sequence finishes
        st.toolhead_confirmed = false;
        st.stuck_active = false;
        st.clog_active = false;
        st.last_encoder = None;
    }

    pub fn notify_unload_complete(&self) {
        self.state.borrow_mut().reset();
    }

    fn notify_engagement_start(&self) {
        self.state.borrow_mut().engagement_in_progress = true;
    }

    fn notify_engagement_end(&self) {
        let mut st = self.state.borrow_mut();
        st.engagement_in_progress = false;
        st.engagement_checked_at = Some(self.reactor.monotonic());
    }

    fn monitor_tick(&self, eventtime: f64) -> f64 {
        let oams = self.oams.borrow().clone();
        let (Some(oams), Some(unit)) = (oams.filter(|_| self.running.get()), self.unit.upgrade()) else { return NEVER };
        if self.state.borrow().state != FpsLoadState::Loaded {
            return eventtime + MONITOR_INTERVAL_IDLE;
        }
        // Only a lane confirmed at the toolhead and then gone is a desync;
        // until the first confirmation the load is still finishing
        let lane_loaded = unit.lanes.borrow().values().any(|l| l.tool_loaded.get() && l.extruder_obj_opt().is_some_and(|e| e.on_shuttle()));
        if lane_loaded {
            self.state.borrow_mut().toolhead_confirmed = true;
        } else if self.state.borrow().toolhead_confirmed {
            unit.afc.logger.debug(&AFC_OPENAMS_OAMSMONITOR__MONITOR_TICK, &format!("{}: lane no longer loaded to toolhead, stopping monitor", self.fps_name));
            self.running.set(false);
            self.state.borrow_mut().reset();
            return NEVER;
        } else {
            return eventtime + MONITOR_INTERVAL;
        }
        // Detection only while actively printing
        if !unit.afc.function.in_print().unwrap_or(false) {
            let mut st = self.state.borrow_mut();
            st.clog_start_time = None;
            return eventtime + MONITOR_INTERVAL_IDLE;
        }
        if self.state.borrow().engagement_in_progress {
            return eventtime + MONITOR_INTERVAL;
        }
        if let Some(since) = self.state.borrow().since.filter(|s| *s != 0.0) {
            if eventtime - since < self.stuck_load_grace {
                return eventtime + MONITOR_INTERVAL;
            }
        }
        let encoder = oams.encoder_clicks.get();
        let pressure = self.fps.fps.as_ref().map_or(0.5, |f| f.fps_value.get());
        let encoder_delta = {
            let mut st = self.state.borrow_mut();
            let d = st.last_encoder.map_or(0, |l| (encoder - l).abs());
            st.last_encoder = Some(encoder);
            d
        };
        if let Err(e) = self.check_stuck_spool(&unit, eventtime, encoder_delta, pressure) {
            unit.afc.logger.error(&AFC_OPENAMS_OAMSMONITOR__MONITOR_TICK, &format!("Monitor error on {}: {}", self.fps_name, e.message()));
        } else if self.enable_clog {
            self.check_clog(eventtime);
        }
        eventtime + MONITOR_INTERVAL
    }

    /// _check_stuck_spool: encoder stopped and FPS pressure low
    fn check_stuck_spool(&self, unit: &AfcUnit, eventtime: f64, encoder_delta: i64, pressure: f64) -> KResult<()> {
        if let Some(t) = self.state.borrow().engagement_checked_at.filter(|t| *t != 0.0) {
            if eventtime - t < 6.0 {
                return Ok(());
            }
        }
        let encoder_moving = encoder_delta >= STUCK_MIN_ENCODER;
        let pressure_low = pressure <= self.stuck_pressure_low;
        let c = &AFC_OPENAMS_OAMSMONITOR__CHECK_STUCK_SPOOL;
        if !encoder_moving && pressure_low {
            let (active, start) = {
                let st = self.state.borrow();
                (st.stuck_active, st.stuck_start_time)
            };
            if !active {
                match start {
                    None => {
                        self.state.borrow_mut().stuck_start_time = Some(eventtime);
                        unit.afc.logger.debug(c, &format!("{}: stuck spool timer started (pressure={:.2}, encoder_delta={})", self.fps_name, pressure, encoder_delta));
                    }
                    Some(s) if eventtime - s >= STUCK_DWELL => {
                        self.state.borrow_mut().stuck_active = true;
                        let msg = format!("Stuck spool on {}: encoder stopped ({} clicks), FPS pressure {:.2}", self.fps_name, encoder_delta, pressure);
                        unit.afc.logger.info(c, &msg);
                        on_stuck_spool_detected(unit, Some(&self.fps_name), Some(&msg))?;
                    }
                    Some(_) => {}
                }
            }
        } else {
            let (start, active) = {
                let st = self.state.borrow();
                (st.stuck_start_time, st.stuck_active)
            };
            if (start.is_some() || active) && (pressure >= STUCK_PRESSURE_CLEAR || encoder_moving) {
                if active {
                    unit.afc.logger.info(&AFC_OPENAMS_AFCAMS__ON_STUCK_SPOOL_CLEARED, &format!("Stuck spool cleared on {}", self.fps_name));
                }
                let mut st = self.state.borrow_mut();
                st.stuck_active = false;
                st.stuck_start_time = None;
            }
        }
        Ok(())
    }

    /// _check_clog: it compares the active extruder's position, read from
    /// the FPS buffer's `extruder` attribute, which AFCFPSBuffer does not
    /// have: before and after its post-load grace it returns with nothing done
    fn check_clog(&self, _eventtime: f64) {}
}

/// FollowerController: the follower motor and error LEDs of one OAMS
pub struct FollowerController {
    oams_name: String,
    oams: Rc<AfcOams>,
    reactor: Rc<Reactor>,
    afc: Rc<Afc>,
    last_state: RefCell<HashMap<String, (i64, i64)>>,
    queue: RefCell<HashMap<String, Vec<(i64, i64)>>>,
    in_flight: RefCell<HashMap<String, bool>>,
    poll_timers: RefCell<HashMap<String, Timer>>,
    led_error_state: RefCell<HashMap<String, i64>>,
    me: Weak<FollowerController>,
}

impl FollowerController {
    /// enable_follower: on in a direction (0 or 1, else 1)
    pub fn enable_follower(&self, oams: Option<&Rc<AfcOams>>, direction: i64, context: &str, force: bool) {
        let Some(oams) = oams else { return };
        let direction = if direction == 0 || direction == 1 { direction } else { 1 };
        if !oams.name.is_empty() {
            self.set_follower_if_changed(&oams.name, oams, 1, direction, context, force);
        }
    }

    pub fn set_follower_state(&self, oams: Option<&Rc<AfcOams>>, enable: i64, direction: i64, context: &str, force: bool) {
        let Some(oams) = oams else { return };
        if oams.name.is_empty() {
            return;
        }
        let direction = if direction == 0 || direction == 1 { direction } else { 1 };
        self.set_follower_if_changed(&oams.name, oams, enable, direction, context, force);
    }

    fn set_follower_if_changed(&self, oams_name: &str, oams: &AfcOams, enable: i64, direction: i64, context: &str, force: bool) {
        let c = &AFC_OPENAMS_FOLLOWERCONTROLLER__SET_FOLLOWER_IF_CHANGED;
        let new_state = (enable, direction);
        if !force && self.last_state.borrow().get(oams_name) == Some(&new_state) {
            return;
        }
        let logger = &self.afc.logger;
        logger.debug(
            c,
            &format!(
                "Follower command for {}: enable={} direction={} context={} forced={}",
                oams_name,
                enable,
                direction,
                if context.is_empty() { "no context" } else { context },
                if force { "True" } else { "False" }
            ),
        );
        match oams.set_oams_follower(enable, direction) {
            Ok(()) => {
                self.last_state.borrow_mut().insert(oams_name.to_string(), new_state);
                logger.debug(c, &format!("Follower {} for {} ({})", if enable != 0 { "enabled" } else { "disabled" }, oams_name, context));
            }
            Err(e) => logger.error(c, &format!("Failed to set follower on {}: {}", oams_name, e.message())),
        }
    }

    pub fn set_led_error_if_changed(&self, oams_name: &str, spool_idx: i64, error_state: i64, context: &str) {
        let key = format!("{}_{}", oams_name, spool_idx);
        if self.led_error_state.borrow().get(&key) == Some(&error_state) {
            return;
        }
        self.led_error_state.borrow_mut().insert(key, error_state);
        self.rate_limited_mcu_command(oams_name, spool_idx, error_state);
        self.afc.logger.debug(
            &AFC_OPENAMS_FOLLOWERCONTROLLER_SET_LED_ERROR_IF_CHANGED,
            &format!("LED error {} for {} spool {} ({})", if error_state != 0 { "set" } else { "cleared" }, oams_name, spool_idx, context),
        );
    }

    pub fn clear_error_led(&self, oams_name: &str, spool_idx: i64, context: &str) {
        self.set_led_error_if_changed(oams_name, spool_idx, 0, context);
    }

    /// rate_limited_mcu_command for oams.set_led_error(idx, value)
    fn rate_limited_mcu_command(&self, oams_name: &str, idx: i64, value: i64) {
        if oams_name != self.oams_name || self.oams.mcu.is_shutdown() {
            return;
        }
        if !self.queue.borrow().contains_key(oams_name) {
            self.queue.borrow_mut().insert(oams_name.to_string(), vec![]);
            self.in_flight.borrow_mut().insert(oams_name.to_string(), false);
        }
        self.queue.borrow_mut().get_mut(oams_name).unwrap().push((idx, value));
        self.process_mcu_command_queue(oams_name);
    }

    fn process_mcu_command_queue(&self, oams_name: &str) {
        if oams_name != self.oams_name || self.in_flight.borrow().get(oams_name).copied().unwrap_or(false) {
            return;
        }
        if self.queue.borrow().get(oams_name).is_none_or(|q| q.is_empty()) {
            return;
        }
        let name = oams_name.to_string();
        if self.oams.action_status.get().is_some() {
            if let Some(t) = self.poll_timers.borrow_mut().remove(oams_name) {
                self.reactor.unregister_timer(&t);
            }
            let w = self.me.clone();
            let n = name.clone();
            let t = self.reactor.register_timer(
                Box::new(move |_| {
                    if let Some(f) = w.upgrade() {
                        f.in_flight.borrow_mut().insert(n.clone(), false);
                        f.process_mcu_command_queue(&n);
                    }
                    NEVER
                }),
                NOW + 0.1,
            );
            self.poll_timers.borrow_mut().insert(name, t);
            return;
        }
        let (idx, value) = self.queue.borrow_mut().get_mut(oams_name).unwrap().remove(0);
        self.in_flight.borrow_mut().insert(name.clone(), true);
        if let Err(e) = self.oams.set_led_error(idx, value) {
            self.afc.logger.error(&AFC_OPENAMS_FOLLOWERCONTROLLER__PROCESS_MCU_COMMAND_QUEUE, &format!("MCU command failed for {}: {}", oams_name, e.message()));
        }
        let w = self.me.clone();
        self.reactor.register_timer(
            Box::new(move |_| {
                if let Some(f) = w.upgrade() {
                    f.in_flight.borrow_mut().insert(name.clone(), false);
                    f.process_mcu_command_queue(&name);
                }
                NEVER
            }),
            NOW + 0.05,
        );
    }
}

/// The OpenAMS unit's part (afcAMS)
pub struct OpenAmsUnit {
    pub oams_name: String,
    pub auto_spoolman_create: bool,
    stuck_spool_auto_recovery: bool,
    stuck_spool_load_grace: f64,
    stuck_spool_pressure_threshold: f64,
    clog_sensitivity: String,
    engagement_length: f64,
    engagement_speed: f64,
    defer_engagement: bool,
    oams: RefCell<Option<Rc<AfcOams>>>,
    follower: RefCell<Option<Rc<FollowerController>>>,
    monitor: RefCell<Option<Rc<OamsMonitor>>>,
    spool_map: RefCell<IndexMap<String, i64>>,
    last_f1s: RefCell<[Option<bool>; 4]>,
    last_hub: RefCell<[Option<bool>; 4]>,
    poll_timer: RefCell<Option<Timer>>,
    operation_active: Cell<bool>,
    prev_states_stale: Cell<bool>,
    hub_load_suppressed: RefCell<HashSet<String>>,
    td1_last_capture_time: Cell<Option<f64>>,
    /// lanes with _oams_runout_detected / _oams_runout_empty set
    runout_detected: RefCell<HashSet<String>>,
    runout_empty: RefCell<HashSet<String>>,
}

fn ams(unit: &AfcUnit) -> &OpenAmsUnit {
    match &unit.kind {
        UnitKind::OpenAms(a) => a,
        _ => unreachable!("not an OpenAMS unit"),
    }
}

fn hw(unit: &AfcUnit) -> Option<Rc<AfcOams>> {
    ams(unit).oams.borrow().clone()
}

fn follower(unit: &AfcUnit) -> Option<Rc<FollowerController>> {
    ams(unit).follower.borrow().clone()
}

fn monitor(unit: &AfcUnit) -> Option<Rc<OamsMonitor>> {
    ams(unit).monitor.borrow().clone()
}

fn pause(unit: &AfcUnit, delay: f64) -> KResult<()> {
    unit.afc.reactor.pause(unit.afc.reactor.monotonic() + delay)?;
    Ok(())
}

/// try: register ... except Exception: pass
fn ignore_err<T>(r: KResult<T>) {
    let _ = r;
}

/// _resolve_lane_name_from_afc: the lane `<unit>:<spool+1>` of the unit on this OAMS
pub fn resolve_lane_name_from_afc(afc: &Afc, oams_name: &str, spool_index: i64) -> Option<String> {
    let target_slot = spool_index + 1;
    for unit in afc.units.borrow().values() {
        let UnitKind::OpenAms(a) = &unit.kind else { continue };
        if a.oams_name != oams_name {
            continue;
        }
        for (lane_name, lane) in unit.lanes.borrow().iter() {
            if let Some((_, slot)) = lane.unit.split_once(':') {
                if super::utils::py_int_of(&PyVal::Str(slot.to_string())).ok() == Some(target_slot) {
                    return Some(lane_name.clone());
                }
            }
        }
    }
    None
}

/// [AFC_OpenAMS <name>]
pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let unit = AfcUnit::new(config, |config, settings: &UnitSettings| {
        let _ = settings;
        let type_name = config.get("type", UNIT_TYPE)?;
        let oams_name = config.get("oams", "oams1")?;
        let auto_spoolman_create = config.getboolean("auto_spoolman_create", false)?;
        let stuck_spool_auto_recovery = config.getboolean("stuck_spool_auto_recovery", false)?;
        let stuck_spool_load_grace = config.getfloat("stuck_spool_load_grace", 8.0, Lim::new().min(0.0))?;
        let stuck_spool_pressure_threshold = config.getfloat("stuck_spool_pressure_threshold", 0.08, Lim::new().min(0.0))?;
        let clog_sensitivity = config.get("clog_sensitivity", "medium")?.to_lowercase();
        let engagement_length = config.getfloat("engagement_length", 20.0, Lim::new().min(1.0))?;
        let engagement_speed = config.getfloat("engagement_speed", 300.0, Lim::new().min(10.0))?;
        let defer_engagement = config.getboolean("defer_engagement", false)?;
        let kind = OpenAmsUnit {
            oams_name,
            auto_spoolman_create,
            stuck_spool_auto_recovery,
            stuck_spool_load_grace,
            stuck_spool_pressure_threshold,
            clog_sensitivity,
            engagement_length,
            engagement_speed,
            defer_engagement,
            oams: RefCell::new(None),
            follower: RefCell::new(None),
            monitor: RefCell::new(None),
            spool_map: RefCell::new(IndexMap::default()),
            last_f1s: RefCell::new([None; 4]),
            last_hub: RefCell::new([None; 4]),
            poll_timer: RefCell::new(None),
            operation_active: Cell::new(false),
            prev_states_stale: Cell::new(false),
            hub_load_suppressed: RefCell::new(HashSet::new()),
            td1_last_capture_time: Cell::new(None),
            runout_detected: RefCell::new(HashSet::new()),
            runout_empty: RefCell::new(HashSet::new()),
        };
        Ok((UnitKind::OpenAms(Box::new(kind)), type_name))
    })?;
    unit.stepperless_drive.set(true);
    let gcode = unit.afc.gcode.clone();
    let n = unit.name.clone();
    let cmds: [(&str, fn(&Rc<AfcUnit>, &GCodeCommand) -> KResult<()>, &str); 4] = [
        ("AFC_OAMS_CALIBRATE_PTFE", cmd_afc_oams_calibrate_ptfe, CMD_AFC_OAMS_CALIBRATE_PTFE_HELP),
        ("AFC_OAMS_CALIBRATE_HUB_HES", cmd_afc_oams_calibrate_hub_hes, CMD_AFC_OAMS_CALIBRATE_HUB_HES_HELP),
        ("AFC_OAMS_CALIBRATE_HUB_HES_ALL", cmd_afc_oams_calibrate_hub_hes_all, CMD_AFC_OAMS_CALIBRATE_HUB_HES_ALL_HELP),
        ("AFC_OAMS_CLEAR_ERRORS", cmd_afc_oams_clear_errors, CMD_AFC_OAMS_CLEAR_ERRORS_HELP),
    ];
    for (cmd, f, help) in cmds {
        let u = unit.clone();
        gcode.register_mux_command(cmd, "UNIT", Some(&n), Rc::new(move |g| f(&u, g)), Some(help))?;
    }
    // Several units share the gcode namespace: the first one registers it
    let u = unit.clone();
    ignore_err(gcode.register_command("_AFC_OAMS_STUCK_RECOVERY", Some(Rc::new(move |g| cmd_stuck_spool_recovery(&u, g))), false, Some(CMD_STUCK_RECOVERY_HELP)));
    Ok(unit)
}

// ── lifecycle ───────────────────────────────────────────────────────

/// handle_connect (after the base unit's): logos, the spool map, lane state
pub fn unit_handle_connect(unit: &AfcUnit) {
    let a = ams(unit);
    let lanes: Vec<(String, Rc<AfcLane>)> = unit.lanes.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect();
    *unit.logo.borrow_mut() = super::ace::ams_box_logo("OpenAMS", lanes.len(), &unit.name);
    *unit.logo_error.borrow_mut() = super::ace::ams_box_logo_error("OpenAMS", lanes.len(), &unit.name);
    for (lane_name, lane) in &lanes {
        let slot = (lane.index - 1).max(0);
        a.spool_map.borrow_mut().insert(lane_name.clone(), slot);
        // (no lane has engagement_length / engagement_speed: no overrides)
        // Virtual sensors driven by the hardware poll, false until ready
        lane.prep_state.set(false);
        lane.load_state_raw.set(false);
        lane.loaded_to_hub.set(false);
        lane.set_status(LaneState::None);
        a.runout_detected.borrow_mut().remove(lane_name);
    }
}

/// _init_follower_and_monitor
fn init_follower_and_monitor(unit: &AfcUnit) {
    let a = ams(unit);
    let Some(oams) = hw(unit) else { return };
    let fps_obj = unit.lanes.borrow().values().find_map(|l| l.buffer_obj().filter(|b| b.fps.is_some()));
    *a.follower.borrow_mut() = Some(Rc::new_cyclic(|me| FollowerController {
        oams_name: a.oams_name.clone(),
        oams: oams.clone(),
        reactor: unit.afc.reactor.clone(),
        afc: unit.afc.clone(),
        last_state: RefCell::new(HashMap::new()),
        queue: RefCell::new(HashMap::new()),
        in_flight: RefCell::new(HashMap::new()),
        poll_timers: RefCell::new(HashMap::new()),
        led_error_state: RefCell::new(HashMap::new()),
        me: me.clone(),
    }));
    if let Some(fps) = fps_obj {
        let clog = clog_sensitivity(&a.clog_sensitivity);
        *a.monitor.borrow_mut() = Some(Rc::new(OamsMonitor {
            fps_name: fps.name.clone(),
            fps,
            unit: Rc::downgrade(&unit.rc()),
            reactor: unit.afc.reactor.clone(),
            enable_clog: clog.is_some(),
            state: RefCell::new(FpsState::new()),
            timer: RefCell::new(None),
            running: Cell::new(false),
            oams: RefCell::new(None),
            stuck_pressure_low: a.stuck_spool_pressure_threshold,
            stuck_load_grace: a.stuck_spool_load_grace,
        }));
    }
}

/// handle_ready (after the base unit's lane sort)
pub fn unit_handle_ready(unit: &AfcUnit) -> KResult<()> {
    let a = ams(unit);
    let oams = oams::lookup(&unit.printer, &a.oams_name);
    *a.oams.borrow_mut() = oams.clone();
    if oams.is_none() {
        unit.afc.logger.warning(
            &AFC_OPENAMS_AFCAMS_HANDLE_READY,
            &format!("OpenAMS hardware '[AFC_OAMS {}]' not found for '{}'. Sensor state will not update.", a.oams_name, unit.name),
        );
        return Ok(());
    }
    init_follower_and_monitor(unit);
    sync_lanes_from_hardware(unit);
    let w = Rc::downgrade(&unit.rc());
    let t = unit.afc.reactor.register_timer(
        Box::new(move |et| match w.upgrade() {
            Some(u) => poll_oams_sensors(&u, et).unwrap_or_else(|e| {
                kaliru::logging::exception_err("OpenAMS sensor poll", &e);
                NEVER
            }),
            None => NEVER,
        }),
        unit.afc.reactor.monotonic() + 1.0,
    );
    *a.poll_timer.borrow_mut() = Some(t);
    Ok(())
}

fn sensor_values(oams: &AfcOams) -> ([i64; 4], [i64; 4]) {
    (*oams.f1s_hes_value.borrow(), *oams.hub_hes_value.borrow())
}

/// _sync_lanes_from_hardware: F1S -> prep_state (and the latched
/// loaded_to_hub), hub HES -> raw_load_state
fn sync_lanes_from_hardware(unit: &AfcUnit) {
    let a = ams(unit);
    let Some(oams) = hw(unit) else { return };
    let (f1s, hub) = sensor_values(&oams);
    for (lane_name, lane) in unit.lanes.borrow().iter() {
        let Some(slot) = a.spool_map.borrow().get(lane_name).copied().filter(|s| *s >= 0) else { continue };
        let i = slot as usize;
        let f1s_present = f1s.get(i).is_some_and(|v| *v != 0);
        let hub_present = hub.get(i).is_some_and(|v| *v != 0);
        lane.prep_state.set(f1s_present);
        lane.load_state_raw.set(hub_present);
        lane.loaded_to_hub.set(f1s_present);
        if i < 4 {
            a.last_f1s.borrow_mut()[i] = Some(f1s_present);
            a.last_hub.borrow_mut()[i] = Some(hub_present);
        }
    }
}

/// _poll_oams_sensors: every 2s, sensor changes into lane state
fn poll_oams_sensors(unit: &AfcUnit, eventtime: f64) -> KResult<f64> {
    let a = ams(unit);
    let Some(oams) = hw(unit) else { return Ok(NEVER) };
    if a.operation_active.get() {
        return Ok(eventtime + 2.0);
    }
    let resync_prev = a.prev_states_stale.replace(false);
    let lanes: Vec<(String, Rc<AfcLane>)> = unit.lanes.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect();
    for (lane_name, lane) in lanes {
        let Some(slot) = a.spool_map.borrow().get(&lane_name).copied().filter(|s| *s >= 0) else { continue };
        let i = slot as usize;
        if i >= 4 {
            continue;
        }
        // F1S -> prep_state; a removed spool can no longer be at the hub
        let new_f1s = oams.f1s_hes_value.borrow()[i] != 0;
        lane.prep_state.set(new_f1s);
        if !new_f1s {
            lane.loaded_to_hub.set(false);
        }
        if resync_prev {
            a.last_f1s.borrow_mut()[i] = Some(new_f1s);
        } else {
            let old_f1s = a.last_f1s.borrow()[i];
            if old_f1s.is_some_and(|o| o != new_f1s) {
                if should_block_sensor_for_runout(unit, &lane, new_f1s)? {
                    a.last_f1s.borrow_mut()[i] = Some(new_f1s);
                    continue;
                }
                if a.hub_load_suppressed.borrow_mut().remove(&lane_name) {
                    lane.set_load_suppressed(true);
                }
                lane.handle_load_runout(eventtime, new_f1s)?;
            }
            a.last_f1s.borrow_mut()[i] = Some(new_f1s);
        }
        // Hub HES -> raw_load_state (the virtual hub's any())
        let new_hub = oams.hub_hes_value.borrow()[i] != 0;
        lane.load_state_raw.set(new_hub);
        a.last_hub.borrow_mut()[i] = Some(new_hub);
    }
    Ok(eventtime + 2.0)
}

// ── engagement verification ─────────────────────────────────────────

fn get_engagement_params(unit: &AfcUnit) -> (f64, f64) {
    let a = ams(unit);
    (a.engagement_length, a.engagement_speed)
}

/// _verify_engagement: extrude the engagement length and see the encoder move
fn verify_engagement(unit: &AfcUnit, cur_lane: &AfcLane) -> KResult<bool> {
    let c = &AFC_OPENAMS_AFCAMS__VERIFY_ENGAGEMENT;
    let (engagement_length, engagement_speed) = get_engagement_params(unit);
    unit.afc.logger.info(c, &format!("Verifying engagement for {}: {:.1}mm at {:.0}mm/min", cur_lane.name, engagement_length, engagement_speed));
    let mon = monitor(unit);
    if let Some(m) = &mon {
        m.notify_engagement_start();
    }
    let r = (|| -> KResult<bool> {
        let oams = hw(unit);
        // The follower feeds forward through the buffer so the encoder moves
        if let Some(f) = follower(unit) {
            f.enable_follower(oams.as_ref(), 1, "engagement verification", true);
        }
        let encoder_before = oams.as_ref().map_or(0, |o| o.encoder_clicks.get());
        pause(unit, 1.5)?;
        let prime_length = f64::min(5.0, engagement_length);
        let remaining = f64::max(0.0, engagement_length - prime_length);
        oams_extrude(unit, prime_length, engagement_speed)?;
        if remaining > 0.0 {
            oams_extrude(unit, remaining, engagement_speed)?;
        }
        let encoder_after = oams.as_ref().map_or(0, |o| o.encoder_clicks.get());
        let encoder_delta = (encoder_after - encoder_before).abs();
        let min_movement = 2;
        if encoder_delta >= min_movement {
            unit.afc.logger.info(c, &format!("Engagement verified: encoder moved {} clicks", encoder_delta));
            return Ok(true);
        }
        pause(unit, 0.3)?;
        let encoder_retry = oams.as_ref().map_or(0, |o| (o.encoder_clicks.get() - encoder_before).abs());
        if encoder_retry >= min_movement {
            unit.afc.logger.info(c, &format!("Engagement verified on retry: encoder moved {} clicks", encoder_retry));
            return Ok(true);
        }
        unit.afc.logger.error(c, &format!("Engagement verification failed: encoder moved only {} clicks", encoder_delta));
        Ok(false)
    })();
    if let Some(m) = &mon {
        m.notify_engagement_end();
    }
    r
}

/// _oams_extrude: an extruder move by gcode
fn oams_extrude(unit: &AfcUnit, length: f64, speed: f64) -> KResult<()> {
    unit.afc.gcode.run_script_from_command(&format!("G92 E0\nG1 E{:.3} F{:.0}\nM400", length, speed))
}

/// _advance_tool_stn_to_nozzle: sensor -> nozzle, less what engagement fed
fn advance_tool_stn_to_nozzle(unit: &AfcUnit, cur_lane: &AfcLane, already_advanced: f64) -> KResult<()> {
    let ext = cur_lane.extruder_obj()?;
    let tool_stn = ext.tool_stn.get();
    let remaining = tool_stn - already_advanced;
    if remaining <= 0.0 {
        return Ok(());
    }
    let speed = ext.tool_load_speed.get() * 60.0;
    unit.afc.logger.info(
        &AFC_OPENAMS_AFCAMS__ADVANCE_TOOL_STN_TO_NOZZLE,
        &format!("OAMS load: advancing {:.1}mm to nozzle (tool_stn {:.0} - {:.0} already fed) for {}", remaining, tool_stn, already_advanced, cur_lane.name),
    );
    oams_extrude(unit, remaining, speed)?;
    unit.afc.delta_time.log_with_time(&AFC_OPENAMS_AFCAMS__ADVANCE_TOOL_STN_TO_NOZZLE, "OAMS load: tool_stn to nozzle", true);
    Ok(())
}

// ── stuck spool / clog ──────────────────────────────────────────────

fn monitor_spool_idx(unit: &AfcUnit) -> Option<i64> {
    monitor(unit).and_then(|m| m.state.borrow().current_spool_idx)
}

/// _on_stuck_spool_detected: error LED, then auto-recovery or a pause
fn on_stuck_spool_detected(unit: &AfcUnit, fps_name: Option<&str>, message: Option<&str>) -> KResult<()> {
    let a = ams(unit);
    let mut msg = message.unwrap_or("OpenAMS stuck spool detected").to_string();
    if let Some(f) = fps_name.filter(|f| !f.is_empty() && !msg.contains(f)) {
        msg = format!("{} (FPS: {})", msg, f);
    }
    let mon = monitor(unit);
    let lane_name = mon.as_ref().and_then(|m| m.state.borrow().current_lane.clone());
    if let (Some(_), Some(f), Some(idx)) = (&mon, follower(unit), monitor_spool_idx(unit)) {
        f.set_led_error_if_changed(&a.oams_name, idx, 1, "stuck spool detected");
    }
    if a.stuck_spool_auto_recovery && lane_name.as_deref().is_some_and(|l| !l.is_empty()) {
        on_stuck_spool_recovery_needed(unit, fps_name, lane_name.as_deref().unwrap_or(""))
    } else {
        if !msg.to_lowercase().contains("paused") {
            msg += ". Print paused — check spool and resume.";
        }
        unit.afc.error.afc_error(&AFC_OPENAMS_AFCAMS__ON_STUCK_SPOOL_DETECTED, &msg, true, None)
    }
}

/// _on_stuck_spool_recovery_needed: the recovery runs as gcode
fn on_stuck_spool_recovery_needed(unit: &AfcUnit, fps_name: Option<&str>, lane_name: &str) -> KResult<()> {
    let c = &AFC_OPENAMS_AFCAMS__ON_STUCK_SPOOL_RECOVERY_NEEDED;
    unit.afc.logger.info(c, &format!("Stuck spool recovery scheduled: fps={}, lane={}", fps_name.unwrap_or("None"), lane_name));
    if let Err(e) = unit.afc.gcode.run_script_from_command(&format!("_AFC_OAMS_STUCK_RECOVERY LANE={} FPS={}", lane_name, fps_name.unwrap_or(""))) {
        unit.afc.logger.error(c, &format!("Failed to schedule stuck spool recovery for {}: {}", lane_name, e.message()));
        if let Err(pe) = unit.afc.gcode.run_script_from_command("PAUSE") {
            unit.afc.logger.error(c, &format!("Failed to pause after recovery-schedule failure: {}", pe.message()));
        }
    }
    Ok(())
}

/// _cmd_stuck_spool_recovery: unload + reload + resume
fn cmd_stuck_spool_recovery(unit: &Rc<AfcUnit>, gcmd: &GCodeCommand) -> KResult<()> {
    let c = &AFC_OPENAMS_AFCAMS__CMD_STUCK_SPOOL_RECOVERY;
    let afc = &unit.afc;
    let lane_name = gcmd.get_opt("LANE");
    let fps_name = gcmd.get_opt("FPS");
    let lane = lane_name.as_deref().filter(|l| !l.is_empty()).and_then(|l| afc.lane(l));
    let Some(lane) = lane else {
        afc.logger.error(c, &format!("Stuck spool recovery: lane '{}' not found, pausing", lane_name.as_deref().unwrap_or("None")));
        return stuck_spool_recovery_fallback(unit, fps_name.as_deref(), lane_name.as_deref(), "lane not found");
    };
    let lane_name = lane.name.clone();
    afc.logger.info(c, &format!("Stuck spool auto-recovery starting: unload then reload for {}", lane_name));
    afc.save_pos()?;
    let pause_resume = afc.error.pause_resume().ok();
    if let Some(pr) = &pause_resume {
        if !afc.function.is_paused()? {
            pr.send_pause_command()?;
        }
    }
    let zhop = (|| -> KResult<()> {
        let z = afc.gcode_move()?.last_position.borrow()[2];
        afc.move_z_pos(z + afc.z_hop, "stuck_spool_recovery_zhop", false)?;
        Ok(())
    })();
    if let Err(e) = zhop {
        afc.logger.warning(c, &format!("Stuck spool recovery: Z-hop failed: {}", e.message()));
    }
    let r = (|| -> KResult<()> {
        afc.logger.info(c, &format!("Stuck spool recovery: unloading {}", lane_name));
        if !afc.tool_unload(&lane, true, false)? {
            return Err(KError::Py("RuntimeError".into(), format!("TOOL_UNLOAD returned False for {}", lane_name)));
        }
        // Filament inertia can re-trigger the hub after the retract: wait
        // for it to settle before TOOL_LOAD checks it
        const HUB_CLEAR_TIMEOUT: f64 = 5.0;
        const HUB_POLL_INTERVAL: f64 = 0.1;
        if let Some(hub) = lane.hub_obj_opt() {
            if hub.state() {
                let mut waited = 0.0;
                while hub.state() && waited < HUB_CLEAR_TIMEOUT {
                    pause(unit, HUB_POLL_INTERVAL)?;
                    waited += HUB_POLL_INTERVAL;
                }
                if hub.state() {
                    afc.logger.warning(c, &format!("Stuck spool recovery: hub still active {:.1}s after unload of {} - proceeding with TOOL_LOAD anyway", waited, lane_name));
                }
            }
        }
        afc.logger.info(c, &format!("Stuck spool recovery: reloading {}", lane_name));
        if !afc.tool_load(&lane, None, true)? {
            return Err(KError::Py("RuntimeError".into(), format!("TOOL_LOAD returned False for {}", lane_name)));
        }
        Ok(())
    })();
    if let Err(e) = r {
        afc.logger.error(c, &format!("Stuck spool auto-recovery FAILED for {}: {}", lane_name, e.message()));
        return stuck_spool_recovery_fallback(unit, fps_name.as_deref(), Some(&lane_name), e.message());
    }
    afc.logger.info(c, &format!("Stuck spool auto-recovery SUCCEEDED for {}, resuming print", lane_name));
    stuck_spool_recovery_clear_oams_state(unit);
    let r = (|| -> KResult<()> {
        afc.error.reset_failure()?;
        // AFC_RESUME only takes its full path while paused
        if !afc.function.is_paused()? {
            if let Some(pr) = &pause_resume {
                pr.is_paused.set(true);
            }
        }
        afc.gcode.run_script_from_command("AFC_RESUME")
    })();
    if let Err(e) = r {
        afc.logger.error(c, &format!("Stuck spool recovery: AFC_RESUME failed: {}", e.message()));
    }
    Ok(())
}

fn stuck_spool_recovery_fallback(unit: &AfcUnit, fps_name: Option<&str>, lane_name: Option<&str>, reason: &str) -> KResult<()> {
    let c = &AFC_OPENAMS_AFCAMS__STUCK_SPOOL_RECOVERY_FALLBACK;
    let who = lane_name.filter(|l| !l.is_empty()).or(fps_name).unwrap_or("None");
    let msg = format!(
        "Stuck spool auto-recovery failed for {}: {}.\nPlease manually correct the issue, then run:\n  SET_LANE_LOADED LANE={}\nfollowed by AFC_OAMS_CLEAR_ERRORS",
        who,
        reason,
        lane_name.unwrap_or("None")
    );
    unit.afc.logger.error(c, &msg);
    if let Err(e) = unit.afc.error.afc_error(c, &msg, true, None) {
        unit.afc.logger.error(c, &format!("Failed to raise AFC error for stuck fallback: {}", e.message()));
        let _ = unit.afc.gcode.run_script_from_command("PAUSE");
    }
    Ok(())
}

fn stuck_spool_recovery_clear_oams_state(unit: &AfcUnit) {
    let a = ams(unit);
    if let (Some(idx), Some(f)) = (monitor_spool_idx(unit), follower(unit)) {
        f.clear_error_led(&a.oams_name, idx, "stuck spool auto-recovery succeeded");
    }
    if let Some(m) = monitor(unit) {
        let mut st = m.state.borrow_mut();
        st.stuck_active = false;
        st.stuck_start_time = None;
    }
}

// ── unit interface ──────────────────────────────────────────────────

pub fn eject_lane(unit: &AfcUnit, lane: &AfcLane) -> KResult<()> {
    unit.afc.logger.info(&AFC_OPENAMS_AFCAMS_EJECT_LANE, &format!("Eject not supported for OpenAMS lane {}. Remove spool physically or use TOOL_UNLOAD.", lane.name));
    Ok(())
}

/// lane_move: the firmware moves filament, not AFC
pub fn lane_move(unit: &AfcUnit, cur_lane: &AfcLane) -> KResult<()> {
    unit.afc.logger.info(&AFC_OPENAMS_AFCAMS_LANE_MOVE, &format!("Lane move not supported for OpenAMS lane {}. OpenAMS firmware controls filament movement.", cur_lane.name));
    Ok(())
}

pub fn system_test(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, assign_tcmd: bool) -> KResult<bool> {
    let a = ams(unit);
    let afc = &unit.afc;
    let mut msg = String::new();
    let mut succeeded = true;
    match hw(unit) {
        None => {
            msg = "<span class=error--text>OAMS NOT CONNECTED</span>".into();
            succeeded = false;
        }
        Some(oams) => {
            let slot = a.spool_map.borrow().get(&cur_lane.name).copied().unwrap_or(-1);
            let (f1s, hub) = sensor_values(&oams);
            let f1s_present = (0..4).contains(&slot) && f1s[slot as usize] != 0;
            let hub_present = (0..4).contains(&slot) && hub[slot as usize] != 0;
            cur_lane.prep_state.set(f1s_present);
            cur_lane.load_state_raw.set(hub_present);
            cur_lane.loaded_to_hub.set(f1s_present);
            if f1s_present {
                unit.lane_loaded(cur_lane, false)?;
                msg += "<span class=success--text>LOCKED</span>";
                cur_lane.set_status(LaneState::Loaded);
                msg += "<span class=success--text> AND LOADED</span>";
                unit.lane_illuminate_spool(cur_lane)?;
                if cur_lane.tool_loaded.get() && cur_lane.extruder_obj()?.lane_loaded.borrow().as_deref() == Some(cur_lane.name.as_str()) {
                    cur_lane.sync_to_extruder(true)?;
                    msg += &cur_lane.extruder_obj()?.prep_on_shuttle_check();
                    if afc.current().as_deref() == Some(cur_lane.name.as_str()) {
                        afc.spool.set_active_spool(&cur_lane.spool_id.borrow().clone())?;
                        unit.lane_tool_loaded(cur_lane)?;
                        cur_lane.set_status(LaneState::Tooled);
                        cur_lane.enable_buffer(false)?;
                    } else {
                        unit.lane_tool_loaded_idle(cur_lane)?;
                    }
                    // Keep the buffer tensioned for filament already in the toolhead
                    if let Some(f) = follower(unit) {
                        f.enable_follower(Some(&oams), 1, "prep tool-loaded", true);
                    }
                    if let Some(m) = monitor(unit) {
                        let spool_index = a.spool_map.borrow().get(&cur_lane.name).copied().unwrap_or(0);
                        m.notify_load_complete(&cur_lane.name, &a.oams_name, spool_index);
                        m.start(&oams);
                    }
                    unit.printer.send_event("afc:tool_loaded", &[cur_lane])?;
                }
            } else {
                if !cur_lane.remember_spool() {
                    afc.spool.clear_values(cur_lane)?;
                }
                cur_lane.unit_obj()?.lane_not_ready(cur_lane)?;
                msg += "EMPTY READY FOR SPOOL";
            }
        }
    }
    if assign_tcmd {
        afc.function.tcmd_assign(cur_lane)?;
    }
    cur_lane.do_enable(false)?;
    afc.logger.info(&AFC_OPENAMS_AFCAMS_SYSTEM_TEST, &format!("{} tool cmd: {:<3} {}", cur_lane.name, cur_lane.map_to_string(), msg));
    cur_lane.set_afc_prep_done();
    Ok(succeeded)
}

pub fn calibrate_lane(unit: &AfcUnit, cur_lane: &AfcLane) -> KResult<(bool, String, f64)> {
    if hw(unit).is_none() {
        return Ok((false, "OpenAMS hardware not available".into(), 0.0));
    }
    let spool_index = get_openams_spool_index(unit, cur_lane);
    if !cur_lane.load_state() {
        return Ok((false, format!("{} not loaded, load before calibration", cur_lane.name), 0.0));
    }
    unit.afc.logger.info(&AFC_OPENAMS_AFCAMS_CALIBRATE_LANE, &format!("Running HUB HES calibration for {}", cur_lane.name));
    if calibrate_hub_hes_spool(unit, spool_index) {
        return Ok((true, "calibration_lane".into(), 0.0));
    }
    Ok((false, format!("HUB HES calibration failed for {}", cur_lane.name), 0.0))
}

pub const CALIBRATION_LANE_MESSAGE: &str = "\nHUB HES calibration complete for lanes: {lanes}\n";

pub fn calibrate_bowden(unit: &AfcUnit, cur_lane: &AfcLane) -> KResult<(bool, String, f64)> {
    if hw(unit).is_none() {
        return Ok((false, "OpenAMS hardware not available".into(), 0.0));
    }
    let spool_index = get_openams_spool_index(unit, cur_lane);
    if !cur_lane.load_state() {
        return Ok((false, format!("{} not loaded, load before calibration", cur_lane.name), 0.0));
    }
    unit.afc.logger.info(&AFC_OPENAMS_AFCAMS_CALIBRATE_BOWDEN, &format!("Running PTFE calibration for {}", cur_lane.name));
    let oams_idx = get_oams_index(unit);
    match unit.afc.gcode.run_script_from_command(&format!("OAMS_CALIBRATE_PTFE_LENGTH OAMS={} SPOOL={}", oams_idx, spool_index)) {
        Ok(()) => Ok((true, format!("PTFE calibration complete for {}", cur_lane.name), 0.0)),
        Err(e) => Ok((false, format!("PTFE calibration failed for {}: {}", cur_lane.name, e.message()), 0.0)),
    }
}

/// _toolhead_sensor_triggered (no U1 motion sensor here: the pre-sensor state)
fn toolhead_sensor_triggered(cur_lane: &AfcLane) -> KResult<bool> {
    cur_lane.get_toolhead_pre_sensor_state()
}

/// unit_load_lane: the OAMS transport; on total failure, pause when printing
pub fn unit_load_lane(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, _cur_extruder: &AfcExtruder) -> KResult<bool> {
    if !oams_load_sequence(unit, cur_lane)? {
        let afc = &unit.afc;
        afc.error.handle_lane_failure(&AFC_OPENAMS_AFCAMS_UNIT_LOAD_LANE, cur_lane, &format!("OpenAMS load failed for {}", cur_lane.name), afc.function.in_print()?)?;
        return Ok(false);
    }
    Ok(true)
}

pub fn unit_unload_lane(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, cur_extruder: &AfcExtruder) -> KResult<bool> {
    let afc = &unit.afc;
    afc.move_e_pos(-2.0, cur_extruder.tool_unload_speed.get(), "Quick Pull", false)?;
    cur_lane.set_status(LaneState::ToolUnloading);
    cur_lane.disable_buffer()?;
    cur_lane.sync_to_extruder(true)?;
    cur_lane.select_lane()?;
    afc.do_tool_cut_tip_form(cur_lane, cur_extruder)?;
    if !oams_unload_sequence(unit, cur_lane)? {
        return Ok(false);
    }
    if let Some(m) = &afc.post_unload_macro {
        afc.gcode.run_script_from_command(m)?;
    }
    cur_lane.set_tool_unloaded(true)?;
    cur_lane.set_status(LaneState::None);
    afc.save_vars()?;
    Ok(true)
}

fn oams_load_sequence(unit: &AfcUnit, cur_lane: &Rc<AfcLane>) -> KResult<bool> {
    let a = ams(unit);
    a.operation_active.set(true);
    let r = (|| -> KResult<bool> {
        // This lane is being loaded on purpose
        a.hub_load_suppressed.borrow_mut().remove(&cur_lane.name);
        if !oams_load(unit, cur_lane, 3)? {
            return Ok(false);
        }
        cur_lane.loaded_to_hub.set(true);
        cur_lane.set_status(LaneState::ToolLoaded);
        unit.afc.save_vars()?;
        Ok(true)
    })();
    a.operation_active.set(false);
    a.prev_states_stale.set(true);
    r
}

fn oams_unload_sequence(unit: &AfcUnit, cur_lane: &Rc<AfcLane>) -> KResult<bool> {
    let a = ams(unit);
    a.operation_active.set(true);
    let r = (|| -> KResult<bool> {
        let afc = &unit.afc;
        if !oams_unload(unit, cur_lane)? {
            afc.error.handle_lane_failure(&AFC_OPENAMS_AFCAMS__OAMS_UNLOAD_INNER, cur_lane, &format!("OAMS unload failed for {}", cur_lane.name), afc.function.in_print()?)?;
            return Ok(false);
        }
        // (loaded_to_hub was latched from the live F1S by the unload)
        unit.lane_tool_unloaded(cur_lane)?;
        a.hub_load_suppressed.borrow_mut().insert(cur_lane.name.clone());
        afc.delta_time.log_with_time(&AFC_OPENAMS_AFCAMS__OAMS_UNLOAD_INNER, "OAMS unload complete", true);
        Ok(true)
    })();
    a.operation_active.set(false);
    a.prev_states_stale.set(true);
    r
}

/// lane_unloading: the base LEDs, then stop the follower (prepare_unload)
pub fn lane_unloading(unit: &AfcUnit) {
    prepare_unload(unit);
}

/// prepare_unload: only stop the forward-feeding follower; the hardware
/// unload drives it in reverse itself
fn prepare_unload(unit: &AfcUnit) {
    if let Some(f) = follower(unit) {
        f.set_follower_state(hw(unit).as_ref(), 0, 0, "stop before unload", true);
    }
}

/// _oams_load: the hardware load with engagement verification and retries
fn oams_load(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, max_retries: i64) -> KResult<bool> {
    let c = &AFC_OPENAMS_AFCAMS__OAMS_LOAD;
    let a = ams(unit);
    let afc = &unit.afc;
    let Some(oams) = hw(unit) else {
        afc.logger.error(c, "OAMS hardware not available");
        return Ok(false);
    };
    let fol = follower(unit);
    let mon = monitor(unit);
    let spool_index = a.spool_map.borrow().get(&cur_lane.name).copied().unwrap_or(0);
    wait_for_idle(unit, 30.0)?;
    // A firmware/host desync leaves the spool loaded in the firmware: the
    // query is answered, a load would not be. Only the toolhead sensor proves
    // a load through to the toolhead (an inserted spool also stages at the hub)
    let hw_spool = match oams.determine_current_spool() {
        Ok(s) => s,
        Err(e) => {
            afc.logger.debug(c, &format!("Could not query OAMS current spool: {}", e.message()));
            None
        }
    };
    let mut toolhead_loaded = false;
    if hw_spool == Some(spool_index) {
        toolhead_loaded = toolhead_sensor_triggered(cur_lane).unwrap_or(false);
        if !toolhead_loaded {
            afc.logger.info(
                c,
                &format!(
                    "OAMS reports spool {} present in hardware but the toolhead sensor does not see filament for {}; treating as not loaded and running a real load",
                    spool_index, cur_lane.name
                ),
            );
        }
    }
    if hw_spool == Some(spool_index) && toolhead_loaded {
        afc.logger.info(
            c,
            &format!(
                "OAMS spool {} already loaded to the toolhead (firmware current_spool={}, toolhead sensor triggered); skipping redundant load for {}",
                spool_index, spool_index, cur_lane.name
            ),
        );
        oams.current_spool.set(Some(spool_index));
        if let Some(f) = &fol {
            f.enable_follower(Some(&oams), 1, "already loaded", true);
        }
        if let Some(m) = &mon {
            m.notify_load_complete(&cur_lane.name, &a.oams_name, spool_index);
            m.start(&oams);
        }
        cur_lane.loaded_to_hub.set(true);
        return Ok(true);
    }
    if let Some(m) = &mon {
        m.stop();
    }
    // The FPS advance latch keeps the toolhead sensor triggered through a
    // brief pressure drop after the feed
    if let Some(b) = cur_lane.buffer_obj() {
        if let Some(f) = &b.fps {
            f.enable_advance_latch();
        }
    }
    // A running follower makes the MCU reject a load
    if let Some(f) = &fol {
        f.set_follower_state(Some(&oams), 0, 0, "stop before load", true);
        pause(unit, 0.5)?;
    }
    if let Some(f) = &fol {
        f.enable_follower(Some(&oams), 1, "before load", true);
        pause(unit, 0.5)?;
    }
    for attempt in 0..max_retries {
        let r = (|| -> KResult<Option<bool>> {
            wait_for_idle(unit, 30.0)?;
            let (success, msg) = oams.load_spool_with_retry(spool_index, None)?;
            if !success {
                afc.logger.error(c, &format!("OAMS load attempt {} failed: {}", attempt + 1, msg));
                return Ok(None);
            }
            let engaged = if a.defer_engagement { true } else { verify_engagement(unit, cur_lane)? };
            if engaged {
                // A no-poop load ends at the nozzle: push what engagement did not
                let already = if a.defer_engagement { 0.0 } else { get_engagement_params(unit).0 };
                advance_tool_stn_to_nozzle(unit, cur_lane, already)?;
                if let Some(f) = &fol {
                    f.enable_follower(Some(&oams), 1, "load complete", true);
                }
                if let Some(m) = &mon {
                    m.notify_load_complete(&cur_lane.name, &a.oams_name, spool_index);
                    m.start(&oams);
                }
                cur_lane.loaded_to_hub.set(true);
                return Ok(Some(true));
            }
            afc.logger.info(c, &format!("Engagement failed attempt {}, cleaning up", attempt + 1));
            // Stop (not reverse) the follower: the extruder still grips the filament
            if let Some(f) = &fol {
                f.set_follower_state(Some(&oams), 0, 0, "engagement cleanup — stop follower", true);
            }
            let ext = cur_lane.extruder_obj()?;
            let retract_dist = ext.tool_stn_unload.get();
            if retract_dist > 0.0 {
                oams_extrude(unit, -(retract_dist + 10.0), ext.tool_unload_speed.get() * 60.0)?;
            }
            if let Some(f) = &fol {
                f.set_follower_state(Some(&oams), 0, 0, "stop before cleanup", true);
                pause(unit, 0.5)?;
            }
            wait_for_idle(unit, 30.0)?;
            oams.abort_current_action(oams::opcode::ERROR_KLIPPER_CALL, true)?;
            wait_for_idle(unit, 30.0)?;
            oams.unload_spool_with_retry(None)?;
            wait_for_idle(unit, 30.0)?;
            // Clear a stuck "busy" before the next attempt
            if let Err(e) = oams.clear_errors() {
                afc.logger.debug(c, &format!("clear_errors after failed load: {}", e.message()));
            }
            wait_for_idle(unit, 30.0)?;
            if attempt < max_retries - 1 {
                pause(unit, 2.0)?;
                if let Some(f) = &fol {
                    f.enable_follower(Some(&oams), 1, "before retry", true);
                    pause(unit, 0.5)?;
                }
            }
            Ok(None)
        })();
        match r {
            Ok(Some(done)) => return Ok(done),
            Ok(None) => {}
            Err(e) => afc.logger.error(c, &format!("OAMS load error attempt {}: {}", attempt + 1, e.message())),
        }
    }
    afc.logger.error(c, &format!("OAMS load failed after {} attempts", max_retries));
    Ok(false)
}

/// _oams_unload: follower reverse, extruder retract, the hardware unload,
/// then the hub HES settle and the post-unload lane state
fn oams_unload(unit: &AfcUnit, cur_lane: &Rc<AfcLane>) -> KResult<bool> {
    let c = &AFC_OPENAMS_AFCAMS__OAMS_UNLOAD;
    let a = ams(unit);
    let afc = &unit.afc;
    let Some(oams) = hw(unit) else {
        afc.logger.error(c, "OAMS hardware not available");
        return Ok(false);
    };
    let fol = follower(unit);
    let mon = monitor(unit);
    let spool_index = a.spool_map.borrow().get(&cur_lane.name).copied().unwrap_or(0);
    wait_for_idle(unit, 30.0)?;
    if let Some(m) = &mon {
        m.stop();
    }
    // The follower reverses with the retract, or it shoves filament back
    // into the gears
    if let Some(f) = &fol {
        f.enable_follower(Some(&oams), 0, "reverse before unload retract", true);
        if let Err(e) = pause(unit, 0.5) {
            afc.logger.warning(c, &format!("OAMS: follower reverse before unload retract failed: {}", e.message()));
        }
    }
    // Clear the extruder gears before the hardware rewinds
    let ext = cur_lane.extruder_obj()?;
    let retract_dist = ext.tool_stn_unload.get();
    if retract_dist > 0.0 {
        let unload_length = retract_dist + 10.0;
        afc.logger.debug(c, &format!("Retracting {:.1}mm from extruder before OAMS unload", unload_length));
        if let Err(e) = oams_extrude(unit, -unload_length, ext.tool_unload_speed.get() * 60.0) {
            afc.logger.warning(c, &format!("Extruder retract before OAMS unload failed: {}", e.message()));
        }
    }
    // A runout-empty spool cannot be rewound
    let runout_empty = a.runout_empty.borrow_mut().remove(&cur_lane.name);
    let r = (|| -> KResult<bool> {
        // The extruder helps pull back while the spool motor rewinds
        let concurrent_retract = if retract_dist > 0.0 { retract_dist } else { 20.0 };
        let concurrent_speed = ext.tool_unload_speed.get() * 60.0;
        let cr = (|| -> KResult<()> {
            afc.gcode.run_script_from_command("M83")?;
            afc.gcode.run_script_from_command(&format!("G1 E-{:.2} F{}", concurrent_retract, concurrent_speed.trunc() as i64))
        })();
        if let Err(e) = cr {
            afc.logger.warning(c, &format!("Concurrent retract failed: {}", e.message()));
        }
        let success = if runout_empty {
            afc.logger.info(c, &format!("Skipping OAMS hardware unload for {} — spool empty from runout", cur_lane.name));
            true
        } else {
            // State-desync guard: nothing loaded in the firmware, so an
            // unload would never be acked
            let hw_spool = match oams.determine_current_spool() {
                Ok(s) => s,
                Err(e) => {
                    afc.logger.debug(c, &format!("Could not query OAMS current spool: {}", e.message()));
                    None
                }
            };
            if hw_spool.is_none() {
                afc.logger.info(c, &format!("OAMS reports no spool loaded; skipping redundant hardware unload for {}", cur_lane.name));
                oams.current_spool.set(None);
                true
            } else {
                oams.unload_spool_with_retry(None)?.0
            }
        };
        wait_for_idle(unit, 30.0)?;
        // Stop the follower so the MCU is idle for the next operation
        if let Some(f) = &fol {
            f.set_follower_state(Some(&oams), 0, 0, "stop after unload", true);
            pause(unit, 0.5)?;
        }
        if let Some(m) = &mon {
            m.notify_unload_complete();
        }
        // The spool stays staged in the bay while F1S sees it
        let f1s = oams.f1s_hes_value.borrow()[spool_index.clamp(0, 3) as usize];
        if (0..4).contains(&spool_index) {
            let f1s_present = f1s != 0;
            cur_lane.prep_state.set(f1s_present);
            cur_lane.loaded_to_hub.set(f1s_present);
        }
        // The hub HES bounces after an unload: the next load's hub check
        // must see it settled clear
        if wait_for_hub_settle(unit, spool_index, 4.0, 0.3)? {
            cur_lane.load_state_raw.set(false);
            if (0..4).contains(&spool_index) {
                a.last_hub.borrow_mut()[spool_index as usize] = Some(false);
            }
        }
        Ok(success)
    })();
    match r {
        Ok(s) => Ok(s),
        Err(e) => {
            afc.logger.error(c, &format!("OAMS unload error: {}", e.message()));
            Ok(false)
        }
    }
}

// ── calibration commands ────────────────────────────────────────────

fn cmd_afc_oams_calibrate_ptfe(unit: &Rc<AfcUnit>, gcmd: &GCodeCommand) -> KResult<()> {
    if hw(unit).is_none() {
        gcmd.respond_info("OAMS hardware not available", true);
        return Ok(());
    }
    let spool = gcmd.get_int("SPOOL", Some(0), Lim::new())?;
    let oams_idx = get_oams_index(unit);
    match unit.afc.gcode.run_script_from_command(&format!("OAMS_CALIBRATE_PTFE_LENGTH OAMS={} SPOOL={}", oams_idx, spool)) {
        Ok(()) => gcmd.respond_info("PTFE calibration complete — check OAMS config for new value", true),
        Err(e) => gcmd.respond_info(&format!("PTFE calibration failed: {}", e.message()), true),
    }
    Ok(())
}

fn cmd_afc_oams_calibrate_hub_hes(unit: &Rc<AfcUnit>, gcmd: &GCodeCommand) -> KResult<()> {
    if hw(unit).is_none() {
        gcmd.respond_info("OAMS hardware not available", true);
        return Ok(());
    }
    let Some(spool) = gcmd.get_int_opt("SPOOL", Lim::new())? else {
        gcmd.respond_info("Usage: AFC_OAMS_CALIBRATE_HUB_HES SPOOL=<index>", true);
        return Ok(());
    };
    let success = calibrate_hub_hes_spool(unit, spool);
    gcmd.respond_info(&format!("Hub HES calibration {} for spool {}", if success { "successful" } else { "failed" }, spool), true);
    Ok(())
}

fn cmd_afc_oams_calibrate_hub_hes_all(unit: &Rc<AfcUnit>, gcmd: &GCodeCommand) -> KResult<()> {
    if hw(unit).is_none() {
        gcmd.respond_info("OAMS hardware not available", true);
        return Ok(());
    }
    let mut count = 0;
    let lanes: Vec<(String, Rc<AfcLane>)> = unit.lanes.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect();
    for (lane_name, lane) in lanes {
        if lane.load_state() {
            let spool_idx = ams(unit).spool_map.borrow().get(&lane_name).copied().unwrap_or(0);
            if calibrate_hub_hes_spool(unit, spool_idx) {
                count += 1;
            }
        }
    }
    gcmd.respond_info(&format!("Calibrated {} hub HES sensor(s)", count), true);
    Ok(())
}

fn cmd_afc_oams_clear_errors(unit: &Rc<AfcUnit>, gcmd: &GCodeCommand) -> KResult<()> {
    let c = &AFC_OPENAMS_AFCAMS_CMD_AFC_OAMS_CLEAR_ERRORS;
    let Some(oams) = hw(unit) else {
        gcmd.respond_info("OAMS hardware not available", true);
        return Ok(());
    };
    let mon = monitor(unit);
    if let Some(m) = &mon {
        m.stop();
    }
    let r = (|| -> KResult<()> {
        oams.abort_current_action(oams::opcode::ERROR_KLIPPER_CALL, true)?;
        pause(unit, 0.1)?;
        oams.clear_errors()?;
        oams.current_spool.set(None);
        Ok(())
    })();
    if let Err(e) = r {
        unit.afc.logger.error(c, &format!("Error clearing OAMS errors: {}", e.message()));
    }
    let lanes: Vec<(String, Rc<AfcLane>)> = unit.lanes.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect();
    for (lane_name, lane) in &lanes {
        if lane.tool_loaded.get() {
            let r = lane.unsync_to_extruder(true).and_then(|_| lane.set_tool_unloaded(false));
            if let Err(e) = r {
                unit.afc.logger.warning(c, &format!("Failed to clear lane_loaded for {}: {}", lane_name, e.message()));
            }
        }
    }
    for (_, lane) in &lanes {
        let _ = if lane.tool_loaded.get() {
            unit.lane_tool_loaded(lane)
        } else if lane.load_state() {
            unit.lane_loaded(lane, false)
        } else {
            unit.lane_unloaded(lane)
        };
    }
    if let Some(m) = &mon {
        m.state.borrow_mut().reset();
        m.start(&oams);
    }
    gcmd.respond_info("OpenAMS errors cleared and state resynced", true);
    Ok(())
}

// ── same-FPS runout ─────────────────────────────────────────────────

/// _should_block_sensor_for_runout: suppress a sensor True while a same-FPS
/// runout reload runs
fn should_block_sensor_for_runout(unit: &AfcUnit, lane: &AfcLane, new_val: bool) -> KResult<bool> {
    let a = ams(unit);
    if !a.runout_detected.borrow().contains(&lane.name) {
        return Ok(false);
    }
    let active_runout = unit.afc.function.is_printing(false).unwrap_or(false)
        && lane.tool_loaded.get()
        && matches!(lane.status(), LaneState::InfiniteRunout | LaneState::ToolUnloading);
    if !active_runout {
        a.runout_detected.borrow_mut().remove(&lane.name);
        return Ok(false);
    }
    if new_val {
        unit.afc.logger.debug(&AFC_OPENAMS_AFCAMS__SHOULD_BLOCK_SENSOR_FOR_RUNOUT, &format!("Blocked sensor True for {} — same-FPS runout active", lane.name));
        return Ok(true);
    }
    a.runout_detected.borrow_mut().remove(&lane.name);
    Ok(false)
}

/// _is_same_extruder
fn is_same_extruder(source_lane: &AfcLane, target_lane: &AfcLane) -> bool {
    let (Some(s), Some(t)) = (source_lane.extruder_obj_opt(), target_lane.extruder_obj_opt()) else { return false };
    if s.name.is_empty() || t.name.is_empty() {
        return false;
    }
    kaliru::pyfmt::py_strip(&s.name).to_lowercase() == kaliru::pyfmt::py_strip(&t.name).to_lowercase()
}

/// handle_same_fps_reload: no pause, no unload; the new spool meets the old
/// filament at the extruder gears
fn handle_same_fps_reload(unit: &AfcUnit, source_lane: &Rc<AfcLane>, target_lane: &Rc<AfcLane>) -> KResult<bool> {
    let c = &AFC_OPENAMS_AFCAMS_HANDLE_SAME_FPS_RELOAD;
    let afc = &unit.afc;
    afc.logger.info(c, &format!("Same-FPS infinite runout: {} -> {}", source_lane.name, target_lane.name));
    source_lane.set_status(LaneState::None);
    unit.lane_unloaded(source_lane)?;
    match oams_load(unit, target_lane, 3) {
        Ok(true) => {}
        Ok(false) => {
            afc.logger.error(c, &format!("Same-FPS reload failed for {}", target_lane.name));
            afc.error.afc_error(c, &format!("Same-FPS reload failed for {}", target_lane.name), true, None)?;
            return Ok(false);
        }
        Err(e) => {
            afc.logger.error(c, &format!("Same-FPS reload exception: {}", e.message()));
            afc.error.afc_error(c, &format!("Same-FPS reload failed for {}: {}", target_lane.name, e.message()), true, None)?;
            return Ok(false);
        }
    }
    afc.gcode.run_script_from_command(&format!("AFC_SWAP_MAPPING FROM={} TO={}", source_lane.name, target_lane.name))?;
    target_lane.set_tool_loaded(false)?;
    unit.lane_tool_loaded(target_lane)?;
    afc.save_vars()?;
    afc.logger.info(c, &format!("Same-FPS reload complete: {} now active", target_lane.name));
    Ok(true)
}

/// check_runout: only while printing with this lane loaded to its extruder
pub fn check_runout(unit: &AfcUnit, lane: &AfcLane) -> KResult<bool> {
    let Ok(printing) = unit.afc.function.is_printing(false) else { return Ok(false) };
    if !printing || !lane.tool_loaded.get() {
        return Ok(false);
    }
    if let Some(ext) = lane.extruder_obj_opt() {
        if ext.lane_loaded.borrow().as_deref() != Some(lane.name.as_str()) {
            return Ok(false);
        }
    }
    Ok(true)
}

/// handle_runout: true when handled here (the OAMS cannot retract past F1S)
pub fn handle_runout(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<bool> {
    let c = &AFC_OPENAMS_AFCAMS_HANDLE_RUNOUT;
    let a = ams(unit);
    let runout_lane_name = lane.runout_lane.borrow().clone().filter(|r| !r.is_empty());
    let Some(runout_lane_name) = runout_lane_name else {
        lane.set_status(LaneState::None);
        unit.lane_unloaded(lane)?;
        unit.afc.error.afc_error(
            c,
            &format!(
                "Runout detected on OAMS {}. No runout lane configured.\nOAMS cannot retract filament past the point of no return. Please manually clear filament and load a new spool.",
                lane.name
            ),
            true,
            None,
        )?;
        return Ok(true);
    };
    let Some(target_lane) = resolve_lane_reference(unit, &runout_lane_name) else {
        lane.set_status(LaneState::None);
        unit.lane_unloaded(lane)?;
        unit.afc.error.afc_error(c, &format!("Runout on OAMS {}: runout lane '{}' not found", lane.name, runout_lane_name), true, None)?;
        return Ok(true);
    };
    if is_same_extruder(lane, &target_lane) {
        a.runout_detected.borrow_mut().insert(lane.name.clone());
        unit.afc.logger.info(c, &format!("OAMS same-FPS runout: {} -> {}, seamless reload", lane.name, target_lane.name));
        handle_same_fps_reload(unit, lane, &target_lane)?;
        return Ok(true);
    }
    // Cross-extruder: the empty spool cannot be rewound; AFC's infinite spool swaps tools
    a.runout_empty.borrow_mut().insert(lane.name.clone());
    unit.afc.logger.info(
        c,
        &format!("OAMS cross-extruder runout: {} -> {}, deferring to AFC infinite spool (hardware unload will be skipped)", lane.name, target_lane.name),
    );
    Ok(false)
}

/// on_filament_insert (then the base unit's)
pub fn on_filament_insert(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<()> {
    lane.loaded_to_hub.set(true);
    unit.lane_loaded(lane, false)?;
    unit.lane_illuminate_spool(lane)
}

/// on_filament_remove: the lane's spool data and the bay's OAMS state cleared
pub fn on_filament_remove(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<()> {
    lane.loaded_to_hub.set(false);
    unit.lane_not_ready(lane)?;
    let spool_index = ams(unit).spool_map.borrow().get(&lane.name).copied();
    // (a print runout still needs the lane's data)
    if !lane.tool_loaded.get() {
        clear_lane_info(unit, lane)?;
        clear_oams_state_for_bay(unit, spool_index);
    }
    Ok(())
}

/// _clear_oams_state_for_bay: an emptied bay is no longer current, fed or monitored
fn clear_oams_state_for_bay(unit: &AfcUnit, spool_index: Option<i64>) {
    let (Some(spool_index), Some(oams)) = (spool_index, hw(unit)) else { return };
    let was_current = oams.current_spool.get() == Some(spool_index);
    if was_current {
        oams.current_spool.set(None);
    }
    if was_current {
        if let Some(f) = follower(unit) {
            f.set_follower_state(Some(&oams), 0, 0, "spool removed — f1s empty", true);
        }
    }
    if let Some(m) = monitor(unit) {
        if m.state.borrow().current_spool_idx == Some(spool_index) {
            m.notify_unload_complete();
            m.stop();
        }
    }
}

/// _clear_lane_info: a new spool starts without the old one's data
fn clear_lane_info(unit: &AfcUnit, lane: &Rc<AfcLane>) -> KResult<()> {
    lane.set_material(PyVal::Str(String::new()))?;
    *lane.color.borrow_mut() = PyVal::Str(String::new());
    let spool_id = lane.spool_id.borrow().clone();
    if !matches!(spool_id, PyVal::None) && spool_id != PyVal::Str(String::new()) && spool_id != PyVal::Int(0) {
        if let Err(e) = unit.afc.spool.set_spool_id(lane, PyVal::Str(String::new()), true, None) {
            unit.afc.logger.warning(&AFC_OPENAMS_AFCAMS__CLEAR_LANE_INFO, &format!("OAMS: failed to clear spool_id on {}: {}", lane.name, e.message()));
        }
    }
    let _ = lane.send_lane_data();
    Ok(())
}

// ── TD-1 ────────────────────────────────────────────────────────────

/// _cancel_and_mark_loaded: the firmware cancel stops the follower and
/// takes the spool as loaded where it is
fn cancel_and_mark_loaded(unit: &AfcUnit, oams: &AfcOams, spool_index: i64) -> KResult<()> {
    oams.load_spool_cancel()?;
    let reactor = &unit.afc.reactor;
    let deadline = reactor.monotonic() + 5.0;
    while oams.action_status.get().is_some() {
        if reactor.monotonic() > deadline {
            unit.afc.logger.warning(&AFC_OPENAMS_AFCAMS__CANCEL_AND_MARK_LOADED, "Cancel response timeout - forcing action_status clear");
            oams.action_status.set(None);
            break;
        }
        pause(unit, 0.2)?;
    }
    oams.current_spool.set(Some(spool_index));
    if let Some(m) = monitor(unit) {
        m.state.borrow_mut().state = FpsLoadState::Loaded;
    }
    Ok(())
}

/// _clear_lane_state_after_td1
fn clear_lane_state_after_td1(unit: &AfcUnit, cur_lane: &AfcLane) -> KResult<()> {
    let r = (|| -> KResult<()> {
        if cur_lane.tool_loaded.get() {
            cur_lane.set_tool_unloaded(false)?;
        } else if let Some(ext) = cur_lane.extruder_obj_opt() {
            if ext.lane_loaded.borrow().as_deref() == Some(cur_lane.name.as_str()) {
                *ext.lane_loaded.borrow_mut() = None;
            }
        }
        Ok(())
    })();
    let _ = r;
    unit.afc.save_vars()
}

/// _unload_after_td1
fn unload_after_td1(unit: &AfcUnit, oams: &AfcOams, cur_lane: &AfcLane) -> KResult<()> {
    for attempt in 0..3 {
        let r = wait_for_idle(unit, 30.0).and_then(|_| oams.unload_spool());
        let success = match r {
            Ok((s, _)) => s,
            Err(e) => {
                unit.afc.logger.debug(&AFC_OPENAMS_AFCAMS__UNLOAD_AFTER_TD1, &format!("TD-1 unload attempt {} failed: {}", attempt + 1, e.message()));
                false
            }
        };
        if success {
            unit.afc.logger.info(&AFC_OPENAMS_AFCAMS__UNLOAD_AFTER_TD1, &format!("TD-1 unload completed for {}", cur_lane.name));
            break;
        }
    }
    let _ = oams.clear_errors();
    clear_lane_state_after_td1(unit, cur_lane)
}

/// _cancel_and_cleanup_td1
fn cancel_and_cleanup_td1(unit: &AfcUnit, oams: &AfcOams, cur_lane: &AfcLane, spool_index: i64) -> KResult<()> {
    let _ = cancel_and_mark_loaded(unit, oams, spool_index);
    let _ = oams.set_oams_follower(0, 0);
    let _ = wait_for_idle(unit, 30.0).and_then(|_| oams.unload_spool());
    let _ = oams.clear_errors();
    clear_lane_state_after_td1(unit, cur_lane)
}

/// _get_td1_snapshot: (scan_time, td, color) of the lane's TD-1
fn get_td1_snapshot(unit: &AfcUnit, cur_lane: &AfcLane) -> Option<(PyVal, PyVal, PyVal)> {
    let moonraker = unit.afc.moonraker.borrow().clone()?;
    let td1_data = moonraker.get_td1_data()?;
    let PyVal::Dict(devices) = td1_data.get() else { return None };
    let id = cur_lane.td1_device_id.borrow().clone()?;
    let data = devices.get(id.as_str())?.clone();
    let PyVal::Dict(d) = data.get() else { return None };
    let scan_time = d.get("scan_time").cloned().unwrap_or(PyVal::None);
    if matches!(scan_time, PyVal::None) {
        return None;
    }
    Some((scan_time, d.get("td").cloned().unwrap_or(PyVal::None), d.get("color").cloned().unwrap_or(PyVal::None)))
}

/// _interpolate_encoder_at_scan: the encoder sample nearest the TD-1 scan
/// time (UTC timestamps: "Z", "+00:00Z", or no offset)
fn interpolate_encoder_at_scan(scan_time_str: &str, encoder_history: &[(DateTime, i64)], fallback: i64) -> i64 {
    let mut st = scan_time_str.to_string();
    if st.ends_with("+00:00Z") {
        st.pop();
    } else if let Some(s) = st.strip_suffix('Z') {
        st = format!("{}+00:00", s);
    } else {
        let tail: String = st.chars().rev().take(6).collect();
        if !(tail.contains('+') || tail.contains('-')) {
            st += "+00:00";
        }
    }
    let Some(scan_dt) = DateTime::fromisoformat(&st) else { return fallback };
    let mut best_encoder = fallback;
    let mut best_delta = f64::INFINITY;
    for (wall_time, encoder) in encoder_history {
        let delta = wall_time.seconds_since(&scan_dt).abs();
        if delta < best_delta {
            best_delta = delta;
            best_encoder = *encoder;
        }
    }
    best_encoder
}

/// calibrate_td1: td1_bowden_length as encoder clicks from the hub to the
/// TD-1's first reading, in one continuous load
pub fn calibrate_td1(unit: &AfcUnit, cur_lane: &Rc<AfcLane>) -> KResult<(bool, String, PyVal)> {
    let c = &AFC_OPENAMS_AFCAMS_CALIBRATE_TD1;
    let afc = &unit.afc;
    let td1_id = cur_lane.td1_device_id.borrow().clone();
    if td1_id.is_none() {
        return Ok((false, format!("Cannot calibrate TD-1 for {}, td1_device_id is a required field in AFC_hub or per AFC_lane", cur_lane.name), PyVal::Int(0)));
    }
    let Some(oams) = hw(unit) else { return Ok((false, "OpenAMS hardware not available".into(), PyVal::Int(0))) };
    let spool_index = get_openams_spool_index(unit, cur_lane);
    let (valid, msg) = afc.function.check_for_td1_id(td1_id.as_deref())?;
    if !valid {
        return Ok((false, msg, PyVal::Int(0)));
    }
    afc.logger.raw(c, &format!("TD-1 calibration: continuous load for {}", cur_lane.name));
    const FPS_STOP_THRESHOLD: f64 = 0.45;
    oams.action_status.set(Some(oams::status::LOADING));
    if let Err(e) = oams.send_load_spool(spool_index) {
        oams.action_status.set(None);
        return Ok((false, format!("Failed to start load: {}", e.message()), PyVal::Int(0)));
    }
    let reactor = afc.reactor.clone();
    let hub_timeout = reactor.monotonic() + 15.0;
    let mut encoder_at_hub = None;
    while reactor.monotonic() < hub_timeout {
        pause(unit, 0.1)?;
        if (0..4).contains(&spool_index) && oams.hub_hes_value.borrow()[spool_index as usize] != 0 {
            let e = oams.encoder_clicks.get();
            encoder_at_hub = Some(e);
            afc.logger.info(c, &format!("TD-1 cal: hub triggered, encoder={}", e));
            break;
        }
    }
    let Some(encoder_at_hub) = encoder_at_hub else {
        cancel_and_cleanup_td1(unit, &oams, cur_lane, spool_index)?;
        return Ok((false, format!("Hub sensor did not trigger for {}", cur_lane.name), PyVal::Int(0)));
    };
    let baseline_td1 = get_td1_snapshot(unit, cur_lane);
    let mut td1_detected = false;
    let mut encoder_at_td1 = None;
    let td1_timeout = reactor.monotonic() + 120.0;
    let mut encoder_history: Vec<(DateTime, i64)> = vec![];
    while reactor.monotonic() < td1_timeout {
        pause(unit, 0.2)?;
        let encoder_now = oams.encoder_clicks.get();
        encoder_history.push((super::utils::datetime_now(), encoder_now));
        if encoder_history.len() > 600 {
            encoder_history.remove(0);
        }
        let fps_pressure = oams.fps_value.borrow().as_f64().unwrap_or(0.0);
        if fps_pressure >= FPS_STOP_THRESHOLD {
            afc.logger.info(c, &format!("TD-1 cal: FPS pressure {:.2}, filament at extruder — stopping", fps_pressure));
            break;
        }
        let total_clicks = (encoder_now - encoder_at_hub).abs();
        let current_td1 = get_td1_snapshot(unit, cur_lane);
        if let Some(cur) = current_td1.filter(|cur| Some(cur) != baseline_td1.as_ref()) {
            td1_detected = true;
            let scan_time_str = cur.0.str();
            let at = interpolate_encoder_at_scan(&scan_time_str, &encoder_history, encoder_now);
            encoder_at_td1 = Some(at);
            let actual_clicks = (at - encoder_at_hub).abs();
            afc.logger.info(c, &format!("TD-1 cal: DETECTED! scan_time={}, interpolated encoder={} clicks from hub (current encoder={})", scan_time_str, actual_clicks, total_clicks));
            break;
        }
    }
    let _ = cancel_and_mark_loaded(unit, &oams, spool_index);
    wait_for_idle(unit, 30.0)?;
    unload_after_td1(unit, &oams, cur_lane)?;
    let Some(encoder_at_td1) = encoder_at_td1.filter(|_| td1_detected) else {
        return Ok((false, format!("TD-1 did not detect filament for {}", cur_lane.name), PyVal::Int(0)));
    };
    let encoder_delta = (encoder_at_td1 - encoder_at_hub).abs();
    afc.logger.info(c, &format!("TD-1 calibration for {}: hub={}, td1={}, distance={} clicks", cur_lane.name, encoder_at_hub, encoder_at_td1, encoder_delta));
    let old = cur_lane.settings().td1_bowden_length;
    let cal_msg = format!("\n td1_bowden_length: New: {} Old: {}", encoder_delta, old.map_or("None".into(), kaliru::pyfmt::py_float_repr));
    cur_lane.settings.borrow_mut().td1_bowden_length = Some(encoder_delta as f64);
    afc.function.config_rewrite(&cur_lane.fullname, "td1_bowden_length", &PyVal::Int(encoder_delta), &cal_msg)?;
    cur_lane.do_enable(false)?;
    cur_lane.unit_obj()?.return_to_home(false)?;
    afc.save_vars()?;
    Ok((true, "td1_bowden_length calibration successful".into(), PyVal::Int(encoder_delta)))
}

/// _capture_td1_with_oams: load to the hub, feed td1_bowden_length clicks,
/// read the TD-1, unload
fn capture_td1_with_oams(unit: &AfcUnit, cur_lane: &Rc<AfcLane>, require_loaded: bool, require_enabled: bool) -> KResult<(bool, String)> {
    let c = &AFC_OPENAMS_AFCAMS__CAPTURE_TD1_WITH_OAMS;
    let a = ams(unit);
    let afc = &unit.afc;
    let reactor = afc.reactor.clone();
    if let Some(last) = a.td1_last_capture_time.get() {
        let settle_delay = 4.2 - (reactor.monotonic() - last);
        if settle_delay > 0.0 {
            pause(unit, settle_delay)?;
        }
    }
    let s = cur_lane.settings();
    if require_enabled && !s.td1_when_loaded.unwrap_or(false) {
        return Ok((false, "TD-1 capture disabled".into()));
    }
    if cur_lane.td1_device_id.borrow().is_none() {
        return Ok((false, "TD-1 device ID not configured".into()));
    }
    if cur_lane.tool_loaded.get() {
        return Ok((false, "Toolhead is loaded".into()));
    }
    let Some(td1_bowden_length) = s.td1_bowden_length else { return Ok((false, "td1_bowden_length not set".into())) };
    drop(s);
    if require_loaded && !(cur_lane.load_state() || cur_lane.prep_state.get()) {
        return Ok((false, "Lane is not loaded".into()));
    }
    let Some(oams) = hw(unit) else { return Ok((false, "OpenAMS hardware not available".into())) };
    let spool_index = get_openams_spool_index(unit, cur_lane);
    let other_loaded = |hub: &[i64; 4]| hub.iter().enumerate().any(|(i, v)| i as i64 != spool_index && *v != 0);
    let mut other_hub_loaded = (0..4).contains(&spool_index) && other_loaded(&oams.hub_hes_value.borrow());
    if other_hub_loaded {
        let settle_deadline = reactor.monotonic() + 5.0;
        while reactor.monotonic() < settle_deadline {
            if oams.hub_hes_value.borrow().iter().all(|v| *v == 0) {
                other_hub_loaded = false;
                break;
            }
            pause(unit, 0.1)?;
        }
    }
    if other_hub_loaded {
        return Ok((false, "Another OpenAMS hub already loaded".into()));
    }
    // Load the spool to the hub
    oams.action_status.set(Some(oams::status::LOADING));
    if let Err(e) = oams.send_load_spool(spool_index) {
        oams.action_status.set(None);
        return Ok((false, format!("Failed to start spool load: {}", e.message())));
    }
    let hub_timeout = reactor.monotonic() + 10.0;
    let mut hub_detected = false;
    while reactor.monotonic() < hub_timeout {
        hub_detected = (0..4).contains(&spool_index) && oams.hub_hes_value.borrow()[spool_index as usize] != 0;
        if hub_detected {
            break;
        }
        pause(unit, 0.1)?;
    }
    if !hub_detected {
        let _ = cancel_and_mark_loaded(unit, &oams, spool_index);
        let _ = oams.set_oams_follower(0, 0);
        let _ = oams.unload_spool();
        let _ = oams.clear_errors();
        clear_lane_state_after_td1(unit, cur_lane)?;
        return Ok((false, "Hub sensor did not trigger".into()));
    }
    let encoder_before = oams.encoder_clicks.get();
    let target_clicks = (td1_bowden_length.trunc() as i64).max(0);
    let td1_timeout = reactor.monotonic() + 30.0;
    let baseline_td1 = get_td1_snapshot(unit, cur_lane);
    while reactor.monotonic() < td1_timeout {
        let clicks_moved = (oams.encoder_clicks.get() - encoder_before).abs();
        if clicks_moved >= target_clicks {
            break;
        }
        pause(unit, 0.1)?;
    }
    let mut td1_detected = false;
    let td1_wait_deadline = reactor.monotonic() + 5.0;
    while reactor.monotonic() < td1_wait_deadline {
        let current_td1 = get_td1_snapshot(unit, cur_lane);
        if current_td1.is_some() && current_td1 != baseline_td1 {
            let r = (|| -> KResult<()> {
                let moonraker = afc.moonraker.borrow().clone().ok_or_else(|| KError::Py("RuntimeError".into(), "moonraker not connected".into()))?;
                let td1_data = moonraker.get_td1_data().unwrap_or(PyVal::None);
                let id = cur_lane.td1_device_id.borrow().clone().unwrap_or_default();
                let data = match td1_data.get() {
                    PyVal::Dict(d) => d.get(id.as_str()).cloned(),
                    _ => None,
                }
                .ok_or_else(|| KError::Py("KeyError".into(), kaliru::pyfmt::py_str_repr(&id)))?;
                *cur_lane.td1_data.borrow_mut() = data.clone();
                let field = |k: &str| match data.get() {
                    PyVal::Dict(d) => d.get(k).map_or("None".into(), |v| v.str()),
                    _ => "None".into(),
                };
                afc.logger.info(c, &format!("{} TD-1 data captured: td={} color={}", cur_lane.name, field("td"), field("color")));
                afc.save_vars()
            })();
            match r {
                Ok(()) => td1_detected = true,
                Err(e) => afc.logger.error(c, &format!("TD-1 capture failed for {}: {}", cur_lane.name, e.message())),
            }
            break;
        }
        pause(unit, 0.1)?;
    }
    let _ = cancel_and_mark_loaded(unit, &oams, spool_index);
    unload_after_td1(unit, &oams, cur_lane)?;
    a.td1_last_capture_time.set(Some(reactor.monotonic()));
    if !td1_detected {
        return Ok((false, "TD-1 data not captured (unload completed)".into()));
    }
    Ok((true, "TD-1 data captured".into()))
}

/// prep_capture_td1: Some(result) when the unit captured (capture_td1_when_loaded)
pub fn prep_capture_td1(unit: &AfcUnit, cur_lane: &Rc<AfcLane>) -> KResult<Option<(bool, String)>> {
    if !cur_lane.settings().td1_when_loaded.unwrap_or(false) {
        return Ok(None);
    }
    if unit.afc.function.get_current_lane_obj().is_some() {
        return Ok(None);
    }
    capture_td1_with_oams(unit, cur_lane, true, false).map(Some)
}

/// capture_td1_data: the OAMS load / read / unload cycle
pub fn capture_td1_data(unit: &AfcUnit, cur_lane: &Rc<AfcLane>) -> KResult<(bool, String)> {
    capture_td1_with_oams(unit, cur_lane, true, false)
}

// ── helpers ─────────────────────────────────────────────────────────

/// _get_oams_index: the number in "oams<N>" (1 when it is not one)
fn get_oams_index(unit: &AfcUnit) -> i64 {
    super::utils::py_int_of(&PyVal::Str(ams(unit).oams_name.replace("oams", ""))).unwrap_or(1)
}

fn get_openams_spool_index(unit: &AfcUnit, lane: &AfcLane) -> i64 {
    ams(unit).spool_map.borrow().get(&lane.name).copied().unwrap_or(0)
}

/// _resolve_lane_reference: by name, else case-insensitively
fn resolve_lane_reference(unit: &AfcUnit, lane_name: &str) -> Option<Rc<AfcLane>> {
    if lane_name.is_empty() {
        return None;
    }
    if let Some(l) = unit.afc.lane(lane_name) {
        return Some(l);
    }
    let lower = lane_name.to_lowercase();
    unit.afc.lanes.borrow().iter().find(|(n, _)| n.to_lowercase() == lower).map(|(_, l)| l.clone())
}

/// _wait_for_idle: until the OAMS has no action in flight
fn wait_for_idle(unit: &AfcUnit, timeout: f64) -> KResult<bool> {
    let Some(oams) = hw(unit) else { return Ok(false) };
    let reactor = &unit.afc.reactor;
    let deadline = reactor.monotonic() + timeout;
    while reactor.monotonic() < deadline {
        if oams.action_status.get().is_none() {
            return Ok(true);
        }
        pause(unit, 0.5)?;
    }
    unit.afc.logger.error(&AFC_OPENAMS_AFCAMS__WAIT_FOR_IDLE, "OAMS idle timeout");
    Ok(false)
}

/// _wait_for_hub_settle: the spool's hub HES clear for stable_time
fn wait_for_hub_settle(unit: &AfcUnit, spool_index: i64, timeout: f64, stable_time: f64) -> KResult<bool> {
    let Some(oams) = hw(unit) else { return Ok(true) };
    let reactor = &unit.afc.reactor;
    let deadline = reactor.monotonic() + timeout;
    let mut clear_since: Option<f64> = None;
    loop {
        let now = reactor.monotonic();
        let hub_present = (0..4).contains(&spool_index) && oams.hub_hes_value.borrow()[spool_index as usize] != 0;
        if !hub_present {
            match clear_since {
                None => clear_since = Some(now),
                Some(s) if now - s >= stable_time => return Ok(true),
                Some(_) => {}
            }
        } else {
            clear_since = None;
        }
        if now >= deadline {
            unit.afc.logger.warning(
                &AFC_OPENAMS_AFCAMS__WAIT_FOR_HUB_SETTLE,
                &format!("OAMS hub HES did not settle clear within {}s (spool {}); proceeding", kaliru::pyfmt::py_float_repr(timeout), spool_index),
            );
            return Ok(false);
        }
        reactor.pause(now + 0.05)?;
    }
}

/// _calibrate_hub_hes_spool
fn calibrate_hub_hes_spool(unit: &AfcUnit, spool_index: i64) -> bool {
    let oams_idx = get_oams_index(unit);
    match unit.afc.gcode.run_script_from_command(&format!("OAMS_CALIBRATE_HUB_HES OAMS={} SPOOL={}", oams_idx, spool_index)) {
        Ok(()) => true,
        Err(e) => {
            unit.afc.logger.error(&AFC_OPENAMS_AFCAMS__CALIBRATE_HUB_HES_SPOOL, &format!("Hub HES calibration failed for spool {}: {}", spool_index, e.message()));
            false
        }
    }
}
