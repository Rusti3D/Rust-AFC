// Armored Turtle Automated Filament Control (port of AFC.py and the AFC
// modules COSMOS runs with the Elegoo Canvas; docs/AFC_PORT.md in Rusticc)
//
// Copyright (C) 2024-2026 Armored Turtle
//
// This file may be distributed under the terms of the GNU GPLv3 license.
pub mod ace;
pub mod ace_transport;
pub mod assist;
pub mod boxturtle;
pub mod buffer;
pub mod button;
pub mod canvas;
pub mod codes;
pub mod error;
pub mod extruder;
pub mod form_tip;
pub mod functions;
pub mod hub;
pub mod lane;
pub mod led;
pub mod logger;
pub mod poop;
pub mod prep;
pub mod respond;
pub mod scope;
pub mod spool;
pub mod stepper;
pub mod stats;
pub mod toolchange;
pub mod unit;
pub mod utils;

use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::extras::gcode_move::GCodeMove;
use kaliru::extras::heaters::{Heater, PrinterHeaters};
use kaliru::extras::idle_timeout::IdleTimeout;
use kaliru::gcode::{GCodeCommand, GCodeDispatch};
use kaliru::pins::{PinChip, SharedPinParams};
use kaliru::printer::{PObj, Printer, PrinterObject};
use kaliru::pyfmt::py_float_repr;
use kaliru::pyval::{PyDict, PyVal};
use kaliru::reactor::{Reactor, Timer, NEVER};
use kaliru::toolhead::ToolHead;
use kaliru::IndexMap;
use codes::*;
use extruder::AfcExtruder;
use hub::AfcHub;
use lane::{AfcLane, AssistActive, LaneState, MoveDirection, SpeedMode};
use logger::{AfcLogger, Code};
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};
use unit::AfcUnit;

pub use scope::native_scope;

pub const AFC_VERSION: &str = "1.2.7";

/// AFC's state (a str enum: str() is the value)
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum State {
    Init,
    Idle,
    Error,
    Loading,
    Unloading,
    ToolSwap,
    ToolDock,
    ToolPickup,
    EjectingLane,
    MovingLane,
    RestoringPos,
}

impl State {
    pub fn as_str(self) -> &'static str {
        match self {
            State::Init => "Initialized",
            State::Idle => "Idle",
            State::Error => "Error",
            State::Loading => "Loading",
            State::Unloading => "Unloading",
            State::ToolSwap => "ToolSwap",
            State::ToolDock => "ToolDock",
            State::ToolPickup => "ToolPickup",
            State::EjectingLane => "Ejecting",
            State::MovingLane => "Moving",
            State::RestoringPos => "Restoring",
        }
    }
}

/// python's AttributeError for a member that was never set
pub fn no_attr(obj: &str, attr: &str) -> KError {
    KError::Py("AttributeError".into(), format!("'{}' object has no attribute '{}'", obj, attr))
}

/// round_floats(value): python's round(v, 6) of each float, repr'd like a list
pub fn round_floats_list(values: &[f64]) -> String {
    let items: Vec<String> = values.iter().map(|v| py_float_repr(kaliru::pyfmt::py_round(*v, 6))).collect();
    format!("[{}]", items.join(", "))
}

pub fn round_float(v: f64) -> String {
    py_float_repr(kaliru::pyfmt::py_round(v, 6))
}

/// capture_toolhead_temp(): the extruder and its target temperature
pub struct TempState {
    extruder: Rc<dyn kaliru::toolhead::ExtraAxis>,
    extruder_name: String,
    target_temp: f64,
}

/// [AFC]: the AFC system, its lanes, units, hubs and extruders
pub struct Afc {
    me: Weak<Afc>,
    pub config: ConfigWrapper,
    pub printer: Rc<Printer>,
    pub reactor: Rc<Reactor>,
    pub webhooks: Rc<kaliru::webhooks::WebHooks>,
    pub logger: Rc<AfcLogger>,
    pub function: Rc<functions::AfcFunction>,
    pub spool: Rc<spool::AfcSpool>,
    pub error: Rc<error::AfcError>,
    pub gcode: Rc<GCodeDispatch>,
    toolhead: RefCell<Option<Rc<ToolHead>>>,
    idle: RefCell<Option<Rc<IdleTimeout>>>,
    gcode_move: RefCell<Option<Rc<GCodeMove>>>,
    pub heater: RefCell<Option<Rc<Heater>>>,

    pub current_loading: RefCell<Option<String>>,
    pub next_lane_load: RefCell<Option<String>>,
    pub error_state: Cell<bool>,
    pub current_state: Cell<State>,
    pub position_saved: Cell<bool>,
    pub spoolman: RefCell<Option<String>>,
    pub moonraker: RefCell<Option<Rc<utils::Moonraker>>>,
    pub print_data_metadata: RefCell<Option<Rc<utils::PrintFileMetaData>>>,
    pub afc_stats: RefCell<Option<Rc<stats::AfcStats>>>,
    pub td1_defined: Cell<bool>,
    td1_present_cache: Cell<bool>,
    last_td1_query: Cell<f64>,
    pub lane_data_enabled: Cell<bool>,
    pub prep_done: Cell<bool>,
    pub last_prep_activity_time: Cell<f64>,
    in_print_timer: RefCell<Option<Timer>>,
    pub db_backup: Cell<bool>,

    pub units: RefCell<IndexMap<String, Rc<AfcUnit>>>,
    pub tools: RefCell<IndexMap<String, Rc<AfcExtruder>>>,
    pub lanes: RefCell<IndexMap<String, Rc<AfcLane>>>,
    pub hubs: RefCell<IndexMap<String, Rc<AfcHub>>>,
    pub buffers: RefCell<IndexMap<String, Rc<buffer::AfcBuffer>>>,
    pub tool_cmds: RefCell<IndexMap<String, String>>,
    pub led_state: Cell<bool>,
    /// led_obj: the AFC_led chains by name
    pub led_obj: RefCell<kaliru::IndexMap<String, Rc<led::AfcLed>>>,
    pub bypass: RefCell<Option<Rc<kaliru::extras::filament_switch_sensor::RunoutHelper>>>,
    bypass_last_state: Cell<bool>,
    quiet_switch: RefCell<Option<Rc<kaliru::extras::filament_switch_sensor::RunoutHelper>>>,
    pub number_of_toolchanges: Cell<i64>,
    pub current_toolchange: Cell<i64>,
    pub print_tool_temperatures: RefCell<Vec<PyVal>>,
    pub active_led_effects: RefCell<Vec<String>>,

    pub in_toolchange: Cell<bool>,

    // Save/resume pos variables
    pub base_position: RefCell<Vec<f64>>,
    pub last_gcode_position: RefCell<Vec<f64>>,
    pub last_toolhead_position: RefCell<Vec<f64>>,
    pub homing_position: RefCell<Vec<f64>>,
    pub speed: Cell<f64>,
    pub speed_factor: Cell<f64>,
    pub absolute_coord: Cell<bool>,
    pub absolute_extrude: Cell<bool>,
    pub extrude_factor: Cell<f64>,

    // Config
    pub moonraker_port: PyVal,
    pub moonraker_host: String,
    pub moonraker_connect_to: i64,
    pub var_file: String,
    pub cfgloc: String,
    pub config_save_method: String,
    var_writer: utils::VarWriter,
    pub default_material_temps: Option<Vec<String>>,
    pub default_material_type: Option<String>,
    pub common_density_values: Vec<String>,
    pub test_extrude_amt: PyVal,
    pub temp_wait_tolerance: f64,
    pub disable_weight_check: bool,
    pub disable_ooze_check: bool,
    pub disable_print_temp_check: bool,
    pub auto_spool_switch: bool,
    pub auto_spool_switch_threshold: f64,
    pub led_off: String,
    pub led_fault: String,
    pub led_ready: String,
    pub led_not_ready: String,
    pub led_loading: String,
    pub led_prep_loaded: String,
    pub led_unloading: String,
    pub led_tool_loaded: String,
    pub led_tool_loaded_idle: String,
    pub led_tool_unloaded: String,
    pub led_buffer_advancing: String,
    pub led_buffer_trailing: String,
    pub led_buffer_neutral: String,
    pub led_buffer_disabled: String,
    pub led_spool_illum: String,
    pub tool_cut: Cell<bool>,
    pub tool_cut_threshold: i64,
    pub tool_cut_cmd: Option<String>,
    pub park_pre_load: bool,
    pub park_pre_load_cmd: Option<String>,
    pub park: Cell<bool>,
    pub park_cmd: Option<String>,
    pub kick: Cell<bool>,
    pub kick_cmd: Option<String>,
    pub wipe: Cell<bool>,
    pub wipe_cmd: Option<String>,
    pub poop: Cell<bool>,
    pub poop_cmd: Option<String>,
    pub enable_standalone_purge: bool,
    pub post_load_macro: Option<String>,
    pub post_unload_macro: Option<String>,
    pub form_tip: Cell<bool>,
    pub form_tip_cmd: Option<String>,
    pub force_assign_map: bool,
    pub quiet_mode: Cell<bool>,
    pub auto_home: bool,
    pub auto_level_macro: Option<String>,
    pub show_quiet_mode: bool,
    pub quiet_moves_speed: Cell<f64>,
    /// quiet_moves_speed is still python's int default (prints "50")
    pub quiet_moves_int: Cell<bool>,
    pub move_ints: MoveInts,
    pub long_moves_speed: f64,
    pub long_moves_accel: f64,
    pub short_moves_speed: f64,
    pub short_moves_accel: f64,
    pub short_move_dis: f64,
    pub tool_homing_distance: f64,
    pub max_move_dis: f64,
    pub n20_break_delay_time: f64,
    pub home_to_hub: bool,
    pub home_to_tool: bool,
    pub homing_enabled: bool,
    pub load_then_home_var: bool,
    pub load_undershoot: f64,
    pub tool_max_unload_attempts: i64,
    pub tool_max_load_checks: i64,
    pub max_move_tries: i64,
    pub rev_long_moves_speed_factor: f64,
    pub z_hop: f64,
    pub resume_speed: f64,
    pub error_timeout: f64,
    pub resume_z_speed: f64,
    pub spool_ratio: f64,
    pub full_weight: f64,
    pub enable_sensors_in_gui: bool,
    pub ignore_spoolman_material_temps: bool,
    pub led_use_filament_color: bool,
    pub restore_extruder_temp_on_load_or_unload: bool,
    pub lower_extruder_temp_on_change: bool,
    pub toolchange_temp_drop: f64,
    pub load_to_hub: bool,
    pub disable_homing_check: bool,
    pub assisted_unload: bool,
    pub bypass_pause: bool,
    pub unload_on_runout: bool,
    /// global_print_current: the lane steppers' current while printing
    pub global_print_current: Option<f64>,
    pub clear_spool_after_eject: bool,
    pub short_stats: bool,
    pub enable_assist: bool,
    pub enable_assist_weight: f64,
    pub enable_hub_runout: bool,
    pub enable_tool_runout: bool,
    pub enable_runout_in_bypass: bool,
    pub debounce_delay: f64,
    pub standalone_auto_load_unload: bool,
    pub td1_when_loaded: bool,
    pub debug: bool,
    pub testing: bool,
    pub enable_multiple_mapping: Cell<bool>,
    pub in_klippy_test_env: bool,
    pub show_macros: bool,
    pub delta_time: functions::DeltaTime,
}

pub const BASE_UNLOAD_FILAMENT: &str = "UNLOAD_FILAMENT";
pub const RENAMED_UNLOAD_FILAMENT: &str = "_AFC_RENAMED_UNLOAD_FILAMENT_";
const BASE_M104: &str = "M104";
const RENAMED_M104: &str = "_AFC_RENAMED_M104_";
const BASE_M109: &str = "M109";
const RENAMED_M109: &str = "_AFC_RENAMED_M109_";

pub const CMD_AFC_TOGGLE_MACRO_HELP: &str = "Enable/disable TOOL_CUT/PARK/POOP/KICK/WIPE/FORM_TIP macros";
pub const CMD_AFC_QUIET_MODE_HELP: &str = "Set quiet mode speed and enable/disable quiet mode";
pub const CMD_UNSET_LANE_LOADED_HELP: &str = "Removes active lane loaded from toolhead loaded status";
pub const CMD_SET_AFC_TOOLCHANGES_HELP: &str = "Sets number of toolchanges for AFC to keep track of";
pub const CMD_LANE_MOVE_HELP: &str = "Lane Manual Movements";
pub const CMD_HUB_LOAD_HELP: &str = "Load lane into hub";
pub const CMD_LANE_UNLOAD_HELP: &str = "Unload lane from extruder";
pub const CMD_TOOL_LOAD_HELP: &str = "Load lane into tool";
pub const CMD_TOOL_UNLOAD_HELP: &str = "Unload from tool head";
pub const CMD_CHANGE_TOOL_HELP: &str = "change filaments in tool head";
pub const CMD_AFC_M104_HELP: &str = "Set extruder temperature";
pub const CMD_AFC_M109_HELP: &str = "Set extruder temperature and wait for it to reach the target";
pub const CMD_TURN_OFF_AFC_LED_HELP: &str = "Turns off all LEDs for AFC_led configurations";
pub const CMD_TURN_ON_AFC_LED_HELP: &str = "Turns on all LEDs for AFC_led configurations and restores state";
pub const CMD_AFC_STATS_HELP: &str = "Prints AFC toolchange statistics to console";
pub const CMD_AFC_CHANGE_BLADE_HELP: &str = "Sets cutter blade changed date and resets total count since blade was changed";
pub const CMD_AFC_RESET_STATS_HELP: &str = "Resets stats for a given extruder or lane";
pub const CMD_AFC_CLEAR_MESSAGE_HELP: &str = "Macro to clear error and warning message from AFC message queue";
pub const CMD__AFC_TEST_MESSAGES_HELP: &str = "Macro to send test messages for testing";

/// A command handler of an Afc method
type AfcCmd = fn(&Rc<Afc>, &GCodeCommand) -> KResult<()>;

impl Afc {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Afc>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let webhooks = printer.load_object(config, "webhooks", true)?.and_then(kaliru::printer::downcast_rc).ok_or_else(|| KError::Other("webhooks".into()))?;
        let logger = Rc::new(AfcLogger::new(&printer)?);
        let function = printer.lookup_or_load::<functions::AfcFunction>(config, "AFC_functions")?;
        let spool = printer.lookup_or_load::<spool::AfcSpool>(config, "AFC_spool")?;
        let error = printer.lookup_or_load::<error::AfcError>(config, "AFC_error")?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;

        // Config get section
        let moonraker_port = match config.get_opt("moonraker_port")? {
            Some(v) => PyVal::Str(v),
            None => PyVal::Int(7125),
        };
        let moonraker_host = config.get("moonraker_host", "http://localhost")?;
        let moonraker_connect_to = config.getint("moonraker_timeout", 30, Lim::new())?;
        let _unit_order_list = config.get("unit_order_list", "")?;
        let var_file = config.get("VarFile", "../printer_data/config/AFC/AFC.var")?;
        let cfgloc = match var_file.rfind('/') {
            Some(i) => var_file[..i + 1].to_string(),
            None => var_file.clone(),
        };
        let config_save_method = config.getchoice("config_save_method", &[("afc", "afc"), ("kalico", "kalico")], "afc")?.to_string();
        let var_writer = utils::VarWriter::start(reactor.clone());
        let strs = |v: &[&str]| PyVal::List(v.iter().map(|s| PyVal::Str(s.to_string())).collect());
        let default_material_temps = config.getlist("default_material_temps", strs(&["default: 235", "PLA:210", "PETG:235", "ABS:235", "ASA:235"]), ",", None)?;
        let default_material_type = config.get_opt("default_material_type")?;
        let common_density_values = config.getlist("common_density_values", strs(&["PLA:1.24", "PETG:1.23", "ABS:1.04", "ASA:1.07"]), ",", None)?.unwrap_or_default();
        let test_extrude_amt = match config.get_opt("test_extrude_amt")? {
            Some(v) => PyVal::Str(v),
            None => PyVal::Int(10),
        };
        let temp_wait_tolerance = config.getfloat("temp_wait_tolerance", 5.0, Lim::new())?;
        let disable_weight_check = config.getboolean("disable_weight_check", false)?;
        let disable_ooze_check = config.getboolean("disable_ooze_check", true)?;
        let disable_print_temp_check = config.getboolean("disable_print_temp_check", false)?;
        let auto_spool_switch = config.getboolean("auto_spool_switch", false)?;
        let auto_spool_switch_threshold = config.getfloat("auto_spool_switch_threshold", 25.0, Lim::new().min(0.0))?;
        // LED SETTINGS
        let _led_name = config.get_opt("led_name")?;
        let led_fault = config.get("led_fault", "1,0,0,0")?;
        let led_ready = config.get("led_ready", "0,0.8,0,0")?;
        let led_not_ready = config.get("led_not_ready", "1,0,0,0")?;
        let led_loading = config.get("led_loading", "1,1,1,0")?;
        let led_prep_loaded = config.get("led_loading", "1,1,1,0")?;
        let led_unloading = config.get("led_unloading", "1,1,.5,0")?;
        let led_tool_loaded = config.get("led_tool_loaded", "0,0,1,0")?;
        let led_tool_loaded_idle = config.get("led_tool_loaded_idle", "0.4,0.4,0,0")?;
        let led_tool_unloaded = config.get("led_tool_unloaded", "1,0,0,0")?;
        let led_buffer_advancing = config.get("led_buffer_advancing", "0,0,1,0")?;
        let led_buffer_trailing = config.get("led_buffer_trailing", "0,1,0,0")?;
        let led_buffer_neutral = config.get("led_buffer_neutral", "1,1,1,1")?;
        let led_buffer_disabled = config.get("led_buffer_disable", "0,0,0,0.25")?;
        let led_spool_illum = config.get("led_spool_illuminate", "1,1,1,1")?;
        // TOOL Cutting Settings
        let tool_cut = config.getboolean("tool_cut", false)?;
        let tool_cut_threshold = config.getint("tool_cut_threshold", 10000, Lim::new())?;
        let tool_cut_cmd = config.get_opt("tool_cut_cmd")?;
        // CHOICES
        let park_pre_load = config.getboolean("park_pre_load", false)?;
        let park_pre_load_cmd = config.get_opt("park_pre_load_cmd")?;
        let park = config.getboolean("park", false)?;
        let park_cmd = config.get_opt("park_cmd")?;
        let kick = config.getboolean("kick", false)?;
        let kick_cmd = config.get_opt("kick_cmd")?;
        let wipe = config.getboolean("wipe", false)?;
        let wipe_cmd = config.get_opt("wipe_cmd")?;
        let poop = config.getboolean("poop", false)?;
        let poop_cmd = config.get_opt("poop_cmd")?;
        let enable_standalone_purge = config.getboolean("enable_standalone_purge", true)?;
        let post_load_macro = config.get_opt("post_load_macro")?;
        let post_unload_macro = config.get_opt("post_unload_macro")?;
        let form_tip = config.getboolean("form_tip", false)?;
        let form_tip_cmd = config.get_opt("form_tip_cmd")?;
        let force_assign_map = config.getboolean("force_assign_map", false)?;
        // MOVE SETTINGS
        let auto_home = config.getboolean("auto_home", false)?;
        let auto_level_macro = config.get_opt("auto_level_macro")?;
        let show_quiet_mode = config.getboolean("show_quiet_mode", true)?;
        let (quiet_moves_speed, quiet_moves_int) = getfloat_int(config, "quiet_moves_speed", PyVal::Int(50))?;
        let (long_moves_speed, long_speed_int) = getfloat_int(config, "long_moves_speed", PyVal::Int(100))?;
        let (long_moves_accel, long_accel_int) = getfloat_int(config, "long_moves_accel", PyVal::Int(400))?;
        let (short_moves_speed, short_speed_int) = getfloat_int(config, "short_moves_speed", PyVal::Int(25))?;
        let (short_moves_accel, short_accel_int) = getfloat_int(config, "short_moves_accel", PyVal::Int(400))?;
        let move_ints = MoveInts { long_speed: long_speed_int, long_accel: long_accel_int, short_speed: short_speed_int, short_accel: short_accel_int };
        let short_move_dis = config.getfloat("short_move_dis", 10.0, Lim::new())?;
        let tool_homing_distance = config.getfloat("tool_homing_distance", 200.0, Lim::new())?;
        let max_move_dis = config.getfloat("max_move_dis", 999999.0, Lim::new())?;
        let n20_break_delay_time = config.getfloat("n20_break_delay_time", 0.200, Lim::new())?;
        let home_to_hub = config.getboolean("home_to_hub", true)?;
        let home_to_tool = config.getboolean("home_to_tool", true)?;
        let homing_enabled = config.getboolean("homing_enabled", true)?;
        let load_then_home_var = config.getboolean("load_then_home", true)?;
        let load_undershoot = config.getfloat("load_undershoot", 20.0, Lim::new())?;
        let tool_max_unload_attempts = config.getint("tool_max_unload_attempts", 4, Lim::new().min(0))?;
        let tool_max_load_checks = config.getint("tool_max_load_checks", 4, Lim::new())?;
        let max_move_tries = config.getint("max_move_tries", 20, Lim::new())?;
        let rev_long_moves_speed_factor = config.getfloat("rev_long_moves_speed_factor", 1.0, Lim::new())?;
        let z_hop = config.getfloat("z_hop", 0.0, Lim::new())?;
        let _xy_resume = config.getboolean("xy_resume", false)?;
        let speed = 25.0;
        let resume_speed = config.getfloat("resume_speed", speed, Lim::new())?;
        let error_timeout = config.getfloat("error_timeout", 36000.0, Lim::new())?;
        let resume_z_speed = config.getfloat("resume_z_speed", speed, Lim::new())?;
        let global_print_current = config.getfloat_opt("global_print_current", Lim::new())?;
        let spool_ratio = config.getfloat("spool_ratio", 2.0, Lim::new())?;
        let full_weight = config.getfloat("full_weight", 1000.0, Lim::new().min(1))?;
        let enable_sensors_in_gui = config.getboolean("enable_sensors_in_gui", false)?;
        let ignore_spoolman_material_temps = config.getboolean("ignore_spoolman_material_temps", false)?;
        let led_use_filament_color = config.getboolean("led_use_filament_color", false)?;
        let restore_extruder_temp_on_load_or_unload = config.getboolean("restore_extruder_temp_on_load_or_unload", false)?;
        let lower_extruder_temp_on_change = config.getboolean("lower_extruder_temp_on_change", true)?;
        let toolchange_temp_drop = config.getfloat("toolchange_temp_drop", 0.0, Lim::new())?;
        let load_to_hub = config.getboolean("load_to_hub", true)?;
        let disable_homing_check = config.getboolean("disable_homing_check", false)?;
        let assisted_unload = config.getboolean("assisted_unload", true)?;
        let bypass_pause = config.getboolean("pause_when_bypass_active", false)?;
        let unload_on_runout = config.getboolean("unload_on_runout", false)?;
        let clear_spool_after_eject = config.getboolean("clear_spool_after_eject", true)?;
        let short_stats = config.getboolean("print_short_stats", false)?;
        let enable_assist = config.getboolean("enable_assist", true)?;
        let enable_assist_weight = config.getfloat("enable_assist_weight", 500.0, Lim::new())?;
        let enable_hub_runout = config.getboolean("enable_hub_runout", true)?;
        let enable_tool_runout = config.getboolean("enable_tool_runout", true)?;
        let enable_runout_in_bypass = config.getboolean("enable_runout_in_bypass", false)?;
        let debounce_delay = config.getfloat("debounce_delay", 0.0, Lim::new())?;
        let standalone_auto_load_unload = config.getboolean("standalone_auto_load_unload", true)?;
        let td1_when_loaded = config.getboolean("capture_td1_when_loaded", false)?;
        let debug = config.getboolean("debug", false)?;
        let log_frame_data = config.getboolean("log_frame_data", true)?;
        let testing = config.getboolean("testing", false)?;
        let enable_multiple_mapping = config.getboolean("enable_multiple_mapping", false)?;
        let in_klippy_test_env = printer.start_arg("debuginput").is_some_and(|v| !matches!(v, PyVal::None));

        logger.log_frame_data.set(log_frame_data);
        logger.set_debug(debug);

        let afc = Rc::new_cyclic(|me| Afc {
            me: me.clone(),
            config: config.clone(),
            printer: printer.clone(),
            reactor: reactor.clone(),
            webhooks,
            logger: logger.clone(),
            function: function.clone(),
            spool: spool.clone(),
            error,
            gcode: gcode.clone(),
            toolhead: RefCell::new(None),
            idle: RefCell::new(None),
            gcode_move: RefCell::new(None),
            heater: RefCell::new(None),
            current_loading: RefCell::new(None),
            next_lane_load: RefCell::new(None),
            error_state: Cell::new(false),
            current_state: Cell::new(State::Init),
            position_saved: Cell::new(false),
            spoolman: RefCell::new(None),
            moonraker: RefCell::new(None),
            print_data_metadata: RefCell::new(None),
            afc_stats: RefCell::new(None),
            td1_defined: Cell::new(false),
            td1_present_cache: Cell::new(false),
            last_td1_query: Cell::new(0.0),
            lane_data_enabled: Cell::new(false),
            prep_done: Cell::new(false),
            last_prep_activity_time: Cell::new(0.0),
            in_print_timer: RefCell::new(None),
            db_backup: Cell::new(false),
            units: RefCell::new(IndexMap::default()),
            tools: RefCell::new(IndexMap::default()),
            lanes: RefCell::new(IndexMap::default()),
            hubs: RefCell::new(IndexMap::default()),
            buffers: RefCell::new(IndexMap::default()),
            tool_cmds: RefCell::new(IndexMap::default()),
            led_state: Cell::new(true),
            led_obj: RefCell::new(kaliru::IndexMap::default()),
            bypass: RefCell::new(None),
            bypass_last_state: Cell::new(false),
            quiet_switch: RefCell::new(None),
            number_of_toolchanges: Cell::new(0),
            current_toolchange: Cell::new(0),
            print_tool_temperatures: RefCell::new(vec![]),
            active_led_effects: RefCell::new(vec![]),
            in_toolchange: Cell::new(false),
            base_position: RefCell::new(vec![0.0; 4]),
            last_gcode_position: RefCell::new(vec![0.0; 4]),
            last_toolhead_position: RefCell::new(vec![0.0; 4]),
            homing_position: RefCell::new(vec![0.0; 4]),
            speed: Cell::new(speed),
            speed_factor: Cell::new(1.0 / 60.0),
            absolute_coord: Cell::new(true),
            absolute_extrude: Cell::new(true),
            extrude_factor: Cell::new(1.0),
            moonraker_port,
            moonraker_host,
            moonraker_connect_to,
            var_file,
            cfgloc,
            config_save_method,
            var_writer,
            default_material_temps,
            default_material_type,
            common_density_values,
            test_extrude_amt,
            temp_wait_tolerance,
            disable_weight_check,
            disable_ooze_check,
            disable_print_temp_check,
            auto_spool_switch,
            auto_spool_switch_threshold,
            led_off: "0,0,0,0".into(),
            led_fault,
            led_ready,
            led_not_ready,
            led_loading,
            led_prep_loaded,
            led_unloading,
            led_tool_loaded,
            led_tool_loaded_idle,
            led_tool_unloaded,
            led_buffer_advancing,
            led_buffer_trailing,
            led_buffer_neutral,
            led_buffer_disabled,
            led_spool_illum,
            tool_cut: Cell::new(tool_cut),
            tool_cut_threshold,
            tool_cut_cmd,
            park_pre_load,
            park_pre_load_cmd,
            park: Cell::new(park),
            park_cmd,
            kick: Cell::new(kick),
            kick_cmd,
            wipe: Cell::new(wipe),
            wipe_cmd,
            poop: Cell::new(poop),
            poop_cmd,
            enable_standalone_purge,
            post_load_macro,
            post_unload_macro,
            form_tip: Cell::new(form_tip),
            form_tip_cmd,
            force_assign_map,
            quiet_mode: Cell::new(false),
            auto_home,
            auto_level_macro,
            show_quiet_mode,
            quiet_moves_speed: Cell::new(quiet_moves_speed),
            quiet_moves_int: Cell::new(quiet_moves_int),
            move_ints,
            long_moves_speed,
            long_moves_accel,
            short_moves_speed,
            short_moves_accel,
            short_move_dis,
            tool_homing_distance,
            max_move_dis,
            n20_break_delay_time,
            home_to_hub,
            home_to_tool,
            homing_enabled,
            load_then_home_var,
            load_undershoot,
            tool_max_unload_attempts,
            tool_max_load_checks,
            max_move_tries,
            rev_long_moves_speed_factor,
            z_hop,
            resume_speed,
            error_timeout,
            resume_z_speed,
            spool_ratio,
            full_weight,
            enable_sensors_in_gui,
            ignore_spoolman_material_temps,
            led_use_filament_color,
            restore_extruder_temp_on_load_or_unload,
            lower_extruder_temp_on_change,
            toolchange_temp_drop,
            load_to_hub,
            disable_homing_check,
            assisted_unload,
            bypass_pause,
            unload_on_runout,
            global_print_current,
            clear_spool_after_eject,
            short_stats,
            enable_assist,
            enable_assist_weight,
            enable_hub_runout,
            enable_tool_runout,
            enable_runout_in_bypass,
            debounce_delay,
            standalone_auto_load_unload,
            td1_when_loaded,
            debug,
            testing,
            enable_multiple_mapping: Cell::new(enable_multiple_mapping),
            in_klippy_test_env,
            show_macros: config.getboolean("show_macros", true).unwrap_or(true),
            delta_time: functions::DeltaTime::new(logger.clone()),
        });
        // (show_macros is read where python reads it, after the trsync update
        // and the version print; a config error there is raised below)
        function.set_afc(&afc);

        let a = afc.clone();
        kaliru::events::KlippyConnect::on(&printer, move || a.handle_connect());
        let a = afc.clone();
        kaliru::events::KlippyReady::on(&printer, move || a.handle_ready());
        let a = afc.clone();
        kaliru::events::KlippyDisconnect::on(&printer, move || {
            a.join_threads();
            Ok(())
        });
        let a = afc.clone();
        printer.register_event_handler(
            lane::REGISTER_MACROS_EVENT,
            Rc::new(move |args| {
                let lane: Rc<AfcLane> = kaliru::events::arg(lane::REGISTER_MACROS_EVENT, args, 0)?;
                a.register_lane_macros(&lane)
            }),
        );
        let a = afc.clone();
        printer.register_event_handler(
            "virtual_sdcard:reset_file",
            Rc::new(move |_| a.reset_file_callback()),
        );
        let f = function.clone();
        printer.register_event_handler(
            "extruder:activate_extruder",
            Rc::new(move |_| f.handle_activate_extruder()),
        );
        let a = afc.clone();
        afc.webhooks.register_endpoint("afc/status", Rc::new(move |web_request| a.webhooks_status(web_request)))?;

        afc.update_trsync(config)?;

        // Setup pin so a virtual filament sensor can be added for bypass and quiet mode
        let ppins = printer.lookup::<kaliru::pins::PinsObject>("pins")?;
        let chip: Rc<dyn PinChip> = Rc::new(VirtualChip);
        ppins.0.borrow_mut().register_chip("afc_virtual_bypass", chip.clone()).map_err(|e| KError::Pins(e.0))?;
        ppins.0.borrow_mut().register_chip("afc_quiet_mode", chip).map_err(|e| KError::Pins(e.0))?;

        // Printing here will not display in console, but it will go to klippy.log
        afc.print_version(false);

        // Register AFC macros
        let show_macros = config.getboolean("show_macros", true)?;
        let reg = |name: &str, f: AfcCmd, help: &str, options: Option<&[(&str, &str, &str)]>| -> KResult<()> {
            let a = afc.clone();
            function.register_commands(show_macros, name, Rc::new(move |g| f(&a, g)), help, options)
        };
        reg("AFC_STATS", Afc::cmd_afc_stats, CMD_AFC_STATS_HELP, Some(&[("SHORT", "int", "1")]))?;
        reg("AFC_QUIET_MODE", Afc::cmd_afc_quiet_mode, CMD_AFC_QUIET_MODE_HELP, Some(&[("SPEED", "float", "50"), ("ENABLE", "int", "0")]))?;
        reg("TURN_ON_AFC_LED", Afc::cmd_turn_on_afc_led, CMD_TURN_ON_AFC_LED_HELP, None)?;
        reg("TURN_OFF_AFC_LED", Afc::cmd_turn_off_afc_led, CMD_TURN_OFF_AFC_LED_HELP, None)?;
        reg("AFC_CHANGE_BLADE", Afc::cmd_afc_change_blade, CMD_AFC_CHANGE_BLADE_HELP, None)?;
        reg(
            "AFC_TOGGLE_MACRO",
            Afc::cmd_afc_toggle_macro,
            CMD_AFC_TOGGLE_MACRO_HELP,
            Some(&[("TOOL_CUT", "int", "0"), ("PARK", "int", "0"), ("POOP", "int", "0"), ("KICK", "int", "0"), ("WIPE", "int", "0"), ("FORM_TIP", "int", "0")]),
        )?;
        reg("UNSET_LANE_LOADED", Afc::cmd_unset_lane_loaded, CMD_UNSET_LANE_LOADED_HELP, None)?;
        reg("AFC_RESET_STATS", Afc::cmd_afc_reset_stats, CMD_AFC_RESET_STATS_HELP, Some(&[("EXTRUDER", "string", ""), ("LANE", "string", "")]))?;
        spool.register_commands(&afc)?;
        Ok(afc)
    }

    pub fn rc(&self) -> Rc<Afc> {
        self.me.upgrade().expect("AFC")
    }

    /// self.current: the lane loaded in the active toolhead
    pub fn current(&self) -> Option<String> {
        self.function.get_current_lane()
    }

    pub fn toolhead(&self) -> KResult<Rc<ToolHead>> {
        self.toolhead.borrow().clone().ok_or_else(|| no_attr("afc", "toolhead"))
    }

    pub fn gcode_move(&self) -> KResult<Rc<GCodeMove>> {
        self.gcode_move.borrow().clone().ok_or_else(|| no_attr("afc", "gcode_move"))
    }

    pub fn afc_stats(&self) -> KResult<Rc<stats::AfcStats>> {
        self.afc_stats.borrow().clone().ok_or_else(|| no_attr("afc", "afc_stats"))
    }

    pub fn lane(&self, name: &str) -> Option<Rc<AfcLane>> {
        self.lanes.borrow().get(name).cloned()
    }

    fn update_trsync(&self, config: &ConfigWrapper) -> KResult<()> {
        // Logic to update trsync values
        let update_trsync = config.getboolean("trsync_update", false)?;
        if update_trsync {
            let trsync_value = config.getfloat("trsync_timeout", 0.05, Lim::new())?;
            let trsync_single_value = config.getfloat("trsync_single_timeout", 0.5, Lim::new())?;
            self.logger.info(&AFC_AFC__UPDATE_TRSYNC, "Applying TRSYNC update");
            // (kalico has neither TRSYNC_TIMEOUT nor TRSYNC_SINGLE_MCU_TIMEOUT)
            let _ = (trsync_value, trsync_single_value);
            self.logger.info(&AFC_AFC__UPDATE_TRSYNC, "TRSYNC_TIMEOUT does not exist in mcu file, not updating");
            self.logger.info(&AFC_AFC__UPDATE_TRSYNC, "TRSYNC_SINGLE_MCU_TIMEOUT does not exist in mcu file, not updating");
        }
        Ok(())
    }

    /// hasattr(Printer, "get_snapmaker_config_dir"): never on kalico
    pub fn snapmaker_printer(&self) -> bool {
        false
    }

    fn register_lane_macros(self: &Rc<Self>, lane_obj: &Rc<AfcLane>) -> KResult<()> {
        let name = lane_obj.name.clone();
        let mux = |cmd: &str, f: AfcCmd, help: &str| -> KResult<()> {
            let a = self.clone();
            self.gcode.register_mux_command(cmd, "LANE", Some(&name), Rc::new(move |g| f(&a, g)), Some(help))
        };
        mux("LANE_MOVE", Afc::cmd_lane_move, CMD_LANE_MOVE_HELP)?;
        mux("LANE_UNLOAD", Afc::cmd_lane_unload, CMD_LANE_UNLOAD_HELP)?;
        mux("TOOL_LOAD", Afc::cmd_tool_load, CMD_TOOL_LOAD_HELP)?;
        if lane_obj.unit_obj()?.type_name() != "ViViD" {
            mux("HUB_LOAD", Afc::cmd_hub_load, CMD_HUB_LOAD_HELP)?;
        }
        Ok(())
    }

    /// Called at the beginning of PREP so that moonraker has time to start
    pub fn handle_moonraker_connect(&self) -> KResult<()> {
        let res = (|| -> KResult<Option<()>> {
            let moonraker = utils::Moonraker::new(&self.moonraker_host, &self.moonraker_port.str(), self.logger.clone(), self.reactor.clone());
            *self.moonraker.borrow_mut() = Some(moonraker.clone());
            if !moonraker.wait_for_moonraker(&*self.toolhead()?, self.moonraker_connect_to)? {
                return Ok(None);
            }
            // Remove current lane_data from database before pushing data back up
            moonraker.delete_lane_data();
            *self.spoolman.borrow_mut() = moonraker.get_spoolman_server();
            if self.spoolman.borrow().is_some() {
                // Wait for remote method to be registered
                for i in 0..30 {
                    if self.webhooks.remote_method_names().iter().any(|m| m == spool::SPOOLMAN_REMOTE_METHOD) {
                        self.logger.debug(&AFC_AFC_HANDLE_MOONRAKER_CONNECT, &format!("{} registered after {}s", spool::SPOOLMAN_REMOTE_METHOD, i));
                        break;
                    }
                    self.toolhead()?.dwell(1.0)?;
                }
            }
            let (td1_defined, td1_present, lane_data) = moonraker.check_for_td1();
            self.td1_defined.set(td1_defined);
            self.td1_present_cache.set(td1_present);
            self.lane_data_enabled.set(lane_data);
            *self.afc_stats.borrow_mut() = Some(Rc::new(stats::AfcStats::new(moonraker.clone(), self.logger.clone(), self.tools.borrow().len() > 1)));
            *self.print_data_metadata.borrow_mut() = Some(Rc::new(utils::PrintFileMetaData::new(moonraker.clone(), self.logger.clone())));
            self.printer.send_event(utils::MOONRAKER_CONNECT_EVENT, &[])?;
            Ok(Some(()))
        })();
        if let Err(e) = res {
            self.logger.debug(
                &AFC_AFC_HANDLE_MOONRAKER_CONNECT,
                &format!("Moonraker/Spoolman/afc_stats/td1 error\nError: {}\n{}", e.message(), kaliru::logging::format_traceback(&e)),
            );
            *self.spoolman.borrow_mut() = None;
        }
        Ok(())
    }

    fn handle_connect(self: &Rc<Self>) -> KResult<()> {
        *self.toolhead.borrow_mut() = Some(self.printer.lookup::<ToolHead>("toolhead")?);
        *self.idle.borrow_mut() = Some(self.printer.lookup::<IdleTimeout>("idle_timeout")?);
        *self.gcode_move.borrow_mut() = Some(self.printer.lookup::<GCodeMove>("gcode_move")?);
        // (kalico's manual_home has no probe_pos)

        // Check if hardware bypass is configured, if not create a virtual bypass sensor
        let bypass = match self.printer.lookup_object_opt("filament_switch_sensor bypass") {
            Some(o) => kaliru::extras::filament_switch_sensor::runout_helper_of(&o).ok_or_else(|| no_attr("SwitchSensor", "runout_helper"))?,
            None => utils::add_filament_switch("virtual_bypass", "afc_virtual_bypass:virtual_bypass", &self.printer, &utils::SwitchOptions::default())?.0.runout_helper.clone(),
        };
        *self.bypass.borrow_mut() = Some(bypass);
        if self.show_quiet_mode {
            let quiet = utils::add_filament_switch("quiet_mode", "afc_quiet_mode:afc_quiet_mode", &self.printer, &utils::SwitchOptions::default())?;
            *self.quiet_switch.borrow_mut() = Some(quiet.0.runout_helper.clone());
        }

        // Register G-Code commands for macros we don't want to show up in mainsail/fluidd
        let reg = |cmd: &str, f: AfcCmd, help: &str| -> KResult<()> {
            let a = self.clone();
            self.gcode.register_command(cmd, Some(Rc::new(move |g| f(&a, g))), false, Some(help)).map(|_| ())
        };
        reg("TOOL_UNLOAD", Afc::cmd_tool_unload, CMD_TOOL_UNLOAD_HELP)?;
        reg("CHANGE_TOOL", Afc::cmd_change_tool, CMD_CHANGE_TOOL_HELP)?;
        reg("SET_AFC_TOOLCHANGES", Afc::cmd_set_afc_toolchanges, CMD_SET_AFC_TOOLCHANGES_HELP)?;
        reg("AFC_CLEAR_MESSAGE", Afc::cmd_afc_clear_message, CMD_AFC_CLEAR_MESSAGE_HELP)?;
        reg("_AFC_TEST_MESSAGES", Afc::cmd_afc_test_messages, CMD__AFC_TEST_MESSAGES_HELP)?;
        reg("AFC_M104", Afc::cmd_afc_m104, CMD_AFC_M104_HELP)?;
        reg("AFC_M109", Afc::cmd_afc_m109, CMD_AFC_M109_HELP)?;

        self.rename_macros()?;
        self.current_state.set(State::Idle);
        Ok(())
    }

    /// Sorts units by their lowest lane number so they show up in order
    fn handle_ready(&self) -> KResult<()> {
        let natural_lane_num = |key: &str, lane: &AfcLane| -> f64 {
            if lane.extruder_obj().is_ok_and(|e| e.name == lane.name) {
                return 9999.0;
            }
            match first_number(key) {
                Some(n) => n as f64,
                None => 9999.0,
            }
        };
        let mut units: Vec<(String, Rc<AfcUnit>)> = self.units.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect();
        let key = |u: &AfcUnit| -> f64 {
            u.lanes.borrow().iter().map(|(k, l)| natural_lane_num(k, l)).fold(f64::INFINITY, f64::min)
        };
        // (sorted() is stable)
        units.sort_by(|a, b| key(&a.1).total_cmp(&key(&b.1)));
        *self.units.borrow_mut() = units.into_iter().collect();
        Ok(())
    }

    fn rename_macros(self: &Rc<Self>) -> KResult<()> {
        let a = self.clone();
        self.function.rename(BASE_M104, RENAMED_M104, Rc::new(move |g| a.cmd_afc_m104(g)), CMD_AFC_M104_HELP)?;
        let a = self.clone();
        self.function.rename(BASE_M109, RENAMED_M109, Rc::new(move |g| a.cmd_afc_m109(g)), CMD_AFC_M109_HELP)
    }

    /// AFC git version to the console and log
    pub fn print_version(&self, console_only: bool) {
        let afc_dir = afc_extras_dir(&self.printer);
        let (mut git_hash, mut git_commit_num) = ("_".to_string(), "_".to_string());
        let res = utils::check_output(&["git", "-C", &afc_dir, "rev-parse", "--short", "HEAD"])
            .and_then(|h| {
                git_hash = h;
                utils::check_output(&["git", "-C", &afc_dir, "rev-list", "HEAD", "--count"])
            })
            .map(|n| git_commit_num = n);
        if let Err(tb) = res {
            self.logger.debug(&AFC_AFC_PRINT_VERSION, &format!("Error fetching repo info: {}", tb));
        }
        let string = format!("AFC Version: v{}-{}-{}", AFC_VERSION, git_commit_num, git_hash);
        self.logger.info_opt(&AFC_AFC_PRINT_VERSION, &string, console_only);
    }

    /// Error string when an enabled macro still has its placeholder position
    pub fn verify_macro_positions(&self) -> String {
        let mut error_str = String::new();
        let var = |obj: &str, key: &str| -> Option<PyVal> {
            let o = self.printer.lookup_object_opt(obj)?;
            let m = kaliru::printer::downcast_rc::<kaliru::extras::gcode_macro::GCodeMacro>(o)?;
            let v = m.variables.borrow().get(key).map(|v| v.to_pyval());
            v
        };
        // (the macro variable's python repr against the placeholder tuple)
        let is_tuple = |v: &Option<PyVal>, t: &[i64]| -> bool {
            let want = format!("({})", t.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(", "));
            v.as_ref().is_some_and(|v| v.repr() == want)
        };
        if self.tool_cut.get() && self.tool_cut_cmd.as_deref() == Some("AFC_CUT") && self.printer.lookup_object_opt("gcode_macro _AFC_CUT_TIP_VARS").is_some() {
            let v = var("gcode_macro _AFC_CUT_TIP_VARS", "pin_loc_xy");
            if is_tuple(&v, &[-99, -99]) {
                error_str += "tool_cut is set to True and variable_pin_loc_xy has not been updated.\n";
                error_str += "Please update variable_pin_loc_xy in AFC\\AFC_Macro_Vars.cfg file.\n\n";
            }
        }
        if self.park.get() && self.park_cmd.as_deref() == Some("AFC_PARK") && self.printer.lookup_object_opt("gcode_macro _AFC_PARK_VARS").is_some() {
            let v = var("gcode_macro _AFC_PARK_VARS", "park_loc_xy");
            if is_tuple(&v, &[-99, -99]) {
                error_str += "park is set to True and variable_park_loc_xy has not been updated.\n";
                error_str += "Please update variable_park_loc_xy in AFC\\AFC_Macro_Vars.cfg file.\n\n";
            }
        }
        if self.poop.get() && self.poop_cmd.as_deref() == Some("AFC_POOP") && self.printer.lookup_object_opt("gcode_macro _AFC_POOP_VARS").is_some() {
            let v = var("gcode_macro _AFC_POOP_VARS", "purge_loc_xy");
            if is_tuple(&v, &[-99, -99]) {
                error_str += "poop is set to True and variable_purge_loc_xy has not been updated.\n";
                error_str += "Please update variable_purge_loc_xy in AFC\\AFC_Macro_Vars.cfg file.\n\n";
            }
        }
        if self.kick.get() && self.kick_cmd.as_deref() == Some("AFC_KICK") && self.printer.lookup_object_opt("gcode_macro _AFC_KICK_VARS").is_some() {
            let v = var("gcode_macro _AFC_KICK_VARS", "kick_start_loc");
            if is_tuple(&v, &[-99, -99, 5]) {
                error_str += "kick is set to True and variable_kick_start_loc has not been updated.\n";
                error_str += "Please update variable_kick_start_loc in AFC\\AFC_Macro_Vars.cfg file.\n\n";
            }
        }
        if self.wipe.get() && self.wipe_cmd.as_deref() == Some("AFC_BRUSH") && self.printer.lookup_object_opt("gcode_macro _AFC_BRUSH_VARS").is_some() {
            let v = var("gcode_macro _AFC_BRUSH_VARS", "brush_loc");
            if is_tuple(&v, &[-99, -99, -1]) {
                error_str += "wipe is set to True and variable_brush_loc has not been updated.\n";
                error_str += "Please update variable_brush_loc in AFC\\AFC_Macro_Vars.cfg file.\n\n";
            }
        }
        error_str
    }

    /// td1_present: re-queried at most every 30 s while ready and not printing
    pub fn td1_present(&self) -> KResult<bool> {
        let mut present = self.td1_present_cache.get();
        let current_time = self.reactor.monotonic();
        let moonraker = self.moonraker.borrow().clone();
        if let Some(m) = moonraker {
            if self.printer.state_message() == "Printer is ready" && (current_time - self.last_td1_query.get()) > 30.0 && !self.function.is_printing(true)? {
                self.last_td1_query.set(current_time);
                present = m.check_for_td1().1;
                self.td1_present_cache.set(present);
            }
        }
        Ok(present)
    }

    fn reset_file_callback(&self) -> KResult<()> {
        let a = self.rc();
        let timer = self.reactor.register_timer(Box::new(move |et| a.in_print_reactor_timer(et)), self.reactor.monotonic() + 5.0);
        *self.in_print_timer.borrow_mut() = Some(timer);
        self.error.reset_failure()?;
        self.gcode.run_script_from_command("CLEAR_PAUSE")?;
        self.number_of_toolchanges.set(0);
        self.current_toolchange.set(-1);
        self.print_tool_temperatures.borrow_mut().clear();
        if let Some(m) = self.print_data_metadata.borrow().as_ref() {
            m.reset();
        }
        self.save_vars()
    }

    fn in_print_reactor_timer(self: &Rc<Self>, _eventtime: f64) -> f64 {
        if let Some(t) = self.in_print_timer.borrow_mut().take() {
            self.reactor.unregister_timer(&t);
        }
        let res = (|| -> KResult<()> {
            let (in_print, print_filename) = self.function.in_print_file()?;
            self.logger.debug(&AFC_AFC_IN_PRINT_REACTOR_TIMER, &format!("In print: {}, Filename: {}", py_bool(in_print), print_filename.str()));
            if in_print {
                self.number_of_toolchanges.set(0);
                let metadata = self.print_data_metadata.borrow().clone();
                if let (true, Some(m)) = (self.moonraker.borrow().is_some(), metadata) {
                    let a = self.clone();
                    m.query_filename(&print_filename.str(), Some(Box::new(move || a.finish_print_start())));
                } else {
                    self.finish_print_start()?;
                }
            }
            Ok(())
        })();
        if let Err(e) = res {
            kaliru::logging::exception_err("AFC in_print_reactor_timer", &e);
        }
        NEVER
    }

    fn finish_print_start(&self) -> KResult<()> {
        let metadata = self.print_data_metadata.borrow().clone();
        if let (true, Some(m)) = (self.moonraker.borrow().is_some(), metadata) {
            self.number_of_toolchanges.set(m.tool_change_count());
            *self.print_tool_temperatures.borrow_mut() = m.tool_temperatures();
        }
        self.current_toolchange.set(-1);
        self.logger.info(&AFC_AFC__FINISH_PRINT_START, &format!("Total number of toolchanges set to {}", self.number_of_toolchanges.get()));
        // Reset fault detection: purging in PRINT_START can trip it
        if let Some(lane) = self.function.get_current_lane_obj() {
            if let Some(b) = lane.buffer_obj() {
                b.update_filament_error_pos()?;
            }
        }
        Ok(())
    }

    /// (temperature, True when a fallback value is used)
    fn get_default_material_temps(&self, cur_lane: &AfcLane) -> KResult<(f64, bool)> {
        let heater = self.heater.borrow().clone().ok_or_else(|| no_attr("afc", "heater"))?;
        let mut temp_value: PyVal = match self.default_material_temps.as_ref().and_then(|t| t.iter().find(|v| v.contains("default")).cloned()) {
            Some(v) => match v.split(':').nth(1) {
                Some(t) => PyVal::Str(t.to_string()),
                None => PyVal::Float(heater.min_extrude_temp.get() + 5.0),
            },
            None => PyVal::Float(heater.min_extrude_temp.get() + 5.0),
        };
        let mut using_min_value = true;
        let lane_temp = cur_lane.extruder_temp.borrow().clone();
        let lane_temp_f = lane_temp.as_f64();
        if lane_temp_f.is_some_and(|t| t > 0.0 && t > heater.min_extrude_temp.get()) {
            temp_value = lane_temp.clone();
            using_min_value = false;
        } else if let (Some(temps), Some(material)) = (self.default_material_temps.as_ref(), cur_lane.material_value()) {
            let lane_material = kaliru::pyfmt::py_strip(&material.str()).to_lowercase();
            for mat in temps {
                let m: Vec<&str> = mat.split(':').collect();
                let mat_key = kaliru::pyfmt::py_strip(m[0]).to_lowercase();
                if lane_material.contains(&mat_key) {
                    temp_value = m.get(1).map(|v| PyVal::Str(v.to_string())).ok_or_else(|| KError::Py("IndexError".into(), "list index out of range".into()))?;
                    using_min_value = false;
                    break;
                }
            }
        }
        Ok((utils::py_float_of(&temp_value)?, using_min_value))
    }

    /// Heats the lane's extruder when needed; True when it should wait
    pub fn check_extruder_temp(&self, cur_lane: &AfcLane, no_wait: bool) -> KResult<bool> {
        let extruder = cur_lane.extruder_obj()?.toolhead_extruder()?;
        let heater = extruder.get_heater()?;
        *self.heater.borrow_mut() = Some(heater.clone());
        let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
        let mut wait;

        if heater.can_extrude.get() && self.function.is_printing(false)? && (self.disable_print_temp_check || self.print_tool_temperatures.borrow().is_empty()) {
            return Ok(false);
        }

        let (target_temp, using_min_value);
        if self.function.is_printing(false)? && !self.print_tool_temperatures.borrow().is_empty() {
            let current_map = cur_lane.current_map.borrow().clone();
            let idx = utils::py_int_of(&PyVal::Str(current_map.trim_start_matches('T').to_string()));
            match idx {
                Ok(idx) if idx >= 0 => {
                    let temps = self.print_tool_temperatures.borrow().clone();
                    match temps.get(idx as usize) {
                        Some(t) => {
                            if matches!(t, PyVal::None) {
                                self.logger.info(&AFC_AFC__CHECK_EXTRUDER_TEMP, &format!("No print_tool_temperatures entry for lane {}", cur_lane.name));
                                return Ok(false);
                            }
                            target_temp = utils::py_float_of(t)?;
                            using_min_value = false;
                        }
                        None => {
                            self.logger.info(
                                &AFC_AFC__CHECK_EXTRUDER_TEMP,
                                &format!(
                                    "Could not resolve print_tool_temperatures index for lane {}: list index out of range. Falling back to AFC lane temperature",
                                    cur_lane.name
                                ),
                            );
                            (target_temp, using_min_value) = self.get_default_material_temps(cur_lane)?;
                        }
                    }
                }
                Ok(_) => {
                    self.logger.info(&AFC_AFC__CHECK_EXTRUDER_TEMP, &format!("Could not resolve print_tool_temperatures index for lane {}: Negative tool index", cur_lane.name));
                    return Ok(false);
                }
                Err(e) => {
                    self.logger.info(&AFC_AFC__CHECK_EXTRUDER_TEMP, &format!("Could not resolve print_tool_temperatures index for lane {}: {}", cur_lane.name, e.message()));
                    return Ok(false);
                }
            }
        } else {
            (target_temp, using_min_value) = self.get_default_material_temps(cur_lane)?;
        }

        let current_temp = heater.get_temp(self.reactor.monotonic());
        let tolerance = self.temp_wait_tolerance;
        if current_temp.0 < (heater.target_temp.get() - tolerance) {
            pheaters.set_temperature(&heater, current_temp.0, false)?;
            self.logger.info(&AFC_AFC__CHECK_EXTRUDER_TEMP, &format!("Current temp {:.1} is below set temp {}", current_temp.0, py_float_repr(target_temp)));
        }
        let need_lower = heater.target_temp.get() >= (target_temp + tolerance) && !using_min_value;
        let need_heat = heater.target_temp.get() <= (target_temp - tolerance);
        let skip_lower = need_lower && !self.lower_extruder_temp_on_change && current_temp.0 >= (target_temp - tolerance);
        wait = false;
        if (need_heat || need_lower) && !skip_lower {
            wait = !need_lower;
            self.logger.info(
                &AFC_AFC__CHECK_EXTRUDER_TEMP,
                &format!(
                    "Setting extruder temperature to {} {}",
                    py_float_repr(target_temp),
                    if wait { "and waiting for extruder to reach temperature" } else { "" }
                ),
            );
            pheaters.set_temperature(&heater, target_temp, false)?;
        }
        if wait && !no_wait {
            self.wait_for_temp_within_tolerance(&heater, target_temp, Some(tolerance * 2.0))?;
        }
        Ok(wait)
    }

    pub fn capture_toolhead_temp(&self, extruder: Option<&AfcExtruder>, async_capture: bool) -> KResult<Option<TempState>> {
        if !self.restore_extruder_temp_on_load_or_unload {
            return Ok(None);
        }
        if self.function.is_printing(false)? && !async_capture {
            return Ok(None);
        }
        let ext = match extruder {
            Some(e) => e.toolhead_extruder()?,
            None => self.toolhead()?.get_extruder(),
        };
        let heater = ext.get_heater()?;
        let name = match extruder {
            Some(e) => e.name.clone(),
            None => ext.get_name(),
        };
        Ok(Some(TempState { extruder: ext, extruder_name: name, target_temp: heater.target_temp.get() }))
    }

    pub fn restore_toolhead_temp(&self, temp_state: Option<&TempState>, async_restore: bool) -> KResult<()> {
        if !self.restore_extruder_temp_on_load_or_unload {
            return Ok(());
        }
        if self.function.is_printing(false)? && !async_restore {
            return Ok(());
        }
        let res = (|| -> KResult<()> {
            let ts = temp_state.ok_or_else(|| KError::Py("TypeError".into(), "'NoneType' object is not subscriptable".into()))?;
            let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
            pheaters.set_temperature(&ts.extruder.get_heater()?, ts.target_temp, false)?;
            self.logger.info(
                &AFC_AFC_RESTORE_TOOLHEAD_TEMP,
                &format!("Restoring extruder temperature to {} for {}", py_float_repr(ts.target_temp), ts.extruder_name),
            );
            Ok(())
        })();
        if res.is_err() {
            self.logger.debug(&AFC_AFC_RESTORE_TOOLHEAD_TEMP, "Unable to restore extruder temperature");
        }
        Ok(())
    }

    /// _AFC_DISPLAY_STATUS VARIABLE=.. VALUE=.. when the user defines it
    pub fn set_display_status(&self, variable: &str, value: bool) {
        if self.printer.lookup_object_opt("gcode_macro _AFC_DISPLAY_STATUS").is_some() {
            let r = self.gcode.run_script_from_command(&format!("_AFC_DISPLAY_STATUS VARIABLE={} VALUE={}", variable, py_bool(value)));
            if r.is_err() {
                self.logger.debug(&AFC_AFC__SET_DISPLAY_STATUS, "_AFC_DISPLAY_STATUS macro raised an error");
            }
        }
    }

    pub fn set_quiet_mode(&self, val: bool) {
        if self.show_quiet_mode {
            if let Some(q) = self.quiet_switch.borrow().as_ref() {
                q.sensor_enabled.set(val);
                q.filament_present.set(val);
            }
        } else {
            self.quiet_mode.set(val);
        }
    }

    pub fn get_quiet_mode(&self) -> bool {
        if self.show_quiet_mode {
            match self.quiet_switch.borrow().as_ref() {
                Some(q) => {
                    let state = q.sensor_enabled.get();
                    q.filament_present.set(state);
                    state
                }
                None => false,
            }
        } else {
            self.quiet_mode.get()
        }
    }

    /// Filament present in the bypass sensor (a virtual one: its switch)
    pub fn get_bypass_state(&self) -> bool {
        let Some(bypass) = self.bypass.borrow().clone() else { return false };
        if bypass.name.contains("virtual") {
            let state = bypass.sensor_enabled.get();
            if state {
                if let Some(cur) = self.current() {
                    self.logger.error(&AFC_AFC_GET_BYPASS_STATE, &format!("Cannot set virtual bypass, {} is currently loaded.", cur));
                    bypass.sensor_enabled.set(false);
                    return false;
                }
            }
            bypass.filament_present.set(state);
            if self.bypass_last_state.get() != state {
                self.bypass_last_state.set(state);
                let _ = self.save_vars();
            }
            state
        } else {
            bypass.filament_present.get()
        }
    }

    /// True when filament is in the bypass (unloads it with unload=True)
    pub fn check_bypass(&self, unload: bool, caller: &Code) -> bool {
        let res = (|| -> KResult<bool> {
            if self.get_bypass_state() {
                if unload {
                    self.logger.info(&AFC_AFC__CHECK_BYPASS, "Bypass detected, calling manual unload filament routine");
                    self.set_display_status("retraction", true);
                    let r = self.gcode.run_script_from_command(RENAMED_UNLOAD_FILAMENT);
                    self.set_display_status("retraction", false);
                    r?;
                    self.logger.info(&AFC_AFC__CHECK_BYPASS, "Filament unloaded");
                } else {
                    let msg = "Filament loaded in bypass, not doing tool load";
                    let pause = self.function.in_print()? && self.bypass_pause;
                    self.error.afc_error(&AFC_AFC__CHECK_BYPASS, msg, pause, Some(caller.func))?;
                }
                return Ok(true);
            }
            Ok(false)
        })();
        res.unwrap_or(false)
    }

    fn cmd_afc_toggle_macro(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let get = |name: &str, cur: bool| -> KResult<bool> { Ok(gcmd.get_int(name, Some(cur as i64), Lim::new().min(0).max(1))? != 0) };
        self.tool_cut.set(get("TOOL_CUT", self.tool_cut.get())?);
        self.park.set(get("PARK", self.park.get())?);
        self.kick.set(get("KICK", self.kick.get())?);
        self.poop.set(get("POOP", self.poop.get())?);
        self.wipe.set(get("WIPE", self.wipe.get())?);
        self.form_tip.set(get("FORM_TIP", self.form_tip.get())?);
        let c = &AFC_AFC_CMD_AFC_TOGGLE_MACRO;
        self.logger.info(c, &format!("Tool Cut {}, Park {}", py_bool(self.tool_cut.get()), py_bool(self.park.get())));
        self.logger.info(c, &format!("Kick {}, Poop {}", py_bool(self.kick.get()), py_bool(self.poop.get())));
        self.logger.info(c, &format!("Wipe {}, Form tip {}", py_bool(self.wipe.get()), py_bool(self.form_tip.get())));
        Ok(())
    }

    fn cmd_afc_quiet_mode(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let enable = gcmd.get_int("ENABLE", Some(self.get_quiet_mode() as i64), Lim::new().min(0).max(1))? != 0;
        self.set_quiet_mode(enable);
        if gcmd.get_opt("SPEED").is_some() {
            self.quiet_moves_int.set(false);
        }
        self.quiet_moves_speed.set(gcmd.get_float("SPEED", Some(self.quiet_moves_speed.get()), Lim::new().min(10.0).max(400.0))?);
        self.logger.info(
            &AFC_AFC_CMD_AFC_QUIET_MODE,
            &format!("QuietMode {}, max speed of {} mm/sec", py_bool(self.get_quiet_mode()), py_float_repr(self.quiet_moves_speed.get())),
        );
        Ok(())
    }

    fn cmd_unset_lane_loaded(self: &Rc<Self>, _gcmd: &GCodeCommand) -> KResult<()> {
        self.function.unset_lane_loaded()
    }

    fn cmd_set_afc_toolchanges(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let number_of_toolchanges = gcmd.get_int("TOOLCHANGES", None, Lim::new())?;
        if number_of_toolchanges > 0 {
            let mut warning_text = String::from("Please remove SET_AFC_TOOLCHANGES from your slicers 'Change Filament G-Code' section as SET_AFC_TOOLCHANGES ");
            warning_text += "is now deprecated and number of toolchanges will be fetched from files metadata in moonraker when a print starts.\n";
            warning_text += "Verify that moonrakers version is at least v0.9.3-64 to utilize this feature.";
            self.logger.info(&AFC_AFC_CMD_SET_AFC_TOOLCHANGES, &format!("<span class=warning--text>{}</span>", warning_text));
            self.logger.message_queue.borrow_mut().push((warning_text, "warning"));
        }
        Ok(())
    }

    fn cmd_lane_move(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_AFC_CMD_LANE_MOVE;
        let force = gcmd.get_int("FORCE", Some(0), Lim::new())? == 1;
        if self.function.is_printing(false)? && !force {
            self.error.afc_error(c, "Cannot move lane while printer is printing", false, None)?;
            return Ok(());
        }
        let lane = gcmd.get_opt("LANE");
        let distance = gcmd.get_float("DISTANCE", Some(0.0), Lim::new())?;
        if distance == 0.0 {
            self.error.afc_error(c, "Distance to move cannot be zero", false, None)?;
            return Ok(());
        }
        let Some(cur_lane) = lane.as_deref().and_then(|l| self.lane(l)) else {
            self.logger.info(c, &format!("{} Unknown", py_opt_str(lane.as_deref())));
            return Ok(());
        };
        self.current_state.set(State::MovingLane);
        let speed_mode = if distance.abs() >= 200.0 { SpeedMode::Long } else { SpeedMode::Short };
        cur_lane.set_load_current()?;
        cur_lane.move_advanced(distance, speed_mode, AssistActive::Yes)?;
        cur_lane.do_enable(false)?;
        self.current_state.set(State::Idle);
        cur_lane.unit_obj()?.return_to_home(false)?;
        // Put CAM back to lane if it is loaded to the toolhead
        self.function.select_loaded_lane()
    }

    fn get_resume_speed(&self) -> f64 {
        if self.resume_speed > 0.0 {
            self.resume_speed
        } else {
            self.speed.get()
        }
    }

    fn get_resume_speedz(&self) -> f64 {
        if self.resume_z_speed > 0.0 {
            self.resume_z_speed
        } else {
            self.speed.get()
        }
    }

    /// Moves z (capped below max z) through gcode_move's own last_position
    pub fn move_z_pos(&self, z_amount: f64, string: &str, wait_moves: bool) -> KResult<f64> {
        let toolhead = self.toolhead()?;
        let max_z = match toolhead.get_status(0.0).as_ref().map(|s| s.get()) {
            Some(PyVal::Dict(d)) => d.get("axis_maximum").and_then(|v| v.items().and_then(|i| i.get(2)).and_then(|z| z.as_f64())).unwrap_or(0.0),
            _ => 0.0,
        };
        let gm = self.gcode_move()?;
        let newpos = {
            let mut lp = gm.last_position.borrow_mut();
            lp[2] = kaliru::pyfmt::py_min(max_z - 1.0, z_amount);
            lp.clone()
        };
        gm.move_with_transform(&newpos, self.get_resume_speedz())?;
        if wait_moves {
            toolhead.wait_moves()?;
        }
        self.function.log_toolhead_pos(&format!("move_z_pos({}): ", string))?;
        Ok(newpos[2])
    }

    /// str(gcode_move.last_position)
    pub fn gcode_move_last_position_repr(&self) -> KResult<String> {
        let gm = self.gcode_move()?;
        let lp = gm.last_position.borrow();
        Ok(format!("[{}]", lp.iter().map(|v| py_float_repr(*v)).collect::<Vec<_>>().join(", ")))
    }

    /// Moves the extruder by e_amount through gcode_move's last_position
    pub fn move_e_pos(&self, e_amount: f64, speed: f64, _log_string: &str, wait_tool: bool) -> KResult<()> {
        let gm = self.gcode_move()?;
        let newpos = {
            let mut lp = gm.last_position.borrow_mut();
            lp[3] += e_amount;
            lp.clone()
        };
        gm.move_with_transform(&newpos, speed)?;
        if wait_tool {
            self.toolhead()?.wait_moves()?;
        }
        Ok(())
    }

    fn position_state(&self, with_toolchange: bool) -> KResult<String> {
        let mut s = format!(
            "Error State: {}, Is Paused {}, Position_saved {}",
            py_bool(self.error_state.get()),
            py_bool(self.function.is_paused()?),
            py_bool(self.position_saved.get())
        );
        if with_toolchange {
            s += &format!(", in toolchange: {}", py_bool(self.in_toolchange.get()));
        }
        Ok(s)
    }

    fn saved_position_msg(&self, what: &str) -> String {
        let mut msg = format!("{} {}", what, round_floats_list(&self.last_toolhead_position.borrow()));
        msg += &format!(" Base position: {}", round_floats_list(&self.base_position.borrow()));
        msg += &format!(" last_gcode_position: {}", round_floats_list(&self.last_gcode_position.borrow()));
        msg += &format!(" homing_position: {}", round_floats_list(&self.homing_position.borrow()));
        msg += &format!(" speed: {}", round_float(self.speed.get()));
        msg += &format!(" speed_factor: {}", round_float(self.speed_factor.get()));
        msg += &format!(" absolute_coord: {}", py_bool(self.absolute_coord.get()));
        msg += &format!(" absolute_extrude: {}", py_bool(self.absolute_extrude.get()));
        msg += &format!(" extrude_factor: {}\n", round_float(self.extrude_factor.get()));
        msg
    }

    /// Saves the position on the first toolchange call (an error state must
    /// not overwrite it)
    pub fn save_pos(&self) -> KResult<()> {
        if !self.function.is_homed(true)? {
            self.function.log_toolhead_pos(&format!("Not Saving unhomed position, {}, POS: ", self.position_state(true)?))?;
            return Ok(());
        }
        if !self.in_toolchange.get() {
            if !self.error_state.get() && !self.function.is_paused()? && !self.position_saved.get() {
                let gm = self.gcode_move()?;
                self.position_saved.set(true);
                *self.last_toolhead_position.borrow_mut() = self.toolhead()?.get_position();
                *self.base_position.borrow_mut() = gm.base_position.borrow().clone();
                *self.last_gcode_position.borrow_mut() = gm.last_position.borrow().clone();
                *self.homing_position.borrow_mut() = gm.homing_position.borrow().clone();
                self.speed.set(gm.speed.get());
                self.speed_factor.set(gm.speed_factor.get());
                self.absolute_coord.set(gm.absolute_coord.get());
                self.absolute_extrude.set(gm.absolute_extrude.get());
                self.extrude_factor.set(gm.extrude_factor.get());
                self.logger.debug(&AFC_AFC_SAVE_POS, &self.saved_position_msg("Saving position"));
            } else {
                self.function.log_toolhead_pos(&format!("Not Saving, {}, POS: ", self.position_state(false)?))?;
            }
        } else {
            self.function.log_toolhead_pos(&format!("Not Saving In a toolchange, {}, POS: ", self.position_state(true)?))?;
        }
        Ok(())
    }

    /// Restores the saved position, speed and coordinate modes: z (with
    /// z_hop) first, then x/y, then back down
    pub fn restore_pos(&self, move_z_first: bool) -> KResult<()> {
        if !self.position_saved.get() {
            self.function.log_toolhead_pos(&format!("Not restoring position, {}, POS: ", self.position_state(true)?))?;
            return Ok(());
        }
        self.logger.debug(&AFC_AFC_RESTORE_POS, &self.saved_position_msg("Restoring Position"));
        self.function.log_toolhead_pos("Resume initial pos: ")?;

        self.current_state.set(State::RestoringPos);
        let gm = self.gcode_move()?;
        let last_gcode = self.last_gcode_position.borrow().clone();
        if move_z_first {
            self.move_z_pos(last_gcode[2] + self.z_hop, "restore_pos", false)?;
        }
        // newpos is gcode_move.last_position itself
        let newpos = {
            let mut lp = gm.last_position.borrow_mut();
            lp[0] = last_gcode[0];
            lp[1] = last_gcode[1];
            lp.clone()
        };
        gm.move_with_transform(&newpos, self.get_resume_speed())?;
        self.function.log_toolhead_pos("Resume prev xy: ")?;

        // Update GCODE STATE variables
        *gm.base_position.borrow_mut() = self.base_position.borrow().clone();
        *gm.homing_position.borrow_mut() = self.homing_position.borrow().clone();
        gm.absolute_coord.set(self.absolute_coord.get());
        gm.absolute_extrude.set(self.absolute_extrude.get());
        gm.extrude_factor.set(self.extrude_factor.get());
        gm.speed.set(self.speed.get());
        gm.speed_factor.set(self.speed_factor.get());

        // Restore the relative E position
        let e_diff = gm.last_position.borrow()[3] - last_gcode[3];
        gm.base_position.borrow_mut()[3] += e_diff;
        let lp = {
            let mut lp = gm.last_position.borrow_mut();
            lp[..3].copy_from_slice(&last_gcode[..3]);
            lp.clone()
        };
        gm.move_with_transform(&lp, self.get_resume_speedz())?;
        self.function.log_toolhead_pos(&format!("Resume final z, {}, POS: ", self.position_state(true)?))?;

        self.current_state.set(State::Idle);
        self.position_saved.set(false);
        Ok(())
    }

    /// Writes the lane variables to VarFile.unit (on the writer thread);
    /// nothing until PREP has run
    pub fn save_vars(&self) -> KResult<()> {
        if !self.prep_done.get() {
            return Ok(());
        }
        let mut data = PyDict::default();
        let units: Vec<Rc<AfcUnit>> = self.units.borrow().values().cloned().collect();
        for cur_unit in units {
            let mut unit_dict = PyDict::default();
            let lane_names: Vec<String> = cur_unit.lanes.borrow().keys().cloned().collect();
            for name in lane_names {
                let cur_lane = self.lane(&name).ok_or_else(|| KError::Py("KeyError".into(), py_repr(&name)))?;
                unit_dict.insert(cur_lane.name.clone().into(), cur_lane.get_status(true)?);
            }
            data.insert(cur_unit.name.clone().into(), PyVal::Dict(unit_dict));
        }
        let mut system = PyDict::default();
        system.insert("current_load".into(), self.current().map(PyVal::Str).unwrap_or(PyVal::None));
        system.insert("num_units".into(), PyVal::Int(self.units.borrow().len() as i64));
        system.insert("num_lanes".into(), PyVal::Int(self.lanes.borrow().len() as i64));
        system.insert("num_extruders".into(), PyVal::Int(self.tools.borrow().len() as i64));
        let mut extruders = PyDict::default();
        system.insert("bypass".into(), PyVal::Dict(kaliru::pydict! { "enabled" => PyVal::Bool(self.get_bypass_state()) }));
        for cur_extruder in self.tools.borrow().values() {
            let mut e = kaliru::pydict! { "lane_loaded" => cur_extruder.lane_loaded.borrow().clone().map(PyVal::Str).unwrap_or(PyVal::None) };
            if cur_extruder.tool_start.as_deref() == Some("virtual") {
                e.insert("virtual_tool_start".into(), PyVal::Bool(cur_extruder.tool_start_state.get()));
            }
            extruders.insert(cur_extruder.name.clone().into(), PyVal::Dict(e));
        }
        // (python fills "extruders" after inserting "bypass" into the dict)
        let bypass = system.shift_remove("bypass");
        system.insert("extruders".into(), PyVal::Dict(extruders));
        if let Some(b) = bypass {
            system.insert("bypass".into(), b);
        }
        data.insert("system".into(), PyVal::Dict(system));
        let target = format!("{}.unit", self.var_file);
        let a = self.rc();
        self.var_writer.write(target, PyVal::Dict(data), Box::new(move |err| a.log_save_vars_error(&err)));
        Ok(())
    }

    fn join_threads(&self) {
        self.var_writer.stop();
        if let Some(m) = self.moonraker.borrow().as_ref() {
            m.join_thread();
        }
    }

    fn log_save_vars_error(&self, err: &str) {
        self.logger.error(&AFC_AFC__LOG_SAVE_VARS_ERROR, "Error happened when trying to save variables, check AFC.log for error");
        self.logger.debug_opt(&AFC_AFC__LOG_SAVE_VARS_ERROR, err, true, None);
    }

    fn cmd_hub_load(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_AFC_CMD_HUB_LOAD;
        if self.function.is_printing(false)? {
            self.error.afc_error(c, "Cannot load lane to hub while printer is printing", false, None)?;
            return Ok(());
        }
        let lane = gcmd.get_opt("LANE");
        let Some(cur_lane) = lane.as_deref().and_then(|l| self.lane(l)) else {
            self.logger.info(c, &format!("{} Unknown", py_opt_str(lane.as_deref())));
            return Ok(());
        };
        let cur_hub = cur_lane.hub_obj()?;
        if !cur_lane.prep_state.get() {
            return Ok(());
        }
        cur_lane.set_status(LaneState::HubLoading);
        while !cur_lane.load_state() {
            cur_lane.move_to(cur_hub.move_dis(), SpeedMode::Short, &cur_lane.load_es(), AssistActive::Dynamic, self.homing_enabled)?;
        }
        if !cur_lane.loaded_to_hub.get() {
            let mut dist_to_hub = cur_lane.dist_hub.get();
            if self.homing_enabled {
                dist_to_hub = cur_lane.dist_hub.get() + cur_hub.move_dis();
            }
            cur_lane.unit_obj()?.move_to_hub(&cur_lane, dist_to_hub, MoveDirection::Pos, self.homing_enabled, SpeedMode::Hub)?;
        }
        while !cur_hub.state() {
            cur_lane.unit_obj()?.move_to_hub(&cur_lane, cur_hub.move_dis(), MoveDirection::Pos, self.homing_enabled, SpeedMode::Hub)?;
        }
        cur_lane.move_to(cur_hub.hub_clear_move_dis() * MoveDirection::Neg.value(), SpeedMode::Hub, &lane::Endstop::None, AssistActive::Yes, false)?;
        cur_lane.set_status(LaneState::None);
        cur_lane.do_enable(false)?;
        cur_lane.loaded_to_hub.set(true);
        self.save_vars()?;
        cur_lane.unit_obj()?.return_to_home(false)?;
        self.function.select_loaded_lane()
    }

    fn cmd_lane_unload(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_AFC_CMD_LANE_UNLOAD;
        if self.function.is_printing(false)? {
            self.error.afc_error(c, "Cannot eject lane while printer is printing", false, None)?;
            return Ok(());
        }
        let lane = gcmd.get_opt("LANE");
        let Some(cur_lane) = lane.as_deref().and_then(|l| self.lane(l)) else {
            self.logger.info(c, &format!("{} Unknown", py_opt_str(lane.as_deref())));
            return Ok(());
        };
        self.lane_unload(&cur_lane)
    }

    /// Ejects a lane that is not loaded in the toolhead
    pub fn lane_unload(&self, cur_lane: &Rc<AfcLane>) -> KResult<()> {
        let c = &AFC_AFC_LANE_UNLOAD;
        self.current_state.set(State::EjectingLane);
        let extruder = cur_lane.extruder_obj()?;
        let lane_loaded = extruder.lane_loaded.borrow().clone();
        if lane_loaded.as_deref() != Some(cur_lane.name.as_str()) && !extruder.is_standalone() {
            cur_lane.unit_obj()?.lane_unloading(cur_lane)?;
            cur_lane.set_status(LaneState::Ejecting);
            self.save_vars()?;
            cur_lane.unit_obj()?.eject_lane(cur_lane)?;
            cur_lane.loaded_to_hub.set(false);
            cur_lane.set_status(LaneState::None);
            cur_lane.unit_obj()?.return_to_home(false)?;
            self.function.select_loaded_lane()?;
            self.save_vars()?;
            if self.clear_spool_after_eject {
                self.spool.set_spool_id(cur_lane, PyVal::None, true, None)?;
                cur_lane.unit_obj()?.lane_not_ready(cur_lane)?;
            }
            self.logger.info(c, &format!("LANE {} eject done", cur_lane.name));
        } else if extruder.is_standalone() && lane_loaded.is_some() {
            cur_lane.set_status(LaneState::Ejecting);
            extruder.load_unload_sequence(-extruder.tool_stn_unload.get())?;
            self.save_vars()?;
        } else if lane_loaded.as_deref() == Some(cur_lane.name.as_str()) {
            self.logger.warning(c, &format!("LANE {} is loaded in toolhead, can't unload. Run TOOL_UNLOAD first.", cur_lane.name));
        } else {
            self.logger.warning(c, &format!("LANE {} not ejected: standalone extruder reports no lane loaded.", cur_lane.name));
        }
        self.current_state.set(State::Idle);
        Ok(())
    }

    /// The enabled poop / wipe / kick macros
    pub fn do_poop_kick_wipe(&self, cur_lane: &AfcLane, cur_extruder: &AfcExtruder, purge_length: Option<&PyVal>) -> KResult<()> {
        let c = &AFC_AFC_DO_POOP_KICK_WIPE;
        let _ = c;
        if self.poop.get() {
            let poop_cmd = py_opt_str(self.poop_cmd.as_deref());
            match purge_length {
                Some(p) => self.gcode.run_script_from_command(&format!("{} PURGE_LENGTH={} EXTRUDER={}", poop_cmd, p.str(), cur_extruder.name))?,
                None => self.gcode.run_script_from_command(&format!("{} EXTRUDER={}", poop_cmd, cur_extruder.name))?,
            }
            self.delta_time.log_with_time(&AFC_AFC_DO_POOP_KICK_WIPE, "TOOL_LOAD: After poop", true);
            self.function.log_toolhead_pos("")?;
            if self.wipe.get() {
                self.gcode.run_script_from_command(&format!("{} EXTRUDER={}", py_opt_str(self.wipe_cmd.as_deref()), cur_extruder.name))?;
                self.delta_time.log_with_time(&AFC_AFC_DO_POOP_KICK_WIPE, "TOOL_LOAD: After first wipe", true);
                self.function.log_toolhead_pos("")?;
            }
        }
        if self.kick.get() {
            self.gcode.run_script_from_command(&format!("{} EXTRUDER={}", py_opt_str(self.kick_cmd.as_deref()), cur_extruder.name))?;
            self.delta_time.log_with_time(&AFC_AFC_DO_POOP_KICK_WIPE, "TOOL_LOAD: After kick", true);
            self.function.log_toolhead_pos("")?;
        }
        if self.wipe.get() {
            self.gcode.run_script_from_command(&format!("{} EXTRUDER={}", py_opt_str(self.wipe_cmd.as_deref()), cur_extruder.name))?;
            self.delta_time.log_with_time(&AFC_AFC_DO_POOP_KICK_WIPE, "TOOL_LOAD: After second wipe", true);
            self.function.log_toolhead_pos("")?;
        }
        self.toolhead()?.wait_moves()?;
        cur_lane.need_purge.set(false);
        Ok(())
    }

    fn cmd_tool_load(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let c = &AFC_AFC_CMD_TOOL_LOAD;
        let lane = gcmd.get_opt("LANE");
        let Some(cur_lane) = lane.as_deref().and_then(|l| self.lane(l)) else {
            self.logger.info(c, &format!("{} Unknown", py_opt_str(lane.as_deref())));
            return Ok(());
        };
        if cur_lane.extruder_obj()?.lane_loaded.borrow().as_deref() == Some(cur_lane.name.as_str()) {
            self.error.afc_error(c, &format!("Not loading {}, already loaded", cur_lane.name), self.function.in_print()?, None)?;
            return Ok(());
        }
        let purge_length = gcmd.get_opt("PURGE_LENGTH").map(PyVal::Str);
        if !self.tool_load(&cur_lane, purge_length.as_ref(), false)? {
            self.afc_stats()?.increase_load_error_count(self)?;
        }
        Ok(())
    }

    /// Loads a lane into its tool; Ok(false) when an error was reported
    pub fn tool_load(&self, cur_lane: &Rc<AfcLane>, purge_length: Option<&PyVal>, set_start_time: bool) -> KResult<bool> {
        let c = &AFC_AFC_TOOL_LOAD;
        if !self.function.check_homed(None, c)? {
            return Ok(false);
        }
        if set_start_time {
            self.delta_time.set_start_time();
        }
        let error_str = self.verify_macro_positions();
        if !error_str.is_empty() {
            self.error.afc_error(c, &format!("Error occurred when verifying macro positions, TOOL_LOAD aborted.\n{}", error_str), self.function.in_print()?, None)?;
            return Ok(false);
        }
        if self.check_bypass(false, c) {
            return Ok(false);
        }
        self.function.check_absolute_mode("TOOL_LOAD")?;

        // If the current extruder is not the one associated with the lane, switch to it.
        if self.function.get_current_extruder()? != Some(cur_lane.extruder_obj()?.th_extruder_name.clone()) {
            cur_lane.tool_swap()?;
        }
        // After a tool swap the active extruder may have another lane loaded
        let cur_extruder_obj = cur_lane.extruder_obj()?;
        let loaded = cur_extruder_obj.lane_loaded.borrow().clone();
        if let Some(lane_name) = loaded.filter(|l| *l != cur_lane.name) {
            let Some(loaded_lane) = self.lane(&lane_name) else {
                self.error.afc_error(
                    c,
                    &format!("Extruder \"{}\" has lane {} that is not in configured lanes.", cur_extruder_obj.name, lane_name),
                    self.function.in_print()?,
                    None,
                )?;
                return Ok(false);
            };
            if !self.tool_unload(&loaded_lane, false, false)? {
                let msg = format!(
                    "Failed to unload currently loaded lane {} from extruder {} before loading new lane {}.",
                    lane_name, cur_extruder_obj.name, cur_lane.name
                );
                self.error.fix(Some(&msg), Some(error::LaneRef::Lane(loaded_lane.clone())))?;
                self.afc_stats()?.increase_unload_error_count(self)?;
                return Ok(false);
            }
        }

        if Some(cur_lane.name.clone()) != self.current() {
            let cur_hub = cur_lane.hub_obj()?;
            if cur_lane.load_state() && (!cur_hub.state() || cur_lane.is_direct_hub()) {
                self.logger.info(c, &format!("Loading {}", cur_lane.name));
                let cur_extruder = cur_lane.extruder_obj()?;
                self.current_state.set(State::Loading);
                *self.current_loading.borrow_mut() = Some(cur_lane.name.clone());
                cur_lane.set_status(LaneState::ToolLoading);
                self.save_vars()?;
                cur_lane.unit_obj()?.lane_loading(cur_lane)?;

                let temp_state = self.capture_toolhead_temp(None, false)?;
                let res = (|| -> KResult<bool> {
                    self.set_display_status("pushing", true);
                    let r = self.load_sequence(cur_lane, &cur_hub, &cur_extruder);
                    self.set_display_status("pushing", false);
                    if !r? {
                        return Ok(false);
                    }
                    cur_lane.unit_obj()?.lane_tool_loaded(cur_lane)?;
                    cur_lane.espooler.do_assist_move(None)?;
                    self.do_poop_kick_wipe(cur_lane, &cur_extruder, purge_length)?;
                    cur_lane.enable_fault_detection()?;
                    *cur_extruder.lane_loaded.borrow_mut() = Some(cur_lane.name.clone());
                    self.spool.set_active_spool(&cur_lane.spool_id.borrow())?;
                    cur_lane.unit_obj()?.lane_tool_loaded(cur_lane)?;
                    self.save_vars()?;
                    self.current_state.set(State::Idle);
                    cur_lane.get_td1_data_load()?;
                    let load_time = self.delta_time.log_major_delta(&AFC_AFC_TOOL_LOAD, &format!("{} is now loaded in toolhead", cur_lane.name), false);
                    self.afc_stats()?.average_tool_load_time.average_time(load_time);
                    cur_lane.extruder_obj()?.estats.tc_tool_load().increase_count()?;
                    cur_lane.lane_load_count()?.increase_count()?;
                    cur_lane.espooler.stats()?.update_database();
                    if let Some(m) = &self.post_load_macro {
                        self.gcode.run_script_from_command(m)?;
                    }
                    Ok(true)
                })();
                self.restore_toolhead_temp(temp_state.as_ref(), false)?;
                if !res? {
                    return Ok(false);
                }
            } else {
                let in_print = self.function.in_print()?;
                if cur_hub.state() {
                    let mut message = String::from("Hub not clear when trying to load.\nPlease check that hub does not contain broken filament and is clear");
                    if in_print {
                        message += &format!("\nOnce issue is resolved please manually load {} ", cur_lane.name);
                        message += &format!("with {} macro and click resume to continue printing.", cur_lane.current_map.borrow());
                    }
                    self.error.handle_lane_failure(c, cur_lane, &message, in_print)?;
                    return Ok(false);
                }
                if !cur_lane.load_state() {
                    let mut message = String::from("Current lane not loaded, LOAD TRIGGER NOT TRIGGERED\n||==>--||----||-----||\nTRG   LOAD   HUB   TOOL");
                    message += "\nPlease load lane before continuing.";
                    if in_print {
                        message += &format!("\nOnce issue is resolved please manually load {} ", cur_lane.name);
                        message += &format!("with {} macro and click resume to continue printing.", cur_lane.current_map.borrow());
                    }
                    self.error.handle_lane_failure(c, cur_lane, &message, in_print)?;
                    return Ok(false);
                }
            }
        }

        // Check if toolhead needs to purge, this normally should only apply for standalone toolheads
        if cur_lane.need_purge.get() {
            let temp_state = self.capture_toolhead_temp(None, false)?;
            let res = (|| -> KResult<()> {
                let ext = cur_lane.extruder_obj()?;
                self.logger.info(c, &format!("Flag set to purge for {}:{}", ext.name, cur_lane.current_map.borrow()));
                if self.check_extruder_temp(cur_lane, false)? {
                    self.delta_time.log_with_time(c, "Done heating toolhead", true);
                }
                self.do_poop_kick_wipe(cur_lane, &ext, purge_length)?;
                if let Some(m) = &self.post_load_macro {
                    self.gcode.run_script_from_command(m)?;
                }
                Ok(())
            })();
            let ok = match res {
                Ok(()) => true,
                Err(e) => {
                    let ext_name = cur_lane.extruder_obj()?.name.clone();
                    self.error.afc_error(c, &format!("Error occurred when trying to purge {}. See AFC.log for error trace.", ext_name), self.function.in_print()?, None)?;
                    self.logger.debug(c, &format!("Exception: {}\n{}", e.message(), kaliru::logging::format_traceback(&e)));
                    false
                }
            };
            self.restore_toolhead_temp(temp_state.as_ref(), false)?;
            self.save_vars()?;
            if !ok {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// The load sequence (or the lane's custom load command)
    fn load_sequence(&self, cur_lane: &Rc<AfcLane>, cur_hub: &AfcHub, cur_extruder: &AfcExtruder) -> KResult<bool> {
        let c = &AFC_AFC_LOAD_SEQUENCE;
        if self.park_pre_load {
            if let Some(cmd) = &self.park_pre_load_cmd {
                self.gcode.run_script_from_command(cmd)?;
            }
        }
        if self.check_extruder_temp(cur_lane, false)? {
            self.delta_time.log_with_time(c, "Done heating toolhead", true);
        }
        let custom_load_cmd = cur_lane.custom_load_cmd.borrow().clone();
        if let Some(cmd) = custom_load_cmd.filter(|c| !c.is_empty()) {
            let load_state_commands = format!(
                "\nSet the lane as unloaded with:\nUNSET_LANE_LOADED LANE={}\nOr set the lane as loaded with:\nSET_LANE_LOADED LANE={}",
                cur_lane.name, cur_lane.name
            );
            if cur_lane.status() == LaneState::Error {
                let message = format!("Cannot load lane {}: lane is in an error state.", cur_lane.name);
                self.error.afc_error(c, &(message + &load_state_commands), false, None)?;
                return Ok(false);
            }
            self.logger.info(c, &format!("Running custom load command for lane {}", cur_lane.name));
            self.gcode.run_script_from_command(&cmd)?;
            if cur_lane.status() == LaneState::Error {
                let message = format!("Custom load command for lane {} left the lane in an error state.", cur_lane.name);
                self.logger.warning(c, &(message + &load_state_commands));
                return Ok(false);
            }
            if cur_lane.get_toolhead_pre_sensor_state()? {
                cur_lane.set_status(LaneState::ToolLoaded);
                self.save_vars()?;
            } else {
                let mut message = String::from("Custom load command did not trigger pre extruder gear toolhead sensor, CHECK FILAMENT PATH\n||=====||====||==>--||\nTRG   LOAD   HUB   TOOL");
                message += &format!("\nTo resolve set lane loaded with `SET_LANE_LOADED LANE={}` macro.", cur_lane.name);
                message += "\nManually move filament until filament is right before toolhead extruder gears,";
                message += "\nthen load into extruder gears with extrude button in your gui of choice until the color fully changes";
                if self.function.in_print()? {
                    message += "\nOnce filament is fully loaded click resume to continue printing";
                }
                self.error.handle_lane_failure(c, cur_lane, &message, true)?;
                return Ok(false);
            }
        } else if let Some(ok) = cur_lane.unit_obj()?.unit_load_lane(cur_lane, cur_extruder)? {
            if !ok {
                return Ok(false);
            }
        } else if !self.standard_load_sequence(cur_lane, cur_hub, cur_extruder)? {
            return Ok(false);
        }
        // Update tool and lane status.
        cur_lane.set_tool_loaded(true)?;
        cur_lane.enable_buffer(true)?;
        self.save_vars()?;
        Ok(true)
    }
}

/// re.search(r'\d+', s) as an int
pub fn first_number(s: &str) -> Option<i64> {
    let start = s.find(|c: char| c.is_ascii_digit())?;
    let digits: String = s[start..].chars().take_while(|c| c.is_ascii_digit()).collect();
    digits.parse().ok()
}

/// str(bool)
pub fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

/// str() of an optional string (None)
pub fn py_opt_str(s: Option<&str>) -> String {
    s.map(|s| s.to_string()).unwrap_or_else(|| "None".into())
}

pub fn py_repr(s: &str) -> String {
    kaliru::pyfmt::py_str_repr(s)
}

/// the directory AFC.py is in (os.path.dirname(os.path.realpath(__file__))):
/// klippy/extras next to the klippy kaliru runs as
fn afc_extras_dir(printer: &Printer) -> String {
    let _ = printer;
    let exe = std::env::current_exe().map(|p| p.to_string_lossy().into_owned()).unwrap_or_default();
    let klippy = std::env::var("KALIRU_KLIPPY_DIR").unwrap_or_else(|_| kaliru::pyglob::dirname(&exe));
    let dir = kaliru::pyglob::join(&klippy, &["extras"]);
    std::fs::canonicalize(&dir).map(|p| p.to_string_lossy().into_owned()).unwrap_or(dir)
}

/// The afc_virtual_bypass / afc_quiet_mode pin chips: AFC's virtual pins
/// (register_config_callback does nothing, so their buttons never fire)
struct VirtualChip;

impl PinChip for VirtualChip {
    fn setup_pin(&self, _pin_type: &str, _pin_params: &SharedPinParams) -> Result<Rc<dyn Any>, String> {
        Err("'afc' object has no attribute 'setup_pin'".into())
    }
    fn register_config_callback(&self, _cb: Rc<dyn Fn() -> KResult<()>>) -> Option<KResult<()>> {
        Some(Ok(()))
    }
}

impl PrinterObject for Afc {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        self.status().ok()
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(Afc::new(config)?)
}

/// Which move speeds are still python's int defaults: config.getfloat hands a
/// default back unchanged, so AFC's 25 / 100 / 400 stay ints through the units
/// and lanes that inherit them, and print as "25", not "25.0"
#[derive(Clone, Copy, Default)]
pub struct MoveInts {
    pub long_speed: bool,
    pub long_accel: bool,
    pub short_speed: bool,
    pub short_accel: bool,
}

/// config.getfloat(name, default): the value as python has it (a default
/// comes back unconverted, so an int default stays an int)
pub fn getfloat_pv(config: &ConfigWrapper, name: &str, default: PyVal) -> KResult<PyVal> {
    Ok(config.getfloat_value(name, &kaliru::configfile::Dflt::Val(default.clone()), &Lim::new())?.unwrap_or(default))
}

/// config.getfloat(name, default) as (value, whether it is still an int)
pub fn getfloat_int(config: &ConfigWrapper, name: &str, default: PyVal) -> KResult<(f64, bool)> {
    let v = config.getfloat_value(name, &kaliru::configfile::Dflt::Val(default), &Lim::new())?.unwrap_or(PyVal::Float(0.0));
    Ok((v.as_f64().unwrap_or(0.0), matches!(v, PyVal::Int(_))))
}
