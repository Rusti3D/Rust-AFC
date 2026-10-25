// An AFC hub: where a unit's lanes merge (port of AFC_hub.py)
//
// Copyright (C) 2024-2026 Armored Turtle
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::lane::AfcLane;
use super::utils::{add_filament_switch, SwitchOptions};
use super::Afc;
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::extras::buttons::PrinterButtons;
use kaliru::extras::filament_switch_sensor::SwitchSensor;
use kaliru::printer::{PObj, Printer, PrinterObject};
use kaliru::pyval::PyVal;
use kaliru::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

/// "afc_hub:register_macros"
pub const REGISTER_MACROS_EVENT: &str = "afc_hub:register_macros";

/// A lane's buffer: AFC_buffer is outside the native scope, so lanes have none
pub struct NoBuffer;

impl NoBuffer {
    pub fn update_filament_error_pos(&self) {}
}

/// [AFC_hub <name>]
pub struct AfcHub {
    me: Weak<AfcHub>,
    printer: Rc<Printer>,
    afc: Rc<Afc>,
    pub fullname: String,
    pub name: String,
    pub lanes: RefCell<IndexMap<String, Rc<AfcLane>>>,
    state_raw: Cell<bool>,
    pub switch_pin: Option<String>,
    hub_clear_move_dis: f64,
    pub afc_bowden_length: Cell<f64>,
    pub td1_bowden_length: f64,
    pub afc_unload_bowden_length: Cell<f64>,
    pub assisted_retract: bool,
    move_dis: f64,
    pub cut: bool,
    pub cut_cmd: Option<String>,
    cut_servo_name: String,
    cut_dist: f64,
    cut_clear: f64,
    cut_min_length: f64,
    cut_servo_pass_angle: f64,
    cut_servo_clip_angle: f64,
    cut_servo_prep_angle: f64,
    cut_confirm: bool,
    pub config_bowden_length: f64,
    pub config_unload_bowden_length: f64,
    use_dist_hub: bool,
    fila: RefCell<Option<Rc<SwitchSensor>>>,
    pub enable_runout: bool,
    /// AFC_ACE: a spent spool's tail below the hub sensor (chained ACEs
    /// share the bowden)
    pub ace_tail_in_bowden: Cell<bool>,
}

impl AfcHub {
    fn new(config: &ConfigWrapper) -> KResult<Rc<AfcHub>> {
        let printer = config.get_printer();
        let afc = printer.lookup_or_load::<Afc>(config, "AFC")?;
        let fullname = config.get_name().to_string();
        let name = fullname.split_whitespace().last().unwrap_or("").to_string();
        let switch_pin = config.get_opt("switch_pin")?;
        let hub_clear_move_dis = config.getfloat("hub_clear_move_dis", 65.0, Lim::new())?;
        let afc_bowden_length = config.getfloat("afc_bowden_length", 900.0, Lim::new())?;
        let td1_bowden_length = config.getfloat("td1_bowden_length", afc_bowden_length - 50.0, Lim::new())?;
        let afc_unload_bowden_length = config.getfloat("afc_unload_bowden_length", afc_bowden_length, Lim::new())?;
        let assisted_retract = config.getboolean("assisted_retract", false)?;
        let move_dis = config.getfloat("move_dis", 75.0, Lim::new())?;
        let cut = config.getboolean("cut", false)?;
        let cut_cmd = config.get_opt("cut_cmd")?;
        let cut_servo_name = config.get("cut_servo_name", "cut")?;
        let cut_dist = config.getfloat("cut_dist", 50.0, Lim::new())?;
        let cut_clear = config.getfloat("cut_clear", 120.0, Lim::new())?;
        let cut_min_length = config.getfloat("cut_min_length", 200.0, Lim::new())?;
        let cut_servo_pass_angle = config.getfloat("cut_servo_pass_angle", 0.0, Lim::new())?;
        let cut_servo_clip_angle = config.getfloat("cut_servo_clip_angle", 160.0, Lim::new())?;
        let cut_servo_prep_angle = config.getfloat("cut_servo_prep_angle", 75.0, Lim::new())?;
        let cut_confirm = config.getboolean("cut_confirm", false)?;
        let enable_sensors_in_gui = config.getboolean("enable_sensors_in_gui", afc.enable_sensors_in_gui)?;
        let debounce_delay = config.getfloat("debounce_delay", afc.debounce_delay, Lim::new())?;
        let enable_runout = config.getboolean("enable_hub_runout", afc.enable_hub_runout)?;
        let use_dist_hub = config.getboolean("use_dist_hub", false)?;
        let hub = Rc::new_cyclic(|me| AfcHub {
            me: me.clone(),
            printer: printer.clone(),
            afc: afc.clone(),
            fullname,
            name: name.clone(),
            lanes: RefCell::new(IndexMap::default()),
            state_raw: Cell::new(false),
            switch_pin: switch_pin.clone(),
            hub_clear_move_dis,
            afc_bowden_length: Cell::new(afc_bowden_length),
            td1_bowden_length,
            afc_unload_bowden_length: Cell::new(afc_unload_bowden_length),
            assisted_retract,
            move_dis,
            cut,
            cut_cmd,
            cut_servo_name,
            cut_dist,
            cut_clear,
            cut_min_length,
            cut_servo_pass_angle,
            cut_servo_clip_angle,
            cut_servo_prep_angle,
            cut_confirm,
            config_bowden_length: afc_bowden_length,
            config_unload_bowden_length: afc_unload_bowden_length,
            use_dist_hub,
            fila: RefCell::new(None),
            enable_runout,
            ace_tail_in_bowden: Cell::new(false),
        });
        let h = hub.clone();
        kaliru::events::KlippyConnect::on(&printer, move || h.handle_connect());
        let h = hub.clone();
        kaliru::events::KlippyReady::on(&printer, move || h.handle_ready());
        if let Some(pin) = switch_pin.filter(|p| p.to_lowercase() != "virtual") {
            let buttons = printer.lookup_or_load::<PrinterButtons>(config, "buttons")?;
            let w = Rc::downgrade(&hub);
            let (fila, _button) = add_filament_switch(
                &format!("{}_Hub", name),
                &pin,
                &printer,
                &SwitchOptions {
                    show_sensor: enable_sensors_in_gui,
                    runout_callback: Some(Rc::new(move |et| w.upgrade().map_or(Ok(()), |h| h.handle_runout(et)))),
                    enable_runout,
                    debounce_delay,
                },
            )?;
            *hub.fila.borrow_mut() = Some(fila);
            let w = Rc::downgrade(&hub);
            buttons.register_buttons(
                &[pin.as_str()],
                Rc::new(move |_et, state| {
                    if let Some(h) = w.upgrade() {
                        h.state_raw.set(state != 0);
                    }
                    Ok(())
                }),
            )?;
        }
        afc.hubs.borrow_mut().insert(name, hub.clone());
        Ok(hub)
    }

    pub fn is_virtual_pin(&self) -> bool {
        self.switch_pin.as_deref().is_some_and(|p| p.to_lowercase() == "virtual")
    }

    pub fn move_dis(&self) -> f64 {
        self.move_dis
    }

    pub fn hub_clear_move_dis(&self) -> f64 {
        self.hub_clear_move_dis
    }

    pub fn use_dist_hub(&self) -> bool {
        self.use_dist_hub
    }

    /// The hub's delayed runout: for the loaded lane, when it is this hub's
    fn handle_runout(&self, _eventtime: f64) -> KResult<()> {
        if let Some(cur) = self.afc.current() {
            let lane = self.lanes.borrow().get(&cur).cloned();
            if let Some(lane) = lane {
                lane.handle_hub_runout(&self.name)?;
            }
        }
        if let Some(f) = self.fila.borrow().as_ref() {
            let rh = &f.runout_helper;
            rh.min_event_systime.set(self.afc.reactor.monotonic() + rh.event_delay.get());
        }
        Ok(())
    }

    fn handle_connect(&self) -> KResult<()> {
        let me = self.me.upgrade().expect("AFC hub");
        self.printer.send_event(REGISTER_MACROS_EVENT, &[&me])
    }

    fn handle_ready(&self) -> KResult<()> {
        if self.is_virtual_pin() {
            let mut msg = String::from("The following lanes need load sensors for virtual hub sensor to work correctly:");
            let mut report_error = false;
            for lane in self.lanes.borrow().values() {
                if lane.unit_obj()?.type_name() == "OpenAMS" {
                    continue;
                }
                if lane.load.is_none() && lane.prep.is_some() {
                    report_error = true;
                    msg += &format!("\n{}", lane.fullname);
                }
            }
            if report_error {
                return Err(KError::config(msg));
            }
        }
        Ok(())
    }

    /// The switch, or for a virtual hub: any lane's load sensor
    pub fn state(&self) -> bool {
        if self.is_virtual_pin() {
            return self.lanes.borrow().values().any(|l| l.raw_load_state());
        }
        self.state_raw.get()
    }

    /// hub_cut: feed to the switch, find its edge, feed cut_dist, cut
    pub fn hub_cut(&self, cur_lane: &AfcLane) -> KResult<()> {
        let gcode = &self.afc.gcode;
        let servo = |angle: f64| gcode.run_script_from_command(&format!("SET_SERVO SERVO={} ANGLE={}", self.cut_servo_name, kaliru::pyfmt::py_float_repr(angle)));
        let reactor = &self.afc.reactor;
        let (speed, accel) = (cur_lane.short_moves_speed(), cur_lane.short_moves_accel());
        servo(self.cut_servo_prep_angle)?;
        while !self.state() {
            cur_lane.move_lane(self.move_dis, speed, accel, false)?;
        }
        while self.state() {
            cur_lane.move_lane(-10.0, speed, accel, self.assisted_retract)?;
        }
        while !self.state() {
            cur_lane.move_lane(2.0, speed, accel, false)?;
        }
        cur_lane.move_lane(self.cut_dist, speed, accel, false)?;
        reactor.pause(reactor.monotonic() + 0.5)?;
        servo(self.cut_servo_clip_angle)?;
        if self.cut_confirm {
            reactor.pause(reactor.monotonic() + 0.5)?;
            servo(self.cut_servo_prep_angle)?;
            reactor.pause(reactor.monotonic() + 1.0)?;
            servo(self.cut_servo_clip_angle)?;
        }
        reactor.pause(reactor.monotonic() + 1.0)?;
        servo(self.cut_servo_pass_angle)?;
        cur_lane.move_lane(-self.cut_clear, speed, accel, self.assisted_retract)
    }

    pub fn get_status(&self) -> PyVal {
        let opt = |v: &Option<String>| v.clone().map(PyVal::Str).unwrap_or(PyVal::None);
        PyVal::Dict(kaliru::pydict! {
            "state" => PyVal::Bool(self.state()),
            "cut" => PyVal::Bool(self.cut),
            "cut_cmd" => opt(&self.cut_cmd),
            "cut_dist" => PyVal::Float(self.cut_dist),
            "cut_clear" => PyVal::Float(self.cut_clear),
            "cut_min_length" => PyVal::Float(self.cut_min_length),
            "cut_servo_pass_angle" => PyVal::Float(self.cut_servo_pass_angle),
            "cut_servo_clip_angle" => PyVal::Float(self.cut_servo_clip_angle),
            "cut_servo_prep_angle" => PyVal::Float(self.cut_servo_prep_angle),
            "lanes" => PyVal::List(self.lanes.borrow().values().map(|l| PyVal::Str(l.name.clone())).collect()),
            "afc_bowden_length" => PyVal::Float(self.afc_bowden_length.get()),
        })
    }
}

impl PrinterObject for AfcHub {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        Some(AfcHub::get_status(self))
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(AfcHub::new(config)?)
}
