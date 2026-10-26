// AFC_button: a lane's button, a short press loading or unloading the tool,
// a long one ejecting the lane (port of AFC_button.py)
//
// Copyright (C) 2025 Armored Turtle
// Copyright (C) 2026 AFCProject
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::lane::AfcLane;
use super::Afc;
use kaliru::configfile::{ConfigWrapper, Dflt, Lim};
use kaliru::error::{KError, KResult};
use kaliru::extras::buttons::PrinterButtons;
use kaliru::printer::{PObj, PrinterObject};
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub struct AfcButton {
    afc: Rc<Afc>,
    lane_id: String,
    lane_obj: RefCell<Option<Rc<AfcLane>>>,
    long_press_duration: f64,
    press_time: Cell<Option<f64>>,
}

impl PrinterObject for AfcButton {}

impl AfcButton {
    fn new(config: &ConfigWrapper) -> KResult<Rc<AfcButton>> {
        let printer = config.get_printer();
        let ready: Rc<RefCell<Weak<AfcButton>>> = Rc::default();
        // (registered before AFC loads, as python does: it runs ahead of AFC's)
        let r = ready.clone();
        kaliru::events::KlippyReady::on(&printer, move || r.borrow().upgrade().map_or(Ok(()), |b| b.handle_ready()));
        let afc = printer.lookup_or_load::<Afc>(config, "AFC")?;
        let lane_id = config.get_name().split_whitespace().last().unwrap_or("").to_string();
        let long_press_duration = config.getfloat("long_press_duration", 1.2, Lim::new())?;
        let pin_name = config.get("pin", Dflt::Required)?;
        let button = Rc::new(AfcButton { afc: afc.clone(), lane_id, lane_obj: RefCell::new(None), long_press_duration, press_time: Cell::new(None) });
        *ready.borrow_mut() = Rc::downgrade(&button);
        // Register the button callback
        let buttons = printer.lookup_or_load::<PrinterButtons>(config, "buttons")?;
        let w = Rc::downgrade(&button);
        buttons.register_buttons(&[pin_name.as_str()], Rc::new(move |et, state| w.upgrade().map_or(Ok(()), |b| b.button_callback(et, state != 0))))?;
        afc.logger.info(&AFC_BUTTON_AFCBUTTON___INIT__, &format!("AFC_button for {} initialized on pin: {}", button.lane_id, pin_name));
        Ok(button)
    }

    fn lane_obj(&self) -> KResult<Rc<AfcLane>> {
        self.lane_obj.borrow().clone().ok_or_else(|| KError::config(format!("Lane {} has not been resolved yet", self.lane_id)))
    }

    fn handle_ready(&self) -> KResult<()> {
        let lane = self.afc.lane(&self.lane_id);
        *self.lane_obj.borrow_mut() = lane.clone();
        if lane.is_none() {
            return Err(KError::config(format!(
                "Lane {} is not defined/found in your configuration file. Please define lane or verify lane name is correct.",
                self.lane_id
            )));
        }
        Ok(())
    }

    fn button_callback(&self, eventtime: f64, state: bool) -> KResult<()> {
        let c = &AFC_BUTTON_AFCBUTTON__BUTTON_CALLBACK;
        let afc = &self.afc;
        if state {
            self.press_time.set(Some(eventtime));
            return Ok(());
        }
        let Some(press_time) = self.press_time.get() else { return Ok(()) };
        if afc.function.is_printing(true)? {
            afc.error.afc_error(c, "Cannot use buttons while printer is actively moving or homing", false, None)?;
            return Ok(());
        }
        let held_time = eventtime - press_time;
        self.press_time.set(None);
        if held_time < 0.05 {
            return Ok(());
        }
        let cur_lane = afc.function.get_current_lane_obj();
        let is_this = cur_lane.as_ref().is_some_and(|l| l.name == self.lane_id);
        if held_time >= self.long_press_duration {
            // Long Press
            afc.logger.info(c, &format!("{}: Long press detected.", self.lane_id));
            if is_this {
                afc.logger.info(c, &format!("Unloading {} before ejecting.", self.lane_id));
                let lane = self.lane_obj()?;
                if afc.tool_unload(&lane, true, false)? {
                    afc.lane_unload(&lane)?;
                } else {
                    afc.afc_stats()?.increase_unload_error_count(afc)?;
                }
            } else {
                // If another lane is active, just eject this one
                afc.logger.info(c, &format!("Ejecting {}.", self.lane_id));
                afc.lane_unload(&self.lane_obj()?)?;
            }
        } else {
            // Short Press
            afc.logger.info(c, &format!("{}: Short press detected.", self.lane_id));
            match cur_lane.filter(|_| is_this) {
                Some(cur_lane) => {
                    afc.logger.info(c, &format!("Unloading tool from {}.", self.lane_id));
                    if !afc.tool_unload(&cur_lane, true, false)? {
                        afc.afc_stats()?.increase_unload_error_count(afc)?;
                    }
                }
                None => {
                    afc.logger.info(c, &format!("Loading tool to {}.", self.lane_id));
                    afc.change_tool(&self.lane_obj()?, None, true, None)?;
                }
            }
        }
        Ok(())
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(AfcButton::new(config)?)
}
