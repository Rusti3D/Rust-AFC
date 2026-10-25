// AFC_led: AFC's neopixel chains (port of AFC_led.py, a neopixel driver
// whose updates run in a reactor callback, with the state AFC sets per
// index kept for TURN_ON_AFC_LED)
//
// Copyright (C) 2019-2022  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::Afc;
use kaliru::configfile::{ConfigWrapper, Dflt, Lim};
use kaliru::error::{KError, KResult};
use kaliru::extras::led::{Color, LEDHelper};
use kaliru::mcu::{get_printer_mcu, CommandQueryWrapper, CommandWrapper, Mcu};
use kaliru::msgproto::Val;
use kaliru::pins::PinsObject;
use kaliru::printer::{PObj, Printer, PrinterObject};
use kaliru::pyval::PyVal;
use kaliru::reactor::{ReactorMutex, NOW};
use kaliru::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const BACKGROUND_PRIORITY_CLOCK: u64 = 0x7fffffff00000000;
const BIT_MAX_TIME: f64 = 0.000004;
const RESET_MIN_TIME: f64 = 0.000050;
const MAX_MCU_SIZE: usize = 500; // Sanity check on LED chain length

/// led_change's index: "a-b", a list of indexes, or one index
pub enum LedIndex<'a> {
    Str(&'a str),
    List(&'a [i64]),
    One(i64),
}

pub struct AfcLed {
    me: Weak<AfcLed>,
    printer: Rc<Printer>,
    afc: Rc<Afc>,
    mutex: Rc<ReactorMutex>,
    mcu: Rc<Mcu>,
    oid: i64,
    pin: String,
    neopixel_update_cmd: RefCell<Option<CommandWrapper>>,
    neopixel_send_cmd: RefCell<Option<CommandQueryWrapper>>,
    color_map: Vec<(usize, (usize, usize))>,
    led_helper: RefCell<Option<Rc<LEDHelper>>>,
    color_data: RefCell<Vec<u8>>,
    old_color_data: RefCell<Vec<u8>>,
    last_led_color: RefCell<IndexMap<String, Color>>,
    keep_leds_off: Cell<bool>,
}

fn sorted(s: &str) -> Vec<char> {
    let mut v: Vec<char> = s.chars().collect();
    v.sort();
    v
}

impl AfcLed {
    fn new(config: &ConfigWrapper) -> KResult<Rc<AfcLed>> {
        let printer = config.get_printer();
        let mutex = printer.get_reactor().mutex(false);
        let fullname = config.get_name().to_string();
        let name = fullname.split_whitespace().last().unwrap_or("").to_string();
        let afc = printer.lookup_or_load::<Afc>(config, "AFC")?;
        // Configure neopixel
        let ppins = printer.lookup::<PinsObject>("pins")?;
        let pin_desc = config.get("pin", Dflt::Required)?;
        let pin_params = ppins.0.borrow_mut().lookup_pin(&pin_desc, false, false, None).map_err(|e| KError::Pins(e.0))?;
        let (chip_name, pin) = {
            let p = pin_params.borrow();
            (p.chip_name.clone(), p.pin.clone())
        };
        let mcu = get_printer_mcu(&printer, &chip_name)?;
        let oid = mcu.create_oid();
        // Build color map
        let chain_count = config.getint("chain_count", 1, Lim::new().min(1))?;
        let mut color_order = config.getlist("color_order", PyVal::List(vec![PyVal::from("GRB")]), ",", None)?.unwrap_or_default();
        if color_order.len() == 1 {
            color_order = vec![color_order[0].clone(); chain_count as usize];
        }
        if color_order.len() != chain_count as usize {
            return Err(KError::config("color_order does not match chain_count"));
        }
        let mut color_indexes = vec![];
        for (lidx, co) in color_order.iter().enumerate() {
            if sorted(co) != sorted("RGB") && sorted(co) != sorted("RGBW") {
                return Err(KError::config(format!("Invalid color_order '{}'", co)));
            }
            color_indexes.extend(co.chars().map(|c| (lidx, "RGBW".find(c).unwrap_or(0))));
        }
        let color_map: Vec<(usize, (usize, usize))> = color_indexes.into_iter().enumerate().collect();
        if color_map.len() > MAX_MCU_SIZE {
            return Err(KError::config("neopixel chain too long"));
        }
        let led = Rc::new_cyclic(|me| AfcLed {
            me: me.clone(),
            printer: printer.clone(),
            afc: afc.clone(),
            mutex,
            mcu: mcu.clone(),
            oid,
            pin,
            neopixel_update_cmd: RefCell::new(None),
            neopixel_send_cmd: RefCell::new(None),
            color_data: RefCell::new(vec![0; color_map.len()]),
            old_color_data: RefCell::new(vec![]),
            color_map,
            led_helper: RefCell::new(None),
            last_led_color: RefCell::new(IndexMap::default()),
            keep_leds_off: Cell::new(false),
        });
        afc.led_obj.borrow_mut().insert(name, led.clone());
        let l = led.clone();
        mcu.register_config_callback(Rc::new(move || l.build_config()));
        // Initialize color data
        let w = led.me.clone();
        let helper = LEDHelper::new(
            config,
            Rc::new(move |state, print_time| match w.upgrade() {
                Some(l) => l.update_leds(state, print_time),
                None => Ok(()),
            }),
            chain_count,
        )?;
        led.update_color_data(&helper.led_state())?;
        *led.old_color_data.borrow_mut() = led.color_data.borrow().iter().map(|d| d ^ 1).collect();
        *led.led_helper.borrow_mut() = Some(helper);
        // Register callbacks
        let l = led.clone();
        kaliru::events::KlippyConnect::on(&printer, move || l.send_data(None));
        Ok(led)
    }

    fn helper(&self) -> KResult<Rc<LEDHelper>> {
        self.led_helper.borrow().clone().ok_or_else(|| super::no_attr("AFCled", "led_helper"))
    }

    fn build_config(&self) -> KResult<()> {
        let bmt = self.mcu.seconds_to_clock(BIT_MAX_TIME);
        let rmt = self.mcu.seconds_to_clock(RESET_MIN_TIME);
        self.mcu.add_config_cmd(
            &format!("config_neopixel oid={} pin={} data_size={} bit_max_ticks={} reset_min_ticks={}", self.oid, self.pin, self.color_data.borrow().len(), bmt, rmt),
            false,
            false,
        );
        let cmd_queue = self.mcu.alloc_command_queue();
        *self.neopixel_update_cmd.borrow_mut() = Some(self.mcu.lookup_command("neopixel_update oid=%c pos=%hu data=%*s", Some(cmd_queue.clone()))?);
        *self.neopixel_send_cmd.borrow_mut() =
            Some(self.mcu.lookup_query_command("neopixel_send oid=%c", "neopixel_result oid=%c success=%c", Some(self.oid), Some(cmd_queue), false)?);
        Ok(())
    }

    fn update_color_data(&self, led_state: &[Color]) -> KResult<()> {
        let mut color_data = self.color_data.borrow_mut();
        for (cdidx, (lidx, cidx)) in &self.color_map {
            let v = led_state.get(*lidx).and_then(|c| c.get(*cidx)).ok_or_else(|| KError::Py("IndexError".into(), "tuple index out of range".into()))?;
            color_data[*cdidx] = (v * 255.0 + 0.5).trunc() as i64 as u8;
        }
        Ok(())
    }

    fn send_data(&self, print_time: Option<f64>) -> KResult<()> {
        let new_data = self.color_data.borrow().clone();
        let old_data = self.old_color_data.borrow().clone();
        if new_data == old_data {
            return Ok(());
        }
        // Find the position of all changed bytes in this framebuffer
        let mut diffs: Vec<[usize; 2]> = new_data.iter().zip(&old_data).enumerate().filter(|(_, (n, o))| n != o).map(|(i, _)| [i, 1]).collect();
        // Batch together changes that are close to each other
        for i in (0..diffs.len().saturating_sub(1)).rev() {
            let [pos, _count] = diffs[i];
            let [nextpos, nextcount] = diffs[i + 1];
            if pos + 5 >= nextpos && nextcount < 16 {
                diffs[i][1] = nextcount + (nextpos - pos);
                diffs.remove(i + 1);
            }
        }
        // Transmit changes
        {
            let ucmd = self.neopixel_update_cmd.borrow();
            let ucmd = ucmd.as_ref().ok_or_else(|| super::no_attr("NoneType", "send"))?;
            for [pos, count] in &diffs {
                let end = (pos + count).min(new_data.len());
                ucmd.send(&[Val::Int(self.oid), Val::Int(*pos as i64), Val::Bytes(new_data[*pos..end].to_vec())], 0, BACKGROUND_PRIORITY_CLOCK)?;
            }
        }
        *self.old_color_data.borrow_mut() = new_data;
        // Instruct mcu to update the LEDs
        let mut minclock = 0;
        if let Some(pt) = print_time {
            minclock = self.mcu.print_time_to_clock(pt);
        }
        if self.printer.start_arg("debugoutput").is_some() {
            return Ok(());
        }
        let scmd = self.neopixel_send_cmd.borrow();
        let scmd = scmd.as_ref().ok_or_else(|| super::no_attr("NoneType", "send"))?;
        for _ in 0..8 {
            let params = scmd.send(&[Val::Int(self.oid)], minclock as u64, BACKGROUND_PRIORITY_CLOCK, true)?;
            if matches!(params.get("success"), Some(Val::Int(v)) if *v != 0) {
                return Ok(());
            }
        }
        kaliru::logging::info("Neopixel update did not succeed");
        Ok(())
    }

    /// update_leds: the new state goes out from a reactor callback
    fn update_leds(&self, led_state: &[Color], print_time: Option<f64>) -> KResult<()> {
        let led_state = led_state.to_vec();
        let Some(me) = self.me.upgrade() else { return Ok(()) };
        self.printer.get_reactor().register_callback(
            move |_| {
                let r = me.mutex.lock().map_err(KError::from).and_then(|()| {
                    let r = me.update_color_data(&led_state).and_then(|()| me.send_data(print_time));
                    me.mutex.unlock();
                    r
                });
                if let Err(e) = r {
                    kaliru::logging::exception_err("AFC_led update", &e);
                }
            },
            NOW,
        );
        Ok(())
    }

    fn set_color_fn(&self, index: i64, color: &Color, update_last: bool) -> KResult<()> {
        if update_last {
            self.last_led_color.borrow_mut().insert(index.to_string(), color.clone());
        }
        if self.keep_leds_off.get() {
            return Ok(());
        }
        self.helper()?.set_color(Some(index), color)
    }

    /// led_change: a color ("r,g,b,w" or a list) for one index, "a-b" or a list
    pub fn led_change(&self, index: LedIndex, status: &PyVal, update_last: bool) -> KResult<()> {
        let colors: Color = match status.get() {
            PyVal::List(l) | PyVal::Tuple(l) => l.iter().map(super::utils::py_float_of).collect::<KResult<_>>()?,
            s => s.str().split(',').map(|v| super::utils::py_float_of(&PyVal::Str(v.to_string()))).collect::<KResult<_>>()?,
        };
        let int = |s: &str| super::utils::py_int_of(&PyVal::Str(s.to_string()));
        match index {
            LedIndex::Str(s) if s.contains('-') => {
                let parts: Vec<&str> = s.split('-').collect();
                if parts.len() != 2 {
                    return Err(KError::Py("ValueError".into(), format!("too many values to unpack (expected 2, got {})", parts.len())));
                }
                for i in int(parts[0])?..int(parts[1])? + 1 {
                    self.set_color_fn(i, &colors, update_last)?;
                }
            }
            LedIndex::Str(s) => self.set_color_fn(int(s)?, &colors, update_last)?,
            LedIndex::List(l) => {
                for &i in l {
                    self.set_color_fn(i, &colors, update_last)?;
                }
            }
            LedIndex::One(i) => self.set_color_fn(i, &colors, update_last)?,
        }
        // Skip further operations if LEDs are set to remain off.
        if self.keep_leds_off.get() {
            return Ok(());
        }
        let helper = self.helper()?;
        self.afc.toolhead()?.register_lookahead_callback(Rc::new(move |print_time| helper.check_transmit(Some(print_time))))
    }

    pub fn turn_off_leds(&self) -> KResult<()> {
        let off = PyVal::from("0,0,0,0");
        for i in 0..self.helper()?.led_count() {
            self.led_change(LedIndex::One(i), &off, false)?;
        }
        self.keep_leds_off.set(true);
        Ok(())
    }

    pub fn turn_on_leds(&self) -> KResult<()> {
        self.keep_leds_off.set(false);
        let last: Vec<(String, Color)> = self.last_led_color.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect();
        for (index, value) in last {
            let value = PyVal::List(value.into_iter().map(PyVal::Float).collect());
            self.led_change(LedIndex::Str(&index), &value, false)?;
        }
        Ok(())
    }
}

impl PrinterObject for AfcLed {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        self.led_helper.borrow().as_ref().map(|h| h.get_status())
    }
    fn teardown(&self) {
        if let Some(h) = self.led_helper.borrow_mut().take() {
            h.teardown();
        }
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(AfcLed::new(config)?)
}
