// Test only (kaliru-afc, the parity runs' binary; not part of the rust_afc
// library): TEST_BUTTON PIN=<pin> STATE=0|1 hands each button group on the
// pin's MCU the buttons_state message that MCU sends when that pin changes.
// Batch mode gets no MCU input, so filament sensors never change otherwise;
// from the message on everything runs as on a printer.
//
// With follow_homing: True, each homing move flips the buttons on its
// endstops' pins as it starts: on a printer the sensor the move homes to
// changes state during the move (and in batch every homing move triggers at
// full distance); the hook then waits FOLLOW_TIME so the switch callbacks
// have run by the time the move ends.
// The python reference's counterpart is tests/plugins/test_button.py, the
// same steps.
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::extras::buttons::PrinterButtons;
use kaliru::gcode::{GCodeCommand, GCodeDispatch};
use kaliru::msgproto::{Params, Val};
use kaliru::native_plugin::NativePlugin;
use kaliru::pins::PinsObject;
use kaliru::printer::{PObj, Printer, PrinterModule, PrinterObject};
use std::rc::Rc;

/// seconds a followed homing move waits for the switch callbacks
const FOLLOW_TIME: f64 = 0.1;

struct TestButton;

impl PrinterObject for TestButton {}

/// Deliver `pin` on `chip` as `state`, or flipped where state is None;
/// false when no button group has the pin
fn press(printer: &Printer, chip: &str, pin: &str, state: Option<bool>) -> KResult<bool> {
    let buttons = printer.lookup::<PrinterButtons>("buttons")?;
    let reactor = printer.get_reactor();
    let mut found = false;
    for mb in buttons.mcu_button_groups(chip) {
        let mut bits = 0i64;
        for (i, p) in mb.pin_names().iter().enumerate() {
            if p == pin {
                bits |= 1 << i;
            }
        }
        if bits == 0 {
            continue;
        }
        found = true;
        let (invert, last_button, ack_count) = mb.state();
        let on = state.unwrap_or(last_button & bits == 0);
        let logical = (last_button & !bits) | if on { bits } else { 0 };
        let raw = logical ^ invert;
        let mut params = Params::default();
        params.insert("ack_count".into(), Val::Int(ack_count & 0xFF));
        params.insert("state".into(), Val::Bytes(vec![(raw & 0xFF) as u8]));
        params.insert("#receive_time".into(), Val::Float(reactor.monotonic()));
        mb.handle_buttons_state(&params)?;
    }
    Ok(found)
}

fn cmd_test_button(printer: &Printer, gcmd: &GCodeCommand) -> KResult<()> {
    let pin = gcmd.get("PIN")?;
    let state = gcmd.get_int("STATE", None, Lim::new().min(0.0).max(1.0))?;
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let pin_params = ppins.0.borrow().parse_pin(&pin, true, true).map_err(|e| KError::Pins(e.0))?;
    if !press(printer, &pin_params.chip_name, &pin_params.pin, Some(state != 0))? {
        return Err(gcmd.error(format!("TEST_BUTTON: no button on {}", pin)));
    }
    Ok(())
}

fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let printer = config.get_printer();
    let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
    let p = printer.clone();
    gcode.register_command("TEST_BUTTON", Some(Rc::new(move |g| cmd_test_button(&p, g))), false, None)?;
    if config.getboolean("follow_homing", false)? {
        let p = printer.clone();
        kaliru::events::HomingMoveBegin::on(&printer, move |hmove| {
            let mut pressed = false;
            for es in hmove.get_mcu_endstops() {
                if let Some(pin) = es.mcu_pin() {
                    pressed |= press(&p, es.get_mcu().get_name(), &pin, None)?;
                }
            }
            if pressed {
                // the move's duration on a printer: batch homing never waits,
                // so without it the switch callbacks would only run after the move
                let reactor = p.get_reactor();
                reactor.pause(reactor.monotonic() + FOLLOW_TIME)?;
            }
            Ok(())
        });
    }
    Ok(Rc::new(TestButton))
}

const MODULES: &[PrinterModule] = &[PrinterModule { name: "test_button", ported: true, load_config: Some(load_config), load_config_prefix: None, register_components: None }];

/// active when the config has [test_button]
fn active(config: &ConfigWrapper) -> KResult<bool> {
    Ok(config.fileconfig.borrow().has_section("test_button"))
}

pub const PLUGIN: NativePlugin = NativePlugin { name: "Rust-AFC test_button", modules: MODULES, active };
