// Test only (kaliru-afc, the parity runs' binary; not part of the rust_afc
// library). Batch mode gets no MCU input:
//   TEST_MCU_MSG MCU=<mcu> NAME=<message> [DELAY=<s>] <field>=<int> ...
// hands the handler registered for an MCU message (oams_cmd_stats,
// oams_action_status, ...) that message's fields, as the MCU would send it;
// DELAY delivers it from a reactor timer that many seconds later, so it
// reaches a command that waits on it. Field names are given upper-cased by
// gcode and passed lower-cased.
// A query is never answered in batch mode (its send waits for an ack that
// never comes). [test_mcu_msg] answer: lines,
//   <mcu> <query command> -> <response> <field>=<int> ...
// and TEST_MCU_ANSWER MCU= QUERY= RESPONSE= <field>=<int> ... make the query's
// send deliver that response and return (kaliru's SerialReader wait-ack hook).
// The python reference's counterpart is tests/plugins/test_mcu_msg.py.
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::gcode::{GCodeCommand, GCodeDispatch};
use kaliru::msgproto::{Params, Val};
use kaliru::native_plugin::NativePlugin;
use kaliru::printer::{PObj, Printer, PrinterModule, PrinterObject};
use kaliru::reactor::NEVER;
use kaliru::serialhdl::SerialReader;
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

const OWN_PARAMS: [&str; 5] = ["MCU", "NAME", "DELAY", "QUERY", "RESPONSE"];

/// per MCU: query command -> (response, its fields), in insertion order
type Answers = Rc<RefCell<Vec<(String, String, Params)>>>;

struct TestMcuMsg {
    printer: Rc<Printer>,
    answers: RefCell<HashMap<String, Answers>>,
}

impl PrinterObject for TestMcuMsg {}

fn int_field(value: &str) -> KResult<Val> {
    value
        .trim()
        .parse::<i64>()
        .map(Val::Int)
        .map_err(|_| KError::Py("ValueError".into(), format!("invalid literal for int() with base 10: {}", kaliru::pyfmt::py_str_repr(value))))
}

fn own_fields(gcmd: &GCodeCommand) -> KResult<Params> {
    let mut params = Params::default();
    for (key, value) in gcmd.get_command_parameters() {
        if !OWN_PARAMS.contains(&key.as_str()) {
            params.insert(key.to_lowercase(), int_field(&value)?);
        }
    }
    Ok(params)
}

impl TestMcuMsg {
    fn set_answer(&self, mcu_name: &str, query: &str, response: &str, fields: Params) -> KResult<()> {
        if !self.answers.borrow().contains_key(mcu_name) {
            let answers: Answers = Rc::new(RefCell::new(vec![]));
            self.answers.borrow_mut().insert(mcu_name.to_string(), answers.clone());
            let serial = kaliru::mcu::get_printer_mcu(&self.printer, mcu_name)?.get_serial();
            serial.set_wait_ack_hook(Some(Rc::new(move |serial: &SerialReader, cmd: &[u8]| {
                for (query, response, fields) in answers.borrow().iter() {
                    if serial.get_msgparser().create_command(query)? == cmd {
                        if let Some(handler) = serial.response_handler(response, None) {
                            handler(fields)?;
                        }
                        return Ok(Some(Params::default()));
                    }
                }
                Ok(None)
            })));
        }
        let answers = self.answers.borrow().get(mcu_name).cloned().unwrap();
        let mut a = answers.borrow_mut();
        match a.iter_mut().find(|(q, _, _)| q == query) {
            Some(entry) => *entry = (query.to_string(), response.to_string(), fields),
            None => a.push((query.to_string(), response.to_string(), fields)),
        }
        Ok(())
    }

    fn cmd_test_mcu_answer(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.set_answer(&gcmd.get_or("MCU", "mcu"), &gcmd.get("QUERY")?, &gcmd.get("RESPONSE")?, own_fields(gcmd)?)
    }

    fn cmd_test_mcu_msg(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let mcu = kaliru::mcu::get_printer_mcu(&self.printer, &gcmd.get_or("MCU", "mcu"))?;
        let name = gcmd.get("NAME")?;
        let delay = gcmd.get_float("DELAY", Some(0.0), Lim::new().min(0.0))?;
        let params = own_fields(gcmd)?;
        let Some(handler) = mcu.get_serial().response_handler(&name, None) else {
            return Err(gcmd.error(format!("TEST_MCU_MSG: no handler for {}", name)));
        };
        if delay == 0.0 {
            return handler(&params);
        }
        let reactor = self.printer.get_reactor();
        reactor.register_timer(
            Box::new(move |_| {
                if let Err(e) = handler(&params) {
                    kaliru::logging::exception_err("TEST_MCU_MSG", &e);
                }
                NEVER
            }),
            reactor.monotonic() + delay,
        );
        Ok(())
    }
}

fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let printer = config.get_printer();
    let t = Rc::new(TestMcuMsg { printer: printer.clone(), answers: RefCell::new(HashMap::new()) });
    for line in config.get("answer", "")?.split('\n') {
        if line.trim().is_empty() {
            continue;
        }
        let (query_part, response_part) = line.split_once("->").ok_or_else(|| KError::Py("ValueError".into(), "not enough values to unpack (expected 2, got 1)".into()))?;
        let q: Vec<&str> = query_part.split_whitespace().collect();
        let [mcu_name, query] = q[..] else { return Err(KError::Py("ValueError".into(), "too many values to unpack (expected 2)".into())) };
        let words: Vec<&str> = response_part.split_whitespace().collect();
        let mut fields = Params::default();
        for w in &words[1..] {
            let (k, v) = w.split_once('=').unwrap_or((w, ""));
            fields.insert(k.to_lowercase(), int_field(v)?);
        }
        t.set_answer(mcu_name, query, words[0], fields)?;
    }
    let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
    let w = Rc::downgrade(&t);
    gcode.register_command("TEST_MCU_MSG", Some(Rc::new(move |g| w.upgrade().map_or(Ok(()), |t| t.cmd_test_mcu_msg(g)))), false, None)?;
    let w = Rc::downgrade(&t);
    gcode.register_command("TEST_MCU_ANSWER", Some(Rc::new(move |g| w.upgrade().map_or(Ok(()), |t| t.cmd_test_mcu_answer(g)))), false, None)?;
    Ok(t)
}

const MODULES: &[PrinterModule] = &[PrinterModule { name: "test_mcu_msg", ported: true, load_config: Some(load_config), load_config_prefix: None, register_components: None }];

/// active when the config has [test_mcu_msg]
fn active(config: &ConfigWrapper) -> KResult<bool> {
    Ok(config.fileconfig.borrow().has_section("test_mcu_msg"))
}

pub const PLUGIN: NativePlugin = NativePlugin { name: "Rust-AFC test_mcu_msg", modules: MODULES, active };
