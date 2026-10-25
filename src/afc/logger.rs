// AFC's log: AFC.log next to klippy.log, and the messages AFC sends to the
// web clients (port of AFC_logger.py)
//
// Copyright (C) 2024-2026 Armored Turtle
// Copyright (C) 2026 AFCProject
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use kaliru::gcode::GCodeDispatch;
use kaliru::logging;
use kaliru::queuelogger::QueueListener;
use kaliru::reactor::Reactor;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

/// The python function a log call comes from: AFC.log names it as
/// `[file:function():co_firstlineno]` (codes.rs has them all)
#[derive(Clone, Copy)]
pub struct Code {
    pub file: &'static str,
    pub func: &'static str,
    pub line: u32,
}

thread_local! {
    // logging.getLogger("AFC") outlives a klippy restart, and with it the
    // queue listener of the first AFC_logger (a later one sees the handler and
    // makes none)
    static AFC_LOG: RefCell<Option<(String, Rc<QueueListener>)>> = const { RefCell::new(None) };
}

/// AFC.log, or klippy's root logger when klippy runs without a log file
enum Sink {
    File(Rc<QueueListener>),
    Root,
}

pub struct AfcLogger {
    reactor: Rc<Reactor>,
    gcode: Rc<GCodeDispatch>,
    sink: Sink,
    /// AFC_logger.afc_ql: the listener this logger started (shutdown stops it)
    afc_ql: Option<Rc<QueueListener>>,
    pub print_debug_console: Cell<bool>,
    adaptive_padding: Cell<usize>,
    /// afc.log_frame_data (getattr default True until AFC reads its config)
    pub log_frame_data: Cell<bool>,
    /// afc.message_queue
    pub message_queue: Rc<RefCell<Vec<(String, &'static str)>>>,
}

/// re.sub("<.*?>", "", s): the shortest run up to a '>' on the same line
fn remove_tags(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut rest = s;
    while let Some(i) = rest.find('<') {
        let after = &rest[i + 1..];
        let end = after.find(['>', '\n']).filter(|&j| after.as_bytes()[j] == b'>');
        match end {
            Some(j) => {
                out.push_str(&rest[..i]);
                rest = &after[j + 1..];
            }
            None => {
                out.push_str(&rest[..i + 1]);
                rest = after;
            }
        }
    }
    out.push_str(rest);
    out
}

/// python str.strip()/lstrip()/rstrip() whitespace
fn lstrip(s: &str) -> &str {
    kaliru::pyfmt::py_lstrip(s)
}

fn strip(s: &str) -> &str {
    kaliru::pyfmt::py_strip(s)
}

impl AfcLogger {
    pub fn new(printer: &Rc<kaliru::printer::Printer>) -> kaliru::error::KResult<AfcLogger> {
        let reactor = printer.get_reactor();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let log_path = printer.start_arg("log_file").filter(|v| v.truthy()).map(|v| v.str());
        let (sink, afc_ql) = match log_path {
            Some(path) => {
                let dirname = kaliru::pyglob::dirname(&path);
                let log_file = kaliru::pyglob::join(&dirname, &["AFC.log"]);
                let existing = AFC_LOG.with(|l| l.borrow().clone());
                match existing {
                    Some((_, ql)) => (Sink::File(ql), None),
                    None => {
                        let ql = Rc::new(
                            QueueListener::new(&log_file, false)
                                .map_err(|e| kaliru::error::KError::Other(kaliru::pyfmt::py_oserror(&e, &log_file)))?,
                        );
                        AFC_LOG.with(|l| *l.borrow_mut() = Some((log_file, ql.clone())));
                        (Sink::File(ql.clone()), Some(ql))
                    }
                }
            }
            None => (Sink::Root, None),
        };
        Ok(AfcLogger {
            reactor,
            gcode,
            sink,
            afc_ql,
            print_debug_console: Cell::new(false),
            adaptive_padding: Cell::new(0),
            log_frame_data: Cell::new(true),
            message_queue: Rc::new(RefCell::new(vec![])),
        })
    }

    /// atexit: stop the background writer
    pub fn shutdown(&self) {
        if let Some(ql) = &self.afc_ql {
            ql.stop();
            AFC_LOG.with(|l| *l.borrow_mut() = None);
        }
    }

    fn format(&self, message: &str, code: &Code) -> String {
        let mut frame_data = String::new();
        if self.log_frame_data.get() {
            let prefix = format!("[{}:{}():{}] ", code.file, code.func, code.line);
            frame_data = format!("{:<width$}", prefix, width = self.adaptive_padding.get());
        }
        self.adaptive_padding.set(self.adaptive_padding.get().max(frame_data.chars().count()));
        let s = remove_tags(lstrip(message));
        format!("{:10.3} {}- {}", self.reactor.monotonic(), frame_data, s)
    }

    /// logger.info/debug/error(...) on the AFC logger
    fn emit(&self, level: i32, line: String) {
        match &self.sink {
            // Formatter('%(asctime)s %(message)s', datefmt='%H:%M:%S')
            Sink::File(ql) => {
                let now = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_secs() as i64)
                    .unwrap_or(0);
                ql.send(&format!("{} {}", kaliru::util::strftime_local("%H:%M:%S", now), line), level >= logging::ERROR);
            }
            Sink::Root => logging::log(level, &line),
        }
    }

    /// gcode output to the web clients only (webhooks' GCodeHelper callbacks)
    pub fn send_callback(&self, msg: &str) {
        for (cb, origin) in self.gcode.output_callbacks() {
            if origin.is_some_and(|o| kaliru::webhooks::is_gcode_helper(&o)) {
                cb(lstrip(msg));
            }
        }
    }

    pub fn raw(&self, code: &Code, message: &str) {
        for line in strip(message).split('\n') {
            self.emit(logging::INFO, self.format(&format!(" RAW:  {}", line), code));
        }
        self.send_callback(message);
    }

    pub fn info(&self, code: &Code, message: &str) {
        self.info_opt(code, message, false);
    }

    pub fn info_opt(&self, code: &Code, message: &str, console_only: bool) {
        if !console_only {
            for line in lstrip(message).split('\n') {
                self.emit(logging::INFO, self.format(&format!("INFO: {}", line), code));
            }
        }
        self.send_callback(message);
    }

    pub fn warning(&self, code: &Code, message: &str) {
        for line in strip(message).split('\n') {
            self.emit(logging::DEBUG, self.format(&format!("WARN:  {}", line), code));
        }
        self.send_callback(&format!("<span class=warning--text>WARNING: {}</span>", message));
        self.message_queue.borrow_mut().push((message.to_string(), "warning"));
    }

    pub fn debug(&self, code: &Code, message: &str) {
        self.debug_opt(code, message, false, None);
    }

    pub fn debug_opt(&self, code: &Code, message: &str, only_debug: bool, traceback: Option<&str>) {
        for line in strip(message).split('\n') {
            self.emit(logging::DEBUG, self.format(&format!("DEBUG:{}", line), code));
        }
        if self.print_debug_console.get() && !only_debug {
            self.send_callback(message);
        }
        if let Some(tb) = traceback {
            for line in strip(tb).split('\n') {
                self.emit(logging::DEBUG, self.format(&format!("DEBUG:{}", line), code));
            }
        }
    }

    pub fn error(&self, code: &Code, message: &str) {
        self.error_opt(code, message, None, "");
    }

    pub fn error_opt(&self, code: &Code, message: &str, traceback: Option<&str>, stack_name: &str) {
        let stack_name = if stack_name.is_empty() { String::new() } else { format!("{}: ", stack_name) };
        for line in strip(message).split('\n') {
            self.emit(logging::ERROR, self.format(&format!("ERROR:{}{}", stack_name, line), code));
        }
        self.send_callback(&format!("!! {}", message));
        self.message_queue.borrow_mut().push((lstrip(message).to_string(), "error"));
        if let Some(tb) = traceback {
            for line in strip(tb).split('\n') {
                self.emit(logging::ERROR, self.format(&format!("ERROR:{}", line), code));
            }
        }
    }

    pub fn set_debug(&self, debug: bool) {
        self.print_debug_console.set(debug);
    }
}

#[cfg(test)]
mod tests {
    use super::remove_tags;

    #[test]
    fn tags() {
        assert_eq!(remove_tags("<span class=x>hi</span>"), "hi");
        assert_eq!(remove_tags("a < b"), "a < b");
        assert_eq!(remove_tags("a <\nb> c"), "a <\nb> c");
        assert_eq!(remove_tags("<<a>"), "");
    }
}
