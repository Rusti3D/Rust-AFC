// AFC helpers: filament switches, moonraker, the var file writer, and the
// python value semantics AFC relies on (port of AFC_utils.py)
//
// Copyright (C) 2024-2026 Armored Turtle
// Copyright (C) 2026 AFCProject
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::logger::AfcLogger;
use kaliru::configfile::{ConfigWrapper, Lim};
use kaliru::error::{KError, KResult};
use kaliru::extras::filament_switch_sensor::{GcodeSlot, SwitchSensor};
use kaliru::pins::PinsObject;
use kaliru::printer::Printer;
use kaliru::pyval::PyVal;
use kaliru::reactor::{AsyncHandle, Reactor, NOW};
use kaliru::toolhead::ToolHead;
use std::cell::{Cell, RefCell};
use std::collections::HashMap;
use std::io::{Read, Write};
use std::rc::Rc;
use std::sync::mpsc;

/// "afc:moonraker_connect"
pub const MOONRAKER_CONNECT_EVENT: &str = "afc:moonraker_connect";
pub const AFC_STATS_KEY: &str = "afc_stats";
const ERROR_STRING: &str = "Error getting data from moonraker, check AFC.log for more information";
const REQUEST_TIMEOUT: u64 = 10;
const FILENAME_PATH: &str = "server/files/metadata?filename=";

// ----------------------------------------------------------------------
// python values
// ----------------------------------------------------------------------

/// int(v)
pub fn py_int_of(v: &PyVal) -> KResult<i64> {
    match v.get() {
        PyVal::Int(i) => Ok(*i),
        PyVal::Bool(b) => Ok(*b as i64),
        PyVal::Float(f) => {
            if f.is_finite() {
                Ok(f.trunc() as i64)
            } else {
                Err(KError::Py("ValueError".into(), "cannot convert float NaN to integer".into()))
            }
        }
        PyVal::Str(s) => {
            let t = kaliru::pyfmt::py_strip(s).replace('_', "");
            t.parse::<i64>().map_err(|_| KError::Py("ValueError".into(), format!("invalid literal for int() with base 10: {}", kaliru::pyfmt::py_str_repr(s))))
        }
        PyVal::None => Err(KError::Py("TypeError".into(), "int() argument must be a string, a bytes-like object or a real number, not 'NoneType'".into())),
        other => Err(KError::Py("TypeError".into(), format!("int() argument must be a string, a bytes-like object or a real number, not '{}'", py_type_name(other)))),
    }
}

/// float(v)
pub fn py_float_of(v: &PyVal) -> KResult<f64> {
    match v.get() {
        PyVal::Int(i) => Ok(*i as f64),
        PyVal::Bool(b) => Ok(*b as i64 as f64),
        PyVal::Float(f) => Ok(*f),
        PyVal::Str(s) => kaliru::pyfmt::py_float(s).ok_or_else(|| KError::Py("ValueError".into(), format!("could not convert string to float: {}", kaliru::pyfmt::py_str_repr(s)))),
        PyVal::None => Err(KError::Py("TypeError".into(), "float() argument must be a string or a real number, not 'NoneType'".into())),
        other => Err(KError::Py("TypeError".into(), format!("float() argument must be a string or a real number, not '{}'", py_type_name(other)))),
    }
}

fn numeric(v: &PyVal) -> Option<PyVal> {
    match v.get() {
        PyVal::Int(_) | PyVal::Float(_) => Some(v.get().clone()),
        PyVal::Bool(b) => Some(PyVal::Int(*b as i64)),
        _ => None,
    }
}

fn binop(a: &PyVal, b: &PyVal, op: &str, fi: fn(i64, i64) -> i64, ff: fn(f64, f64) -> f64) -> KResult<PyVal> {
    match (numeric(a), numeric(b)) {
        (Some(PyVal::Int(x)), Some(PyVal::Int(y))) => Ok(PyVal::Int(fi(x, y))),
        (Some(x), Some(y)) => Ok(PyVal::Float(ff(x.as_f64().unwrap_or(0.0), y.as_f64().unwrap_or(0.0)))),
        _ => Err(KError::Py("TypeError".into(), format!("unsupported operand type(s) for {}: '{}' and '{}'", op, py_type_name(a), py_type_name(b)))),
    }
}

/// type(v).__name__
pub fn py_type_name(v: &PyVal) -> &'static str {
    match v.get() {
        PyVal::None => "NoneType",
        PyVal::Bool(_) => "bool",
        PyVal::Int(_) => "int",
        PyVal::Float(_) => "float",
        PyVal::Str(_) => "str",
        PyVal::List(_) => "list",
        PyVal::Tuple(_) => "tuple",
        PyVal::Dict(_) => "dict",
        _ => "object",
    }
}

/// f"{x:g}"
pub fn py_format_g(x: f64) -> String {
    if x == 0.0 {
        return "0".into();
    }
    let exp = x.abs().log10().floor() as i32;
    if (-4..6).contains(&exp) {
        let s = format!("{:.*}", (5 - exp).max(0) as usize, x);
        let s = if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s };
        s
    } else {
        let m = x / 10f64.powi(exp);
        let ms = format!("{:.5}", m);
        let ms = ms.trim_end_matches('0').trim_end_matches('.');
        format!("{}e{}{:02}", ms, if exp < 0 { '-' } else { '+' }, exp.abs())
    }
}

/// days since 1970-01-01 of a proleptic gregorian date
pub fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146097 + doe - 719468
}

/// a + b
pub fn py_add(a: &PyVal, b: &PyVal) -> KResult<PyVal> {
    binop(a, b, "+", |x, y| x + y, |x, y| x + y)
}

/// a - b
pub fn py_sub(a: &PyVal, b: &PyVal) -> KResult<PyVal> {
    binop(a, b, "-", |x, y| x - y, |x, y| x - y)
}

/// f"{s:^{width}}": the extra space goes to the right
pub fn py_center(s: &str, width: usize) -> String {
    let n = s.chars().count();
    if n >= width {
        return s.to_string();
    }
    let pad = width - n;
    let left = pad / 2;
    format!("{}{}{}", " ".repeat(left), s, " ".repeat(pad - left))
}

#[derive(PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum SortChunk {
    Int(i64),
    Str(String),
}

/// natural_sort_key: digit runs as ints (T2 before T10), the rest lowercased
pub fn natural_sort_key(value: &str) -> Vec<SortChunk> {
    let mut out = vec![];
    let mut cur = String::new();
    let mut digits = false;
    for ch in value.chars() {
        if ch.is_ascii_digit() != digits {
            out.push(if digits { SortChunk::Int(cur.parse().unwrap_or(0)) } else { SortChunk::Str(cur.to_lowercase()) });
            cur.clear();
            digits = !digits;
        }
        cur.push(ch);
    }
    out.push(if digits { SortChunk::Int(cur.parse().unwrap_or(0)) } else { SortChunk::Str(cur.to_lowercase()) });
    if digits {
        out.push(SortChunk::Str(String::new()));
    }
    out
}

/// random.shuffle
pub fn shuffle<T>(v: &mut [T]) {
    let mut seed = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos() as u64).unwrap_or(1) | 1;
    for i in (1..v.len()).rev() {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        v.swap(i, (seed % (i as u64 + 1)) as usize);
    }
}

/// A wall clock time (datetime.now())
#[derive(Clone, Copy)]
pub struct DateTime {
    secs: f64,
}

pub fn datetime_now() -> DateTime {
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0);
    DateTime { secs: (secs * 1e6).trunc() / 1e6 }
}

impl DateTime {
    pub fn strftime(&self, fmt: &str) -> String {
        kaliru::util::strftime_local(fmt, self.secs.floor() as i64)
    }

    /// str(datetime): local time with microseconds
    pub fn str(&self) -> String {
        let us = ((self.secs - self.secs.floor()) * 1e6).round() as i64;
        let base = self.strftime("%Y-%m-%d %H:%M:%S");
        if us == 0 {
            base
        } else {
            format!("{}.{:06}", base, us)
        }
    }

    /// datetime.fromisoformat() of "YYYY-MM-DDTHH:MM:SS[.f]+00:00" (UTC)
    pub fn fromisoformat(s: &str) -> Option<DateTime> {
        let s = s.strip_suffix("+00:00")?;
        let (date, time) = s.split_once(['T', ' '])?;
        let d: Vec<i64> = date.split('-').map(|x| x.parse().ok()).collect::<Option<_>>()?;
        let (hms, frac) = match time.split_once('.') {
            Some((h, f)) => (h, format!("0.{}", f).parse::<f64>().ok()?),
            None => (time, 0.0),
        };
        let t: Vec<i64> = hms.split(':').map(|x| x.parse().ok()).collect::<Option<_>>()?;
        if d.len() != 3 || t.len() != 3 {
            return None;
        }
        let days = days_from_civil(d[0], d[1], d[2]);
        Some(DateTime { secs: (days * 86400 + t[0] * 3600 + t[1] * 60 + t[2]) as f64 + frac })
    }

    pub fn after(&self, other: &DateTime) -> bool {
        self.secs > other.secs
    }

    pub fn seconds_since(&self, earlier: &DateTime) -> f64 {
        self.secs - earlier.secs
    }
}

/// subprocess.check_output(args).decode('ascii').strip(), or the
/// traceback.format_exc() text of its failure
pub fn check_output(args: &[&str]) -> Result<String, String> {
    let argv = format!("[{}]", args.iter().map(|a| kaliru::pyfmt::py_str_repr(a)).collect::<Vec<_>>().join(", "));
    match std::process::Command::new(args[0]).args(&args[1..]).stderr(std::process::Stdio::inherit()).output() {
        Ok(o) if o.status.success() => Ok(String::from_utf8_lossy(&o.stdout).trim().to_string()),
        Ok(o) => Err(format!(
            "Traceback (most recent call last):\n  File \"<kaliru>\"\nsubprocess.CalledProcessError: Command {} returned non-zero exit status {}.\n",
            argv,
            o.status.code().unwrap_or(-1)
        )),
        Err(e) => Err(format!(
            "Traceback (most recent call last):\n  File \"<kaliru>\"\n{}: {}\n",
            kaliru::pyfmt::py_oserror_class(&e),
            kaliru::pyfmt::py_oserror(&e, args[0])
        )),
    }
}

// ----------------------------------------------------------------------
// filament switches
// ----------------------------------------------------------------------

pub type RunoutCallback = Rc<dyn Fn(f64) -> KResult<()>>;

pub struct SwitchOptions {
    pub show_sensor: bool,
    pub runout_callback: Option<RunoutCallback>,
    pub enable_runout: bool,
    pub debounce_delay: f64,
}

impl Default for SwitchOptions {
    fn default() -> Self {
        SwitchOptions { show_sensor: true, runout_callback: None, enable_runout: false, debounce_delay: 0.0 }
    }
}

/// add_filament_switch: a [filament_switch_sensor <name>] built from a
/// config of its own, hidden from the web interfaces unless show_sensor,
/// with AFC taking its runout (runout_callback) and a debounced handler
pub fn add_filament_switch(switch_name: &str, switch_pin: &str, printer: &Rc<Printer>, opts: &SwitchOptions) -> KResult<(Rc<SwitchSensor>, Rc<DebounceButton>)> {
    let new_switch_name = format!("filament_switch_sensor {}", switch_name);
    let ppins = printer.lookup::<PinsObject>("pins")?;
    ppins.0.borrow_mut().allow_multi_use_pin(switch_pin.trim_matches(|c| c == '!' || c == '^')).map_err(|e| KError::Pins(e.0))?;
    let fileconfig = Rc::new(RefCell::new(kaliru::configparser::RawConfigParser::new(true, &[], Box::new(kaliru::configparser::NoInterpolation))));
    {
        let mut fc = fileconfig.borrow_mut();
        fc.add_section(&new_switch_name)?;
        fc.set(&new_switch_name, "switch_pin", switch_pin)?;
        fc.set(&new_switch_name, "pause_on_runout", "False")?;
        fc.set(&new_switch_name, "debounce_delay", "0.0")?;
    }
    let cfg_wrap = ConfigWrapper::from_parts(printer, fileconfig.clone(), Rc::new(RefCell::new(Default::default())), &new_switch_name);
    let fila = printer
        .load_object(&cfg_wrap, &new_switch_name, true)?
        .and_then(kaliru::printer::downcast_rc::<SwitchSensor>)
        .ok_or_else(|| KError::Other(format!("{} is not a filament switch sensor", new_switch_name)))?;
    if !opts.show_sensor {
        if let Some(o) = printer.objects_remove(&new_switch_name) {
            printer.objects_set(&format!("_{}", new_switch_name), o);
        }
    }
    let rh = fila.runout_helper.clone();
    rh.sensor_enabled.set(opts.enable_runout);
    rh.runout_pause.set(false);
    fileconfig.borrow_mut().set(&new_switch_name, "debounce_delay", &kaliru::pyfmt::py_float_repr(opts.debounce_delay))?;
    let debounce_button = DebounceButton::new(&cfg_wrap, &fila)?;
    if let Some(cb) = &opts.runout_callback {
        *rh.insert_gcode.borrow_mut() = None;
        *rh.runout_gcode.borrow_mut() = Some(GcodeSlot::Native);
        *rh.runout_handler_hook.borrow_mut() = Some(cb.clone());
    }
    Ok((fila, debounce_button))
}

pub type ButtonAction = Rc<dyn Fn(f64, bool) -> KResult<()>>;

/// DebounceButton: the sensor's note_filament_present, debounced, calling
/// button_action (the sensor's own handler, or the lane's)
pub struct DebounceButton {
    printer: Rc<Printer>,
    reactor: Rc<Reactor>,
    fila: Rc<SwitchSensor>,
    button_action: RefCell<Option<ButtonAction>>,
    pub debounce_delay: Cell<f64>,
    logical_state: Cell<Option<bool>>,
    physical_state: Cell<Option<bool>>,
    latest_eventtime: Cell<f64>,
}

impl DebounceButton {
    fn new(config: &ConfigWrapper, fila: &Rc<SwitchSensor>) -> KResult<Rc<DebounceButton>> {
        let printer = config.get_printer();
        let b = Rc::new(DebounceButton {
            printer: printer.clone(),
            reactor: printer.get_reactor(),
            fila: fila.clone(),
            button_action: RefCell::new(None),
            debounce_delay: Cell::new(0.0),
            logical_state: Cell::new(None),
            physical_state: Cell::new(None),
            latest_eventtime: Cell::new(0.0),
        });
        // kalico's signature (eventtime, is_filament_present, force, immediate)
        let w = Rc::downgrade(&b);
        *fila.runout_helper.note_hook.borrow_mut() = Some(Rc::new(move |eventtime, state, _force, _immediate| match w.upgrade() {
            Some(b) => b.button_handler(eventtime, state.unwrap_or(false)),
            None => Ok(()),
        }));
        b.debounce_delay.set(config.getfloat("debounce_delay", 0.0, Lim::new().min(0.0))?);
        Ok(b)
    }

    pub fn set_action(&self, action: ButtonAction) {
        *self.button_action.borrow_mut() = Some(action);
    }

    /// _old_note_filament_present(eventtime, state): the sensor's own handler
    pub fn old_note_filament_present(&self, eventtime: f64, state: bool) -> KResult<()> {
        self.fila.runout_helper.note_filament_present_orig(eventtime, Some(state), false, false)
    }

    fn button_handler(self: &Rc<Self>, eventtime: f64, state: bool) -> KResult<()> {
        self.physical_state.set(Some(state));
        self.latest_eventtime.set(eventtime);
        if self.logical_state.get() == self.physical_state.get() {
            return Ok(());
        }
        let b = self.clone();
        self.reactor.register_callback(move |et| b.debounce_event(et), eventtime + self.debounce_delay.get());
        Ok(())
    }

    fn debounce_event(&self, eventtime: f64) {
        if self.logical_state.get() == self.physical_state.get() {
            return;
        }
        if (eventtime - self.debounce_delay.get()) < self.latest_eventtime.get() {
            return;
        }
        self.logical_state.set(self.physical_state.get());
        let state = self.logical_state.get().unwrap_or(false);
        let action = self.button_action.borrow().clone();
        let r = match action {
            Some(a) => a(eventtime, state),
            None => self.old_note_filament_present(eventtime, state),
        };
        if r.is_err() {
            // Last ditch effort to call klipper pause since something bad happened
            let res = (|| -> KResult<()> {
                let pause_resume = self.printer.lookup::<kaliru::extras::pause_resume::PauseResume>("pause_resume")?;
                let gcode = self.printer.lookup::<kaliru::gcode::GCodeDispatch>("gcode")?;
                let pause_cmd = gcode.create_gcode_command("PAUSE", "PAUSE", Default::default());
                pause_resume.cmd_pause(&pause_cmd)
            })();
            if let Err(e) = res {
                kaliru::logging::exception_err("AFC debounce", &e);
            }
        }
    }
}

// ----------------------------------------------------------------------
// moonraker
// ----------------------------------------------------------------------

/// An HTTP request: urllib's Request(url, data, method, headers)
#[derive(Clone)]
struct Request {
    url: String,
    method: String,
    body: Option<Vec<u8>>,
    content_type: Option<String>,
}

impl Request {
    fn get(url: String) -> Request {
        Request { url, method: "GET".into(), body: None, content_type: None }
    }

    fn form(url: String, fields: &[(&str, String)], method: Option<&str>) -> Request {
        let body = fields.iter().map(|(k, v)| format!("{}={}", urlencode(k), urlencode(v))).collect::<Vec<_>>().join("&");
        Request { url, method: method.unwrap_or("POST").into(), body: Some(body.into_bytes()), content_type: Some("application/x-www-form-urlencoded".into()) }
    }
}

/// urllib.parse.quote_plus (urlencode's)
fn urlencode(s: &str) -> String {
    let mut out = String::new();
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'_' | b'.' | b'-' | b'~' => out.push(b as char),
            b' ' => out.push('+'),
            _ => out += &format!("%{:02X}", b),
        }
    }
    out
}

/// urllib.parse.quote (path segment: '/' kept)
fn quote(s: &str) -> String {
    let mut out = String::new();
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'_' | b'.' | b'-' | b'~' | b'/' => out.push(b as char),
            _ => out += &format!("%{:02X}", b),
        }
    }
    out
}

/// urljoin(base, path) for the absolute-path joins AFC does
fn urljoin(base: &str, path: &str) -> String {
    if let Some(q) = path.strip_prefix('?') {
        let b = base.split('?').next().unwrap_or(base);
        return format!("{}?{}", b, q);
    }
    let (scheme, rest) = base.split_once("://").unwrap_or(("http", base));
    let host = rest.split('/').next().unwrap_or(rest);
    let dir = match rest.rfind('/') {
        Some(i) if i > host.len() => &rest[host.len()..=i],
        _ => "/",
    };
    format!("{}://{}{}{}", scheme, host, dir, path)
}

/// One HTTP/1.1 exchange; Ok((status, reason, body)) or the urllib
/// exception text
fn http_request(req: &Request) -> Result<(u16, String, Vec<u8>), (String, String)> {
    let urlerror = |reason: String| ("urllib.error.URLError".to_string(), format!("<urlopen error {}>", reason));
    let rest = req.url.strip_prefix("http://").ok_or_else(|| urlerror("unknown url type: https".into()))?;
    let (hostport, path) = match rest.find('/') {
        Some(i) => (&rest[..i], &rest[i..]),
        None => (rest, "/"),
    };
    let addr = if hostport.contains(':') { hostport.to_string() } else { format!("{}:80", hostport) };
    let addrs: Vec<std::net::SocketAddr> = std::net::ToSocketAddrs::to_socket_addrs(&addr.as_str()).map_err(|e| urlerror(format!("[Errno -2] {}", e)))?.collect();
    let mut last_err = None;
    let mut stream = None;
    for a in addrs {
        match std::net::TcpStream::connect_timeout(&a, std::time::Duration::from_secs(REQUEST_TIMEOUT)) {
            Ok(s) => {
                stream = Some(s);
                break;
            }
            Err(e) => last_err = Some(e),
        }
    }
    let mut stream = stream.ok_or_else(|| {
        let e = last_err.unwrap_or_else(|| std::io::Error::from(std::io::ErrorKind::ConnectionRefused));
        urlerror(format!("[Errno {}] {}", e.raw_os_error().unwrap_or(111), kaliru::pyfmt::strerror(e.raw_os_error().unwrap_or(0))))
    })?;
    let _ = stream.set_read_timeout(Some(std::time::Duration::from_secs(REQUEST_TIMEOUT)));
    let mut head = format!("{} {} HTTP/1.1\r\nAccept-Encoding: identity\r\nHost: {}\r\nUser-Agent: Python-urllib/3.15\r\nConnection: close\r\n", req.method, path, hostport);
    if let Some(body) = &req.body {
        head += &format!("Content-Length: {}\r\n", body.len());
    }
    if let Some(ct) = &req.content_type {
        head += &format!("Content-Type: {}\r\n", ct);
    }
    head += "\r\n";
    let mut msg = head.into_bytes();
    if let Some(body) = &req.body {
        msg.extend_from_slice(body);
    }
    stream.write_all(&msg).map_err(|e| urlerror(kaliru::pyfmt::strerror(e.raw_os_error().unwrap_or(0))))?;
    let mut raw = vec![];
    stream.read_to_end(&mut raw).map_err(|_| ("TimeoutError".to_string(), "timed out".to_string()))?;
    let split = raw.windows(4).position(|w| w == b"\r\n\r\n").ok_or_else(|| ("http.client.RemoteDisconnected".to_string(), "Remote end closed connection without response".to_string()))?;
    let header = String::from_utf8_lossy(&raw[..split]).to_string();
    let mut body = raw[split + 4..].to_vec();
    let mut lines = header.split("\r\n");
    let status_line = lines.next().unwrap_or("");
    let mut parts = status_line.splitn(3, ' ');
    let _ = parts.next();
    let status: u16 = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0);
    let reason = parts.next().unwrap_or("").to_string();
    if lines.any(|l| l.to_lowercase().starts_with("transfer-encoding:") && l.to_lowercase().contains("chunked")) {
        let mut out = vec![];
        let mut rest = &body[..];
        while let Some(i) = rest.windows(2).position(|w| w == b"\r\n") {
            let size = usize::from_str_radix(String::from_utf8_lossy(&rest[..i]).split(';').next().unwrap_or("0").trim(), 16).unwrap_or(0);
            if size == 0 {
                break;
            }
            let start = i + 2;
            out.extend_from_slice(&rest[start..(start + size).min(rest.len())]);
            rest = &rest[(start + size + 2).min(rest.len())..];
        }
        body = out;
    }
    if status >= 400 {
        return Err(("urllib.error.HTTPError".into(), format!("HTTP Error {}: {}", status, reason)));
    }
    Ok((status, reason, body))
}

/// traceback.format_exc() of a urllib failure (frames are kaliru's)
fn http_traceback(cls: &str, msg: &str) -> String {
    if cls == "urllib.error.URLError" && msg.contains("Errno 111") {
        return format!(
            "Traceback (most recent call last):\n  File \"<kaliru>\"\nConnectionRefusedError: [Errno 111] Connection refused\n\nDuring handling of the above exception, another exception occurred:\n\nTraceback (most recent call last):\n  File \"<kaliru>\"\n{}: {}\n",
            cls, msg
        );
    }
    format!("Traceback (most recent call last):\n  File \"<kaliru>\"\n{}: {}\n", cls, msg)
}

/// A log call queued for the reactor thread
enum LogFn {
    Error,
    Debug,
    Info,
}

/// A queued write's job (runs on the writer thread)
enum Job {
    Request(Request, u64),
    Stop,
}

/// The moonraker results returned from the writer thread
enum JobResult {
    Data(Option<String>),
    Failed { cls: String, msg: String, status: Option<(u16, String)> },
}

thread_local! {
    static PENDING: RefCell<HashMap<u64, (Rc<Moonraker>, Box<dyn FnOnce(&Moonraker, JobResult)>)>> = RefCell::new(HashMap::new());
    static NEXT_JOB: Cell<u64> = const { Cell::new(1) };
}

/// AFC_moonraker: moonraker's HTTP API, writes on a background thread
pub struct Moonraker {
    pub logger: Rc<AfcLogger>,
    reactor: Rc<Reactor>,
    host: String,
    database_url: String,
    afc_stats: RefCell<Option<PyVal>>,
    last_stats_time: Cell<Option<f64>>,
    lane_data: Cell<bool>,
    tx: mpsc::Sender<Job>,
    thread: RefCell<Option<std::thread::JoinHandle<()>>>,
    me: std::rc::Weak<Moonraker>,
}

impl Moonraker {
    pub fn new(host: &str, port: &str, logger: Rc<AfcLogger>, reactor: Rc<Reactor>) -> Rc<Moonraker> {
        let host = format!("{}:{}", host.trim_end_matches('/'), port);
        let database_url = urljoin(&host, "server/database/item");
        let (tx, rx) = mpsc::channel::<Job>();
        let handle: AsyncHandle = reactor.async_handle();
        let thread = std::thread::Builder::new()
            .name("afc_moonraker".into())
            .spawn(move || {
                while let Ok(Job::Request(req, id)) = rx.recv() {
                    let result = match http_request(&req) {
                        Ok((_, _, body)) => JobResult::Data(Some(String::from_utf8_lossy(&body).into_owned())),
                        Err((cls, msg)) => JobResult::Failed { cls, msg, status: None },
                    };
                    // (the result crosses as plain data; the reactor thread
                    // finds its callback by id)
                    handle.register_async_callback(
                        Box::new(move |_| {
                            let r = result;
                            PENDING.with(|p| {
                                if let Some((m, cb)) = p.borrow_mut().remove(&id) {
                                    cb(&m, r);
                                }
                            });
                        }),
                        NOW,
                    );
                }
            })
            .ok();
        let m = Rc::new_cyclic(|me| Moonraker {
            logger,
            reactor,
            host,
            database_url,
            afc_stats: RefCell::new(None),
            last_stats_time: Cell::new(None),
            lane_data: Cell::new(false),
            tx,
            thread: RefCell::new(thread),
            me: me.clone(),
        });
        m.logger.debug(&AFC_UTILS_AFC_MOONRAKER___INIT__, &format!("Moonraker url: {}", m.host));
        m
    }

    fn rc(&self) -> Rc<Moonraker> {
        self.me.upgrade().expect("moonraker")
    }

    /// _log_async: the log call on the reactor thread
    fn log_async(&self, log_fn: LogFn, message: String, traceback: Option<String>) {
        let logger = self.logger.clone();
        self.reactor.register_async_callback_local(move |_| Self::log_now(&logger, log_fn, &message, traceback.as_deref()), NOW);
    }

    /// The body of _log_async's callback (its lambda's frame in AFC.log)
    fn log_now(logger: &AfcLogger, log_fn: LogFn, message: &str, traceback: Option<&str>) {
        let c = &AFC_UTILS_AFC_MOONRAKER__LOG_ASYNC_LAMBDA_488;
        match log_fn {
            LogFn::Error => logger.error_opt(c, message, traceback, ""),
            LogFn::Debug => logger.debug_opt(c, message, false, traceback),
            LogFn::Info => logger.info(c, message),
        }
    }

    pub fn join_thread(&self) {
        let _ = self.tx.send(Job::Stop);
        if let Some(t) = self.thread.borrow_mut().take() {
            let _ = t.join();
        }
    }

    /// _get_results: the response's "result", None (logged) on failure
    fn get_results(&self, req: &Request, print_error: bool) -> Option<PyVal> {
        let log = || if print_error { LogFn::Error } else { LogFn::Debug };
        match http_request(req) {
            Ok((status, reason, body)) => {
                if (200..=300).contains(&status) {
                    match PyVal::json_loads(&String::from_utf8_lossy(&body)) {
                        Ok(data) => Self::result_of(&data),
                        Err(e) => {
                            self.log_async(log(), ERROR_STRING.into(), Some(format!("Traceback (most recent call last):\n  File \"<kaliru>\"\njson.decoder.JSONDecodeError: {}\n", e)));
                            None
                        }
                    }
                } else {
                    self.log_async(log(), ERROR_STRING.into(), None);
                    self.log_async(log(), format!("Response: {} Reason: {}", status, reason), None);
                    None
                }
            }
            Err((cls, msg)) => {
                self.log_async(log(), ERROR_STRING.into(), Some(http_traceback(&cls, &msg)));
                None
            }
        }
    }

    fn result_of(data: &PyVal) -> Option<PyVal> {
        match data.get() {
            PyVal::Dict(d) => d.get("result").cloned(),
            _ => None,
        }
    }

    /// Queues a request on the writer thread; done(result) on the reactor thread
    fn queue(&self, req: Request, done: Box<dyn FnOnce(&Moonraker, Option<PyVal>)>, print_error: bool) {
        let id = NEXT_JOB.with(|n| {
            let id = n.get();
            n.set(id + 1);
            id
        });
        // (python's writer thread queues its log calls and result callbacks
        // in one FIFO: a job's all run here, on the reactor thread, in order,
        // before the next job's)
        let cb: Box<dyn FnOnce(&Moonraker, JobResult)> = Box::new(move |m, r| {
            let log = || if print_error { LogFn::Error } else { LogFn::Debug };
            let result = match r {
                JobResult::Data(Some(text)) => match PyVal::json_loads(&text) {
                    Ok(d) => Self::result_of(&d),
                    Err(_) => None,
                },
                JobResult::Data(None) => None,
                JobResult::Failed { cls, msg, status } => {
                    match status {
                        Some((s, reason)) => {
                            Self::log_now(&m.logger, log(), ERROR_STRING, None);
                            Self::log_now(&m.logger, log(), &format!("Response: {} Reason: {}", s, reason), None);
                        }
                        None => Self::log_now(&m.logger, log(), ERROR_STRING, Some(&http_traceback(&cls, &msg))),
                    }
                    None
                }
            };
            done(m, result);
        });
        PENDING.with(|p| p.borrow_mut().insert(id, (self.rc(), cb)));
        let _ = self.tx.send(Job::Request(req, id));
    }

    /// Waits up to timeout seconds (toolhead dwells) for moonraker
    pub fn wait_for_moonraker(&self, toolhead: &ToolHead, timeout: i64) -> KResult<bool> {
        let c = &AFC_UTILS_AFC_MOONRAKER_WAIT_FOR_MOONRAKER;
        self.logger.info(c, &format!("Waiting max {}s for moonraker to connect", timeout));
        for i in 0..timeout {
            if self.get_results(&Request::get(urljoin(&self.host, "server/info")), false).is_some() {
                self.logger.debug(c, &format!("Connected to moonraker after {} tries", i));
                return Ok(true);
            }
            toolhead.dwell(1.0)?;
        }
        self.logger.warning(c, &format!("Failed to connect to moonraker after {} seconds, check AFC.log for more information", timeout));
        Ok(false)
    }

    fn field(v: &PyVal, k: &str) -> Option<PyVal> {
        match v.get() {
            PyVal::Dict(d) => d.get(k).cloned(),
            _ => None,
        }
    }

    pub fn get_spoolman_server(&self) -> Option<String> {
        let resp = self.get_results(&Request::get(urljoin(&self.host, "server/config")), true);
        match resp.as_ref().and_then(|r| Self::field(r, "orig")).and_then(|o| Self::field(&o, "spoolman")) {
            Some(s) => Some(Self::field(&s, "server").map(|v| v.str()).unwrap_or_default()),
            None => {
                self.logger.debug(&AFC_UTILS_AFC_MOONRAKER_GET_SPOOLMAN_SERVER, "Spoolman server is not defined");
                None
            }
        }
    }

    pub fn get_file_metadata(&self, filename: &str, callback: Box<dyn FnOnce(Option<PyVal>)>) {
        let req = Request::get(urljoin(&self.host, &format!("{}{}", FILENAME_PATH, quote(filename))));
        self.queue(req, Box::new(move |_, r| callback(r)), true);
    }

    /// The afc_stats namespace (refetched after 60 s)
    pub fn get_afc_stats(&self) -> Option<PyVal> {
        let now = datetime_now().secs;
        let mut refetch = true;
        match self.last_stats_time.get() {
            Some(t) => {
                refetch = false;
                if (now - t).floor() as i64 % 86400 > 60 {
                    refetch = true;
                    self.last_stats_time.set(Some(now));
                }
            }
            None => self.last_stats_time.set(Some(now)),
        }
        if self.afc_stats.borrow().is_none() || refetch {
            let resp = self.get_results(&Request::get(urljoin(&self.database_url, &format!("?namespace={}", AFC_STATS_KEY))), true);
            match resp {
                Some(r) => *self.afc_stats.borrow_mut() = Some(r),
                None => self.logger.debug(&AFC_UTILS_AFC_MOONRAKER_GET_AFC_STATS, "AFC_stats not in database"),
            }
        }
        self.afc_stats.borrow().as_ref().and_then(|s| Self::field(s, "value"))
    }

    pub fn update_afc_stats(&self, key: &str, value: PyVal) {
        let req = Request::form(
            self.database_url.clone(),
            &[("request_method", "POST".into()), ("namespace", AFC_STATS_KEY.into()), ("key", key.to_string()), ("value", value.str())],
            None,
        );
        let key = key.to_string();
        self.queue(
            req,
            Box::new(move |m, r| {
                if r.is_none() {
                    Self::log_now(&m.logger, LogFn::Error, &format!("Error when trying to update {} in moonraker, see AFC.log for more info", key), None);
                }
            }),
            true,
        );
    }

    pub fn get_spool(&self, id: i64, callback: Box<dyn FnOnce(Option<PyVal>) -> KResult<()>>) {
        let req = Request::form(urljoin(&self.host, "server/spoolman/proxy"), &[("request_method", "GET".into()), ("path", format!("/v1/spool/{}", id))], None);
        self.queue(
            req,
            Box::new(move |m, r| {
                if r.is_none() {
                    Self::log_now(&m.logger, LogFn::Info, &format!("SpoolID: {} not found", id), None);
                }
                if let Err(e) = callback(r) {
                    kaliru::logging::exception_err("AFC spoolman", &e);
                }
            }),
            true,
        );
    }

    /// ([td1] defined, a TD-1 found, [lane_data] defined)
    pub fn check_for_td1(&self) -> (bool, bool, bool) {
        let mut td1 = false;
        let mut td1_defined = false;
        if let Some(resp) = self.get_results(&Request::get(urljoin(&self.host, "server/config")), true) {
            let orig = Self::field(&resp, "orig");
            if orig.as_ref().and_then(|o| Self::field(o, "td1")).is_some() {
                td1_defined = true;
                if self.get_td1_data().is_some_and(|d| matches!(d.get(), PyVal::Dict(m) if !m.is_empty())) {
                    td1 = true;
                }
            }
            if orig.as_ref().and_then(|o| Self::field(o, "lane_data")).is_some() {
                self.lane_data.set(true);
            }
        }
        (td1_defined, td1, self.lane_data.get())
    }

    pub fn get_td1_data(&self) -> Option<PyVal> {
        let resp = self.get_results(&Request::get(urljoin(&self.host, "machine/td1/data")), true)?;
        Self::field(&resp, "devices").filter(|d| d.truthy())
    }

    pub fn get_td1_data_async(&self, callback: Box<dyn FnOnce(Option<PyVal>) -> KResult<()>>) {
        let req = Request::get(urljoin(&self.host, "machine/td1/data"));
        self.queue(
            req,
            Box::new(move |m, r| {
                let devices = r.as_ref().and_then(|r| Self::field(r, "devices"));
                let m = m.rc();
                m.reactor.register_async_callback_local(
                    move |_| {
                        if let Err(e) = callback(devices) {
                            kaliru::logging::exception_err("AFC TD-1", &e);
                        }
                    },
                    NOW,
                );
            }),
            true,
        );
    }

    pub fn reboot_td1(&self, serial_number: &str) -> Option<PyVal> {
        let req = Request::form(urljoin(&self.host, "machine/td1/reboot"), &[("request_method", "POST".into()), ("serial", serial_number.to_string())], None);
        self.get_results(&req, true)
    }

    pub fn send_lane_data(&self, data: PyVal) {
        let req = Request { url: self.database_url.clone(), method: "POST".into(), body: Some(data.json().into_bytes()), content_type: Some("application/json".into()) };
        self.queue(
            req,
            Box::new(|m, r| {
                if r.is_none() {
                    Self::log_now(&m.logger, LogFn::Error, "Error sending lane data, check AFC.log for more information", None);
                }
            }),
            true,
        );
    }

    pub fn remove_database_entry(&self, namespace: &str, key: &str) {
        let req = Request::form(
            self.database_url.clone(),
            &[("request_method", "DELETE".into()), ("namespace", namespace.to_string()), ("key", key.to_string())],
            Some("DELETE"),
        );
        let (ns, k) = (namespace.to_string(), key.to_string());
        self.queue(req, Box::new(move |m, _| Self::log_now(&m.logger, LogFn::Debug, &format!("Removing {} from {}", k, ns), None)), false);
    }

    /// Removes the lane_data namespace (stale lanes from another setup)
    pub fn delete_lane_data(&self) {
        if let Some(resp) = self.get_results(&Request::get(urljoin(&self.database_url, "?namespace=lane_data")), false) {
            if let Some(PyVal::Dict(v)) = Self::field(&resp, "value").as_ref().map(|v| v.get().clone()) {
                for key in v.keys() {
                    self.remove_database_entry("lane_data", key);
                }
            }
        }
    }

    /// True on error
    pub fn trigger_db_backup(&self) -> bool {
        let c = &AFC_UTILS_AFC_MOONRAKER_TRIGGER_DB_BACKUP;
        let req = Request { url: urljoin(&self.host, "server/database/backup"), method: "POST".into(), body: None, content_type: Some("application/json".into()) };
        match self.get_results(&req, true) {
            None => {
                self.logger.error(c, "Error trying to backup moonraker database, check AFC.log for more information");
                true
            }
            Some(r) => {
                self.logger.info(c, &format!("Moonrakers database backed up to {}", Self::field(&r, "backup_path").map(|v| v.str()).unwrap_or_default()));
                false
            }
        }
    }
}

/// AFC_PrintFileMetaData: the printed file's metadata, cached
pub struct PrintFileMetaData {
    moonraker: Rc<Moonraker>,
    logger: Rc<AfcLogger>,
    filename: RefCell<String>,
    metadata: RefCell<PyVal>,
}

impl PrintFileMetaData {
    pub fn new(moonraker: Rc<Moonraker>, logger: Rc<AfcLogger>) -> PrintFileMetaData {
        PrintFileMetaData { moonraker, logger, filename: RefCell::new(String::new()), metadata: RefCell::new(PyVal::dict()) }
    }

    pub fn query_filename(self: &Rc<Self>, value: &str, on_fetched: Option<Box<dyn FnOnce() -> KResult<()>>>) {
        *self.filename.borrow_mut() = value.to_string();
        if !value.is_empty() {
            let (me, v) = (self.clone(), value.to_string());
            self.moonraker.get_file_metadata(
                value,
                Box::new(move |resp| {
                    if v == *me.filename.borrow() {
                        *me.metadata.borrow_mut() = resp.filter(|r| r.truthy()).unwrap_or_else(PyVal::dict);
                    }
                    if let Some(f) = on_fetched {
                        if let Err(e) = f() {
                            kaliru::logging::exception_err("AFC print start", &e);
                        }
                    }
                }),
            );
        } else if let Some(f) = on_fetched {
            if let Err(e) = f() {
                kaliru::logging::exception_err("AFC print start", &e);
            }
        }
    }

    pub fn tool_change_count(&self) -> i64 {
        let md = self.metadata.borrow().clone();
        match Moonraker::field(&md, "filament_change_count") {
            Some(v) if md.truthy() => py_int_of(&v).unwrap_or(0),
            _ => {
                self.logger.debug(
                    &AFC_UTILS_AFC_PRINTFILEMETADATA_TOOL_CHANGE_COUNT,
                    &format!("Filament change count metadata not found for file:{}", self.filename.borrow()),
                );
                0
            }
        }
    }

    pub fn tool_temperatures(&self) -> Vec<PyVal> {
        let md = self.metadata.borrow().clone();
        if !md.truthy() {
            return vec![];
        }
        let mut l = Moonraker::field(&md, "filament_temps").unwrap_or(PyVal::List(vec![]));
        if !l.truthy() {
            l = Moonraker::field(&md, "nozzle_temp").unwrap_or(PyVal::List(vec![]));
        }
        l.items().map(|i| i.to_vec()).unwrap_or_default()
    }

    pub fn reset(&self) {
        self.filename.borrow_mut().clear();
        *self.metadata.borrow_mut() = PyVal::dict();
    }
}

// ----------------------------------------------------------------------
// the var file
// ----------------------------------------------------------------------

type ErrorCallback = Box<dyn FnOnce(String)>;

thread_local! {
    static VAR_ERRORS: RefCell<HashMap<u64, ErrorCallback>> = RefCell::new(HashMap::new());
}

enum VarJob {
    Write(String, String, u64),
    Stop,
}

/// save_vars' writer thread: tmp + fsync + rename, in queue order
pub struct VarWriter {
    tx: mpsc::Sender<VarJob>,
    thread: RefCell<Option<std::thread::JoinHandle<()>>>,
    next: Cell<u64>,
}

impl VarWriter {
    pub fn start(reactor: Rc<Reactor>) -> VarWriter {
        let (tx, rx) = mpsc::channel::<VarJob>();
        let handle = reactor.async_handle();
        let thread = std::thread::Builder::new()
            .name("afc_save_vars".into())
            .spawn(move || {
                while let Ok(VarJob::Write(target, text, id)) = rx.recv() {
                    let temp_path = format!("{}.tmp", target);
                    let r = (|| -> std::io::Result<()> {
                        let mut f = std::fs::File::create(&temp_path)?;
                        f.write_all(text.as_bytes())?;
                        f.flush()?;
                        f.sync_all()?;
                        std::fs::rename(&temp_path, &target)
                    })();
                    if let Err(e) = r {
                        let path = if std::path::Path::new(&temp_path).parent().is_some_and(|p| p.exists()) { target.clone() } else { temp_path.clone() };
                        let err = format!(
                            "Error:{}\nTraceback (most recent call last):\n  File \"<kaliru>\"\n{}: {}\n",
                            kaliru::pyfmt::py_oserror(&e, &path),
                            kaliru::pyfmt::py_oserror_class(&e),
                            kaliru::pyfmt::py_oserror(&e, &path)
                        );
                        handle.register_async_callback(
                            Box::new(move |_| {
                                VAR_ERRORS.with(|v| {
                                    if let Some(cb) = v.borrow_mut().remove(&id) {
                                        cb(err);
                                    }
                                })
                            }),
                            NOW,
                        );
                    } else {
                        handle.register_async_callback(Box::new(move |_| VAR_ERRORS.with(|v| drop(v.borrow_mut().remove(&id)))), NOW);
                    }
                }
            })
            .ok();
        VarWriter { tx, thread: RefCell::new(thread), next: Cell::new(1) }
    }

    /// json.dumps(data, indent=4) to target (on the writer thread)
    pub fn write(&self, target: String, data: PyVal, on_error: ErrorCallback) {
        let id = self.next.get();
        self.next.set(id + 1);
        VAR_ERRORS.with(|v| v.borrow_mut().insert(id, on_error));
        let _ = self.tx.send(VarJob::Write(target, data.json_indent(4), id));
    }

    pub fn stop(&self) {
        let _ = self.tx.send(VarJob::Stop);
        if let Some(t) = self.thread.borrow_mut().take() {
            let _ = t.join();
        }
    }
}

