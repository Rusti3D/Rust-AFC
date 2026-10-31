// The ACE serial links of AFC_ACE.py (cosmoace_afc): AceTransport, the ACE
// Pro's thread (framed JSON over USB), and Ace2Bus / Ace2Transport, the ACE 2
// Pro's shared RS-485 bus (protobuf). Each runs on its own thread, as in
// python; the reactor side submits requests and polls them.
use kaliru::pyfmt::{py_float_repr, py_str_repr, strerror};
use serde_json::{json, Map, Value};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{channel, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::time::Duration;

// ACE firmware states that mean "a motor is moving"
pub const ACTIVE_STATES: [&str; 5] = ["busy", "feeding", "unwinding", "shifting", "preload"];
pub const STOP_METHODS: [&str; 3] = ["stop_feed_filament", "stop_unwind_filament", "stop_feed_assist"];
const STOP_REPLY_S: f64 = 1.0;
const RPC_REPLY_S: f64 = 5.0;
pub const HEARTBEAT_S: f64 = 0.8;

const ACE2_BAUD: u32 = 230400;
const ACE2_USB_ID: (&str, &str) = ("1a86", "55d3");
const ACE2_REPLY: u8 = 0x80;
const ACE2_DISCOVER: u8 = 0;
const ACE2_ASSIGN: u8 = 1;
const ACE2_GET_STATUS: u8 = 6;
const ACE2_FEED: u8 = 8;
const ACE2_STOP: u8 = 9;
const ACE2_DRYING: u8 = 11;
const ACE2_FILAMENT_INFO: u8 = 13;
const ACE2_PRINTER_STATUS: u8 = 20;
pub const ACE2_FORBIDDEN_WAIT_S: f64 = 30.0;
const ACE2_FORBIDDEN: u64 = 2;
pub const ACE2_MOTION_METHODS: [&str; 3] = ["feed_filament", "unwind_filament", "start_feed_assist"];
pub const ACE2_ASSIST_ERROR: u64 = 131;
const ACE2_MAX_SPEED: i64 = 100;
const ACE2_STOP_AFTER_START_S: f64 = 0.25;
pub const ACE2_PRELOAD_MAX_MM: f64 = 750.0;

pub fn monotonic() -> f64 {
    kaliru::chelper::pyhelper::get_monotonic()
}

fn sleep(secs: f64) {
    std::thread::sleep(Duration::from_secs_f64(secs.max(0.0)));
}

/// str() of a json value as python prints it in an f-string
pub fn py_str(v: &Value) -> String {
    match v {
        Value::String(s) => s.clone(),
        v => py_repr(v),
    }
}

/// repr() of a json value (a json.loads result) as python prints it
pub fn py_repr(v: &Value) -> String {
    match v {
        Value::Null => "None".into(),
        Value::Bool(b) => if *b { "True" } else { "False" }.into(),
        Value::Number(n) => match n.as_i64() {
            Some(i) => i.to_string(),
            None => match n.as_u64() {
                Some(u) => u.to_string(),
                None => py_float_repr(n.as_f64().unwrap_or(0.0)),
            },
        },
        Value::String(s) => py_str_repr(s),
        Value::Array(a) => format!("[{}]", a.iter().map(py_repr).collect::<Vec<_>>().join(", ")),
        Value::Object(m) => format!("{{{}}}", m.iter().map(|(k, v)| format!("{}: {}", py_str_repr(k), py_repr(v))).collect::<Vec<_>>().join(", ")),
    }
}

/// python truthiness of a json value
pub fn truthy(v: &Value) -> bool {
    match v {
        Value::Null => false,
        Value::Bool(b) => *b,
        Value::Number(n) => n.as_f64().is_some_and(|f| f != 0.0),
        Value::String(s) => !s.is_empty(),
        Value::Array(a) => !a.is_empty(),
        Value::Object(m) => !m.is_empty(),
    }
}

/// "<params or ''>": the params dict, '' when there are none
fn params_str(params: &Option<Map<String, Value>>) -> String {
    match params {
        Some(p) if !p.is_empty() => py_repr(&Value::Object(p.clone())),
        _ => String::new(),
    }
}

/// An rpc's result dict: {"ok", "error", "response", "sent", ...}
#[derive(Clone, Default, Debug)]
pub struct RpcResult {
    pub ok: bool,
    pub error: Option<String>,
    pub response: Option<Value>,
    pub sent: bool,
    pub assumed_started: bool,
    pub assumed_success: bool,
}

impl RpcResult {
    pub fn err(e: impl Into<String>) -> Self {
        RpcResult { error: Some(e.into()), ..Default::default() }
    }

    pub fn ok_with(response: Value) -> Self {
        RpcResult { ok: true, response: Some(response), ..Default::default() }
    }

    /// res.get('error') in an f-string
    pub fn error_str(&self) -> String {
        self.error.clone().unwrap_or_else(|| "None".into())
    }

    /// res.get("response", {}).get("result")
    pub fn result(&self) -> Option<&Value> {
        self.response.as_ref().and_then(|r| r.get("result"))
    }
}

/// A queued request: filled in place, then done is set
pub struct Pending {
    pub result: Mutex<RpcResult>,
    pub done: AtomicBool,
    pub cancelled: AtomicBool,
}

impl Pending {
    fn new() -> Arc<Pending> {
        Arc::new(Pending { result: Mutex::new(RpcResult::default()), done: AtomicBool::new(false), cancelled: AtomicBool::new(false) })
    }

    fn set_sent(&self) {
        self.result.lock().unwrap().sent = true;
    }

    fn cancelled(&self) -> bool {
        self.cancelled.load(Ordering::SeqCst)
    }

    /// result.update(res): res's keys over the ones already set ("sent")
    fn update(&self, res: RpcResult) {
        let mut r = self.result.lock().unwrap();
        let sent = r.sent || res.sent;
        *r = res;
        r.sent = sent;
    }

    fn finish(&self) {
        self.done.store(true, Ordering::SeqCst);
    }
}

type Request = (String, Option<Map<String, Value>>, Arc<Pending>);

/// What the unit reads from either transport (AceTransport's attributes)
#[derive(Default)]
pub struct TransportState {
    status: Mutex<(Option<Value>, f64)>,
    notes: Mutex<Vec<String>>,
    pub last_error: Mutex<Option<String>>,
    pub connected: AtomicBool,
    pub generation: AtomicU64,
    shutdown: AtomicBool,
    last_state: Mutex<Option<(Value, Value, Vec<Value>)>>,
    assist_pumping: AtomicBool,
    // slot -> when status last showed it in the ACE's own insert preload
    preload_at: Mutex<HashMap<i64, f64>>,
}

impl TransportState {
    pub fn cached_status(&self) -> (Option<Value>, f64) {
        self.status.lock().unwrap().clone()
    }

    /// preloaded_since: whether status showed the slot in the ACE's own
    /// insert preload at or after `since` (monotonic)
    pub fn preloaded_since(&self, slot: i64, since: f64) -> bool {
        self.preload_at.lock().unwrap().get(&slot).is_some_and(|&t| t >= since)
    }

    pub fn drain_notes(&self) -> Vec<String> {
        std::mem::take(&mut *self.notes.lock().unwrap())
    }

    pub fn note(&self, message: String) {
        self.notes.lock().unwrap().push(message);
    }

    pub fn last_error(&self) -> Option<String> {
        self.last_error.lock().unwrap().clone()
    }

    fn set_error(&self, e: Option<String>) {
        *self.last_error.lock().unwrap() = e;
    }

    pub fn is_connected(&self) -> bool {
        self.connected.load(Ordering::SeqCst)
    }

    pub fn generation(&self) -> u64 {
        self.generation.load(Ordering::SeqCst)
    }

    fn shutting_down(&self) -> bool {
        self.shutdown.load(Ordering::SeqCst)
    }

    /// _store_status: every successful get_status refreshes the cache (the
    /// stale-tag cleanup lands in the reply itself too, as in python)
    fn store_status(&self, res: &mut RpcResult) {
        if !res.ok {
            return;
        }
        let Some(Value::Object(payload)) = res.response.as_mut().and_then(|r| r.get_mut("result")) else { return };
        // The ACE Pro keeps a slot's last tag after the spool leaves; only
        // rfid 2 (identified) is live. ACE 2 slots carry no "rfid" key.
        let now = monotonic();
        if let Some(Value::Array(slots)) = payload.get_mut("slots") {
            for (index, slot) in slots.iter_mut().enumerate() {
                if let Value::Object(s) = slot {
                    if s.get("status").and_then(|v| v.as_str()) == Some("preload") {
                        self.preload_at.lock().unwrap().insert(index as i64, now);
                    }
                    let rfid = s.get("rfid").map(|v| v.as_f64()).unwrap_or(Some(2.0));
                    if rfid != Some(2.0) {
                        s.insert("sku".into(), json!(""));
                        s.insert("type".into(), json!(""));
                        s.insert("color".into(), json!([0, 0, 0]));
                        s.insert("colors".into(), json!([[0, 0, 0, 0]]));
                    }
                }
            }
        }
        let payload = Value::Object(payload.clone());
        *self.status.lock().unwrap() = (Some(payload.clone()), now);
        if let Err(e) = self.log_state_change(&payload) {
            kaliru::logging::debug(&format!("AFC_ACE: state log skipped: {}", e));
        }
    }

    /// _log_state_change: the motion state whenever it changes, and the
    /// start of each run of continuous feed-assist pumping
    fn log_state_change(&self, status: &Value) -> Result<(), String> {
        let slots: Vec<Value> = match status.get("slots") {
            Some(Value::Array(a)) => a.iter().filter(|s| s.is_object()).map(|s| s.get("status").cloned().unwrap_or(Value::Null)).collect(),
            _ => vec![],
        };
        let get = |k: &str| status.get(k).cloned().unwrap_or(Value::Null);
        let state = (get("status"), get("action"), slots);
        let mut last = self.last_state.lock().unwrap();
        if last.as_ref() != Some(&state) {
            kaliru::logging::info(&format!(
                "AFC_ACE: t={:.2} state {}/{} slots [{}]",
                monotonic(),
                py_str(&state.0),
                py_str(&state.1),
                state.2.iter().map(py_repr).collect::<Vec<_>>().join(", ")
            ));
            *last = Some(state);
        }
        drop(last);
        let cat = get("cont_assist_time");
        let cat_f = if truthy(&cat) {
            match &cat {
                Value::Number(n) => n.as_f64().unwrap_or(0.0),
                Value::Bool(b) => *b as i64 as f64,
                Value::String(s) => s.trim().parse::<f64>().map_err(|_| format!("could not convert string to float: {}", py_str_repr(s)))?,
                _ => return Err(format!("float() argument must be a string or a real number, not '{}'", if cat.is_array() { "list" } else { "dict" })),
            }
        } else {
            0.0
        };
        let pumping = cat_f >= 2.0;
        if pumping != self.assist_pumping.load(Ordering::SeqCst) {
            self.assist_pumping.store(pumping, Ordering::SeqCst);
            kaliru::logging::info(&format!(
                "AFC_ACE: t={:.2} assist {} (cont_assist_time={}, feed_assist_count={})",
                monotonic(),
                if pumping { "pumping" } else { "idle" },
                py_str(&cat),
                py_str(&get("feed_assist_count"))
            ));
        }
        Ok(())
    }
}

// ── the serial port, as pyserial uses it ────────────────────────────

/// "[Errno N] message" (str() of an OSError without a filename)
fn errno_str(e: &std::io::Error) -> String {
    let errno = e.raw_os_error().unwrap_or(0);
    format!("[Errno {}] {}", errno, strerror(errno))
}

struct Port {
    fd: i32,
    timeout: f64,
    write_timeout: f64,
}

impl Port {
    /// serial.Serial(port, baud, timeout=, write_timeout=, exclusive=)
    fn open(port: &str, baud: u32, timeout: f64, write_timeout: f64, exclusive: Option<bool>) -> Result<Port, String> {
        let uart = kaliru::serialhdl::Uart::open(port, baud, exclusive, None).map_err(|e| e.message().to_string())?;
        Ok(Port { fd: uart.into_fd(), timeout, write_timeout })
    }

    /// read(n): up to n bytes within the read timeout
    fn read(&self, n: usize) -> Result<Vec<u8>, String> {
        let mut out = vec![];
        let deadline = monotonic() + self.timeout;
        while out.len() < n {
            let left = deadline - monotonic();
            if left <= 0.0 {
                break;
            }
            let mut pfd = libc::pollfd { fd: self.fd, events: libc::POLLIN, revents: 0 };
            // SAFETY: pfd is one live pollfd
            let r = unsafe { libc::poll(&mut pfd, 1, (left * 1000.0).ceil() as i32) };
            if r < 0 {
                let e = std::io::Error::last_os_error();
                if e.raw_os_error() == Some(libc::EINTR) {
                    continue;
                }
                return Err(format!("read failed: {}", errno_str(&e)));
            }
            if r == 0 {
                break;
            }
            let mut buf = vec![0u8; n - out.len()];
            // SAFETY: buf is a live buffer of its length
            let got = unsafe { libc::read(self.fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
            if got < 0 {
                let e = std::io::Error::last_os_error();
                match e.raw_os_error() {
                    Some(libc::EAGAIN) | Some(libc::EALREADY) | Some(libc::EINPROGRESS) | Some(libc::EINTR) => continue,
                    _ => return Err(format!("read failed: {}", errno_str(&e))),
                }
            }
            if got == 0 {
                return Err("device reports readiness to read but returned no data (device disconnected or multiple access on port?)".into());
            }
            out.extend_from_slice(&buf[..got as usize]);
        }
        Ok(out)
    }

    /// write(data) then flush()
    fn write(&self, data: &[u8]) -> Result<(), String> {
        let deadline = monotonic() + self.write_timeout;
        let mut sent = 0;
        while sent < data.len() {
            // SAFETY: the unsent tail of data is live
            let r = unsafe { libc::write(self.fd, data[sent..].as_ptr() as *const libc::c_void, data.len() - sent) };
            if r < 0 {
                let e = std::io::Error::last_os_error();
                if e.raw_os_error() != Some(libc::EAGAIN) {
                    return Err(format!("write failed: {}", errno_str(&e)));
                }
            } else {
                sent += r as usize;
                continue;
            }
            let left = deadline - monotonic();
            if left <= 0.0 {
                return Err("Write timeout".into());
            }
            let mut pfd = libc::pollfd { fd: self.fd, events: libc::POLLOUT, revents: 0 };
            // SAFETY: pfd is one live pollfd
            unsafe { libc::poll(&mut pfd, 1, (left * 1000.0).ceil() as i32) };
        }
        // SAFETY: the port's own fd
        unsafe { libc::tcdrain(self.fd) };
        Ok(())
    }

    fn reset_input_buffer(&self) {
        // SAFETY: the port's own fd
        unsafe { libc::tcflush(self.fd, libc::TCIFLUSH) };
    }
}

impl Drop for Port {
    fn drop(&mut self) {
        // SAFETY: the port's own fd, closed once
        unsafe { libc::close(self.fd) };
    }
}

fn crc16_mcrf4xx(data: &[u8]) -> u16 {
    let mut crc: u32 = 0xFFFF;
    for &byte in data {
        let mut v = (byte as u32) ^ (crc & 0xFF);
        v ^= (v & 0x0F) << 4;
        crc = (((v << 8) | (crc >> 8)) ^ (v >> 4) ^ (v << 3)) & 0xFFFF;
    }
    crc as u16
}

fn build_frame(payload: &[u8]) -> Vec<u8> {
    let mut frame = vec![0xFF, 0xAA];
    frame.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    frame.extend_from_slice(payload);
    frame.extend_from_slice(&crc16_mcrf4xx(payload).to_le_bytes());
    frame.push(0xFE);
    frame
}

fn read_exact(ser: &Port, count: usize, deadline: f64) -> Result<Vec<u8>, String> {
    let mut out = vec![];
    while out.len() < count && monotonic() < deadline {
        out.extend(ser.read(count - out.len())?);
    }
    Ok(out)
}

/// _usb_path_key: orders usb path 1-1.4.3 before 1-1.4.10
fn usb_path_key(path: &str) -> Vec<(u8, u64, String)> {
    let mut out = vec![];
    let mut cur = String::new();
    let mut digits = false;
    let flush = |cur: &mut String, digits: bool, out: &mut Vec<(u8, u64, String)>| {
        if digits {
            out.push((1, cur.parse().unwrap_or(0), String::new()));
        } else {
            out.push((0, 0, cur.clone()));
        }
        cur.clear();
    };
    // re.split(r"(\d+)") always starts and ends with a (maybe empty) text part
    for ch in path.chars() {
        let d = ch.is_ascii_digit();
        if d != digits {
            flush(&mut cur, digits, &mut out);
            digits = d;
        }
        cur.push(ch);
    }
    flush(&mut cur, digits, &mut out);
    if digits {
        out.push((0, 0, String::new()));
    }
    out
}

fn list_dir(path: &str) -> Vec<String> {
    match std::fs::read_dir(path) {
        Ok(rd) => rd.filter_map(|e| e.ok()).map(|e| e.file_name().to_string_lossy().into_owned()).collect(),
        Err(_) => vec![],
    }
}

/// scan_ace_ports: {usb device path: tty device} of every ACE on USB
pub fn scan_ace_ports() -> Vec<(String, String)> {
    let mut found: Vec<(String, String)> = vec![];
    for name in list_dir("/sys/bus/usb/devices") {
        let dev = format!("/sys/bus/usb/devices/{}", name);
        if !std::path::Path::new(&dev).is_dir() {
            continue;
        }
        let Ok(product) = std::fs::read(format!("{}/product", dev)) else { continue };
        if !String::from_utf8_lossy(&product).to_uppercase().contains("ACE") {
            continue;
        }
        for intf in list_dir(&dev) {
            for tty in list_dir(&format!("{}/{}/tty", dev, intf)).into_iter().filter(|t| t.starts_with("tty")) {
                let tty = format!("/dev/{}", tty);
                match found.iter_mut().find(|(d, _)| *d == dev) {
                    Some(e) => e.1 = tty,
                    None => found.push((dev.clone(), tty)),
                }
            }
        }
    }
    found
}

// ── ACE Pro ─────────────────────────────────────────────────────────

/// AceTransport: owns an ACE Pro's serial port on its own thread
pub struct AceTransport {
    pub state: Arc<TransportState>,
    tx: Mutex<Option<Sender<Request>>>,
}

struct ProWorker {
    state: Arc<TransportState>,
    configured_port: String,
    unit_index: i64,
    ser: Option<Port>,
    request_id: i64,
    pinned_path: Option<String>,
    discover_after: f64,
}

impl AceTransport {
    pub fn start(serial_port: &str, unit_index: i64) -> Arc<AceTransport> {
        let state = Arc::new(TransportState::default());
        let (tx, rx) = channel();
        let mut worker = ProWorker {
            state: state.clone(),
            configured_port: serial_port.to_string(),
            unit_index,
            ser: None,
            // Seeded from the PID so a restart can't match a stale reply
            request_id: (std::process::id() as i64 * 100) % 300000,
            pinned_path: None,
            discover_after: 0.0,
        };
        std::thread::Builder::new().name("AFC_ACE_serial".into()).spawn(move || worker.run(rx)).expect("AFC_ACE thread");
        Arc::new(AceTransport { state, tx: Mutex::new(Some(tx)) })
    }

    pub fn submit(&self, method: &str, params: Option<Map<String, Value>>) -> Arc<Pending> {
        let p = Pending::new();
        if let Some(tx) = self.tx.lock().unwrap().as_ref() {
            let _ = tx.send((method.to_string(), params, p.clone()));
        }
        p
    }

    pub fn stop(&self) {
        self.state.shutdown.store(true, Ordering::SeqCst);
    }
}

impl ProWorker {
    fn run(&mut self, rx: Receiver<Request>) {
        while !self.state.shutting_down() {
            let (method, params, pending) = match rx.recv_timeout(Duration::from_secs_f64(HEARTBEAT_S)) {
                Ok(r) => r,
                Err(RecvTimeoutError::Timeout) => {
                    // Nothing may escape here: a dead thread stops the heartbeat
                    match self.rpc("get_status", &None, None) {
                        Ok(mut res) => self.state.store_status(&mut res),
                        Err(e) => {
                            self.disconnect();
                            kaliru::logging::debug(&format!("AFC_ACE: heartbeat raised: {}", e));
                        }
                    }
                    continue;
                }
                Err(RecvTimeoutError::Disconnected) => break,
            };
            if pending.cancelled() {
                pending.update(RpcResult::err("request abandoned"));
                pending.finish();
                continue;
            }
            match self.rpc(&method, &params, Some(&pending)) {
                Ok(mut res) => {
                    if method == "get_status" {
                        self.state.store_status(&mut res);
                    }
                    pending.update(res);
                }
                Err(e) => {
                    self.disconnect();
                    pending.update(RpcResult::err(format!("rpc {} raised: {}", method, e)));
                }
            }
            pending.finish();
        }
        self.disconnect();
    }

    fn resolve_port(&mut self) -> Option<String> {
        if !self.configured_port.is_empty() && self.configured_port.to_lowercase() != "auto" {
            return Some(self.configured_port.clone());
        }
        if self.pinned_path.is_none() {
            if monotonic() < self.discover_after {
                return None;
            }
            self.pinned_path = self.discover_unit_path();
            match &self.pinned_path {
                None => {
                    self.discover_after = monotonic() + 30.0;
                    return None;
                }
                Some(p) => self.state.note(format!("ACE unit_index {} pinned to USB path {}", self.unit_index, p)),
            }
        }
        let pinned = self.pinned_path.clone()?;
        scan_ace_ports().into_iter().find(|(d, _)| *d == pinned).map(|(_, t)| t)
    }

    /// _discover_unit_path: ACE USB paths seen over a full watchdog cycle
    fn discover_unit_path(&mut self) -> Option<String> {
        let mut seen: Vec<String> = vec![];
        let deadline = monotonic() + 6.0;
        while monotonic() < deadline && !self.state.shutting_down() {
            for (d, _) in scan_ace_ports() {
                if !seen.contains(&d) {
                    seen.push(d);
                }
            }
            sleep(0.5);
        }
        seen.sort_by_key(|p| usb_path_key(p));
        if (self.unit_index as usize) < seen.len() {
            return Some(seen[self.unit_index as usize].clone());
        }
        self.state.set_error(Some(format!("only {} ACE unit(s) on USB, unit_index {} not found", seen.len(), self.unit_index)));
        None
    }

    fn connect(&mut self) -> bool {
        if self.ser.is_some() {
            return true;
        }
        let Some(port) = self.resolve_port() else {
            self.state.set_error(Some(format!("no ACE found on USB (unit_index {})", self.unit_index)));
            self.state.connected.store(false, Ordering::SeqCst);
            return false;
        };
        match Port::open(&port, 115200, 0.1, 2.0, None) {
            Ok(p) => {
                self.ser = Some(p);
                self.state.connected.store(true, Ordering::SeqCst);
                self.state.set_error(None);
                self.state.generation.fetch_add(1, Ordering::SeqCst);
                let pinned = self.pinned_path.as_ref().map(|p| format!(" ({})", p)).unwrap_or_default();
                kaliru::logging::debug(&format!("AFC_ACE: connected on {}{}", port, pinned));
                true
            }
            Err(e) => {
                self.state.set_error(Some(format!("open {} failed: {}", port, e)));
                self.state.connected.store(false, Ordering::SeqCst);
                self.ser = None;
                false
            }
        }
    }

    fn disconnect(&mut self) {
        self.ser = None;
        self.state.connected.store(false, Ordering::SeqCst);
    }

    fn next_id(&mut self) -> i64 {
        let current = self.request_id;
        self.request_id = (self.request_id + 1) % 300000;
        current
    }

    /// _rpc: Err is an exception escaping it (the caller disconnects)
    fn rpc(&mut self, method: &str, params: &Option<Map<String, Value>>, pending: Option<&Arc<Pending>>) -> Result<RpcResult, String> {
        // A call in the watchdog's re-enumeration gap fails on write before
        // the frame went out: reconnect once and resend. Read-side failures
        // are not retried: the command may already be running.
        for attempt in 0..2 {
            if attempt > 0 && pending.is_some_and(|p| p.cancelled()) {
                return Ok(RpcResult::err("request abandoned"));
            }
            if !self.connect() {
                if attempt == 0 {
                    sleep(0.7);
                    continue;
                }
                return Ok(RpcResult::err(self.state.last_error().unwrap_or_else(|| "not connected".into())));
            }
            let mut req = Map::new();
            req.insert("id".into(), json!(self.next_id()));
            req.insert("method".into(), json!(method));
            if let Some(p) = params.as_ref().filter(|p| !p.is_empty()) {
                req.insert("params".into(), Value::Object(p.clone()));
            }
            let frame = build_frame(serde_json::to_string(&Value::Object(req.clone())).unwrap_or_default().as_bytes());
            if let Err(e) = self.ser.as_ref().unwrap().write(&frame) {
                self.state.set_error(Some(format!("write failed: {}", e)));
                self.disconnect();
                if attempt == 0 {
                    sleep(0.7);
                    continue;
                }
                return Ok(RpcResult::err(self.state.last_error().unwrap_or_default()));
            }
            if let Some(p) = pending {
                p.set_sent();
            }
            let sent_at = monotonic();
            let id = req["id"].as_i64().unwrap_or(0);
            let timeout = if STOP_METHODS.contains(&method) { STOP_REPLY_S } else { RPC_REPLY_S };
            let mut response = match self.read_matching(id, timeout) {
                Ok(r) => r,
                Err(e) => {
                    // The ACE dropped off USB mid-read: close it so the next
                    // call reopens the new tty
                    self.disconnect();
                    RpcResult::err(format!("read failed: {}", e))
                }
            };
            if method != "get_status" {
                kaliru::logging::info(&format!(
                    "AFC_ACE: t={:.2} {} {} -> {} ({:.2}s)",
                    sent_at,
                    method,
                    params_str(params),
                    if response.ok { "ok".to_string() } else { response.error_str() },
                    monotonic() - sent_at
                ));
            }
            if !response.ok {
                // The frame went out: the command may be executing
                response.sent = true;
                self.state.set_error(Some(response.error_str()));
                // drain twice with a gap so the next read starts on a frame
                if let Some(ser) = &self.ser {
                    ser.reset_input_buffer();
                    sleep(0.15);
                    ser.reset_input_buffer();
                }
                return Ok(response);
            }
            let parsed = response.response.clone().unwrap_or(Value::Null);
            let code = parsed.get("code").cloned().unwrap_or(Value::Null);
            let code_ok = match &code {
                Value::Null => true,
                Value::Number(n) => n.as_f64() == Some(0.0),
                Value::String(s) => s == "0",
                Value::Bool(b) => !*b,
                _ => false,
            };
            if !code_ok {
                let msg = parsed.get("msg").map(py_str).unwrap_or_default();
                let msg = msg.trim();
                let msg = if msg.is_empty() { "unknown ACE error" } else { msg };
                let e = format!("{} failed with ACE code {}: {}", method, py_str(&code), msg);
                self.state.set_error(Some(e.clone()));
                return Ok(RpcResult { error: Some(e), response: Some(parsed), ..Default::default() });
            }
            self.state.set_error(None);
            return Ok(RpcResult::ok_with(parsed));
        }
        Ok(RpcResult::err(self.state.last_error().unwrap_or_else(|| "rpc failed".into())))
    }

    /// _read_frame: Ok(Err(msg)) is a bad frame, Err an exception
    fn read_frame(&self, deadline: f64) -> Result<Result<Value, String>, String> {
        let ser = self.ser.as_ref().ok_or("'NoneType' object has no attribute 'read'")?;
        let mut header: Vec<u8> = vec![];
        let mut found = false;
        while monotonic() < deadline {
            let b = read_exact(ser, 1, deadline)?;
            if b.is_empty() {
                continue;
            }
            header.push(b[0]);
            if header.len() > 2 {
                header.remove(0);
            }
            if header == [0xFF, 0xAA] {
                found = true;
                break;
            }
        }
        if !found {
            return Ok(Err("timeout waiting for frame header".into()));
        }
        let raw_len = read_exact(ser, 2, deadline)?;
        if raw_len.len() != 2 {
            return Ok(Err("timeout waiting for frame length".into()));
        }
        let payload_len = u16::from_le_bytes([raw_len[0], raw_len[1]]) as usize;
        if !(0 < payload_len && payload_len <= 4096) {
            return Ok(Err(format!("invalid frame payload length {}", payload_len)));
        }
        let payload = read_exact(ser, payload_len, deadline)?;
        if payload.len() != payload_len {
            return Ok(Err("timeout waiting for frame payload".into()));
        }
        let crc_raw = read_exact(ser, 2, deadline)?;
        if crc_raw.len() != 2 {
            return Ok(Err("timeout waiting for frame crc".into()));
        }
        if u16::from_le_bytes([crc_raw[0], crc_raw[1]]) != crc16_mcrf4xx(&payload) {
            return Ok(Err("crc mismatch".into()));
        }
        let mut terminated = false;
        while monotonic() < deadline {
            if read_exact(ser, 1, deadline)? == [0xFE] {
                terminated = true;
                break;
            }
        }
        if !terminated {
            return Ok(Err("timeout waiting for frame terminator".into()));
        }
        match std::str::from_utf8(&payload).ok().and_then(|s| serde_json::from_str::<Value>(s).ok()) {
            Some(v) => Ok(Ok(v)),
            None => Ok(Err("frame payload is not JSON".into())),
        }
    }

    fn read_matching(&self, request_id: i64, timeout_s: f64) -> Result<RpcResult, String> {
        let deadline = monotonic() + timeout_s;
        while monotonic() < deadline {
            match self.read_frame(deadline)? {
                Err(e) => return Ok(RpcResult::err(e)),
                Ok(parsed) => {
                    if parsed.is_object() && parsed.get("id").and_then(|v| v.as_f64()) == Some(request_id as f64) {
                        return Ok(RpcResult::ok_with(parsed));
                    }
                    // stale frame from a previous request: keep reading
                }
            }
        }
        Ok(RpcResult::err(format!("timeout waiting for rpc response id={}", request_id)))
    }
}

// ── ACE 2 Pro ───────────────────────────────────────────────────────
//
// Untested on hardware in python too (simulated unit only). Frame:
// FF AA | flags | seq lo | seq hi | cmd | len | protobuf | crc16 LE | FE

/// A protobuf field value: varints as int, everything else as bytes
#[derive(Clone, Debug)]
enum Pb {
    Int(u64),
    Bytes(Vec<u8>),
}

type PbFields = Vec<(u64, Vec<Pb>)>;

fn pb_varint(mut value: u64) -> Vec<u8> {
    let mut out = vec![];
    while value > 0x7F {
        out.push(((value & 0x7F) | 0x80) as u8);
        value >>= 7;
    }
    out.push(value as u8);
    out
}

/// _pb_msg: uint32/bool fields 1..n in order
fn pb_msg(values: &[i64]) -> Vec<u8> {
    let mut out = vec![];
    for (num, v) in values.iter().enumerate() {
        out.extend(pb_varint(((num + 1) as u64) << 3));
        out.extend(pb_varint(*v as u64));
    }
    out
}

fn pb_read_varint(data: &[u8], mut pos: usize) -> Result<(u64, usize), String> {
    let (mut value, mut shift) = (0u64, 0u32);
    loop {
        let byte = *data.get(pos).ok_or("index out of range")?;
        pos += 1;
        if shift < 64 {
            value |= ((byte & 0x7F) as u64) << shift;
        }
        if byte & 0x80 == 0 {
            return Ok((value, pos));
        }
        shift += 7;
    }
}

fn pb_decode(data: &[u8]) -> Result<PbFields, String> {
    let mut fields: PbFields = vec![];
    let mut pos = 0;
    while pos < data.len() {
        let (key, p) = pb_read_varint(data, pos)?;
        pos = p;
        let value = match key & 7 {
            0 => {
                let (v, p) = pb_read_varint(data, pos)?;
                pos = p;
                Pb::Int(v)
            }
            2 => {
                let (size, p) = pb_read_varint(data, pos)?;
                let end = (p + size as usize).min(data.len());
                pos = p + size as usize;
                Pb::Bytes(data[p.min(data.len())..end].to_vec())
            }
            w @ (1 | 5) => {
                let size = if w == 1 { 8 } else { 4 };
                let end = (pos + size).min(data.len());
                let v = Pb::Bytes(data[pos.min(data.len())..end].to_vec());
                pos += size;
                v
            }
            w => return Err(format!("protobuf wire type {}", w)),
        };
        match fields.iter_mut().find(|(n, _)| *n == key >> 3) {
            Some(f) => f.1.push(value),
            None => fields.push((key >> 3, vec![value])),
        }
    }
    Ok(fields)
}

fn pb_field(fields: &PbFields, num: u64) -> Option<&Pb> {
    fields.iter().find(|(n, _)| *n == num).and_then(|(_, v)| v.first())
}

fn pb_int(fields: &PbFields, num: u64) -> Result<u64, String> {
    match pb_field(fields, num) {
        None => Ok(0),
        Some(Pb::Int(v)) => Ok(*v),
        Some(Pb::Bytes(_)) => Err("unsupported operand type(s): bytes".into()),
    }
}

fn pb_bytes(fields: &PbFields, num: u64) -> Result<Vec<u8>, String> {
    match pb_field(fields, num) {
        None => Ok(vec![]),
        Some(Pb::Bytes(b)) => Ok(b.clone()),
        Some(Pb::Int(_)) => Err("object of type 'int' has no len()".into()),
    }
}

fn ace2_frame(flags: u8, seq: u16, cmd: u8, payload: &[u8]) -> Vec<u8> {
    let mut inner = vec![flags, (seq & 0xFF) as u8, (seq >> 8) as u8, cmd, payload.len() as u8];
    inner.extend_from_slice(payload);
    let mut out = vec![0xFF, 0xAA];
    out.extend_from_slice(&inner);
    out.extend_from_slice(&crc16_mcrf4xx(&inner).to_le_bytes());
    out.push(0xFE);
    out
}

fn param_int(params: &Option<Map<String, Value>>, key: &str) -> Result<i64, String> {
    params
        .as_ref()
        .and_then(|p| p.get(key))
        .and_then(|v| v.as_f64())
        .map(|f| f as i64)
        .ok_or_else(|| py_str_repr(key))
}

/// _ace2_request: an ACE Pro method + params as ACE 2 [(command, payload)]
fn ace2_request(method: &str, params: &Option<Map<String, Value>>) -> Result<Vec<(u8, Vec<u8>)>, String> {
    let p = |k| param_int(params, k);
    Ok(match method {
        "get_status" => vec![(ACE2_GET_STATUS, vec![])],
        m if STOP_METHODS.contains(&m) => vec![(ACE2_STOP, pb_msg(&[p("index")?]))],
        "feed_filament" | "unwind_filament" => {
            let mode = if method == "feed_filament" { 0 } else { 1 };
            vec![(ACE2_FEED, pb_msg(&[p("index")?, p("speed")?.min(ACE2_MAX_SPEED), p("length")?, mode]))]
        }
        "start_feed_assist" => vec![(ACE2_STOP, pb_msg(&[p("index")?])), (ACE2_FEED, pb_msg(&[p("index")?, 10, 0, 2]))],
        "drying" => vec![(ACE2_DRYING, pb_msg(&[0, 0])), (ACE2_DRYING, pb_msg(&[p("temp")?, p("duration")?, 0]))],
        "drying_stop" => vec![(ACE2_DRYING, pb_msg(&[0, 0]))],
        _ => return Err(format!("no ACE 2 equivalent for {}", method)),
    })
}

fn ace2_slot_motion(state: u64) -> Option<&'static str> {
    match state {
        1 => Some("feeding"),
        2 => Some("unwinding"),
        5 => Some("busy"),
        _ => None,
    }
}

/// _ace2_status: a GET_STATUS reply in the ACE Pro's get_status shape
fn ace2_status(fields: &PbFields, rfid: &HashMap<i64, Map<String, Value>>) -> Result<Value, String> {
    let mut slots = vec![];
    let mut moving = false;
    let raws: Vec<Pb> = fields.iter().find(|(n, _)| *n == 9).map(|(_, v)| v.clone()).unwrap_or_default();
    for (index, raw) in raws.iter().enumerate() {
        let Pb::Bytes(raw) = raw else { return Err("object of type 'int' has no len()".into()) };
        let sub = pb_decode(raw)?;
        let (state, filament) = (pb_int(&sub, 1)?, pb_int(&sub, 2)?);
        moving = moving || ace2_slot_motion(state).is_some();
        let mut slot = Map::new();
        slot.insert("index".into(), json!(index));
        slot.insert("status".into(), json!(if filament == 0 { "empty" } else { ace2_slot_motion(state).unwrap_or("ready") }));
        slot.insert("ace2_state".into(), json!(state));
        slot.insert("ace2_filament".into(), json!(filament));
        if let Some(r) = rfid.get(&(index as i64)) {
            for (k, v) in r {
                slot.insert(k.clone(), v.clone());
            }
        }
        slots.push(Value::Object(slot));
    }
    let dry = pb_decode(&pb_bytes(fields, 2)?)?;
    let drying = matches!(pb_int(&dry, 1)? & 0x0F, 1 | 2);
    let mut out = Map::new();
    out.insert("status".into(), json!(if moving { "busy" } else { "ready" }));
    out.insert("temp".into(), json!(pb_int(fields, 3)?));
    out.insert("humidity".into(), json!(pb_int(fields, 4)?));
    let mut ds = Map::new();
    ds.insert("status".into(), json!(if drying { "drying" } else { "stop" }));
    ds.insert("target_temp".into(), json!(pb_int(&dry, 2)?));
    ds.insert("duration".into(), json!(pb_int(&dry, 3)?));
    ds.insert("remain_time".into(), json!(pb_int(&dry, 4)?));
    out.insert("dryer_status".into(), Value::Object(ds));
    out.insert("feed_assist_count".into(), json!(pb_int(fields, 7)?));
    out.insert("cont_assist_time".into(), json!(pb_int(fields, 8)? as f64 / 1000.0));
    out.insert("slots".into(), Value::Array(slots));
    Ok(Value::Object(out))
}

/// _ace2_filament: a GET_FILAMENT_INFO reply as the slot's RFID keys
fn ace2_filament(fields: &PbFields) -> Result<Map<String, Value>, String> {
    let mut info = Map::new();
    if pb_int(fields, 12)? != 0 {
        return Ok(info);
    }
    info.insert("type".into(), json!(kaliru::pyfmt::py_strip(&String::from_utf8_lossy(&pb_bytes(fields, 4)?)).to_string()));
    if let Some((_, colors)) = fields.iter().find(|(n, _)| *n == 5) {
        if let Some(Pb::Bytes(c)) = colors.first() {
            let rgba = pb_int(&pb_decode(c)?, 1)?;
            info.insert("color".into(), json!([(rgba >> 24) & 0xFF, (rgba >> 16) & 0xFF, (rgba >> 8) & 0xFF]));
        }
    }
    Ok(info)
}

/// scan_ace2_ports: ttys of Anycubic's ACE 2 USB/RS-485 cable
fn scan_ace2_ports() -> Vec<String> {
    let mut devs: Vec<String> = list_dir("/sys/bus/usb/devices").into_iter().map(|n| format!("/sys/bus/usb/devices/{}", n)).collect();
    devs.sort_by_key(|d| usb_path_key(&format!("{}/", d)));
    let mut found = vec![];
    for dev in devs {
        let read = |n: &str| std::fs::read_to_string(format!("{}/{}", dev, n)).map(|s| s.trim().to_string());
        let (Ok(v), Ok(p)) = (read("idVendor"), read("idProduct")) else { continue };
        if (v.as_str(), p.as_str()) != ACE2_USB_ID {
            continue;
        }
        for intf in list_dir(&dev) {
            for t in list_dir(&format!("{}/{}", dev, intf)).into_iter().filter(|t| t.starts_with("tty")) {
                if t == "tty" {
                    found.extend(list_dir(&format!("{}/{}/tty", dev, intf)).into_iter().map(|n| format!("/dev/{}", n)));
                } else {
                    found.push(format!("/dev/{}", t));
                }
            }
        }
    }
    found
}

/// One ACE 2 unit on a bus (Ace2Transport's own state)
pub struct Ace2Unit {
    pub state: Arc<TransportState>,
    configured_port: String,
    unit_index: i64,
    inner: Mutex<Ace2UnitInner>,
    bus: Mutex<Option<Arc<Ace2Bus>>>,
}

#[derive(Default)]
struct Ace2UnitInner {
    device_id: Option<i64>,
    uid: Option<(u64, u64, u64)>,
    misses: i64,
    started: HashMap<i64, f64>,
    rfid: HashMap<i64, Map<String, Value>>,
}

type BusRequest = (Arc<Ace2Unit>, String, Option<Map<String, Value>>, Arc<Pending>);

pub struct Ace2Bus {
    port: String,
    units: Mutex<Vec<Arc<Ace2Unit>>>,
    tx: Sender<BusRequest>,
    shutdown: AtomicBool,
    discover_at: Mutex<f64>,
}

static ACE2_BUSES: Mutex<Vec<(String, Arc<Ace2Bus>)>> = Mutex::new(vec![]);

struct BusWorker {
    bus: Arc<Ace2Bus>,
    ser: Option<Port>,
    seq: u16,
}

impl Ace2Unit {
    pub fn start(serial_port: &str, unit_index: i64) -> Arc<Ace2Unit> {
        let unit = Arc::new(Ace2Unit {
            state: Arc::new(TransportState::default()),
            configured_port: serial_port.to_string(),
            unit_index,
            inner: Mutex::new(Ace2UnitInner::default()),
            bus: Mutex::new(None),
        });
        *unit.bus.lock().unwrap() = Some(Ace2Bus::attach(&unit));
        unit
    }

    pub fn submit(self: &Arc<Self>, method: &str, params: Option<Map<String, Value>>) -> Arc<Pending> {
        let p = Pending::new();
        if let Some(bus) = self.bus.lock().unwrap().as_ref() {
            let _ = bus.tx.send((self.clone(), method.to_string(), params, p.clone()));
        }
        p
    }

    pub fn stop(self: &Arc<Self>) {
        if let Some(bus) = self.bus.lock().unwrap().as_ref() {
            bus.detach(self);
        }
    }

    fn device_id(&self) -> Option<i64> {
        self.inner.lock().unwrap().device_id
    }

    fn fail(&self, error: &str) {
        self.state.connected.store(false, Ordering::SeqCst);
        self.inner.lock().unwrap().device_id = None;
        self.state.set_error(Some(error.to_string()));
    }

    fn status_time(&self) -> f64 {
        self.state.status.lock().unwrap().1
    }
}

impl Ace2Bus {
    fn attach(unit: &Arc<Ace2Unit>) -> Arc<Ace2Bus> {
        let mut buses = ACE2_BUSES.lock().unwrap();
        let existing = buses.iter().find(|(p, b)| *p == unit.configured_port && !b.shutdown.load(Ordering::SeqCst)).map(|(_, b)| b.clone());
        let bus = match existing {
            Some(b) => b,
            None => {
                let (tx, rx) = channel();
                let bus = Arc::new(Ace2Bus {
                    port: unit.configured_port.clone(),
                    units: Mutex::new(vec![]),
                    tx,
                    shutdown: AtomicBool::new(false),
                    discover_at: Mutex::new(0.0),
                });
                buses.retain(|(p, _)| *p != unit.configured_port);
                buses.push((unit.configured_port.clone(), bus.clone()));
                let mut worker = BusWorker { bus: bus.clone(), ser: None, seq: ((std::process::id() as u64 * 100) % 0x10000) as u16 };
                std::thread::Builder::new().name("AFC_ACE2_bus".into()).spawn(move || worker.run(rx)).expect("AFC_ACE2 thread");
                bus
            }
        };
        bus.units.lock().unwrap().push(unit.clone());
        *bus.discover_at.lock().unwrap() = 0.0;
        bus
    }

    fn detach(&self, unit: &Arc<Ace2Unit>) {
        let mut buses = ACE2_BUSES.lock().unwrap();
        let mut units = self.units.lock().unwrap();
        units.retain(|u| !Arc::ptr_eq(u, unit));
        if units.is_empty() {
            self.shutdown.store(true, Ordering::SeqCst);
            buses.retain(|(p, b)| !(*p == self.port && std::ptr::eq(Arc::as_ptr(b), self)));
        }
    }
}

impl BusWorker {
    fn units(&self) -> Vec<Arc<Ace2Unit>> {
        self.bus.units.lock().unwrap().clone()
    }

    fn run(&mut self, rx: Receiver<BusRequest>) {
        while !self.bus.shutdown.load(Ordering::SeqCst) {
            let item = match rx.recv_timeout(Duration::from_millis(100)) {
                Ok(i) => Some(i),
                Err(RecvTimeoutError::Timeout) => None,
                Err(RecvTimeoutError::Disconnected) => break,
            };
            // Nothing may escape: a dead thread stops every unit on the bus
            let res = (|| -> Result<(), String> {
                if self.connect() && monotonic() >= *self.bus.discover_at.lock().unwrap() {
                    self.discover()?;
                }
                if let Some((unit, method, params, pending)) = &item {
                    if pending.cancelled() {
                        pending.update(RpcResult::err("request abandoned"));
                    } else {
                        let mut r = self.call(unit, method, params, Some(pending))?;
                        if method == "get_status" {
                            unit.state.store_status(&mut r);
                        }
                        pending.update(r);
                    }
                }
                self.heartbeat()
            })();
            if let Err(e) = res {
                kaliru::logging::info(&format!("AFC_ACE: ACE 2 bus {} raised: {}", self.bus.port, e));
                self.disconnect();
                if let Some((_, _, _, pending)) = &item {
                    let mut r = pending.result.lock().unwrap();
                    if r.error.is_none() && !r.ok {
                        r.error = Some(format!("ACE 2 bus raised: {}", e));
                    }
                }
            }
            if let Some((_, _, _, pending)) = &item {
                pending.finish();
            }
        }
        self.disconnect();
    }

    /// Per unit, so one unit's busy load loop never starves another's status
    fn heartbeat(&mut self) -> Result<(), String> {
        for unit in self.units() {
            if unit.device_id().is_none() || monotonic() - unit.status_time() < HEARTBEAT_S {
                continue;
            }
            let mut res = self.call(&unit, "get_status", &None, None)?;
            if res.ok {
                // Tags before the status is published
                let mut status = res.result().cloned().unwrap_or(Value::Null);
                self.fetch_rfid(&unit, &status)?;
                let rfid = unit.inner.lock().unwrap().rfid.clone();
                if let Some(Value::Array(slots)) = status.get_mut("slots") {
                    for slot in slots.iter_mut() {
                        let idx = slot.get("index").and_then(|v| v.as_i64()).unwrap_or(-1);
                        if let (Some(r), Value::Object(s)) = (rfid.get(&idx), slot) {
                            for (k, v) in r {
                                s.insert(k.clone(), v.clone());
                            }
                        }
                    }
                }
                res.response = Some(json!({"code": 0, "result": status}));
            }
            unit.state.store_status(&mut res);
        }
        Ok(())
    }

    /// _fetch_rfid: read once per identified spool, forget when it empties
    fn fetch_rfid(&mut self, unit: &Arc<Ace2Unit>, status: &Value) -> Result<(), String> {
        let Some(Value::Array(slots)) = status.get("slots") else { return Ok(()) };
        for slot in slots {
            let index = slot.get("index").and_then(|v| v.as_i64()).unwrap_or(0);
            let filament = slot.get("ace2_filament").and_then(|v| v.as_u64()).unwrap_or(0);
            if filament == 0 {
                unit.inner.lock().unwrap().rfid.remove(&index);
            } else if filament == 2 && !unit.inner.lock().unwrap().rfid.contains_key(&index) {
                match self.exchange(Some(unit), ACE2_FILAMENT_INFO, &pb_msg(&[index]), RPC_REPLY_S)? {
                    None => continue,
                    Some(fields) => {
                        let info = ace2_filament(&fields).unwrap_or_default();
                        unit.inner.lock().unwrap().rfid.insert(index, info);
                    }
                }
            }
        }
        Ok(())
    }

    fn connect(&mut self) -> bool {
        if self.ser.is_some() {
            return true;
        }
        let mut port = self.bus.port.clone();
        if port.to_lowercase() == "auto" {
            let ports = scan_ace2_ports();
            match ports.first() {
                None => {
                    sleep(1.0);
                    return self.fail_all("no ACE 2 USB/RS-485 cable (1a86:55d3) found");
                }
                Some(p) => port = p.clone(),
            }
        }
        match Port::open(&port, ACE2_BAUD, 0.05, 1.0, Some(true)) {
            Ok(p) => self.ser = Some(p),
            Err(e) => {
                sleep(1.0);
                return self.fail_all(&format!("open {} failed: {}", port, e));
            }
        }
        *self.bus.discover_at.lock().unwrap() = 0.0;
        kaliru::logging::info(&format!("AFC_ACE: ACE 2 bus connected on {}", port));
        true
    }

    fn fail_all(&self, error: &str) -> bool {
        for unit in self.units() {
            unit.fail(error);
        }
        false
    }

    fn disconnect(&mut self) {
        self.ser = None;
        self.fail_all("ACE 2 bus disconnected");
    }

    fn discover_uids(&mut self, rounds: usize) -> Result<Vec<(u64, u64, u64)>, String> {
        let mut uids = vec![];
        for _ in 0..rounds {
            // A broadcast: every unit answers the same seq
            let seq = self.send(0, ACE2_DISCOVER, &[])?;
            let deadline = monotonic() + 0.3;
            while monotonic() < deadline {
                if let Some(f) = self.read_frame(deadline)? {
                    if f.cmd == ACE2_DISCOVER && f.seq == seq {
                        if let Ok(fields) = pb_decode(&f.payload) {
                            if let (Ok(a), Ok(b), Ok(c)) = (pb_int(&fields, 1), pb_int(&fields, 2), pb_int(&fields, 3)) {
                                if !uids.contains(&(a, b, c)) {
                                    uids.push((a, b, c));
                                }
                            }
                        }
                    }
                }
            }
        }
        Ok(uids)
    }

    fn discover(&mut self) -> Result<(), String> {
        let units = self.units();
        let uids = self.discover_uids(units.len() + 1)?;
        let mut ordered = uids.clone();
        ordered.sort();
        for unit in &units {
            if unit.device_id().is_some() {
                continue;
            }
            let known = unit.inner.lock().unwrap().uid;
            let uid = known.or_else(|| ordered.get(unit.unit_index as usize).copied());
            let Some(uid) = uid.filter(|u| uids.contains(u)) else {
                unit.state.connected.store(false, Ordering::SeqCst);
                unit.state.set_error(Some(format!("ACE 2 unit_index {} not answering ({} unit(s) on the bus did)", unit.unit_index, ordered.len())));
                continue;
            };
            let device_id = unit.unit_index + 1;
            let reply = self.exchange(None, ACE2_ASSIGN, &pb_msg(&[uid.0 as i64, uid.1 as i64, uid.2 as i64, device_id]), RPC_REPLY_S)?;
            if reply.as_ref().is_none_or(|f| pb_int(f, 1).unwrap_or(1) != 0) {
                unit.state.set_error(Some(format!("ACE 2 device id assignment failed for UID ({}, {}, {})", uid.0, uid.1, uid.2)));
                continue;
            }
            if known.is_none() {
                unit.state.note(format!("ACE 2 unit_index {} pinned to UID {:08X}-{:08X}-{:08X}", unit.unit_index, uid.0, uid.1, uid.2));
            }
            {
                let mut i = unit.inner.lock().unwrap();
                i.uid = Some(uid);
                i.device_id = Some(device_id);
                i.misses = 0;
            }
            unit.state.connected.store(true, Ordering::SeqCst);
            unit.state.set_error(None);
            unit.state.generation.fetch_add(1, Ordering::SeqCst);
            // Pin the insert preload to its cap (printer status 0); best effort
            self.exchange(Some(unit), ACE2_PRINTER_STATUS, &pb_msg(&[0]), RPC_REPLY_S)?;
        }
        let missing = self.units().iter().any(|u| u.device_id().is_none());
        *self.bus.discover_at.lock().unwrap() = monotonic() + if missing { 10.0 } else { f64::INFINITY };
        Ok(())
    }

    fn send(&mut self, flags: u8, cmd: u8, payload: &[u8]) -> Result<u16, String> {
        let seq = self.seq;
        self.seq = self.seq.wrapping_add(1);
        let ser = self.ser.as_ref().ok_or("'NoneType' object has no attribute 'reset_input_buffer'")?;
        ser.reset_input_buffer();
        ser.write(&ace2_frame(flags, seq, cmd, payload))?;
        Ok(seq)
    }

    /// _exchange: one frame, its decoded reply fields or None
    fn exchange(&mut self, unit: Option<&Arc<Ace2Unit>>, cmd: u8, payload: &[u8], timeout: f64) -> Result<Option<PbFields>, String> {
        let device_id = unit.and_then(|u| u.device_id()).unwrap_or(0);
        let seq = self.send(device_id as u8, cmd, payload)?;
        let deadline = monotonic() + timeout;
        while monotonic() < deadline {
            if let Some(f) = self.read_frame(deadline)? {
                if f.seq == seq && f.cmd == cmd && (device_id == 0 || [0, device_id].contains(&((f.flags & 0x7F) as i64))) {
                    return pb_decode(&f.payload).map(Some);
                }
            }
        }
        Ok(None)
    }

    /// _step: holds a STOP back until the move it stops has run long enough
    fn step(&mut self, unit: &Arc<Ace2Unit>, params: &Option<Map<String, Value>>, cmd: u8, payload: &[u8]) -> Result<Option<PbFields>, String> {
        let index = params.as_ref().and_then(|p| p.get("index")).and_then(|v| v.as_i64()).unwrap_or(-1);
        if cmd == ACE2_STOP {
            let started = unit.inner.lock().unwrap().started.get(&index).copied().unwrap_or(0.0);
            let wait = started + ACE2_STOP_AFTER_START_S - monotonic();
            if wait > 0.0 {
                sleep(wait);
            }
        }
        let fields = self.exchange(Some(unit), cmd, payload, if cmd == ACE2_STOP { STOP_REPLY_S } else { RPC_REPLY_S })?;
        if cmd == ACE2_FEED && fields.is_some() {
            unit.inner.lock().unwrap().started.insert(index, monotonic());
        }
        Ok(fields)
    }

    /// _call: one ACE Pro-style rpc to one unit
    fn call(&mut self, unit: &Arc<Ace2Unit>, method: &str, params: &Option<Map<String, Value>>, pending: Option<&Arc<Pending>>) -> Result<RpcResult, String> {
        if unit.device_id().is_none() || self.ser.is_none() {
            return Ok(RpcResult::err(unit.state.last_error().unwrap_or_else(|| "ACE 2 not discovered".into())));
        }
        let steps = match ace2_request(method, params) {
            Ok(s) => s,
            Err(e) => return Ok(RpcResult::err(format!("{}: {}", method, e))),
        };
        let sent_at = monotonic();
        if let Some(p) = pending {
            p.set_sent();
        }
        let cancelled = || pending.is_some_and(|p| p.cancelled());
        for (cmd, payload) in &steps[..steps.len() - 1] {
            // a garbled reply to a clearing step changes nothing
            let _ = self.step(unit, params, *cmd, payload);
        }
        if steps.len() > 1 && cancelled() {
            return Ok(RpcResult { sent: true, ..RpcResult::err("request abandoned") });
        }
        let (cmd, payload) = steps.last().cloned().unwrap();
        let mut fields;
        let mut status = None;
        loop {
            match self.step(unit, params, cmd, &payload) {
                Ok(f) => {
                    fields = f;
                    if let (Some(f), true) = (&fields, cmd == ACE2_GET_STATUS) {
                        let rfid = unit.inner.lock().unwrap().rfid.clone();
                        match ace2_status(f, &rfid) {
                            Ok(s) => status = Some(s),
                            Err(e) => {
                                fields = None;
                                unit.state.set_error(Some(format!("bad reply: {}", e)));
                            }
                        }
                    }
                }
                Err(e) if e.starts_with("protobuf wire type") || e == "index out of range" => {
                    fields = None;
                    unit.state.set_error(Some(format!("bad reply: {}", e)));
                }
                Err(e) => return Err(e),
            }
            let forbidden = fields.as_ref().is_some_and(|f| pb_int(f, 1).unwrap_or(0) == ACE2_FORBIDDEN);
            if fields.is_none() || !ACE2_MOTION_METHODS.contains(&method) || !forbidden || monotonic() - sent_at > ACE2_FORBIDDEN_WAIT_S || cancelled() {
                break;
            }
            // keep every unit's status fresh while this one waits
            self.heartbeat()?;
            sleep(0.3);
            if cancelled() {
                break;
            }
        }
        if method != "get_status" {
            kaliru::logging::info(&format!(
                "AFC_ACE: t={:.2} ACE2 {} {} -> {} ({:.2}s)",
                sent_at,
                method,
                params_str(params),
                match &fields {
                    None => "no reply".to_string(),
                    Some(f) => format!("code {}", pb_int(f, 1).unwrap_or(0)),
                },
                monotonic() - sent_at
            ));
        }
        let Some(fields) = fields else {
            let mut i = unit.inner.lock().unwrap();
            i.misses += 1;
            if i.misses >= 3 {
                // Power-cycled or unplugged: it lost its bus id
                i.device_id = None;
                unit.state.connected.store(false, Ordering::SeqCst);
                *self.bus.discover_at.lock().unwrap() = 0.0;
            }
            drop(i);
            let e = unit.state.last_error().unwrap_or_else(|| format!("timeout waiting for ACE 2 reply to {}", method));
            unit.state.set_error(Some(e.clone()));
            return Ok(RpcResult { sent: true, ..RpcResult::err(e) });
        };
        unit.inner.lock().unwrap().misses = 0;
        if cmd == ACE2_GET_STATUS {
            unit.state.set_error(None);
            return Ok(RpcResult::ok_with(json!({"code": 0, "result": status.unwrap_or(Value::Null)})));
        }
        let code = pb_int(&fields, 1).unwrap_or(0);
        if code != 0 && STOP_METHODS.contains(&method) {
            // what the caller needs is the slot idle: accept that
            let check = self.call(unit, "get_status", &None, None)?;
            let slots: Vec<Value> = if check.ok { check.result().and_then(|r| r.get("slots")).and_then(|s| s.as_array()).cloned().unwrap_or_default() } else { vec![] };
            let index = params.as_ref().and_then(|p| p.get("index")).and_then(|v| v.as_i64());
            if let Some(slot) = index.and_then(|i| slots.get(i as usize)) {
                let st = slot.get("ace2_state").and_then(|v| v.as_u64()).unwrap_or(0);
                if ace2_slot_motion(st).is_none() && st != 3 && st != 4 {
                    unit.state.set_error(None);
                    return Ok(RpcResult { assumed_success: true, ..RpcResult::ok_with(json!({"code": 0})) });
                }
            }
        }
        if code != 0 {
            let name = match code {
                1 => "PARAM_ERROR",
                2 => "FORBIDDEN (previous operation still running)",
                3 => "FAILED",
                _ => "unknown ACE 2 error",
            };
            let e = format!("{} failed with ACE code {}: {}", method, code, name);
            unit.state.set_error(Some(e.clone()));
            return Ok(RpcResult { error: Some(e), response: Some(json!({"code": code})), ..Default::default() });
        }
        unit.state.set_error(None);
        Ok(RpcResult::ok_with(json!({"code": 0})))
    }

    fn read_frame(&mut self, deadline: f64) -> Result<Option<Ace2Frame>, String> {
        let ser = self.ser.as_ref().ok_or("'NoneType' object has no attribute 'read'")?;
        let mut last: Vec<u8> = vec![];
        while monotonic() < deadline {
            let b = read_exact(ser, 1, deadline)?;
            let mut pair = last.clone();
            pair.extend_from_slice(&b);
            if pair != [0xFF, 0xAA] {
                last = b;
                continue;
            }
            let head = read_exact(ser, 5, deadline)?;
            if head.len() != 5 {
                return Ok(None);
            }
            let body = read_exact(ser, head[4] as usize + 3, deadline)?;
            if body.len() != head[4] as usize + 3 || body[body.len() - 1] != 0xFE {
                continue;
            }
            let payload = body[..body.len() - 3].to_vec();
            let mut crc_in = head.clone();
            crc_in.extend_from_slice(&payload);
            if u16::from_le_bytes([body[body.len() - 3], body[body.len() - 2]]) != crc16_mcrf4xx(&crc_in) {
                continue;
            }
            if head[0] & ACE2_REPLY != 0 {
                return Ok(Some(Ace2Frame { flags: head[0], seq: head[1] as u16 | (head[2] as u16) << 8, cmd: head[3], payload }));
            }
            last = vec![];
        }
        Ok(None)
    }
}

struct Ace2Frame {
    flags: u8,
    seq: u16,
    cmd: u8,
    payload: Vec<u8>,
}

/// count_ace_units: how many ACEs are attached now (`units: auto`)
pub fn count_ace_units(protocol: &str, serial_port: &str) -> usize {
    if protocol == "ace2" {
        let (tx, _rx) = channel();
        let bus = Arc::new(Ace2Bus { port: serial_port.to_string(), units: Mutex::new(vec![]), tx, shutdown: AtomicBool::new(false), discover_at: Mutex::new(0.0) });
        let mut w = BusWorker { bus, ser: None, seq: ((std::process::id() as u64 * 100) % 0x10000) as u16 };
        let n = if w.connect() { w.discover_uids(3).map(|u| u.len()).unwrap_or(0) } else { 0 };
        w.disconnect();
        return n;
    }
    // An idle ACE Pro drops off USB for ~0.5 s: look every 0.25 s
    let mut seen: Vec<String> = vec![];
    let deadline = monotonic() + 1.5;
    loop {
        for (d, _) in scan_ace_ports() {
            if !seen.contains(&d) {
                seen.push(d);
            }
        }
        if monotonic() >= deadline {
            return seen.len();
        }
        sleep(0.25);
    }
}

/// Either transport, as the unit uses it
pub enum Transport {
    Pro(Arc<AceTransport>),
    Two(Arc<Ace2Unit>),
}

impl Transport {
    pub fn state(&self) -> &Arc<TransportState> {
        match self {
            Transport::Pro(t) => &t.state,
            Transport::Two(u) => &u.state,
        }
    }

    pub fn submit(&self, method: &str, params: Option<Map<String, Value>>) -> Arc<Pending> {
        match self {
            Transport::Pro(t) => t.submit(method, params),
            Transport::Two(u) => u.submit(method, params),
        }
    }

    pub fn stop(&self) {
        match self {
            Transport::Pro(t) => t.stop(),
            Transport::Two(u) => u.stop(),
        }
    }

    pub fn is_ace2(&self) -> bool {
        matches!(self, Transport::Two(_))
    }
}
