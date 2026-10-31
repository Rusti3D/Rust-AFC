# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Anycubic ACE Pro (and ACE 2 Pro) unit driver. ACE Pro protocol knowledge from the CosmoACE
# addon (framed JSON-RPC over USB-CDC) and the printers-for-people/ACEResearch
# reverse-engineering effort.
#
# The ACE Pro is a closed 4-slot feeder speaking a proprietary framed JSON-RPC
# protocol over USB serial (115200). It has no Klipper MCU and no steppers AFC
# can drive, so this unit is stepperless (like OpenAMS): the ACE firmware
# moves filament (feed/unwind/feed-assist) and AFC orchestrates sensors,
# toolhead engagement, toolchanges and runout on top of it.
#
# Filament path: spool -> ACE slot feeder -> splitter/merger -> extruder
# gears -> nozzle, with ONE path sensor homing every feed, never both:
#   hub:        the frame runout sensor, with a blind afc_bowden_length push
#               from it to just short of the gears
#   tool_start: a sensor just before the gears (CANVAS toolhead)
# With neither, loads fall back to a blind dist_hub + bowden push.

from __future__ import annotations

import glob
import json
import logging
import os
import queue
import re
import subprocess
import threading
import time
import traceback
from configparser import Error as error
from typing import Any, Callable, Dict, Optional, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from gcode import GCodeCommand
    from extras.AFC_lane import AFCLane, SpeedMode
    from extras.AFC_extruder import AFCExtruder

# Unit type string: the config default, and the name AFC's lane-type gates key on
UNIT_TYPE = "ACE"

try: from extras.AFC_utils import ERROR_STR
except: raise error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_lane import AFCLaneState
except: raise error(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))

try: from extras.AFC_unit import afcUnit
except: raise error(ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc()))

try:
    from extras import AFC_lane as _afc_lane_mod
    # Register our unit type with AFC's lane-type gates from here rather than
    # editing AFC_lane.py, so this driver drops into a stock AFC install as a
    # single file. ONLY_LOAD_TYPES routes load_callback/handle_load_runout
    # (insert, runout, endless spool) to units whose "load switch" is a
    # firmware flag rather than a real pin; EXCLUDE_TYPES is derived from it at
    # AFC_lane import time, so both need the entry.
    for _type_list in (_afc_lane_mod.ONLY_LOAD_TYPES, _afc_lane_mod.EXCLUDE_TYPES):
        if UNIT_TYPE not in _type_list:
            _type_list.append(UNIT_TYPE)
except: raise error(ERROR_STR.format(import_lib="AFC_lane type lists",
                                     trace=traceback.format_exc()))

try: from extras.AFC_OpenAMS import _ams_box_logo, _ams_box_logo_error
except: raise error(ERROR_STR.format(import_lib="AFC_OpenAMS", trace=traceback.format_exc()))

try:
    import serial
except ImportError:
    serial = None

# ACE firmware states that mean "a motor is moving"
ACTIVE_STATES = frozenset(("busy", "feeding", "unwinding", "shifting", "preload"))
# The ACE sometimes eats the reply to a stop while its motor winds down (the
# stop still lands). Those replies are short-waited: idle is then confirmed
# from status, instead of stalling every sensor stop for the full read timeout.
STOP_METHODS = frozenset(("stop_feed_filament", "stop_unwind_filament", "stop_feed_assist"))
STOP_REPLY_S = 1.0
RPC_REPLY_S = 5.0

# _feed_until(extend=False) ran its length without reaching the sensor
NOT_REACHED = "sensor not reached"

# Hub -> toolhead length assumed when a lane has no hub object
DEFAULT_BOWDEN_MM = 500.0
# ACE firmware dryer limit
DRYER_MAX_C = 65
# The firmware can report idle for a moment after acking a motion command,
# before the motor spins up; status read inside this window proves nothing.
SPIN_UP_S = 0.6

# The ACE drops its USB link (~0.5s re-enumeration) if it sees no complete
# frame for ~3.5s, and pauses motion mid-feed if status isn't polled roughly
# every second. One heartbeat interval serves both.
HEARTBEAT_S = 0.8
# How long a lost reply waits for the ACE to come back on USB before its
# motion state counts as unknown: a re-enumeration is ~0.5s, each reconnect
# attempt ~0.7s.
RECONNECT_WAIT_S = 4.0
# Longest the driver waits for the ACE's own insert preload to finish (an
# ACE 2's runs up to 15 s)
PRELOAD_WAIT_S = 60.0


def _crc16_mcrf4xx(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        v = byte ^ (crc & 0xFF)
        v ^= (v & 0x0F) << 4
        crc = (((v << 8) | (crc >> 8)) ^ (v >> 4) ^ (v << 3)) & 0xFFFF
    return crc


def _build_frame(payload: bytes) -> bytes:
    frame = bytearray(b"\xFF\xAA")
    frame.extend(len(payload).to_bytes(2, "little"))
    frame.extend(payload)
    frame.extend(_crc16_mcrf4xx(payload).to_bytes(2, "little"))
    frame.append(0xFE)
    return bytes(frame)


def _read_exact(ser, count: int, deadline: float) -> bytes:
    out = bytearray()
    while len(out) < count and time.monotonic() < deadline:
        out.extend(ser.read(count - len(out)))
    return bytes(out)


def _usb_path_key(path: str) -> list:
    """Sort key that orders usb path 1-1.4.3 before 1-1.4.10."""
    return [int(p) if p.isdigit() else p for p in re.split(r"(\d+)", path)]


def scan_ace_ports() -> Dict[str, str]:
    """One snapshot of ACE units: {usb device path: tty device}.

    Found by USB product string, not device name: chained ACEs all report the
    same by-id name and ttyACM numbering swaps between units on every
    watchdog re-enumeration. Only the USB topology path is a stable identity.
    """
    found: Dict[str, str] = {}
    for dev in glob.glob("/sys/bus/usb/devices/*/"):
        try:
            with open(os.path.join(dev, "product"), encoding="utf-8", errors="replace") as f:
                product = f.read()
        except OSError:
            continue
        if "ACE" not in product.upper():
            continue
        for tty in glob.glob(os.path.join(dev, "*", "tty", "tty*")):
            found[dev.rstrip("/")] = "/dev/" + os.path.basename(tty)
    return found


class AceTransport(threading.Thread):
    """Owns the ACE serial port in a background thread.

    RPCs are queued from the klippy reactor thread and answered via a
    threading.Event the caller polls with reactor.pause. When idle, the
    thread sends get_status every HEARTBEAT_S: this feeds the ACE's comms
    watchdog, keeps motors alive during long feeds, and refreshes the status
    cache the unit reads for lane sync and motion waits.
    """

    def __init__(self, serial_port: str, unit_index: int) -> None:
        super().__init__(name="AFC_ACE_serial", daemon=True)
        self.configured_port = serial_port
        self.unit_index = unit_index
        self.baud = 115200
        self._ser = None
        self._queue: "queue.Queue[tuple]" = queue.Queue()
        # Messages for the reactor thread to log; see note()
        self._notes: "queue.Queue[str]" = queue.Queue()
        self._status_lock = threading.Lock()
        self._status: Optional[Dict[str, Any]] = None
        self._status_time = 0.0
        self._last_state = None
        self._assist_pumping = False
        # slot -> when status last showed it in the ACE's own insert preload
        self._preload_at: Dict[int, float] = {}
        self._shutdown = threading.Event()
        # Seed request ids from the PID so a klippy restart can't match a
        # stale reply frame from the previous process.
        self._request_id = (os.getpid() * 100) % 300000
        self._pinned_path: Optional[str] = None
        # A failed discovery scan takes 6s; don't repeat it on every RPC and
        # heartbeat while the unit is absent
        self._discover_after = 0.0
        self.last_error: Optional[str] = None
        self.connected = False
        # Bumped on every (re)connect. The ACE forgets feed assist when its
        # USB watchdog resets it, so the unit re-arms assist when this moves.
        self.generation = 0

    # ── reactor-side API ────────────────────────────────────────────

    def submit(self, method: str, params: Optional[dict] = None):
        """Queue an RPC; returns (result-dict, done-event, cancelled-event).
        Result is filled in place before done is set. Setting cancelled before
        the worker picks the request up abandons it — the command is never
        sent (a caller that timed out must not leave a motion command armed
        in the queue to fire later). A request taken before the cancel is
        dropped too if its frame has not gone out yet (it may be waiting on
        a reconnect). The worker sets result["sent"] once the frame is
        written, so a caller that cancels and then finds no "sent" knows the
        command never reached the wire."""
        result: Dict[str, Any] = {}
        done = threading.Event()
        cancelled = threading.Event()
        self._queue.put((method, params, result, done, cancelled))
        return result, done, cancelled

    def cached_status(self) -> Tuple[Optional[Dict[str, Any]], float]:
        with self._status_lock:
            return self._status, self._status_time

    def preloaded_since(self, slot: int, since: float) -> bool:
        """Whether status showed the slot in the ACE's own insert preload
        at or after `since` (time.monotonic)."""
        with self._status_lock:
            return self._preload_at.get(slot, -1.0) >= since

    def drain_notes(self) -> list:
        """Take the messages this thread wanted logged (reactor side)."""
        out = []
        while True:
            try:
                out.append(self._notes.get_nowait())
            except queue.Empty:
                return out

    def stop(self) -> None:
        self._shutdown.set()

    # ── worker-side logging ─────────────────────────────────────────

    def note(self, message: str) -> None:
        """Queue a message for the reactor thread to log.

        Every AFC logger method can end in send_callback(), which walks
        gcode.output_callbacks and writes client sockets — klippy work that
        belongs to the reactor thread (debug() does too when AFC debug is
        on). From this thread, log only through note() or python's logging
        module, which is thread-safe.
        """
        self._notes.put(message)

    # ── worker thread ───────────────────────────────────────────────

    def run(self) -> None:
        while not self._shutdown.is_set():
            try:
                method, params, result, done, cancelled = self._queue.get(timeout=HEARTBEAT_S)
            except queue.Empty:
                # Nothing may escape here: a dead thread stops the heartbeat
                # and every later RPC just times out.
                try:
                    self._store_status(self._rpc("get_status"))
                except Exception as exc:
                    self._disconnect()
                    logging.debug(f"AFC_ACE: heartbeat raised: {exc}")
                continue
            if cancelled.is_set():
                result.update({"ok": False, "error": "request abandoned"})
                done.set()
                continue
            try:
                res = self._rpc(method, params, result, cancelled)
                if method == "get_status":
                    self._store_status(res)
                result.update(res)
            except Exception as exc:
                # result["sent"] is already set if the frame went out
                self._disconnect()
                result.update({"ok": False, "error": f"rpc {method} raised: {exc}"})
            finally:
                done.set()
        self._disconnect()

    def _store_status(self, res: Dict[str, Any]) -> None:
        """Every successful get_status refreshes the cache, heartbeat or not:
        while the reactor is polling status itself the heartbeat never runs."""
        if res.get("ok"):
            payload = res.get("response", {}).get("result")
            if isinstance(payload, dict):
                # The ACE Pro keeps a slot's last tag after the spool leaves;
                # only rfid 2 (identified, as SnapAce/U1-Ace read it) is live.
                # ACE 2 slots carry no "rfid" key and are left alone.
                now = time.monotonic()
                for index, slot in enumerate(payload.get("slots") or []):
                    if isinstance(slot, dict) and slot.get("rfid", 2) != 2:
                        slot.update(sku="", type="", color=[0, 0, 0], colors=[[0, 0, 0, 0]])
                    if isinstance(slot, dict) and slot.get("status") == "preload":
                        self._preload_at[index] = now
                with self._status_lock:
                    self._status = payload
                    self._status_time = now
                try:
                    self._log_state_change(payload)
                except (TypeError, ValueError, AttributeError) as exc:
                    # A malformed field is a logging problem, not a dead link:
                    # escaping here would close the port
                    logging.debug(f"AFC_ACE: state log skipped: {exc}")

    def _log_state_change(self, status: Dict[str, Any]) -> None:
        """Log the ACE's motion state whenever it changes, and the start of
        each run of continuous feed-assist pumping: the firmware decides how
        much assist pushes, so this is the only record of what it did."""
        slots = status.get("slots") if isinstance(status.get("slots"), list) else []
        state = (status.get("status"), status.get("action"),
                 tuple(s.get("status") for s in slots if isinstance(s, dict)))
        if state != self._last_state:
            self._last_state = state
            logging.info(f"AFC_ACE: t={time.monotonic():.2f} state {state[0]}/{state[1]} "
                         f"slots {list(state[2])}")
        pumping = float(status.get("cont_assist_time") or 0.0) >= 2.0
        if pumping != self._assist_pumping:
            self._assist_pumping = pumping
            logging.info(f"AFC_ACE: t={time.monotonic():.2f} assist "
                         f"{'pumping' if pumping else 'idle'} "
                         f"(cont_assist_time={status.get('cont_assist_time')}, "
                         f"feed_assist_count={status.get('feed_assist_count')})")

    def _resolve_port(self) -> Optional[str]:
        if self.configured_port and self.configured_port.lower() != "auto":
            return self.configured_port
        # Once a USB path is pinned, only ever return that unit's current tty
        # (the tty name changes on every watchdog re-enumeration; the unit
        # must not silently become a different ACE when a sibling is absent).
        if self._pinned_path is None:
            if time.monotonic() < self._discover_after:
                return None
            self._pinned_path = self._discover_unit_path()
            if self._pinned_path is None:
                self._discover_after = time.monotonic() + 30.0
                return None
            self.note(
                f"ACE unit_index {self.unit_index} pinned to USB path {self._pinned_path}")
        return scan_ace_ports().get(self._pinned_path)

    def _discover_unit_path(self) -> Optional[str]:
        """Accumulate ACE USB paths over a full watchdog cycle: an idle ACE
        drops USB every ~3.5s and is invisible ~0.5s, so a single snapshot
        can miss a unit and mis-index the survivors."""
        seen: set = set()
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and not self._shutdown.is_set():
            seen.update(scan_ace_ports().keys())
            time.sleep(0.5)
        ordered = sorted(seen, key=_usb_path_key)
        if self.unit_index < len(ordered):
            return ordered[self.unit_index]
        self.last_error = (f"only {len(ordered)} ACE unit(s) on USB, "
                           f"unit_index {self.unit_index} not found")
        return None

    def _connect(self) -> bool:
        if self._ser is not None and self._ser.is_open:
            return True
        if serial is None:
            self.last_error = "pyserial not available"
            return False
        port = self._resolve_port()
        if port is None:
            self.last_error = f"no ACE found on USB (unit_index {self.unit_index})"
            self.connected = False
            return False
        try:
            self._ser = serial.Serial(port, self.baud, timeout=0.1, write_timeout=2.0)
            self.connected = True
            self.last_error = None
            self.generation += 1
            logging.debug(f"AFC_ACE: connected on {port}"
                          + (f" ({self._pinned_path})" if self._pinned_path else ""))
            return True
        except Exception as exc:
            self.last_error = f"open {port} failed: {exc}"
            self.connected = False
            self._ser = None
            return False

    def _disconnect(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self.connected = False

    def _next_id(self) -> int:
        current = self._request_id
        self._request_id = (self._request_id + 1) % 300000
        return current

    def _rpc(self, method: str, params: Optional[dict] = None,
             result: Optional[Dict[str, Any]] = None,
             cancelled: Optional[threading.Event] = None) -> Dict[str, Any]:
        # A call landing in the watchdog's re-enumeration gap fails on write
        # before the frame went through, so reconnect once and resend.
        # Read-side failures are NOT retried: the command may already be
        # executing and a resend would double the motion.
        for attempt in range(2):
            if attempt and cancelled is not None and cancelled.is_set():
                # The caller gave up during the reconnect wait: a late
                # motion command must not fire after it reported failure
                return {"ok": False, "error": "request abandoned"}
            if not self._connect():
                if attempt == 0:
                    time.sleep(0.7)
                    continue
                return {"ok": False, "error": self.last_error or "not connected"}
            req: Dict[str, Any] = {"id": self._next_id(), "method": method}
            if params:
                req["params"] = params
            frame = _build_frame(json.dumps(req, separators=(",", ":")).encode())
            try:
                self._ser.write(frame)
                self._ser.flush()
            except Exception as exc:
                self.last_error = f"write failed: {exc}"
                self._disconnect()
                if attempt == 0:
                    time.sleep(0.7)
                    continue
                return {"ok": False, "error": self.last_error}
            if result is not None:
                result["sent"] = True
            sent_at = time.monotonic()
            try:
                response = self._read_matching(
                    req["id"], timeout_s=STOP_REPLY_S if method in STOP_METHODS else RPC_REPLY_S)
            except Exception as exc:
                # The ACE dropped off USB mid-read (pyserial raises on a dead
                # fd). Close it so the next call reopens the new tty; holding
                # the deleted one leaves the unit unreachable for good.
                self._disconnect()
                response = {"ok": False, "error": f"read failed: {exc}"}
            if method != "get_status":
                # Every command, with its reply time, so a load or unload can
                # be replayed step by step from klippy.log
                logging.info(f"AFC_ACE: t={sent_at:.2f} {method} {params or ''} -> "
                             f"{'ok' if response.get('ok') else response.get('error')} "
                             f"({time.monotonic() - sent_at:.2f}s)")
            if not response.get("ok"):
                # The frame went out: the command may be executing
                response["sent"] = True
                self.last_error = str(response.get("error"))
                # A bad frame usually means we are mid-stream (partial frame
                # tail still arriving). Drain twice with a gap so the next
                # read starts on a frame boundary.
                try:
                    self._ser.reset_input_buffer()
                    time.sleep(0.15)
                    self._ser.reset_input_buffer()
                except Exception:
                    pass
                return response
            parsed = response["response"]
            code = parsed.get("code")
            if code not in (None, 0, "0"):
                msg = str(parsed.get("msg", "")).strip() or "unknown ACE error"
                self.last_error = f"{method} failed with ACE code {code}: {msg}"
                return {"ok": False, "error": self.last_error, "response": parsed}
            self.last_error = None
            return {"ok": True, "response": parsed}
        return {"ok": False, "error": self.last_error or "rpc failed"}

    def _read_frame(self, deadline: float) -> Dict[str, Any]:
        header = bytearray()
        while time.monotonic() < deadline:
            b = _read_exact(self._ser, 1, deadline)
            if not b:
                continue
            header.append(b[0])
            header = header[-2:]
            if bytes(header) == b"\xFF\xAA":
                break
        else:
            return {"ok": False, "error": "timeout waiting for frame header"}
        raw_len = _read_exact(self._ser, 2, deadline)
        if len(raw_len) != 2:
            return {"ok": False, "error": "timeout waiting for frame length"}
        payload_len = int.from_bytes(raw_len, "little")
        if not 0 < payload_len <= 4096:
            return {"ok": False, "error": f"invalid frame payload length {payload_len}"}
        payload = _read_exact(self._ser, payload_len, deadline)
        if len(payload) != payload_len:
            return {"ok": False, "error": "timeout waiting for frame payload"}
        crc_raw = _read_exact(self._ser, 2, deadline)
        if len(crc_raw) != 2:
            return {"ok": False, "error": "timeout waiting for frame crc"}
        if int.from_bytes(crc_raw, "little") != _crc16_mcrf4xx(payload):
            return {"ok": False, "error": "crc mismatch"}
        while time.monotonic() < deadline:
            tail = _read_exact(self._ser, 1, deadline)
            if tail == b"\xFE":
                break
        else:
            return {"ok": False, "error": "timeout waiting for frame terminator"}
        try:
            return {"ok": True, "payload": json.loads(payload.decode("utf-8"))}
        except Exception:
            return {"ok": False, "error": "frame payload is not JSON"}

    def _read_matching(self, request_id: int, timeout_s: float) -> Dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = self._read_frame(deadline)
            if not frame.get("ok"):
                return frame
            parsed = frame.get("payload")
            if isinstance(parsed, dict) and parsed.get("id") == request_id:
                return {"ok": True, "response": parsed}
            # stale frame from a previous request — keep reading
        return {"ok": False, "error": f"timeout waiting for rpc response id={request_id}"}


# ── ACE 2 Pro ───────────────────────────────────────────────────────
#
# UNTESTED ON HARDWARE. Checked only against a simulated unit
# (tools/test_ace2.py). Sources and their disagreements: docs/ACE2_PROTOCOL.md.
#
# Frame layout, command codes, protobuf field numbers and the bus discovery
# / id assignment are ported from ACEPRO (github.com/Kobra-S1/ACEPRO,
# extras/ace/protocol_ace2.py and manager.py, GPL-3.0; read at the
# WojciechBednarski/ACEPRO fork, 547d57c), cross-checked against hakimio's
# ACE 2 firmware analysis, Simon-CR/ace2-pro-firmware-research,
# decay71/multiACE (GPL-3.0) and tobecwb/ace2k.
#
# The ACE 2 talks protobuf over RS-485 at 230400 baud, through Anycubic's
# USB conversion cable (a CH343). Several units can share that one bus, so
# one Ace2Bus thread owns the port and every ACE 2 unit gets an
# Ace2Transport: the same API as AceTransport, with requests and replies
# translated to and from the ACE Pro's JSON shapes, so the unit code above
# the transport does not know which protocol it is driving.
#
# Frame: FF AA | flags | seq lo | seq hi | cmd | len | protobuf | crc16 LE | FE
# flags = bus device id (0 = broadcast), | 0x80 on replies. The CRC is the
# ACE Pro's (MCRF4XX) over flags..payload.

ACE2_BAUD = 230400
ACE2_USB_ID = ("1a86", "55d3")
ACE2_REPLY = 0x80
ACE2_DISCOVER, ACE2_ASSIGN, ACE2_GET_STATUS = 0, 1, 6
ACE2_FEED, ACE2_STOP, ACE2_DRYING, ACE2_FILAMENT_INFO = 8, 9, 11, 13
ACE2_PRINTER_STATUS = 20
ACE2_CODES = {1: "PARAM_ERROR", 2: "FORBIDDEN (previous operation still running)",
              3: "FAILED"}
# Slot motion states, as the ACE Pro status words the unit code keys on.
# Feed assist (3, 4) is not motion the driver waits on; error states
# (129+) are idle and accept the next command.
ACE2_SLOT_MOTION = {1: "feeding", 2: "unwinding", 5: "busy"}
ACE2_FORBIDDEN = 2
# A motion command answered FORBIDDEN did not run (the previous operation
# is still finishing), so it is resent until this runs out. Simon-CR saw
# that window last 20-30 s after stopping feed assist, which every unload
# does just before its retract.
ACE2_FORBIDDEN_WAIT_S = 30.0
ACE2_MOTION_METHODS = frozenset(("feed_filament", "unwind_filament", "start_feed_assist"))
ACE2_ASSIST_ERROR = 131
ACE2_MAX_SPEED = 100    # firmware PARAM_ERROR above this (Simon-CR 8.1)
# The firmware drops a STOP that lands within ~200 ms of the move it stops
# starting, and the move then runs its full length (Simon-CR docs/05)
ACE2_STOP_AFTER_START_S = 0.25
# How far an ACE 2 feeds an inserted spool by itself, at printer status 0
# (Simon-CR docs/08). A fresh lane's tip is anywhere up to here.
ACE2_PRELOAD_MAX_MM = 750.0


def _pb_varint(value: int) -> bytes:
    out = bytearray()
    while value > 0x7F:
        out.append((value & 0x7F) | 0x80)
        value >>= 7
    out.append(value)
    return bytes(out)


def _pb_msg(*values) -> bytes:
    """Every request the driver sends is uint32/bool fields 1..n in order."""
    return b"".join(_pb_varint(num << 3) + _pb_varint(int(v))
                    for num, v in enumerate(values, 1))


def _pb_read_varint(data: bytes, pos: int) -> Tuple[int, int]:
    value = shift = 0
    while True:
        byte = data[pos]  # IndexError on a truncated varint
        pos += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, pos
        shift += 7


def _pb_decode(data: bytes) -> Dict[int, list]:
    """{field number: [values]}; varints as int, everything else as bytes."""
    fields: Dict[int, list] = {}
    pos = 0
    while pos < len(data):
        key, pos = _pb_read_varint(data, pos)
        wire = key & 7
        if wire == 0:
            value, pos = _pb_read_varint(data, pos)
        elif wire == 2:
            size, pos = _pb_read_varint(data, pos)
            value, pos = data[pos:pos + size], pos + size
        elif wire in (1, 5):
            size = 8 if wire == 1 else 4
            value, pos = data[pos:pos + size], pos + size
        else:
            raise ValueError(f"protobuf wire type {wire}")
        fields.setdefault(key >> 3, []).append(value)
    return fields


def _pb_get(fields: Dict[int, list], num: int, default: Any = 0) -> Any:
    return fields.get(num, [default])[0]


def _ace2_frame(flags: int, seq: int, cmd: int, payload: bytes) -> bytes:
    inner = bytes((flags, seq & 0xFF, (seq >> 8) & 0xFF, cmd, len(payload))) + payload
    return b"\xFF\xAA" + inner + _crc16_mcrf4xx(inner).to_bytes(2, "little") + b"\xFE"


def _ace2_request(method: str, params: Optional[dict]) -> list:
    """ACE Pro method + params -> ACE 2 [(command, payload)]. Only the last
    frame's reply counts; the ones before it clear the way."""
    p = params or {}
    if method == "get_status":
        return [(ACE2_GET_STATUS, b"")]
    if method in STOP_METHODS:
        # One stop for every motion, assist included (Simon-CR 8.3)
        return [(ACE2_STOP, _pb_msg(p["index"]))]
    if method in ("feed_filament", "unwind_filament"):
        mode = 0 if method == "feed_filament" else 1
        return [(ACE2_FEED, _pb_msg(p["index"], min(int(p["speed"]), ACE2_MAX_SPEED),
                                    p["length"], mode))]
    if method == "start_feed_assist":
        # A slot reading "ready" may still hold a live assist, which refuses
        # a new one: stop first (Simon-CR docs/05). The firmware runs assist
        # at a fixed 50 mm/s whatever speed is sent; 10 is what others send.
        return [(ACE2_STOP, _pb_msg(p["index"])),
                (ACE2_FEED, _pb_msg(p["index"], 10, 0, 2))]
    if method == "drying":
        # A DRYING sent mid-cycle keeps the old cycle's start time, so it can
        # end at once (Simon-CR docs/07): stop first. Duration in minutes;
        # auto_roll off, the factory default.
        return [(ACE2_DRYING, _pb_msg(0, 0)),
                (ACE2_DRYING, _pb_msg(p["temp"], p["duration"], 0))]
    if method == "drying_stop":
        return [(ACE2_DRYING, _pb_msg(0, 0))]
    raise ValueError(f"no ACE 2 equivalent for {method}")


def _ace2_status(fields: Dict[int, list], rfid: Dict[int, dict]) -> Dict[str, Any]:
    """GET_STATUS reply -> the ACE Pro's get_status result shape."""
    slots = []
    moving = False
    for index, raw in enumerate(fields.get(9, [])):
        sub = _pb_decode(raw)
        state, filament = _pb_get(sub, 1), _pb_get(sub, 2)
        moving = moving or state in ACE2_SLOT_MOTION
        # filament: 0 empty, 1 present, 2 RFID identified, 3 identifying
        slot = {"index": index,
                "status": "empty" if filament == 0 else ACE2_SLOT_MOTION.get(state, "ready"),
                "ace2_state": state, "ace2_filament": filament}
        slot.update(rfid.get(index, {}))
        slots.append(slot)
    dry = _pb_decode(_pb_get(fields, 2, b""))
    # Low nibble only: custom firmware puts flags in the high bits (multiACE)
    drying = (_pb_get(dry, 1) & 0x0F) in (1, 2)
    return {
        "status": "busy" if moving else "ready",
        "temp": _pb_get(fields, 3),
        "humidity": _pb_get(fields, 4),
        # Seconds, as the ACE Pro's remain_time (DRYING itself takes minutes)
        "dryer_status": {"status": "drying" if drying else "stop",
                         "target_temp": _pb_get(dry, 2),
                         "duration": _pb_get(dry, 3),
                         "remain_time": _pb_get(dry, 4)},
        "feed_assist_count": _pb_get(fields, 7),
        "cont_assist_time": _pb_get(fields, 8) / 1000.0,  # ms on the ACE 2
        "slots": slots,
    }


def _ace2_filament(fields: Dict[int, list]) -> Dict[str, Any]:
    """GET_FILAMENT_INFO (cmd 13) reply -> the slot's RFID keys."""
    if _pb_get(fields, 12):
        return {}
    info: Dict[str, Any] = {"type": _pb_get(fields, 4, b"").decode(errors="replace").strip()}
    colors = fields.get(5)
    if colors:
        rgba = _pb_get(_pb_decode(colors[0]), 1)
        info["color"] = [(rgba >> 24) & 0xFF, (rgba >> 16) & 0xFF, (rgba >> 8) & 0xFF]
    return info


def scan_ace2_ports() -> list:
    """ttys of Anycubic's ACE 2 USB/RS-485 cable, in USB topology order."""
    found = []
    for dev in sorted(glob.glob("/sys/bus/usb/devices/*/"), key=_usb_path_key):
        try:
            ids = tuple(open(os.path.join(dev, name)).read().strip()
                        for name in ("idVendor", "idProduct"))
        except OSError:
            continue
        if ids != ACE2_USB_ID:
            continue
        # cdc_acm: <intf>/tty/ttyACM0; vendor ch343 driver: <intf>/ttyCH343USB0
        for path in glob.glob(os.path.join(dev, "*", "tty*")):
            names = os.listdir(path) if os.path.basename(path) == "tty" else [os.path.basename(path)]
            found.extend("/dev/" + n for n in names)
    return found


_ACE2_BUSES: Dict[str, "Ace2Bus"] = {}
_ACE2_BUSES_LOCK = threading.Lock()


class Ace2Bus(threading.Thread):
    """Owns one ACE 2 RS-485 bus for every ACE 2 unit configured on it.

    On connect it broadcasts DISCOVER, sorts the UIDs that answer, and hands
    the unit at unit_index N the bus device id N+1 (ASSIGN). Every later
    frame is addressed by that id. A unit that stops answering is
    rediscovered: a power-cycled ACE 2 forgets its id.
    """

    def __init__(self, port: str) -> None:
        super().__init__(name="AFC_ACE2_bus", daemon=True)
        self.port = port
        self.units: list = []
        self._queue: "queue.Queue[tuple]" = queue.Queue()
        self._ser = None
        self._seq = (os.getpid() * 100) % 0x10000
        self._discover_at = 0.0
        self._shutdown = threading.Event()

    @classmethod
    def attach(cls, unit: "Ace2Transport") -> "Ace2Bus":
        with _ACE2_BUSES_LOCK:
            bus = _ACE2_BUSES.get(unit.configured_port)
            if bus is None or bus._shutdown.is_set():
                bus = _ACE2_BUSES[unit.configured_port] = cls(unit.configured_port)
                bus.start()
            bus.units.append(unit)
            bus._discover_at = 0.0
            return bus

    def detach(self, unit: "Ace2Transport") -> None:
        with _ACE2_BUSES_LOCK:
            if unit in self.units:
                self.units.remove(unit)
            if not self.units:
                self._shutdown.set()
                if _ACE2_BUSES.get(self.port) is self:
                    del _ACE2_BUSES[self.port]

    def submit(self, unit: "Ace2Transport", method: str, params: Optional[dict]):
        result: Dict[str, Any] = {}
        done, cancelled = threading.Event(), threading.Event()
        self._queue.put((unit, method, params, result, done, cancelled))
        return result, done, cancelled

    # ── worker thread ───────────────────────────────────────────────

    def run(self) -> None:
        while not self._shutdown.is_set():
            try:
                item = self._queue.get(timeout=0.1)
            except queue.Empty:
                item = None
            # Nothing may escape: a dead thread stops every unit on the bus
            try:
                if self._connect() and time.monotonic() >= self._discover_at:
                    self._discover()
                if item is not None:
                    unit, method, params, result, done, cancelled = item
                    if cancelled.is_set():
                        result.update({"ok": False, "error": "request abandoned"})
                    else:
                        res = self._call(unit, method, params, result, cancelled)
                        if method == "get_status":
                            unit._store_status(res)
                        result.update(res)
                self._heartbeat()
            except Exception as exc:
                logging.info(f"AFC_ACE: ACE 2 bus {self.port} raised: {exc}")
                self._disconnect()
                if item is not None:
                    item[3].setdefault("ok", False)
                    item[3].setdefault("error", f"ACE 2 bus raised: {exc}")
            finally:
                if item is not None:
                    item[4].set()
        self._disconnect()

    def _heartbeat(self) -> None:
        """Per unit, so one unit's busy load loop never starves another's
        status (each ACE Pro has its own thread for the same reason)."""
        for unit in list(self.units):
            if unit.device_id is None or time.monotonic() - unit._status_time < HEARTBEAT_S:
                continue
            res = self._call(unit, "get_status", None)
            if res.get("ok"):
                # Tags before the status is published: a status without them
                # would read as the tag changing when they arrive, and the
                # lane would overwrite the user's restored color/material
                status = res["response"]["result"]
                self._fetch_rfid(unit, status)
                for slot in status["slots"]:
                    slot.update(unit._rfid.get(slot["index"], {}))
            unit._store_status(res)

    def _fetch_rfid(self, unit: "Ace2Transport", status: Dict[str, Any]) -> None:
        """The ACE 2 keeps RFID data out of GET_STATUS: read it once per
        identified spool, forget it when the slot empties."""
        for slot in status["slots"]:
            index = slot["index"]
            if slot["ace2_filament"] == 0:
                unit._rfid.pop(index, None)
            elif slot["ace2_filament"] == 2 and index not in unit._rfid:
                try:
                    fields = self._exchange(unit, ACE2_FILAMENT_INFO, _pb_msg(index), RPC_REPLY_S)
                    if fields is None:
                        continue  # no reply: try again on the next heartbeat
                    unit._rfid[index] = _ace2_filament(fields)
                except (IndexError, ValueError, UnicodeError):
                    unit._rfid[index] = {}  # unreadable: not asked again until it empties

    def _connect(self) -> bool:
        if self._ser is not None:
            return True
        if serial is None:
            return self._fail_all("pyserial not available")
        port = self.port
        if port.lower() == "auto":
            ports = scan_ace2_ports()
            if not ports:
                time.sleep(1.0)
                return self._fail_all("no ACE 2 USB/RS-485 cable (1a86:55d3) found")
            port = ports[0]  # ponytail: first cable only; pin serial: for a second bus
        try:
            # exclusive: a bus left over from a klippy restart still holds it
            self._ser = serial.Serial(port, ACE2_BAUD, timeout=0.05,
                                      write_timeout=1.0, exclusive=True)
        except Exception as exc:
            time.sleep(1.0)
            return self._fail_all(f"open {port} failed: {exc}")
        self._discover_at = 0.0
        logging.info(f"AFC_ACE: ACE 2 bus connected on {port}")
        return True

    def _fail_all(self, error: str) -> bool:
        for unit in self.units:
            unit.connected, unit.device_id, unit.last_error = False, None, error
        return False

    def _disconnect(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self._fail_all("ACE 2 bus disconnected")

    def _discover_uids(self, rounds: int) -> set:
        """Hardware UIDs of every ACE 2 answering on the bus."""
        uids = set()
        for _ in range(rounds):
            # A broadcast: every unit answers the same seq, so several
            # rounds, collecting every reply (ACEPRO does the same)
            seq = self._send(0, ACE2_DISCOVER, b"")
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                frame = self._read_frame(deadline)
                if frame and frame["cmd"] == ACE2_DISCOVER and frame["seq"] == seq:
                    try:
                        f = _pb_decode(frame["payload"])
                        uids.add(tuple(_pb_get(f, n) for n in (1, 2, 3)))
                    except (IndexError, ValueError):
                        pass
        return uids

    def _discover(self) -> None:
        uids = self._discover_uids(len(self.units) + 1)
        ordered = sorted(uids)
        for unit in self.units:
            if unit.device_id is not None:
                continue  # still answering: leave its id (and assist) alone
            # A unit keeps the UID it first bound to: with a sibling missing,
            # bus order would hand it the wrong unit
            uid = unit.uid or (ordered[unit.unit_index]
                               if unit.unit_index < len(ordered) else None)
            if uid not in uids:
                unit.connected = False
                unit.last_error = (f"ACE 2 unit_index {unit.unit_index} not answering "
                                   f"({len(ordered)} unit(s) on the bus did)")
                continue
            device_id = unit.unit_index + 1
            fields = self._exchange(None, ACE2_ASSIGN, _pb_msg(*uid, device_id), RPC_REPLY_S)
            if fields is None or _pb_get(fields, 1):
                unit.last_error = f"ACE 2 device id assignment failed for UID {uid}"
                continue
            if unit.uid is None:
                unit.note(f"ACE 2 unit_index {unit.unit_index} pinned to UID "
                          f"{'-'.join('%08X' % u for u in uid)}")
            unit.uid, unit.device_id, unit.connected, unit.last_error = uid, device_id, True, None
            unit._misses = 0
            unit.generation += 1
            # Pin the preload an inserted spool triggers to its cap
            # (printer status 0) instead of relying on the boot default
            # (Simon-CR docs/08). Best effort: no reply changes nothing.
            self._exchange(unit, ACE2_PRINTER_STATUS, _pb_msg(0), RPC_REPLY_S)
        missing = any(u.device_id is None for u in self.units)
        self._discover_at = time.monotonic() + (10.0 if missing else float("inf"))

    def _send(self, flags: int, cmd: int, payload: bytes) -> int:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        self._ser.reset_input_buffer()
        self._ser.write(_ace2_frame(flags, seq, cmd, payload))
        self._ser.flush()
        return seq

    def _exchange(self, unit: Optional["Ace2Transport"], cmd: int, payload: bytes,
                  timeout: float) -> Optional[Dict[int, list]]:
        """Send one frame and return the decoded reply fields, or None.
        unit None = unaddressed (bus setup)."""
        device_id = unit.device_id if unit is not None else 0
        seq = self._send(device_id, cmd, payload)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self._read_frame(deadline)
            # Skips our own echo (no reply bit) and other units' replies
            if (frame and frame["seq"] == seq and frame["cmd"] == cmd
                    and (not device_id or (frame["flags"] & 0x7F) in (0, device_id))):
                return _pb_decode(frame["payload"])
        return None

    def _step(self, unit: "Ace2Transport", params: Optional[dict], cmd: int,
              payload: bytes) -> Optional[Dict[int, list]]:
        """One frame of a call; holds a STOP back until the move it stops has
        been running long enough for the firmware to take it."""
        index = (params or {}).get("index")
        if cmd == ACE2_STOP:
            wait = unit._started.get(index, 0.0) + ACE2_STOP_AFTER_START_S - time.monotonic()
            if wait > 0:
                time.sleep(wait)
        fields = self._exchange(unit, cmd, payload,
                                STOP_REPLY_S if cmd == ACE2_STOP else RPC_REPLY_S)
        if cmd == ACE2_FEED and fields is not None:
            unit._started[index] = time.monotonic()
        return fields

    def _call(self, unit: "Ace2Transport", method: str, params: Optional[dict],
              result: Optional[Dict[str, Any]] = None,
              cancelled: Optional[threading.Event] = None) -> Dict[str, Any]:
        """One ACE Pro-style RPC to one unit, in AceTransport._rpc's result shape."""
        if unit.device_id is None or self._ser is None:
            return {"ok": False, "error": unit.last_error or "ACE 2 not discovered"}
        try:
            steps = _ace2_request(method, params)
        except (KeyError, ValueError) as exc:
            return {"ok": False, "error": f"{method}: {exc}"}
        sent_at = time.monotonic()
        if result is not None:
            result["sent"] = True
        try:
            for cmd, payload in steps[:-1]:
                self._step(unit, params, cmd, payload)
        except (IndexError, ValueError):
            pass  # a garbled reply to a clearing step changes nothing
        if len(steps) > 1 and cancelled is not None and cancelled.is_set():
            return {"ok": False, "sent": True, "error": "request abandoned"}
        cmd, payload = steps[-1]
        while True:
            try:
                fields = self._step(unit, params, cmd, payload)
                status = _ace2_status(fields, unit._rfid) if fields is not None and cmd == ACE2_GET_STATUS else None
            except (IndexError, ValueError) as exc:
                fields, unit.last_error = None, f"bad reply: {exc}"
            if (fields is None or method not in ACE2_MOTION_METHODS
                    or _pb_get(fields, 1) != ACE2_FORBIDDEN
                    or time.monotonic() - sent_at > ACE2_FORBIDDEN_WAIT_S
                    or (cancelled is not None and cancelled.is_set())):
                break
            # keep every unit's status fresh while this one waits
            self._heartbeat()
            time.sleep(0.3)
            if cancelled is not None and cancelled.is_set():
                break  # the caller gave up: never send it late
        if method != "get_status":
            logging.info(f"AFC_ACE: t={sent_at:.2f} ACE2 {method} {params or ''} -> "
                         f"{'no reply' if fields is None else 'code %s' % _pb_get(fields, 1)} "
                         f"({time.monotonic() - sent_at:.2f}s)")
        if fields is None:
            unit._misses += 1
            if unit._misses >= 3:
                # Power-cycled or unplugged: it lost its bus id
                unit.connected, unit.device_id = False, None
                self._discover_at = 0.0
            unit.last_error = unit.last_error or f"timeout waiting for ACE 2 reply to {method}"
            return {"ok": False, "sent": True, "error": unit.last_error}
        unit._misses = 0
        if cmd == ACE2_GET_STATUS:
            unit.last_error = None
            return {"ok": True, "response": {"code": 0, "result": status}}
        code = _pb_get(fields, 1)
        if code and method in STOP_METHODS:
            # Whether a stop to an idle slot answers 0 is not known: what
            # the caller needs is the slot idle, so accept that
            check = self._call(unit, "get_status", None)
            slots = check.get("response", {}).get("result", {}).get("slots", []) if check.get("ok") else []
            index = (params or {}).get("index")
            if (isinstance(index, int) and index < len(slots)
                    and slots[index]["ace2_state"] not in ACE2_SLOT_MOTION
                    and slots[index]["ace2_state"] not in (3, 4)):
                unit.last_error = None
                return {"ok": True, "response": {"code": 0}, "assumed_success": True}
        if code:
            unit.last_error = (f"{method} failed with ACE code {code}: "
                               f"{ACE2_CODES.get(code, 'unknown ACE 2 error')}")
            return {"ok": False, "error": unit.last_error, "response": {"code": code}}
        unit.last_error = None
        return {"ok": True, "response": {"code": 0}}

    def _read_frame(self, deadline: float) -> Optional[Dict[str, Any]]:
        """Next CRC-valid reply frame before the deadline, or None."""
        last = b""
        while time.monotonic() < deadline:
            b = _read_exact(self._ser, 1, deadline)
            if last + b != b"\xFF\xAA":
                last = b
                continue
            head = _read_exact(self._ser, 5, deadline)
            if len(head) != 5:
                return None
            body = _read_exact(self._ser, head[4] + 3, deadline)
            if len(body) != head[4] + 3 or body[-1] != 0xFE:
                continue
            payload = body[:-3]
            if int.from_bytes(body[-3:-1], "little") != _crc16_mcrf4xx(head + payload):
                continue
            if head[0] & ACE2_REPLY:
                return {"flags": head[0], "seq": head[1] | (head[2] << 8),
                        "cmd": head[3], "payload": payload}
            last = b""
        return None


class Ace2Transport(AceTransport):
    """One ACE 2 unit: AceTransport's API (submit, cached_status, notes,
    connected, generation) over a shared Ace2Bus instead of its own thread."""

    def __init__(self, serial_port: str, unit_index: int) -> None:
        super().__init__(serial_port, unit_index)
        self.device_id: Optional[int] = None
        self.uid: Optional[Tuple[int, int, int]] = None
        self._misses = 0
        # slot -> when its last FEED_OR_ROLLBACK was accepted; see Ace2Bus._step
        self._started: Dict[int, float] = {}
        self._rfid: Dict[int, Dict[str, Any]] = {}
        self._bus: Optional[Ace2Bus] = None

    def start(self) -> None:
        self._bus = Ace2Bus.attach(self)

    def submit(self, method: str, params: Optional[dict] = None):
        return self._bus.submit(self, method, params)

    def stop(self) -> None:
        if self._bus is not None:
            self._bus.detach(self)


class _BypassRunoutGcode:
    """Stands in for the frame sensor's runout_gcode while AFC's bypass is
    on, as COSMOS's stock machine.cfg runout does, but a plain PAUSE: its
    STATE=runout ends in LOAD_FILAMENT, which COSMOS's afc.cfg deletes."""

    def __init__(self, helper) -> None:
        self.helper = helper

    def render(self, context=None) -> str:
        # Re-checked when the runout distance is used up, like the stock one
        if self.helper.filament_present:
            return ""
        return ('RESPOND MSG="Bypass spool ran out: load a new one, then RESUME"'
                '\nPAUSE')


class AceDryerHeater:
    """Duck-typed Klipper heater for the ACE's firmware dryer.

    Registered with the heaters registry so SET_HEATER_TEMPERATURE (what UIs
    send — grumpyscreen's AFC panel, fluidd) starts drying. Temperature and
    target are synced from the ACE status poll.

    A drying run is a timed firmware cycle, not a hotend: it must not be
    stopped by TURN_OFF_HEATERS, which idle_timeout fires 10 minutes after
    the printer goes idle and print end/cancel macros fire too. That sweep
    reaches the heater as set_temp(0), so set_temp only ever starts the
    dryer. Stops are explicit: AFC_ACE_DRYER_STOP, SET_HEATER_TEMPERATURE
    TARGET=0 on Kalico (routed to the unit's own mux handler), or M112.
    """

    def __init__(self, unit: "afcACE") -> None:
        self.unit = unit
        self.temperature = 0.0
        self.target = 0.0

    def set_temp(self, degrees: float) -> None:
        if degrees > 0:
            self.unit.set_dryer(int(degrees))

    def get_temp(self, eventtime=None):
        return self.temperature, self.target

    def alter_target(self, target: float) -> None:
        self.target = target

    def check_busy(self, eventtime) -> bool:
        return False

    def get_status(self, eventtime=None):
        return {"temperature": round(self.temperature, 1), "target": self.target}


class AceHumiditySensor:
    """Read-only sensor for the ACE enclosure: humidity (%) and temperature
    (C), shaped like Klipper's own humidity sensors (bme280, sht3x)."""

    def __init__(self) -> None:
        self.humidity = 0.0
        self.temperature = 0.0

    def get_temp(self, eventtime=None):
        # TEMPERATURE_WAIT SENSOR= reads any listed sensor through get_temp
        return self.temperature, 0.0

    def get_status(self, eventtime=None):
        return {"humidity": round(self.humidity, 1),
                "temperature": round(self.temperature, 1)}


class afcACE(afcUnit):
    """Anycubic ACE Pro unit — stepperless, firmware-managed feeder."""

    def __init__(self, config: ConfigWrapper) -> None:
        _inherit_setup_defaults(config)
        super().__init__(config)
        # Not configurable: AFC's lane-type gates match this exact string
        self.type = UNIT_TYPE
        self.stepperless_drive: bool = True

        # ace1: ACE Pro (JSON over USB). ace2: ACE 2 Pro (protobuf over
        # RS-485, through Anycubic's USB cable; untested on hardware)
        self.protocol = config.getchoice("protocol", {"ace1": "ace1", "ace2": "ace2"}, "ace1")
        self.serial_port = config.get("serial", "auto")
        # Which ACE this unit is. ACE Pro, with serial auto: USB-topology
        # order, the directly connected unit 0, the first chained unit 1.
        # ACE 2: position on its RS-485 bus by hardware UID, lowest first.
        self.ace_unit_index = config.getint("unit_index", 0, minval=0)
        # Monitor-only: slot status, RFID and the dryer stay live, but every
        # filament-motion command is refused. For a chained ACE whose filament
        # feeds another printer, or a unit not yet plumbed into the splitter.
        self.monitor_only = config.getboolean("monitor_only", False)
        # Tube length for this unit's lanes: slot -> runout sensor (runout
        # path, only a limit) or slot -> merger (toolhead path). A lane's own
        # [AFC_lane] dist_hub overrides it.
        self.dist_hub = config.getfloat("dist_hub", None, minval=0.0)
        # A unit that may legitimately not be there: a chained ACE that is
        # unplugged, or one kept configured for later. Absence is then reported
        # as a plain fact at PREP instead of a fault, and AFC is not handed a
        # failed lane check. Its lanes still appear, empty and not loadable.
        self.optional = config.getboolean("optional", False)
        self.feed_speed = config.getint("feed_speed", 50, minval=10)
        self.retract_speed = config.getint("retract_speed", 75, minval=10)
        # unwind_filament mode: 0 = normal, 1 = "enhanced" (drives the spool
        # take-up harder; believed to pile up slack on some units)
        self.retract_mode = config.getint("retract_mode", 0)
        # Extra mm commanded beyond afc_bowden_length when feeding toward a
        # sensor; the feed is stopped by the sensor, not by this length.
        self.bowden_overshoot = config.getfloat("bowden_overshoot", 300.0, minval=0.0)
        # Extra retract after the homing sensor clears, so filament parks
        # behind the splitter (staged for fast reload) and the next lane can
        # load. Must exceed the sensor -> splitter-merge distance.
        self.hub_clear_mm = config.getfloat("hub_clear_mm", 100.0, minval=0.0)
        # A toolhead sensor sits a few mm short of the extruder gears, and a
        # stopped ACE feed lands a few mm late at full speed, so the last
        # sensor_approach_mm before it run at sensor_approach_speed.
        self.sensor_approach_mm = config.getfloat("sensor_approach_mm", 30.0, minval=0.0)
        self.sensor_approach_speed = config.getint("sensor_approach_speed", 15, minval=5)
        # How much of tool_stn the ACE pushes in step with the extruder, to
        # get the filament through the gears; the extruder pulls the rest
        # alone with feed assist on. Default: all of it.
        self.sync_load_mm = config.getfloat("sync_load_mm", None, minval=0.0)
        # Endless spool: speed (mm/s) for pushing a spent spool's tail through
        # the melt zone with the extruder in step with the ACE.
        self.tail_purge_speed = config.getfloat("tail_purge_speed", 10.0, minval=1.0)
        # Where a load extrudes (the gear engagement, a tail push). AFC's
        # park_cmd when unset. On COSMOS that is MOVE_TO_TRAY, which parks on
        # the tray's v-slot line: COSMOS's own PURGE steps off it to extrude,
        # so the config points this at the same spot.
        self.purge_park_cmd = config.get("purge_park_cmd", None)
        self.dryer_temp = config.getint("dryer_temp", 45)
        self.dryer_duration = config.getint("dryer_duration", 240)  # minutes
        self.dryer_fan_speed = config.getint("dryer_fan_speed", 7000)
        # Existing [filament_switch_sensor] objects to home feeds against, for
        # printers whose sensors are already declared elsewhere in the config
        # (a pin can only be claimed once). hub_sensor_name sits between the
        # splitter and the toolhead; toolhead_sensor_name is at the extruder
        # inlet. Both are alternatives to [AFC_hub] switch_pin / [AFC_extruder]
        # pin_tool_start, which take priority when set.
        self.hub_sensor_name = config.get("hub_sensor_name", None)
        self.toolhead_sensor_name = config.get("toolhead_sensor_name", None)
        # Bypass (a spool fed straight in, AFC's virtual bypass on): the
        # frame sensor pauses this far after a runout, COSMOS's stock value
        self.bypass_runout_distance = config.getfloat("bypass_runout_distance", 770.0, minval=0.0)

        # Expose the firmware dryer as a heater (+ humidity sensor) so UIs
        # control it with SET_HEATER_TEMPERATURE and see live values.
        self.dryer_heater = AceDryerHeater(self)
        self.humidity_sensor = AceHumiditySensor()
        dryer_name = f"ace_dryer_{self.name}"
        hum_name = f"ace_humidity_{self.name}"
        pheaters = self.printer.load_object(config, "heaters")
        pheaters.heaters[dryer_name] = self.dryer_heater
        pheaters.available_heaters.append(dryer_name)
        pheaters.available_sensors.append(hum_name)
        self.printer.add_object(dryer_name, self.dryer_heater)
        self.printer.add_object(hum_name, self.humidity_sensor)
        # Mainline Klipper routes SET_HEATER_TEMPERATURE through
        # heaters.lookup_heater (the dict insert above covers it); Kalico
        # instead registers it as a per-heater mux command, so register ours —
        # on mainline the global command already exists and this raises.
        try:
            self.gcode.register_mux_command(
                'SET_HEATER_TEMPERATURE', "HEATER", dryer_name,
                self._cmd_SET_DRYER_TEMPERATURE,
                desc=f"Set {self.name} ACE dryer temperature")
        except error:
            pass

        self.transport: Optional[AceTransport] = None
        self._slot_map: Dict[str, int] = {}
        self._last_prep: Dict[int, Optional[bool]] = {}
        # Last RFID (type, color) seen per slot; see _sync_lane
        self._last_rfid: Dict[int, Tuple[Any, str]] = {}
        self._operation_active = False
        # (lane name, ok) of the ACE-only feed that lane_loading
        # ran ahead of TOOL_LOAD's heat-up; unit_load_lane finishes from there
        self._early_feed: Optional[Tuple[str, bool]] = None
        self._poll_timer = None
        # first-seen / last-logged times for throttled fault logging
        self._log_throttle: Dict[str, float] = {}
        # Endless-spool state: lanes printing out their ungripped tail. Whether
        # a spent tail still occupies the bowden lives on the hub (see
        # _tail_in_bowden), since chained ACEs share one.
        self._tail_pending: set = set()
        # Lane whose runout paused the print (no runout lane): a new spool in
        # it is loaded and purged, and the user only presses resume
        self._reload_lane: Optional[str] = None
        # Slot that should have print-time feed assist, and the transport
        # generation it was armed on (re-armed after a USB reset)
        self._assist_slot: Optional[int] = None
        self._assist_gen: Optional[int] = None
        # Set once a load has pushed filament into the extruder gears, so a
        # failed load backs the extruder out before the ACE retracts
        self._gears_engaged = False

        self.gcode.register_mux_command(
            'AFC_ACE_DRYER_START', "UNIT", self.name, self.cmd_AFC_ACE_DRYER_START,
            desc="Start the ACE dryer: AFC_ACE_DRYER_START UNIT=%s [TEMP=] [DURATION=] [FAN_SPEED=]" % self.name)
        self.gcode.register_mux_command(
            'AFC_ACE_DRYER_STOP', "UNIT", self.name, self.cmd_AFC_ACE_DRYER_STOP,
            desc="Stop the ACE dryer")
        self.gcode.register_mux_command(
            'AFC_ACE_STATUS', "UNIT", self.name, self.cmd_AFC_ACE_STATUS,
            desc="Report raw ACE status")
        self.gcode.register_mux_command(
            'AFC_ACE_STOP', "UNIT", self.name, self.cmd_AFC_ACE_STOP,
            desc="Stop all ACE motion for a lane: AFC_ACE_STOP UNIT=%s LANE=" % self.name)
        self.gcode.register_mux_command(
            'AFC_ACE_RESET', "UNIT", self.name, self.cmd_AFC_ACE_RESET,
            desc="Retract a lane clear of the sensors and park it behind the "
                 "splitter: AFC_ACE_RESET UNIT=%s LANE=" % self.name)
        # One command each for all units: the first unit registers them
        try:
            self.gcode.register_command(
                'AFC_ACE_BYPASS', self.cmd_AFC_ACE_BYPASS,
                desc="Feed a spool directly, bypassing the ACE and AFC: "
                     "AFC_ACE_BYPASS [ENABLE=0|1] (no ENABLE toggles)")
        except error:
            pass
        try:
            self.gcode.register_command(
                'AFC_ACE_PANEL', self.cmd_AFC_ACE_PANEL,
                desc="Install or remove the optional ACE Pro Mainsail panel: "
                     "AFC_ACE_PANEL ACTION=INSTALL|REMOVE")
        except error:
            pass

    def _create_lanes(self, config: ConfigWrapper) -> None:
        """This unit's four lanes, numbered by unit_index: lane1-4 (T0-T3) on
        the first ACE, lane5-8 (T4-T7) on the second, and so on. A slot that
        already has a lane section (unit: ACE_1:2) keeps it. A section named
        after the lane but without a unit is an override, e.g.
        [AFC_lane lane1] dist_hub: 800: its missing keys are filled in and
        Klipper loads it as usual. Missing lanes are added to the config and
        built here, as AFC_extruder builds its own lane."""
        fc = config.fileconfig
        taken = {fc.get(sec, "unit").strip() for sec in fc.sections()
                 if sec.startswith("AFC_lane ") and fc.has_option(sec, "unit")}
        for slot in range(1, 5):
            unit_ref = f"{self.name}:{slot}"
            if unit_ref in taken:
                continue
            number = self.ace_unit_index * 4 + slot
            section = f"AFC_lane lane{number}"
            existed = fc.has_section(section)
            if not existed:
                fc.add_section(section)
            elif fc.has_option(section, "unit"):
                raise error(f"[{section}] belongs to unit {fc.get(section, 'unit')}, but "
                            f"{self.name} (unit_index {self.ace_unit_index}) numbers slot "
                            f"{slot} {section}. Remove its unit line or change unit_index.")
            keys = [("unit", unit_ref), ("map", f"T{number - 1}"), ("load_to_hub", "False")]
            if self.dist_hub is not None:
                keys.append(("dist_hub", str(self.dist_hub)))
            for key, value in keys:
                if not fc.has_option(section, key):
                    fc.set(section, key, value)
            if not existed:
                self.printer.load_object(config, section)

    # ── lifecycle ───────────────────────────────────────────────────

    def handle_connect(self) -> None:
        super().handle_connect()
        model = "ACE 2 PRO" if self.protocol == "ace2" else "ACE PRO"
        self.logo = _ams_box_logo(model, 4, self.name)
        self.logo_error = _ams_box_logo_error(model, 4, self.name)

    def handle_ready(self) -> None:
        super().handle_ready()
        # AFC_spool subscribes to afc_stepper:register_macros inside its own
        # klippy:connect handler, which runs AFTER this unit's connect handler
        # has already had the lanes fire that event — so SET_COLOR /
        # SET_MATERIAL / SET_MAP / SET_WEIGHT / SET_SPOOL_ID never register
        # for unit-driven lanes. Register them directly; tolerate versions
        # where the ordering is fixed and they already exist.
        for lane in self.lanes.values():
            try:
                self.afc.spool.register_lane_macros(lane)
            except error:
                pass  # already registered
        for lane in self.lanes.values():
            # Some frontends only look lanes up as "AFC_stepper <name>";
            # register that alias like AFC_canvas does.
            try:
                self.printer.add_object(f"AFC_stepper {lane.name}", lane)
            except error:
                pass  # already exists
            slot = lane.index - 1
            if not 0 <= slot <= 3:
                self.logger.error(
                    f"AFC_ACE {self.name}: lane {lane.name} has unit index {lane.index}, "
                    f"expected 1..4 (e.g. 'unit: {self.name}:1') — lane disabled")
                continue
            if self._hub_sensor_fn(lane) and self._toolhead_sensor_fn(lane):
                self.logger.error(
                    f"AFC_ACE {self.name}: lane {lane.name} has both a hub (frame) "
                    "sensor and a toolhead sensor; use one — lane disabled")
                continue
            self._slot_map[lane.name] = slot
        transport = Ace2Transport if self.protocol == "ace2" else AceTransport
        self.transport = transport(self.serial_port, self.ace_unit_index)
        self.transport.start()
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        self._poll_timer = self.reactor.register_timer(
            self._poll_status, self.reactor.monotonic() + 2.0)

    def _handle_shutdown(self) -> None:
        """On M112/MCU shutdown, halt all ACE motion — a feed in flight would
        otherwise keep pushing up to its commanded length with nobody driving
        the sequence — and stop the dryer, which Klipper's heater shutdown
        does not reach. Fire-and-forget via the transport thread: the reactor
        is not usable for waiting here."""
        if self.transport is None:
            return
        for slot in set(self._slot_map.values()):
            for method in ("stop_feed_filament", "stop_unwind_filament",
                           "stop_feed_assist"):
                self.transport.submit(method, {"index": slot})
        self.transport.submit("drying_stop")
        self.dryer_heater.target = 0.0

    def _handle_disconnect(self) -> None:
        if self.transport:
            self.transport.stop()

    # ── RPC plumbing (reactor side) ─────────────────────────────────

    def _rpc(self, method: str, params: Optional[dict] = None, timeout: float = 8.0) -> Dict[str, Any]:
        if self.transport is None:
            return {"ok": False, "error": "ACE transport not started"}
        # After M112 / an MCU shutdown a load or calibration loop would carry
        # on sending feeds: only stops and status get out
        if (self.printer.is_shutdown()
                and method not in STOP_METHODS and method != "get_status"):
            return {"ok": False, "error": "printer is shut down"}
        if isinstance(self.transport, Ace2Transport) and method in ACE2_MOTION_METHODS:
            # the bus may sit out a FORBIDDEN window before this goes out
            timeout = max(timeout, ACE2_FORBIDDEN_WAIT_S + RPC_REPLY_S + 2.0)
        result, done, cancelled = self.transport.submit(method, params)
        deadline = self.reactor.monotonic() + timeout
        while not done.is_set():
            if self.reactor.monotonic() > deadline:
                # Abandon the request so a not-yet-sent motion command cannot
                # fire after we have reported failure. "sent" means the frame
                # was written, not just taken from the queue.
                cancelled.set()
                return {"ok": False, "error": f"timeout waiting for ACE rpc {method}",
                        "sent": bool(result.get("sent"))}
            self.reactor.pause(self.reactor.monotonic() + 0.05)
        return result

    def _get_status(self, max_age: float = 0.0, newer_than: float = 0.0,
                    strict: bool = False) -> Optional[Dict[str, Any]]:
        """ACE status dict, from cache when fresh enough, else via RPC.
        newer_than rejects cache entries fetched before a command was issued —
        age alone can't tell a pre-command snapshot from a live one. strict
        returns None instead of falling back to a stale/pre-command snapshot
        when a fresh read fails, for callers deciding whether motion is safe."""
        status, when = self.transport.cached_status() if self.transport else (None, 0.0)
        if (status is not None and max_age > 0 and when > newer_than
                and (time.monotonic() - when) <= max_age):
            return status
        res = self._rpc("get_status")
        if res.get("ok"):
            payload = res.get("response", {}).get("result")
            if isinstance(payload, dict):
                return payload
        if strict or (status is not None and when <= newer_than):
            return None
        return status

    def _slot_info(self, slot: int, status: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        slots = (status or {}).get("slots")
        if isinstance(slots, list) and 0 <= slot < len(slots) and isinstance(slots[slot], dict):
            return slots[slot]
        return {}

    def _slot_present(self, slot: int, status: Optional[Dict[str, Any]]) -> bool:
        """Callers must rule out status=None first: no status is "unknown",
        and reading it as an empty slot would mistake a comms glitch for a
        spent spool."""
        return str(self._slot_info(slot, status).get("status", "")).lower() not in ("", "empty")

    def _motion_active(self, slot: int, status: Dict[str, Any]) -> bool:
        vals = (status.get("status"), status.get("action"),
                self._slot_info(slot, status).get("status"))
        return any(str(v or "").lower() in ACTIVE_STATES for v in vals)

    # ── ACE motion primitives ───────────────────────────────────────

    def _motion_blocked(self, what: str) -> bool:
        if self.monitor_only:
            self.logger.error(
                f"{self.name} is monitor_only — refusing {what}. Remove "
                "monitor_only from its config once its filament path feeds "
                "this printer.")
            return True
        return False

    def _motion_rpc(self, slot: int, method: str, params: Dict[str, Any],
                    done_fn: Optional[Callable[[], bool]] = None) -> Dict[str, Any]:
        """Issue a motion RPC, tolerating a lost or garbled reply frame: the
        command may have landed even though its reply didn't survive. Verify
        via fresh status — motion running means it took. Idle is ambiguous
        once the move could already have finished, so a resend happens only
        when that is impossible (less time has passed than the move takes)
        or harmless: a homed move (`done_fn` given, a sensor stops it) whose
        sensor has not yet reached its target state. Once it has, the first
        command landed and ran unwatched; a resend would move again."""
        issued = self.reactor.monotonic()
        res = self._rpc(method, params)
        if res.get("ok"):
            return res
        if not res.get("sent"):
            return res  # never reached the ACE: nothing to verify
        self.logger.info(
            f"{method} reply lost/garbled ({res.get('error')}); verifying via status")
        # A lost reply is usually the ACE dropping off USB: give it time to
        # re-enumerate before reading its state. Fresh reads or nothing.
        deadline = self.reactor.monotonic() + RECONNECT_WAIT_S
        status = self._get_status(strict=True)
        while status is None and self.reactor.monotonic() < deadline:
            self.reactor.pause(self.reactor.monotonic() + 0.2)
            status = self._get_status(strict=True)
        if status is None:
            # Motion state indeterminate: the command may be executing.
            # Resending non-idempotent motion here could double it — fail.
            return {"ok": False,
                    "error": f"{method} reply lost and motion state indeterminate"}
        if self._motion_active(slot, status) or (done_fn is not None and done_fn()):
            return {"ok": True, "assumed_started": True}
        move_s = params["length"] / max(params["speed"], 1)
        if done_fn is not None or self.reactor.monotonic() - issued < move_s:
            return self._rpc(method, params)
        return {"ok": False,
                "error": f"{method} reply lost and the move may already have run"}

    def _stop_motion(self, slot: int, method: str) -> Dict[str, Any]:
        res = self._rpc(method, {"index": slot})
        # The ACE occasionally eats the reply frame for stop commands while
        # motors wind down; the stop itself still lands. Treat a read timeout
        # as success (verified against hardware by CosmoACE) — but only when
        # the stop actually went out.
        if (not res.get("ok") and res.get("sent")
                and "timeout" in str(res.get("error", "")).lower()):
            self.logger.debug(f"{method} reply timed out; assuming success")
            return {"ok": True, "assumed_success": True}
        return res

    def _wait_motion_idle(self, slot: int, timeout: float, settle: float = SPIN_UP_S) -> bool:
        # Every read is a fresh get_status (a cached heartbeat snapshot can be
        # up to HEARTBEAT_S old, and two reads of the same snapshot prove
        # nothing). Two consecutive idle reads are required: the firmware can
        # briefly report idle between command ack and spin-up, which `settle`
        # also waits out. After a stop there is no spin-up, so stops pass a
        # short settle.
        self.reactor.pause(self.reactor.monotonic() + settle)
        deadline = self.reactor.monotonic() + timeout
        idle_reads = 0
        while self.reactor.monotonic() < deadline:
            status = self._get_status(strict=True)
            if status is not None and not self._motion_active(slot, status):
                idle_reads += 1
                if idle_reads >= 2:
                    return True
            else:
                idle_reads = 0
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        return False

    def _halt(self, slot: int, method: str) -> Tuple[bool, str]:
        """Stop a move and confirm the ACE actually went idle. A stop whose
        reply is lost has usually landed, but anything that follows (the
        blind push to the gears, the park) assumes the filament is where the
        sensor left it, so never carry on while the motor may still turn."""
        for _ in range(2):
            stop = self._stop_motion(slot, method)
            if stop.get("ok") and self._wait_motion_idle(slot, 3.0, settle=0.1):
                return True, ""
        return False, f"{method}: the ACE did not stop"

    def _feed_until(self, slot: int, sensor_fn: Optional[Callable[[], bool]],
                    length: float, speed: float, extend: bool = True) -> Tuple[bool, str]:
        """Start an ACE feed and stop it when sensor_fn goes True.

        With a sensor, `length` is the EXPECTED run, not a hard cap: if the
        commanded feed runs out before the sensor trips, the feed is extended
        in chunks. Only a generous safety budget (a dead feeder or wrong path
        would otherwise grind forever) fails the move. With extend=False the
        feed stops at `length` and returns (False, NOT_REACHED) instead. With
        no sensor, the exact length is fed and waited out.
        """
        homed = sensor_fn is not None
        res = self._motion_rpc(slot, "feed_filament",
                               {"index": slot, "length": int(length), "speed": int(speed)},
                               done_fn=sensor_fn)
        if not res.get("ok"):
            return False, f"feed_filament failed: {res.get('error')}"
        if not homed:
            if not self._wait_motion_idle(slot, length / max(speed, 1.0) + 30.0):
                self._stop_motion(slot, "stop_feed_filament")
                return False, "timeout waiting for blind feed to finish"
            return True, ""
        budget = length * 2.0 + 500.0
        fed = length
        # Only a status read taken after the feed has had time to spin up can
        # say it ran out; anything earlier (cached, or read in the gap after
        # the ack) reads idle and would fire extension chunks into a running
        # feed.
        settled = time.monotonic() + SPIN_UP_S
        deadline = self.reactor.monotonic() + budget / max(speed, 1.0) + 60.0
        while self.reactor.monotonic() < deadline:
            if sensor_fn():
                ok, msg = self._halt(slot, "stop_feed_filament")
                if not ok:
                    return False, msg
                # Confirm the sensor held through the stop wind-down
                self.reactor.pause(self.reactor.monotonic() + 0.2)
                if not sensor_fn():
                    return False, "sensor lost after feed stop"
                return True, ""
            # Commanded length exhausted without the sensor: extend the feed
            if time.monotonic() >= settled:
                status = self._get_status(max_age=HEARTBEAT_S * 2, newer_than=settled)
                if status is not None and not self._motion_active(slot, status):
                    if not extend:
                        return False, NOT_REACHED
                    if fed >= budget:
                        break
                    step = min(200.0, budget - fed)
                    res = self._motion_rpc(slot, "feed_filament",
                                           {"index": slot, "length": int(step),
                                            "speed": int(speed)},
                                           done_fn=sensor_fn)
                    if not res.get("ok"):
                        return False, f"feed extension failed: {res.get('error')}"
                    fed += step
                    settled = time.monotonic() + SPIN_UP_S
            self.reactor.pause(self.reactor.monotonic() + 0.05)
        self._stop_motion(slot, "stop_feed_filament")
        return False, (f"sensor did not trigger within {fed:.0f}mm "
                       f"(expected ~{length:.0f}mm) — check the filament path")

    def _home(self, slot: int, feed: Callable[[], Tuple[bool, str]]) -> Tuple[bool, str]:
        """Run a feed that homes on a path sensor. A slot sensor that drops
        out for a moment mid-feed makes the ACE start its own insert preload
        (seen on an ACE Pro: the slot reads "empty", then "preload"). The
        preload moves the filament and ignores stop, so the feed fails with
        the spool fine. Wait the preload out and home once more: the feed
        never assumes where the tip is."""
        started = time.monotonic()
        ok, msg = feed()
        if ok or not self.transport.preloaded_since(slot, started):
            return ok, msg
        self.logger.info(f"{self.name}: slot {slot + 1} ran the ACE's own preload "
                         f"during the feed ({msg}); homing again")
        if not self._wait_motion_idle(slot, PRELOAD_WAIT_S, settle=0.0):
            return False, f"{msg}; the ACE's preload did not finish"
        return feed()

    def _approach_sensor(self, slot: int, sensor_fn: Callable[[], bool],
                         near: float, far: float) -> Tuple[bool, str]:
        """Feed to a sensor just short of the extruder gears, slowly for the
        last sensor_approach_mm, so the stop cannot carry the tip into the
        stopped gears.

        near: the furthest a parked tip can be from the sensor; that much is
        fed slowly first, since it may end at the sensor. far: the expected
        run from anywhere else (the slot). Every phase stops at the sensor.
        """
        if near > 0:
            ok, msg = self._feed_until(slot, sensor_fn, near,
                                       self.sensor_approach_speed, extend=False)
            if msg != NOT_REACHED:
                return ok, msg
        fast = far - near - self.sensor_approach_mm
        if self.protocol == "ace2":
            # A fresh spool's tip may be up to a preload past the slot
            fast -= ACE2_PRELOAD_MAX_MM
        if fast > 0 and not sensor_fn():
            ok, msg = self._feed_until(slot, sensor_fn, fast, self.feed_speed, extend=False)
            if ok:
                self.logger.warning(
                    f"{self.name}: reached the toolhead sensor at full speed: the "
                    "tip was further along than the park, or dist_hub is longer "
                    "than the real path")
            if msg != NOT_REACHED:
                return ok, msg
        if sensor_fn():
            return True, ""
        # Whatever the fast phase left, slowly (its extension budget scales
        # with the length asked for)
        rest = max(far - near - max(fast, 0.0), self.sensor_approach_mm)
        return self._feed_until(slot, sensor_fn, rest, self.sensor_approach_speed)

    def _retract_until_clear(self, slot: int, sensor_name: str,
                             sensor_fn: Callable[[], bool], cap: float,
                             park: float) -> Tuple[bool, str]:
        """One continuous unwind, stopped the moment the sensor clears, then a
        park move of `park` mm behind it. Used for unloads, ejects and
        recovery alike. A stopped unwind may skip the spool take-up (untested
        on hardware). `cap` bounds the move for the case where the sensor
        never clears (broken filament, wrong path).
        """
        res = self._motion_rpc(slot, "unwind_filament",
                               {"index": slot, "length": int(cap),
                                "speed": int(self.retract_speed),
                                "mode": self.retract_mode},
                               done_fn=lambda: not sensor_fn())
        if not res.get("ok"):
            return False, f"unwind_filament failed: {res.get('error')}"
        if res.get("assumed_started") and not sensor_fn():
            # The reply was lost and the sensor cleared while nobody watched:
            # the tip is anywhere from just behind it to `cap` back. A park
            # could run it toward the slot; no park could leave it in the
            # merger for the next lane to jam on. Stop and let the user look.
            ok, msg = self._halt(slot, "stop_unwind_filament")
            if not ok:
                return False, msg
            return False, (f"{sensor_name} sensor cleared while the unwind reply was lost: "
                           "filament position unknown, check the merger before the "
                           "next load")

        deadline = self.reactor.monotonic() + cap / max(self.retract_speed, 1.0) + 30.0
        while self.reactor.monotonic() < deadline:
            if not sensor_fn():
                break
            self.reactor.pause(self.reactor.monotonic() + 0.05)
        else:
            self._stop_motion(slot, "stop_unwind_filament")
            return False, f"{sensor_name} sensor did not clear during retract"

        ok, msg = self._halt(slot, "stop_unwind_filament")
        if not ok:
            return False, msg

        # The tip is now at the sensor: everything past here is measured from
        # that known point.
        if park > 0:
            return self._retract(slot, park, self.retract_speed)
        return True, ""

    def _retract(self, slot: int, length: float, speed: float,
                 wait: bool = True) -> Tuple[bool, str]:
        res = self._motion_rpc(slot, "unwind_filament", {"index": slot, "length": int(length),
                                                          "speed": int(speed), "mode": self.retract_mode})
        if not res.get("ok"):
            return False, f"unwind_filament failed: {res.get('error')}"
        if wait and not self._wait_motion_idle(slot, length / max(speed, 1.0) + 30.0):
            self._stop_motion(slot, "stop_unwind_filament")
            return False, "timeout waiting for retract to finish"
        return True, ""

    def _sync_load(self, slot: int, cur_extruder: AFCExtruder, length: float,
                   chunk: float = 5.0) -> Tuple[bool, str]:
        """Thread filament into the extruder gears: ACE feed chunks matched
        to extruder moves at the same speed, so the ACE pushes while the
        extruder pulls. Each chunk waits for the ACE to go idle before the
        next, as CosmoACE's ACE_SYNC_LOAD does: a feed sent while the last
        one is still running is not reliably accepted."""
        self._gears_engaged = True
        fed = 0.0
        speed = max(cur_extruder.tool_load_speed, 1.0)
        while fed < length:
            step = min(chunk, length - fed)
            # The ACE takes whole mm; a sub-mm remainder is the extruder's alone
            ace_mm = int(round(step))
            if ace_mm > 0:
                res = self._motion_rpc(slot, "feed_filament", {"index": slot, "length": ace_mm,
                                                                "speed": int(speed)})
                if not res.get("ok"):
                    return False, f"sync load feed failed: {res.get('error')}"
            self.afc.move_e_pos(step, speed, "sync load", wait_tool=True)
            if ace_mm > 0 and not self._wait_motion_idle(slot, step / speed + 5.0):
                self._stop_motion(slot, "stop_feed_filament")
                return False, "timeout waiting for sync load chunk to finish"
            self.reactor.pause(self.reactor.monotonic() + 0.25)
            fed += step
        return True, ""

    def _set_assist(self, slot: int, enable: bool) -> None:
        """Print-time feed assist on/off. The wanted state is remembered so
        the status poll can re-arm it after the ACE's USB watchdog resets the
        unit, which drops assist (CosmoACE's daemon reconciled it the same
        way). Without assist the extruder drags filament the whole way from
        the spool and under-extrudes."""
        if enable:
            self._assist_slot = slot
            self._arm_assist()
        else:
            self._assist_slot = None
            self._stop_motion(slot, "stop_feed_assist")

    def _assist_after_load(self, slot: int) -> None:
        """Feed assist for a load that ends with the extruder standing still.
        An ACE 2 armed now would push into the stopped gears (see
        _assist_needs_arming), so the status poll arms it once the purge
        extrudes; an ACE Pro is armed at once."""
        if self.protocol == "ace2":
            self._assist_slot, self._assist_gen = slot, None
        else:
            self._set_assist(slot, True)

    def _assist_needs_arming(self, status: Dict[str, Any]) -> bool:
        """Whether the status poll should (re)arm feed assist: it is wanted
        and was never armed on this connection (PREP, a USB reset, a failed
        arm), or an ACE 2 dropped it with an assist error, the firmware's
        timeout ~1 s after the extruder stops pulling, cleared only by a new
        start (hakimio). An ACE 2 is armed only while the extruder is moving
        forward: armed on a still one it pushes into stopped gears, then
        errors, every poll (heat-ups and M109 waits count as printing)."""
        if self._assist_slot is None or self.monitor_only:
            return False
        if self.protocol != "ace2":
            return self._assist_gen != self.transport.generation
        dropped = (self._slot_info(self._assist_slot, status).get("ace2_state")
                   == ACE2_ASSIST_ERROR)
        return ((dropped or self._assist_gen != self.transport.generation)
                and self._extruding())

    def _extruding(self) -> bool:
        try:
            report = self.printer.lookup_object("motion_report")
            return report.get_status(self.reactor.monotonic())["live_extruder_velocity"] > 0.0
        except Exception:
            return False

    def _arm_assist(self) -> None:
        generation = self.transport.generation if self.transport else None
        res = self._rpc("start_feed_assist", {"index": self._assist_slot})
        if res.get("ok"):
            self._assist_gen = generation
        else:
            self._log_once(
                "assist", f"{self.name}: feed assist did not start ({res.get('error')}) — "
                "the extruder is pulling the spool unaided, expect under-extrusion. "
                "Retrying on each status poll.")

    # ── sensors ─────────────────────────────────────────────────────

    def _named_sensor_fn(self, sensor_name: str) -> Optional[Callable[[], bool]]:
        """State reader for an existing [filament_switch_sensor <name>]."""
        try:
            sensor = self.printer.lookup_object(f"filament_switch_sensor {sensor_name}")
        except error:
            self.logger.error(
                f"{self.name}: filament_switch_sensor '{sensor_name}' not found in config")
            return None
        return lambda: bool(sensor.runout_helper.filament_present)

    def _path_sensor(self, lane: AFCLane) -> Optional[Tuple[str, Callable[[], bool]]]:
        """The one sensor that homes this lane's feeds, as (kind, reader):
        "hub" for the frame runout sensor, "tool_start" for the pre-gear
        toolhead sensor, or None. handle_ready disables a lane with both."""
        hub_fn = self._hub_sensor_fn(lane)
        if hub_fn is not None:
            return "hub", hub_fn
        tool_fn = self._toolhead_sensor_fn(lane)
        return ("tool_start", tool_fn) if tool_fn is not None else None

    def _hub_sensor_fn(self, lane: AFCLane) -> Optional[Callable[[], bool]]:
        """Hub (frame) sensor reader: an [AFC_hub] switch_pin when it is a
        real pin, else the named existing sensor, else None."""
        hub = lane.hub_obj
        if hub is not None and not hub.is_virtual_pin():
            return lambda: bool(hub.state)
        if self.hub_sensor_name:
            return self._named_sensor_fn(self.hub_sensor_name)
        return None

    def _afc_watches(self, lane: AFCLane, sensor: str) -> bool:
        """Whether AFC's own runout handling is armed on this path sensor."""
        if sensor == "tool_start":
            return bool(lane.extruder_obj and lane.extruder_obj.enable_runout
                        and lane.extruder_obj.tool_start)
        hub = lane.hub_obj
        return bool(hub is not None and not hub.is_virtual_pin() and hub.enable_runout)

    def _toolhead_sensor_fn(self, lane: AFCLane) -> Optional[Callable[[], bool]]:
        """Pre-gear toolhead sensor reader: AFC's pin_tool_start when
        configured, else the named existing sensor, else None."""
        extruder = lane.extruder_obj
        if extruder is not None and extruder.tool_start and extruder.tool_start != "buffer":
            return lane.get_toolhead_pre_sensor_state
        if self.toolhead_sensor_name:
            return self._named_sensor_fn(self.toolhead_sensor_name)
        return None

    # ── bypass ──────────────────────────────────────────────────────

    def _bypass_filament_present(self) -> bool:
        """Filament at the path sensor the bypass spool goes through."""
        for lane_name in self._slot_map:
            sensor = self._path_sensor(self.lanes[lane_name])
            if sensor is not None:
                return bool(sensor[1]())
        return False

    def _sync_bypass(self) -> None:
        """Follow AFC's bypass switch, however it was flipped (this unit's
        AFC_ACE_BYPASS, the virtual_bypass switch in a UI, or restored at
        PREP). The ACE config mutes runout on its path sensor, because the
        ACE's toolchanges empty it; a directly fed spool needs it back:
          frame sensor path: COSMOS's stock pause, bypass_runout_distance
                             after the sensor clears;
          toolhead path:     AFC's own toolhead runout in bypass.
        The saved state lives on the shared sensor or extruder object, so
        several units on one path apply and restore it once."""
        try:
            bypass = bool(self.afc.get_bypass_state())
        except Exception:
            return
        if self.hub_sensor_name:
            try:
                helper = self.printer.lookup_object(
                    f"filament_switch_sensor {self.hub_sensor_name}").runout_helper
            except error:
                return
            saved = getattr(helper, "_afc_ace_saved", None)
            if bypass and saved is None:
                helper._afc_ace_saved = (helper.runout_gcode, helper.runout_distance)
                helper.runout_gcode = _BypassRunoutGcode(helper)
                helper.runout_distance = self.bypass_runout_distance
                helper.sensor_enabled = True
                self.logger.info("Bypass on: frame sensor runout pause restored")
            elif not bypass and saved is not None:
                helper.runout_gcode, helper.runout_distance = saved
                # muted again, as _afc_ace_mute_sensor_gates leaves it
                helper.sensor_enabled = False
                del helper._afc_ace_saved
            return
        for lane_name in self._slot_map:
            ext = self.lanes[lane_name].extruder_obj
            fila = getattr(ext, "fila_tool_start", None)
            if fila is None:
                continue
            saved = getattr(ext, "_afc_ace_saved", None)
            if bypass and saved is None:
                ext._afc_ace_saved = (ext.enable_runout, ext.enable_runout_in_bypass,
                                      fila.runout_helper.sensor_enabled)
                ext.enable_runout = ext.enable_runout_in_bypass = True
                fila.runout_helper.sensor_enabled = True
                self.logger.info("Bypass on: toolhead sensor runout pause on")
            elif not bypass and saved is not None:
                (ext.enable_runout, ext.enable_runout_in_bypass,
                 fila.runout_helper.sensor_enabled) = saved
                del ext._afc_ace_saved

    def cmd_AFC_ACE_BYPASS(self, gcmd: GCodeCommand) -> None:
        bypass = bool(self.afc.get_bypass_state())
        enable = gcmd.get_int("ENABLE", int(not bypass), minval=0, maxval=1)
        if "virtual" not in self.afc.bypass.name:
            raise gcmd.error("A hardware [filament_switch_sensor bypass] is configured: "
                             "it decides bypass, not this command")
        if enable == bypass:
            gcmd.respond_info(f"Bypass already {'on' if bypass else 'off'}")
            return
        if self.afc.function.in_print():
            raise gcmd.error("Not during a print")
        if enable:
            if self.afc.current is not None:
                # AFC refuses bypass while a lane is loaded
                gcmd.respond_info(f"Unloading {self.afc.current} first")
                self.gcode.run_script_from_command("TOOL_UNLOAD")
                if self.afc.current is not None:
                    raise gcmd.error("Unload failed, bypass stays off")
        elif self._bypass_filament_present():
            raise gcmd.error("The bypass spool is still in the path sensor: UNLOAD_FILAMENT, "
                             "pull the filament out, then turn bypass off")
        self.gcode.run_script_from_command(
            f"SET_FILAMENT_SENSOR SENSOR=virtual_bypass ENABLE={enable}")
        self._sync_bypass()
        if enable:
            gcmd.respond_info(
                "Bypass on: AFC will not load or change lanes. Feed the spool through the "
                "path sensor to the extruder, heat the nozzle and extrude until it flows. "
                "To go back: UNLOAD_FILAMENT, pull the filament out, AFC_ACE_BYPASS ENABLE=0.")
        else:
            gcmd.respond_info("Bypass off: the ACE lanes are back in use")

    # ── status polling / lane sync ──────────────────────────────────

    def _log_once(self, key: str, message: str, every: float = 60.0) -> None:
        """Log the first occurrence, then at most one per `every` seconds.
        A fault that repeats every poll would otherwise bury the log."""
        now = time.monotonic()
        if now - self._log_throttle.get(key, 0.0) < every:
            return
        self._log_throttle[key] = now
        self.logger.error(message)

    def _poll_status(self, eventtime: float) -> float:
        """Timer entry point. The reactor calls timer callbacks without a
        guard of its own and blanks the waketime before the call, so anything
        escaping here takes klippy down AND leaves this timer dead. Nothing
        escapes."""
        try:
            return self._poll_status_inner(eventtime)
        except Exception:
            self._log_once("poll", f"AFC_ACE {self.name}: status poll failed, "
                                   f"continuing:\n{traceback.format_exc()}")
            return eventtime + 2.0

    def _poll_status_inner(self, eventtime: float) -> float:
        if self.transport is not None:
            # the serial thread cannot log for itself
            for note in self.transport.drain_notes():
                self.logger.info(note)
        self._sync_bypass()
        if self._operation_active or self.transport is None:
            return eventtime + 2.0
        if self._reload_lane is not None and not self.afc.function.is_paused():
            # Resumed (or cancelled) without a new spool in that lane
            self._reload_lane = None

        # Hub-path occupancy is AFC's own state, not the ACE's: the virtual hub
        # aggregates every lane's raw load flag, so a unit that has stopped
        # answering would otherwise hold the shared hub "occupied" and TOOL_LOAD
        # would refuse every load with "Hub not clear". Derive it from
        # tool_loaded, which needs no conversation with the unit.
        for lane_name, lane in self.lanes.items():
            if lane_name in self._slot_map:
                lane._load_state = bool(lane.tool_loaded)

        status, when = self.transport.cached_status()
        if status is None or (time.monotonic() - when) > 10.0:
            # No word from this ACE (unplugged, powered down, or its watchdog
            # mid-re-enumeration). Its lanes keep whatever they were restored
            # with from AFC.var, so clear the loadable flag: a stale one has
            # TOOL_LOAD and every UI offering a load this unit cannot perform.
            # prep_state is left alone so the last known spool still shows.
            self._mark_lanes_unloadable()
            return eventtime + 2.0
        if self._assist_needs_arming(status):
            self._arm_assist()
            # The RPC paused this timer: a load or unload may have started,
            # and `status` predates it
            if self._operation_active:
                return eventtime + 2.0
            status, when = self.transport.cached_status()
            if status is None:
                return eventtime + 2.0
        # Sync the dryer heater + humidity objects for UIs
        try:
            self.dryer_heater.temperature = float(status.get("temp") or 0.0)
            self.humidity_sensor.temperature = self.dryer_heater.temperature
            # ACE firmware reports this as "dryer_status" ("dryer" on some builds)
            dryer = status.get("dryer_status") or status.get("dryer") or {}
            if not isinstance(dryer, dict):
                dryer = {}
            if str(dryer.get("status", "")).lower() in ("stop", "stopped", ""):
                self.dryer_heater.target = 0.0
            else:
                self.dryer_heater.target = float(dryer.get("target_temp") or 0.0)
            if status.get("humidity") is not None:
                self.humidity_sensor.humidity = float(status["humidity"])
        except (TypeError, ValueError):
            pass
        for lane_name, lane in self.lanes.items():
            # A lane's runout handling can pause this timer in an RPC; stop
            # syncing from the old snapshot once an operation has started
            if self._operation_active:
                break
            slot = self._slot_map.get(lane_name)
            if slot is None:
                continue
            try:
                self._sync_lane(lane_name, lane, slot, status, eventtime)
            except Exception:
                # one lane's RFID payload or runout handler must not stop the
                # others being synced, nor kill the timer
                self._log_once(f"sync:{lane_name}",
                               f"AFC_ACE {self.name}: syncing {lane_name} failed, "
                               f"continuing:\n{traceback.format_exc()}")
        return eventtime + 1.5

    def _mark_lanes_unloadable(self) -> None:
        """Withdraw the "staged and loadable" flag from this unit's lanes."""
        for lane_name, lane in self.lanes.items():
            if lane_name in self._slot_map and lane.loaded_to_hub:
                lane.loaded_to_hub = False

    def _sync_lane(self, lane_name: str, lane: AFCLane, slot: int,
                   status: Dict[str, Any], eventtime: float) -> None:
        if "status" not in self._slot_info(slot, status):
            # No data for this slot in the frame: unknown, not empty. Read
            # as empty, a loaded lane would drop feed assist mid-print.
            return
        present = self._slot_present(slot, status)
        lane.prep_state = present
        # Virtual-hub idiom (same as OpenAMS): loaded_to_hub is the latched
        # "staged and loadable" flag TOOL_LOAD gates on, and a present spool is
        # staged. The raw _load_state (hub-path occupancy) is kept in step with
        # tool_loaded by the caller, for every unit, answering or not.
        # A monitor_only unit refuses every motion command, so its lanes
        # must not look loadable either: loaded_to_hub is what
        # AFC_lane.load_state reports through a virtual hub, so it is what
        # TOOL_LOAD gates on and what a UI greys its load button by.
        lane.loaded_to_hub = present and not self.monitor_only
        prev = self._last_prep.get(slot)
        if prev is not None and present != prev:
            if present:
                self._tail_pending.discard(lane_name)
                lane.handle_load_runout(eventtime, True)
                # After AFC's insert handling, not before: its set_loaded()
                # resets material/color to defaults when no Spoolman spool is
                # set (the COSMOS AFC 1.2.1 fork does so even when a color is
                # already there). A fresh spool's RFID tag is ground truth.
                self._seed_spool_weight(lane)
                self._apply_rfid(lane, self._slot_info(slot, status), force=True)
                self._publish_lane(lane)
                if lane_name == self._reload_lane:
                    self._reload_lane = None
                    loaded = lane.extruder_obj.lane_loaded
                    if self.afc.function.is_paused() and loaded in (None, lane_name):
                        if loaded is not None:
                            # The pause left the spent spool booked in
                            # (unload_on_runout off): book it out as an
                            # unload of a spent spool does, its remnant left
                            # for the load to push through
                            self._tail_unload(lane, slot)
                        self._reload(lane_name)
            elif lane.tool_loaded:
                # Spool ran out while loaded: an ungripped tail now spans
                # slot->nozzle, so an immediate swap could not retract it.
                # Keep consuming the tail; the swap fires when the hub
                # sensor clears (the extruder still grips the remainder).
                self.logger.info(
                    f"{self.name}: {lane_name} spool ran out — printing out the "
                    "tail, endless-spool swap fires when the hub sensor clears")
                self._set_assist(slot, False)
                self._tail_pending.add(lane_name)
            else:
                # Spool removed from an idle slot
                lane.handle_load_runout(eventtime, False)
        self._last_prep[slot] = present

        # A tag can be read after the spool went in (an ACE 2 reads it only
        # once the spool turns), so apply it whenever it changes, not only
        # on insert. The first sighting only records: restored edits win.
        info = self._slot_info(slot, status)
        tag = (info.get("type"), str(info.get("color")))
        seen = self._last_rfid.get(slot)
        self._last_rfid[slot] = tag
        if present and seen is not None and tag != seen and info.get("type"):
            self._apply_rfid(lane, info, force=True)
            self._publish_lane(lane)

        # Broken filament: the spool is still in its slot but the first path
        # sensor (the frame runout sensor, or the toolhead sensor) reads
        # empty. AFC never sees that when it does not own the sensor (a named
        # frame sensor) or its runout is off (the endless-spool tail passes
        # it too), so hand it AFC's own break/jam pause, which checks it is
        # mid-print and not mid-toolchange.
        sensor = self._path_sensor(lane)
        if present and lane.tool_loaded and lane_name not in self._tail_pending:
            if sensor and not sensor[1]() and not self._afc_watches(lane, sensor[0]):
                lane.handle_toolhead_runout(sensor=sensor[0])

        # Endless-spool stage 2: the tail end passed the path sensor
        if lane_name in self._tail_pending:
            checkpoint = sensor
            if checkpoint is None or not checkpoint[1]():
                self._tail_pending.discard(lane_name)
                # Below a hub sensor the tail is unsensed and the next
                # load must push it through; past a toolhead sensor only
                # ~tool_stn remains, which the normal load engagement
                # (sync-load) pushes through by itself.
                if checkpoint is not None and checkpoint[0] == "hub":
                    self._set_tail_in_bowden(lane, True)
                self.logger.info(
                    f"{self.name}: {lane_name} tail passed the "
                    f"{checkpoint[0] if checkpoint else 'path'} — starting swap")
                lane.handle_load_runout(eventtime, False)
                # A runout that paused (no runout lane): a new spool in this
                # lane is loaded (see _reload) while the pause lasts
                if lane.runout_lane is None and self.afc.function.is_paused():
                    self._reload_lane = lane_name

    def _reload(self, lane_name: str) -> None:
        """Load and purge a spool put into the lane whose runout paused the
        print. Off the status timer: TOOL_LOAD runs for minutes and polls the
        ACE itself. The head goes back to the saved print position on resume."""
        self.logger.info(f"{self.name}: new spool in {lane_name} — loading it; "
                         "press resume when the purge is done")

        def run(eventtime: float) -> None:
            try:
                self.gcode.run_script(f"TOOL_LOAD LANE={lane_name}")
            except Exception as exc:
                self.logger.error(f"{self.name}: loading {lane_name} failed: {exc}. "
                                  "Load it by hand, then resume.")
        self.reactor.register_callback(run)

    def _apply_rfid(self, lane: AFCLane, info: Dict[str, Any], force: bool = False) -> None:
        """Apply a slot's RFID data (material type, color) to its lane.

        On a fresh insert the tag is ground truth (force=True, overwrite); at
        PREP only fill in blanks so user edits persisted in AFC.var win. The
        material setter derives filament density automatically. Callers
        publish the lane afterwards.
        """
        material = str(info.get("type") or "").strip()
        if material and (force or not lane.material):
            lane.material = material
        rgb = info.get("color")
        if (isinstance(rgb, (list, tuple)) and len(rgb) >= 3 and any(rgb)
                and (force or not lane.color)):
            lane.color = "#%02X%02X%02X" % tuple(int(c) & 0xFF for c in rgb[:3])

    def _publish_lane(self, lane: AFCLane) -> None:
        """Push lane data to moonraker's lane_data namespace (Orca's filament
        sync reads it). A moonraker hiccup must not fail the caller."""
        try:
            lane.send_lane_data()
        except Exception as exc:
            self.logger.debug(f"{self.name}: publishing {lane.name} lane data failed: {exc}")

    # ── shared-bowden tail state ────────────────────────────────────

    def _tail_in_bowden(self, lane: AFCLane) -> bool:
        """Whether a spent spool's tail sits unsensed below the hub sensor,
        so the next load must push it through the nozzle. Kept on the hub,
        not the unit: chained ACEs feed one bowden, and a tail left by one
        unit is the next unit's problem."""
        return bool(getattr(lane.hub_obj or self, "ace_tail_in_bowden", False))

    def _set_tail_in_bowden(self, lane: AFCLane, value: bool) -> None:
        setattr(lane.hub_obj or self, "ace_tail_in_bowden", value)

    def _path_lengths(self, lane: AFCLane) -> Tuple[float, float]:
        """(hub -> toolhead bowden, slot -> hub sensor) for a lane."""
        bowden = lane.hub_obj.afc_bowden_length if lane.hub_obj else DEFAULT_BOWDEN_MM
        return bowden, getattr(lane, "dist_hub", 0.0) or 0.0

    def _seed_spool_weight(self, lane: AFCLane) -> None:
        """A spool with weight 0 renders as an empty reel in the UIs (fill
        percent = weight/full) and disables AFC's weight-based runout logic.
        The ACE has no scale, so seed a freshly seen spool with the unit's
        full_weight default; user-set weights are never touched."""
        if not getattr(lane, "weight", 0):
            lane.weight = self.full_weight

    # ── AFC unit hooks ──────────────────────────────────────────────

    def prep_load(self, lane: AFCLane) -> None:
        pass  # ACE firmware stages filament at the slot on insert

    def prep_post_load(self, lane: AFCLane) -> None:
        pass

    def lane_move(self, cur_lane: AFCLane, distance: float, speed_mode: SpeedMode) -> None:
        """LANE_MOVE support: positive feeds toward the toolhead, negative
        retracts toward the spool, firmware-driven. A forward move never runs
        into the toolhead sensor: it is refused while the sensor shows
        filament, and otherwise stops at the sensor, at the approach speed,
        since the gears sit just past it."""
        if self._motion_blocked("LANE_MOVE"):
            return
        slot = self._slot_map.get(cur_lane.name)
        if slot is None:
            return
        tool_fn = self._toolhead_sensor_fn(cur_lane)
        if distance >= 0 and tool_fn is not None:
            if tool_fn():
                self.logger.error(f"LANE_MOVE refused: the toolhead sensor already shows "
                                  f"filament, {cur_lane.name} would feed into the gears")
                return
            ok, msg = self._feed_until(slot, tool_fn, distance, self.sensor_approach_speed,
                                       extend=False)
            if msg == NOT_REACHED:
                ok, msg = True, ""
            elif ok:
                self.logger.info(f"LANE_MOVE {cur_lane.name} stopped at the toolhead sensor")
        elif distance >= 0:
            ok, msg = self._feed_until(slot, None, distance, self.feed_speed)
        else:
            # An unwind past the slot runs the filament out of the ACE
            cap = self._retract_cap(cur_lane)
            if -distance > cap:
                self.logger.error(f"LANE_MOVE refused: {-distance:.0f}mm retract is over the "
                                  f"{cap:.0f}mm unload cap for {cur_lane.name}")
                return
            ok, msg = self._retract(slot, -distance, self.retract_speed)
        if not ok:
            self.logger.error(f"ACE lane_move failed for {cur_lane.name}: {msg}")

    def reset_lane(self, cur_lane: AFCLane) -> bool:
        """AFC_ACE_RESET and eject: retract a lane from wherever it is until
        the path sensor clears, then park it behind the merger. Never a full
        unwind: past the slot the ACE runs the filament out of the unit."""
        if self._motion_blocked("lane reset"):
            return False
        slot = self._slot_map.get(cur_lane.name)
        if slot is None:
            return False
        if cur_lane.tool_loaded:
            # The ACE cannot pull filament out through gripping extruder gears
            self.logger.error(f"{cur_lane.name} is loaded in the toolhead — "
                              "run TOOL_UNLOAD instead")
            return False
        self._operation_active = True
        # Any ACE operation invalidates a pending early feed
        self._early_feed = None
        try:
            # Filament at the path sensor may reach into the gears or the
            # hotend (an aborted load): heat, cut and pull it out of the
            # toolhead before the ACE retracts, as an unload does. Behind
            # the sensor nothing is in the toolhead.
            sensor = self._path_sensor(cur_lane)
            if sensor is not None and sensor[1]() and cur_lane.extruder_obj is not None:
                temp_state = self.afc.capture_toolhead_temp()
                try:
                    self.afc._check_extruder_temp(cur_lane)
                    ok, msg = self._clear_toolhead(cur_lane, cur_lane.extruder_obj)
                finally:
                    self.afc.restore_toolhead_temp(temp_state)
                if not ok:
                    self.logger.error(f"ACE reset failed for {cur_lane.name}: {msg}")
                    return False
            ok, msg = self._retract_clear_of_sensors(cur_lane, slot)
            if not ok:
                self.logger.error(f"ACE reset failed for {cur_lane.name}: {msg}")
                return False
            cur_lane.loaded_to_hub = False
            cur_lane._load_state = False
        finally:
            self._operation_active = False
        return True

    def get_lane_reset_command(self, lane: AFCLane, dis: float) -> str:
        """AFC_RESET hands stepperless lanes to this command. Note AFC_RESET
        refuses earlier when its hub reads clear, which for the ACE's virtual
        hub means "no lane tool-loaded" — so AFC_ACE_RESET is usually the
        command to run directly."""
        return f"AFC_ACE_RESET UNIT={self.name} LANE={lane.name}"

    def check_runout(self, cur_lane: AFCLane) -> bool:
        """Whether a load-switch runout on this lane should pause or swap
        (endless spool). AFC's base returns False, which turns every runout
        into a silent set_unloaded() while the print carries on. Same test as
        AFC_HTLF: the active lane, while printing, and not ejecting or
        calibrating."""
        return (cur_lane.name == self.afc.function.get_current_lane()
                and self.afc.function.is_printing()
                and cur_lane.status not in (AFCLaneState.EJECTING,
                                            AFCLaneState.CALIBRATING))

    def eject_lane(self, lane: AFCLane) -> None:
        """LANE_UNLOAD: the reset retract; the spool is then released with
        the slot lever."""
        slot = self._slot_map.get(lane.name)
        status = self._get_status() if slot is not None else None
        if (status is not None and "status" in self._slot_info(slot, status)
                and not self._slot_present(slot, status)):
            # A spent spool (a runout with unload_on_runout ejects it): the
            # ACE no longer grips its end, so there is nothing to retract,
            # as OpenAMS skips the hardware unload of an empty spool
            lane.loaded_to_hub = False
            lane._load_state = False
            self.logger.info(f"Lane {lane.name} is empty: nothing to retract. Remove "
                             f"the empty spool from {self.name}.")
            return
        if self.reset_lane(lane):
            self.logger.info(
                f"Lane {lane.name} retracted clear of the merger. Press the slot lever on "
                f"{self.name} to release and remove the spool.")

    # ── load ────────────────────────────────────────────────────────

    def lane_loading(self, lane: AFCLane) -> None:
        super().lane_loading(lane)
        self._early_feed = None
        # TOOL_LOAD calls this before it waits for the hotend: park and do
        # the ACE-only feed now, cold, so only the extruder part of the
        # load waits for the heat-up.
        if (lane.status == AFCLaneState.TOOL_LOADING
                and self.afc.current_loading == lane.name
                and not self._operation_active
                and lane.extruder_obj is not None
                and not self._tail_in_bowden(lane)):
            self._park(lane.extruder_obj)
            ok = self.unit_load_lane(lane, lane.extruder_obj, to_sensor=True)
            self._early_feed = (lane.name, ok)

    def unit_load_lane(self, cur_lane: AFCLane, cur_extruder: AFCExtruder,
                       to_sensor: bool = False) -> bool:
        """Full spool→nozzle load; replaces AFC's stepper load path. The
        caller then sets tool_loaded, enables the buffer and purges.
        to_sensor: stop after the ACE-only part, when the filament is at the
        toolhead sensor or the blind push past the frame sensor is done (the
        early feed from lane_loading); the next call engages the gears."""
        early, self._early_feed = self._early_feed, None
        resuming = early is not None and early[0] == cur_lane.name and not to_sensor
        if resuming and not early[1]:
            return False  # the early feed failed and has been reported
        if resuming:
            # Left over from a TOOL_LOAD that aborted before finishing? Only
            # a tip still at the path sensor is the early feed's
            sensor = self._path_sensor(cur_lane)
            resuming = sensor is not None and sensor[1]()
        if self._motion_blocked("TOOL_LOAD"):
            return False
        # Pre-flight: a lit path sensor at load start means untracked filament
        # occupies the bowden (e.g. tool state lost after a killed print) —
        # feeding into it would double-load. A spent tail is the one tracked
        # exception (the push-through branch consumes it). No auto-retract
        # here: the ACE may not even grip whatever is in the tube.
        if not resuming and not self._tail_in_bowden(cur_lane):
            sensor = self._path_sensor(cur_lane)
            if sensor and sensor[1]():
                self.afc.error.handle_lane_failure(
                    cur_lane,
                    f"Refusing to load {cur_lane.name}: {sensor[0]} sensor "
                    "already shows filament but no lane is tool-loaded. Clear the "
                    "path, or if filament is legitimately loaded run "
                    "SET_LANE_LOADED LANE=<lane>.",
                    pause=self.afc.function.in_print())
                return False
        self._operation_active = True
        # Any ACE operation invalidates a pending early feed
        self._early_feed = None
        self._gears_engaged = False
        try:
            # A resumed load is fed and still parked from the early feed
            if not self._load_inner(cur_lane, cur_extruder, to_sensor, fed=resuming):
                # Best-effort rescue: pull whatever was fed back clear of the
                # sensors and the merger, so the failure leaves a clean path.
                slot = self._slot_map.get(cur_lane.name)
                if slot is not None:
                    if self._assist_slot == slot:
                        # Armed once the gears had it; it would fight the
                        # retract
                        self._set_assist(slot, False)
                    self._stop_motion(slot, "stop_feed_filament")
                    ok, msg = True, ""
                    if self._gears_engaged:
                        # Filament reached the gears (the nozzle is hot by
                        # now): cut and pull it out before the ACE retracts
                        ok, msg = self._clear_toolhead(cur_lane, cur_extruder)
                    if ok:
                        ok, msg = self._retract_clear_of_sensors(cur_lane, slot)
                    if ok:
                        cur_lane.loaded_to_hub = False
                        cur_lane._load_state = False
                    else:
                        self.logger.error(f"ACE post-failure retract: {msg}")
                self.afc.error.handle_lane_failure(
                    cur_lane, f"ACE load failed for {cur_lane.name}",
                    pause=self.afc.function.in_print())
                return False
            return True
        finally:
            self._operation_active = False

    def _load_inner(self, cur_lane: AFCLane, cur_extruder: AFCExtruder,
                    to_sensor: bool = False, fed: bool = False) -> bool:
        slot = self._slot_map.get(cur_lane.name)
        if slot is None:
            self.logger.error(f"No ACE slot mapped for {cur_lane.name}")
            return False
        status = self._get_status()
        if status is not None and self._motion_active(slot, status):
            # The ACE's own insert preload: wait it out rather than fail the
            # load (an ACE 2's runs up to 15 s)
            self._wait_motion_idle(slot, PRELOAD_WAIT_S, settle=0.0)
            status = self._get_status()
        state = str(self._slot_info(slot, status).get("status", "")).lower()
        if state != "ready":
            self.logger.error(
                f"ACE slot {slot + 1} on {self.name} is not ready (status={state or 'unknown'})")
            return False

        # Path model (standard AFC semantics):
        #   slot --dist_hub--> [frame sensor] --afc_bowden_length--> gears
        # With the frame (hub) sensor the feed homes on it, then the bowden
        # is a blind push; with the toolhead sensor the feed homes on that.
        bowden, dist_hub = self._path_lengths(cur_lane)
        sensor = self._path_sensor(cur_lane)
        kind, sensor_fn = sensor if sensor else (None, None)

        cur_lane.status = AFCLaneState.TOOL_LOADING
        self.lane_loading(cur_lane)

        if self._tail_in_bowden(cur_lane):
            # A spent spool's tail occupies sensor->nozzle: push it through
            # the melt zone with the extruder in step (this also engages the
            # new filament — its tip ends at the nozzle), purging over the
            # tray.
            if kind == "hub" and not sensor_fn():
                ok, msg = self._home(slot, lambda: self._feed_until(
                    slot, sensor_fn, dist_hub + self.bowden_overshoot, self.feed_speed))
                if not ok:
                    self.logger.error(f"ACE feed to hub sensor failed for {cur_lane.name}: {msg}")
                    return False
            cur_lane.sync_to_extruder()
            if not self._tail_push(slot, cur_extruder,
                                   bowden + cur_extruder.tool_stn):
                return False
            self._set_tail_in_bowden(cur_lane, False)
            cur_lane.loaded_to_hub = True
            cur_lane._load_state = True
            self.afc.afcDeltaTime.log_with_time("Tail pushed through, new filament at nozzle")
            self._assist_after_load(slot)
            cur_lane.status = AFCLaneState.TOOL_LOADED
            self.afc.save_vars()
            return True

        if fed:
            pass  # the early feed already did the ACE-only part
        elif kind == "hub":
            # Home on the frame sensor, then the blind push to the gears
            if not sensor_fn():
                ok, msg = self._home(slot, lambda: self._feed_until(
                    slot, sensor_fn, dist_hub + self.bowden_overshoot, self.feed_speed))
                if not ok:
                    self.logger.error(f"ACE feed to hub sensor failed for {cur_lane.name}: {msg}")
                    return False
                self.afc.afcDeltaTime.log_with_time("ACE fed to hub sensor")
            ok, msg = self._feed_until(slot, None, bowden, self.feed_speed)
            if not ok:
                self.logger.error(f"ACE bowden push failed for {cur_lane.name}: {msg}")
                return False
        elif kind == "tool_start":
            if not sensor_fn():
                # The tip is parked by the last unload (hub_clear_mm behind
                # the sensor) or at the slot. The feed homes on the sensor
                # either way; its position is never assumed.
                ok, msg = self._home(slot, lambda: self._approach_sensor(
                    slot, sensor_fn, self.hub_clear_mm + self.sensor_approach_mm,
                    bowden + dist_hub))
                if not ok:
                    self.logger.error(
                        f"ACE feed to tool_start sensor failed for {cur_lane.name}: {msg}")
                    return False
                self.afc.afcDeltaTime.log_with_time("ACE fed to tool_start sensor")
        else:
            blind = bowden + dist_hub
            self.logger.info(f"{self.name}: no path sensor — blind feeding {blind:.0f}mm")
            ok, msg = self._feed_until(slot, None, blind, self.feed_speed)
            if not ok:
                self.logger.error(f"ACE blind feed failed: {msg}")
                return False
        cur_lane.loaded_to_hub = True
        cur_lane._load_state = True  # filament now occupies the hub path
        if to_sensor:
            return True

        # Toolhead engagement. The filament tip sits at the extruder inlet and
        # nothing grips it yet, so the extruder cannot pull it in alone: feed
        # the ACE in small chunks in step with the extruder (CosmoACE's
        # sync-load) until the gears have it: sync_load_mm of tool_stn. The
        # extruder then pulls the rest alone, with the ACE on feed assist
        # (print-time buffer mode: the ACE feeds on demand so the extruder
        # never fights the slot's rewind clutch; stays on while tool-loaded).
        # This extrudes through the hotend, so park over the purge area first.
        if not fed:
            self._park(cur_extruder)
        cur_lane.sync_to_extruder()
        sync_mm = cur_extruder.tool_stn
        if self.sync_load_mm is not None:
            sync_mm = min(self.sync_load_mm, sync_mm)
        ok, msg = self._sync_load(slot, cur_extruder, sync_mm)
        if not ok:
            self.logger.error(f"ACE sync load failed for {cur_lane.name}: {msg}")
            return False
        if cur_extruder.tool_stn > sync_mm:
            # The extruder pulls the rest alone, at once: assist on first
            self._set_assist(slot, True)
            self.afc.move_e_pos(cur_extruder.tool_stn - sync_mm,
                                cur_extruder.tool_load_speed, "tool stn", wait_tool=True)
        else:
            self._assist_after_load(slot)
        if cur_extruder.tool_end:
            attempts = 0
            while not cur_extruder.tool_end_state:
                attempts += 1
                if attempts > 20:
                    self.logger.error(
                        f"Filament failed to trigger post-gear sensor for {cur_lane.name}")
                    return False
                self.afc.move_e_pos(cur_lane.short_move_dis, cur_extruder.tool_load_speed,
                                    "Tool end", wait_tool=True)
        self.afc.afcDeltaTime.log_with_time("Filament loaded to nozzle")

        cur_lane.status = AFCLaneState.TOOL_LOADED
        self.afc.save_vars()
        return True

    # ── unload ──────────────────────────────────────────────────────

    def _clear_toolhead(self, cur_lane: AFCLane, cur_extruder: AFCExtruder) -> Tuple[bool, str]:
        """Get the filament out of the toolhead so the ACE can retract it:
        cut, then the extruder pulls it out of the gears. Never retract a
        lane from the toolhead without this: an uncut filament is held in
        the hotend and the ACE only winds against it. The nozzle must be hot
        (the caller heats). Used by the unload and by every recovery."""
        self.afc.move_e_pos(-2, cur_extruder.tool_unload_speed, "Quick Pull",
                            wait_tool=False)
        cur_lane.disable_buffer()
        cur_lane.sync_to_extruder()
        cur_lane.select_lane()
        self.afc.do_tool_cut_tip_form(cur_lane, cur_extruder)

        # Extruder-driven retract until the filament is out of the gears
        # (post-gear sensor clear). The ACE cannot pull through gripping
        # extruder gears. A pre-gear sensor is left for the ACE retract
        # to clear: once the tip is out of the gears the extruder cannot
        # move it any further.
        if cur_extruder.tool_stn_unload > 0:
            self.afc.move_e_pos(-cur_extruder.tool_stn_unload,
                                cur_extruder.tool_unload_speed, "tool stn unload",
                                wait_tool=True)
        attempts = 0
        while cur_extruder.tool_end_state:
            attempts += 1
            if attempts > cur_lane.tool_max_unload_attempts:
                cur_lane.unsync_to_extruder()
                return False, (f"Failed to clear toolhead sensors unloading {cur_lane.name}; "
                               "filament may be stuck in the toolhead")
            self.afc.move_e_pos(-cur_lane.short_move_dis,
                                cur_extruder.tool_unload_speed, "clear sensors",
                                wait_tool=True)
        # Same extra pull as AFC's own unload when a sensor sits after
        # the extruder gears
        if cur_extruder.tool_sensor_after_extruder > 0:
            self.afc.move_e_pos(-cur_extruder.tool_sensor_after_extruder,
                                cur_extruder.tool_unload_speed, "After extruder",
                                wait_tool=True)
        cur_lane.unsync_to_extruder()
        return True, ""

    def unit_unload_lane(self, cur_lane: AFCLane, cur_extruder: AFCExtruder) -> bool:
        """Full nozzle→slot unload; replaces AFC's stepper unload path."""
        if self._motion_blocked("TOOL_UNLOAD"):
            return False
        self._operation_active = True
        # Any ACE operation invalidates a pending early feed
        self._early_feed = None
        try:
            slot = self._slot_map.get(cur_lane.name)
            if slot is None:
                return False

            status = self._get_status(strict=True)
            if status is None:
                # Unknown is not empty: guessing "spent spool" here would
                # skip the cut and retract with the filament still loaded
                self.afc.error.handle_lane_failure(
                    cur_lane, f"Cannot unload {cur_lane.name}: no status from {self.name}",
                    pause=self.afc.function.in_print())
                return False
            # Spent spool: the slot is empty and only an ungripped tail
            # remains in the path — there is nothing to cut or retract.
            if not self._slot_present(slot, status):
                return self._tail_unload(cur_lane, slot)

            # Assist off FIRST — with assist on, the ACE fights every
            # backward move that follows (CosmoACE hardware lesson)
            self._set_assist(slot, False)

            cur_lane.status = AFCLaneState.TOOL_UNLOADING
            ok, msg = self._clear_toolhead(cur_lane, cur_extruder)
            if not ok:
                self.afc.error.handle_lane_failure(
                    cur_lane, msg, pause=self.afc.function.in_print())
                return False
            self.afc.afcDeltaTime.log_with_time("Unloaded from toolhead")

            # ACE-driven retract: pull the bowden empty and park the filament
            # behind the splitter.
            ok, msg = self._retract_clear_of_sensors(cur_lane, slot)
            if not ok:
                self.afc.error.handle_lane_failure(
                    cur_lane, f"ACE retract failed for {cur_lane.name}: {msg}",
                    pause=self.afc.function.in_print())
                return False
            # The spool is still in its slot and the tip parked behind the
            # merger: staged and loadable, as the status poll keeps it (a
            # load right after, a swap back, must not wait for the poll)
            cur_lane.loaded_to_hub = True
            cur_lane._load_state = False
            self.afc.afcDeltaTime.log_with_time("ACE retract complete")

            if self.afc.post_unload_macro is not None:
                self.gcode.run_script_from_command(self.afc.post_unload_macro)
            cur_lane.set_tool_unloaded(normal_toolchange=True)
            cur_lane.status = AFCLaneState.NONE
            self.lane_tool_unloaded(cur_lane)
            self.afc.save_vars()
            return True
        finally:
            self._operation_active = False

    def _tail_unload(self, cur_lane: AFCLane, slot: int) -> bool:
        """Book-keeping unload for a spent spool: the tail stays in the path
        (below the hub) and is pushed through the nozzle by the next load."""
        cur_lane.status = AFCLaneState.TOOL_UNLOADING
        cur_lane.disable_buffer()
        self._set_assist(slot, False)
        cur_lane.unsync_to_extruder()
        self._tail_pending.discard(cur_lane.name)
        # Only the frame (hub) sensor path leaves unsensed tail below the
        # sensor that the next load must push through; with a toolhead
        # sensor the normal load homing + sync-load consumes the remnant.
        sensor = self._path_sensor(cur_lane)
        if sensor and sensor[0] == "hub":
            self._set_tail_in_bowden(cur_lane, True)
        cur_lane.loaded_to_hub = False
        cur_lane._load_state = False
        if self.afc.post_unload_macro is not None:
            self.gcode.run_script_from_command(self.afc.post_unload_macro)
        cur_lane.set_tool_unloaded(normal_toolchange=True)
        cur_lane.status = AFCLaneState.NONE
        self.lane_tool_unloaded(cur_lane)
        self.afc.save_vars()
        self.logger.info(
            f"{cur_lane.name} spent-spool unload: tail left in bowden, "
            "next load pushes it through the nozzle")
        return True

    def _tail_push(self, slot: int, cur_extruder: AFCExtruder, total: float) -> bool:
        """Drive a spent tail through the melt zone: park over the purge
        area, then feed the ACE in chunks matched to extruder moves (the new
        filament pushes from behind while the extruder pulls the tail
        through), kicking the purge blob away after each chunk when AFC's
        kick is enabled. Like _sync_load, each chunk waits for the ACE to go
        idle before the next."""
        self._park(cur_extruder)
        self._gears_engaged = True
        speed = self.tail_purge_speed
        pushed = 0.0
        while pushed < total:
            step = min(100.0, total - pushed)
            # The ACE takes whole mm; a sub-mm remainder is the extruder's alone
            ace_mm = int(round(step))
            if ace_mm > 0:
                res = self._motion_rpc(slot, "feed_filament", {"index": slot, "length": ace_mm,
                                                                "speed": int(speed)})
                if not res.get("ok"):
                    self.logger.error(f"Tail push feed failed: {res.get('error')}")
                    return False
            self.afc.move_e_pos(step, speed, "tail push", wait_tool=True)
            if ace_mm > 0 and not self._wait_motion_idle(slot, step / speed + 5.0):
                self._stop_motion(slot, "stop_feed_filament")
                self.logger.error("Tail push: timeout waiting for ACE feed to finish")
                return False
            pushed += step
            if self.afc.kick and self.afc.kick_cmd:
                self.gcode.run_script_from_command(self.afc.kick_cmd)
        return True

    def _park(self, cur_extruder: AFCExtruder) -> None:
        """Park where a load may extrude: purge_park_cmd, else AFC's
        park_cmd. Every caller extrudes next."""
        cmd = self.purge_park_cmd or self.afc.park_cmd
        if self.afc.park and cmd:
            self.gcode.run_script_from_command(f"{cmd} EXTRUDER={cur_extruder.name}")

    def _retract_cap(self, cur_lane: AFCLane) -> float:
        """Longest unwind an unload or recovery may make: the sensor -> nozzle
        run (tool_stn), the merger (afc_unload_bowden_length, else
        afc_bowden_length) and the park. The path sensor clears well inside
        it; an unwind is never allowed to head on toward the slot, which runs
        the filament out of the ACE."""
        hub = cur_lane.hub_obj
        bowden = DEFAULT_BOWDEN_MM
        if hub is not None:
            bowden = getattr(hub, "afc_unload_bowden_length", 0) or hub.afc_bowden_length
        tool_stn = cur_lane.extruder_obj.tool_stn if cur_lane.extruder_obj else 0.0
        return bowden + tool_stn + self.hub_clear_mm

    def _retract_clear_of_sensors(self, cur_lane: AFCLane, slot: int) -> Tuple[bool, str]:
        """Unload / recovery retract: unwind until the path sensor
        clears, then park hub_clear_mm behind it, clear of the merger
        for the next lane (COSMOS parks a CANVAS lane the same way, its
        hub_clear_move_dis behind the toolhead sensor). A sensor that has not
        cleared within _retract_cap fails the move instead of unwinding on."""
        cap = self._retract_cap(cur_lane)
        sensor = self._path_sensor(cur_lane)
        if sensor is None:
            # Nothing to home against: the configured path, blind
            return self._retract(slot, cap, self.retract_speed)
        name, fn = sensor
        ok, msg = self._retract_until_clear(slot, name, fn, cap, self.hub_clear_mm)
        if not ok:
            return ok, msg
        if fn():
            return False, f"{name} sensor re-triggered after retract"
        return True, ""

    # ── PREP / system test ──────────────────────────────────────────

    def system_Test(self, cur_lane: AFCLane, delay: float, assignTcmd: bool,
                    enable_movement: bool) -> bool:
        msg = ''
        succeeded = True
        status = self._get_status()
        slot = self._slot_map.get(cur_lane.name, -1)

        if status is None:
            if self.optional:
                # Configured as possibly-absent: say so without dressing it up
                # as a failure, and let AFC's lane check pass.
                msg = 'ACE NOT CONNECTED (optional unit, lanes inactive)'
            else:
                msg = '<span class=error--text>ACE NOT CONNECTED</span>'
                if self.transport and self.transport.last_error:
                    msg += f' ({self.transport.last_error})'
                succeeded = False
        else:
            present = self._slot_present(slot, status)
            cur_lane.prep_state = present
            cur_lane._load_state = bool(cur_lane.tool_loaded)
            # Same rule as _sync_lane: a monitor_only unit never looks loadable
            cur_lane.loaded_to_hub = present and not self.monitor_only
            self._last_prep[slot] = present
            if present:
                self._seed_spool_weight(cur_lane)
                # Fill blanks from RFID; AFC.var-persisted user edits win
                self._apply_rfid(cur_lane, self._slot_info(slot, status))
                # Re-publish to moonraker's lane_data namespace — AFC wipes it
                # on every moonraker connect and units must push their lanes
                # back (Orca's filament sync reads this)
                self._publish_lane(cur_lane)
                self.lane_loaded(cur_lane)
                cur_lane.status = AFCLaneState.LOADED
                msg += "<span class=success--text>SPOOL READY</span>"
                if (cur_lane.tool_loaded
                        and cur_lane.extruder_obj.lane_loaded == cur_lane.name):
                    cur_lane.sync_to_extruder()
                    if self.afc.current == cur_lane.name:
                        self.afc.spool.set_active_spool(cur_lane.spool_id)
                        self.lane_tool_loaded(cur_lane)
                        cur_lane.status = AFCLaneState.TOOLED
                        cur_lane.enable_buffer()
                        # Print-time feed assist for the loaded lane: armed
                        # by the status poll (an ACE 2 only once it extrudes)
                        self._assist_slot, self._assist_gen = slot, None
                    else:
                        self.lane_tool_loaded_idle(cur_lane)
                    self.printer.send_event("afc:tool_loaded", cur_lane)
            else:
                if cur_lane.tool_loaded:
                    # Booted mid-tail: spool spent but its filament is still
                    # loaded — with a hub sensor the next load must push the
                    # unsensed tail through
                    sensor = self._path_sensor(cur_lane)
                    if sensor and sensor[0] == "hub":
                        self._set_tail_in_bowden(cur_lane, True)
                    msg += "<span class=warning--text>SPENT SPOOL (tail loaded) </span>"
                if not cur_lane.remember_spool:
                    self.afc.spool.clear_values(cur_lane)
                self.lane_not_ready(cur_lane)
                msg += 'EMPTY READY FOR SPOOL'

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)
        cur_lane.do_enable(False)
        # lane.map is a list of T commands since AFC's 2026-10 rebase (a list
        # can't take the :3 format); older AFC keeps a string and has no
        # map_to_string
        tcmd = getattr(cur_lane, "map_to_string", lambda: cur_lane.map)()
        self.logger.info('{lane_name} tool cmd: {tcmd:3} {msg}'.format(
            lane_name=cur_lane.name, tcmd=tcmd, msg=msg))
        cur_lane.set_afc_prep_done()
        return succeeded

    # ── calibration ─────────────────────────────────────────────────

    def calibrate_bowden(self, cur_lane: AFCLane, dis, tol) -> Tuple[bool, str, float]:
        """Nothing to measure: every load homes on the sensor. AFC's
        calibration menu unpacks (ok, message, distance); anything else is an
        exception in a gcode handler, which shuts Klipper down."""
        return False, self.calibration_lane_message(), 0.0

    def calibrate_lane(self, cur_lane: AFCLane, tol) -> Tuple[bool, str, float]:
        """AFC's all-lanes calibration lands here for non-direct hubs. There
        is nothing to calibrate per lane; "calibration_lane" makes AFC show
        calibration_lane_message instead."""
        return True, "calibration_lane", 0.0

    def calibration_lane_message(self) -> str:
        return ("ACE lanes are not calibrated by AFC: loads home on the path "
                "sensor. With the frame runout sensor, tune afc_bowden_length "
                "by hand (see the cosmoace_afc README).")

    # ── gcode commands ──────────────────────────────────────────────

    def set_dryer(self, temp: int, duration: Optional[int] = None,
                  fan_speed: Optional[int] = None) -> None:
        """Start (temp > 0) or stop (temp <= 0) the ACE dryer. Only explicit
        commands reach the stop branch; see AceDryerHeater."""
        if temp > 0:
            temp = min(temp, DRYER_MAX_C)
            res = self._rpc("drying", {"temp": temp,
                                       "fan_speed": fan_speed or self.dryer_fan_speed,
                                       "duration": duration or self.dryer_duration})
            if res.get("ok"):
                self.dryer_heater.target = float(temp)
                self.logger.info(f"{self.name} dryer set to {temp}C "
                                 f"for {duration or self.dryer_duration} minutes")
            else:
                self.logger.error(f"{self.name} dryer start failed: {res.get('error')}")
        else:
            # A unit that is not plugged in has no dryer running, so there is
            # nothing to stop and nothing to report.
            if self.transport is None or not self.transport.connected:
                self.dryer_heater.target = 0.0
                self.logger.debug(f"{self.name} dryer stop skipped, unit not connected")
                return
            res = self._rpc("drying_stop")
            if res.get("ok"):
                self.dryer_heater.target = 0.0
                self.logger.info(f"{self.name} dryer stopped")
            else:
                # The unit answered before, so the dryer may well still be
                # running: worth saying, but not a fault to fail a print on.
                self.logger.warning(f"{self.name} dryer stop failed: {res.get('error')}")

    def _cmd_SET_DRYER_TEMPERATURE(self, gcmd: GCodeCommand) -> None:
        self.set_dryer(int(gcmd.get_float("TARGET", 0.0)))

    def cmd_AFC_ACE_DRYER_START(self, gcmd: GCodeCommand) -> None:
        self.set_dryer(gcmd.get_int("TEMP", self.dryer_temp),
                       gcmd.get_int("DURATION", self.dryer_duration),
                       gcmd.get_int("FAN_SPEED", self.dryer_fan_speed))

    def cmd_AFC_ACE_DRYER_STOP(self, gcmd: GCodeCommand) -> None:
        self.set_dryer(0)

    def cmd_AFC_ACE_STATUS(self, gcmd: GCodeCommand) -> None:
        status = self._get_status()
        if status is None:
            err = self.transport.last_error if self.transport else "transport not started"
            self.logger.info(f"{self.name}: not connected ({err})")
        else:
            self.logger.info(f"{self.name}: {json.dumps(status, indent=1)}")

    def cmd_AFC_ACE_STOP(self, gcmd: GCodeCommand) -> None:
        lane_name = gcmd.get("LANE")
        slot = self._slot_map.get(lane_name)
        if slot is None:
            self.logger.error(f"Unknown lane {lane_name}")
            return
        self._stop_motion(slot, "stop_feed_filament")
        self._stop_motion(slot, "stop_unwind_filament")
        if self._assist_slot == slot:
            self._set_assist(slot, False)
        else:
            # Another lane's print-time assist stays wanted
            self._stop_motion(slot, "stop_feed_assist")

    def cmd_AFC_ACE_RESET(self, gcmd: GCodeCommand) -> None:
        lane = self.lanes.get(gcmd.get("LANE"))
        if lane is None or lane.name not in self._slot_map:
            self.logger.error(f"Unknown lane {gcmd.get('LANE')} on {self.name}")
            return
        if self.reset_lane(lane):
            self.logger.info(f"{lane.name} reset: parked behind the merger")

    def cmd_AFC_ACE_PANEL(self, gcmd: GCodeCommand) -> None:
        action = gcmd.get("ACTION").lower()
        if action not in ("install", "remove"):
            raise gcmd.error("ACTION must be INSTALL or REMOVE")
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "webui", "setup-panel.sh")
        # Run in the background and poll from a timer: the script restarts
        # the web UI init script, which klippy must not block on
        proc = subprocess.Popen(["sh", script, action], stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT)
        gcmd.respond_info(f"AFC_ACE_PANEL: {action} started")

        def check(eventtime):
            if proc.poll() is None:
                return eventtime + 0.5
            out = proc.stdout.read().decode(errors="replace").strip()
            msg = f"AFC_ACE_PANEL {action}: {out or 'exit ' + str(proc.returncode)}"
            if proc.returncode:
                self.logger.error(msg)
            else:
                self.logger.info(msg)
            self.reactor.unregister_timer(timer)
            return self.reactor.NEVER
        timer = self.reactor.register_timer(check, self.reactor.monotonic() + 0.5)

    # ── status for UI ───────────────────────────────────────────────

    def get_status(self, eventtime=None):
        response = super().get_status(eventtime)
        status, _ = self.transport.cached_status() if self.transport else (None, 0.0)
        response["connected"] = bool(self.transport and self.transport.connected)
        if status is not None:
            response["ace"] = {
                "temp": status.get("temp"),
                "humidity": status.get("humidity"),
                "dryer": status.get("dryer_status") or status.get("dryer"),
                "slots": status.get("slots"),
            }
        return response


# [AFC_ACE] ace: -> unit protocol
ACE_MODELS = {"pro": "ace1", "2pro": "ace2"}
# units: auto, ACE Pro: how long the startup count watches USB
COUNT_WINDOW_S = 1.5


def _inherit_setup_defaults(config: ConfigWrapper) -> None:
    """Give a unit section every key it does not set itself from [AFC_ACE],
    written into its own section so AFC's unit base class and Klipper's
    option check see them as the unit's. `ace` becomes protocol; unit_index
    and optional stay per unit, and so does an ACE Pro's serial (each unit
    has its own port; ACE 2 units share one bus). Without an [AFC_ACE]
    section nothing changes."""
    fc, section = config.fileconfig, config.get_name()
    if not fc.has_section("AFC_ACE"):
        return
    if not fc.has_option(section, "protocol") and fc.has_option("AFC_ACE", "ace"):
        model = fc.get("AFC_ACE", "ace").strip().lower()
        if model in ACE_MODELS:
            fc.set(section, "protocol", ACE_MODELS[model])
    protocol = fc.get(section, "protocol").strip() if fc.has_option(section, "protocol") else "ace1"
    per_unit = {"ace", "units", "unit_index", "optional"}
    if protocol == "ace1":
        per_unit.add("serial")
    for option in fc.options("AFC_ACE"):
        if option not in per_unit and not fc.has_option(section, option):
            fc.set(section, option, fc.get("AFC_ACE", option))


def count_ace_units(protocol: str, serial_port: str) -> int:
    """How many ACEs are attached now, for `units: auto` at startup."""
    if protocol == "ace2":
        bus = Ace2Bus(serial_port)
        try:
            return len(bus._discover_uids(3)) if bus._connect() else 0
        finally:
            bus._disconnect()
    # An idle ACE Pro drops off USB for ~0.5 s (watchdog re-enumeration), so
    # one look can miss a unit; looking every 0.25 s for three gaps' worth
    # sees each one. Klipper's startup waits for this, so no longer.
    seen: set = set()
    deadline = time.monotonic() + COUNT_WINDOW_S
    while True:
        seen.update(scan_ace_ports())
        if time.monotonic() >= deadline:
            return len(seen)
        time.sleep(0.25)


class AceSetup:
    """[AFC_ACE]: one unit (AFC_ACE ACE_1, ACE_2, ...) per ACE, each with its
    four lanes. `ace: pro|2pro` picks the model, `units: auto` counts the
    ACEs attached at startup (or a number, fixed); every other key is a
    default for all units. A unit section of your own ([AFC_ACE ACE_1]
    serial: ...) overrides keys for that unit."""

    def __init__(self, config: ConfigWrapper) -> None:
        self.printer = config.get_printer()
        protocol = config.getchoice("ace", ACE_MODELS, "pro")
        units = config.get("units", "auto").strip().lower()
        serial_port = config.get("serial", "auto")
        if units == "auto":
            found = count_ace_units(protocol, serial_port)
            logging.info(f"AFC_ACE: {found} ACE unit(s) found at startup")
            count = max(found, 1)
        else:
            try:
                count = int(units)
            except ValueError:
                raise config.error("[AFC_ACE] units must be auto or a number, not %r" % units)
            if count < 1:
                raise config.error("[AFC_ACE] units must be at least 1")
        # Every key here is read by the units; read them once here too, or
        # Klipper's option check rejects them as unused in [AFC_ACE]
        for option in config.fileconfig.options(config.get_name()):
            config.get(option)
        fc = config.fileconfig
        for number in range(1, count + 1):
            section = f"AFC_ACE ACE_{number}"
            if not fc.has_section(section):
                fc.add_section(section)
            defaults = [("unit_index", str(number - 1))]
            # A chained unit unplugged at startup is reported, not a failure
            if number > 1:
                defaults.append(("optional", "True"))
            # [AFC_ACE] serial on an ACE Pro is the first unit's port
            if number == 1 and protocol == "ace1" and fc.has_option("AFC_ACE", "serial"):
                defaults.append(("serial", fc.get("AFC_ACE", "serial")))
            for key, value in defaults:
                if not fc.has_option(section, key):
                    fc.set(section, key, value)
            self.printer.load_object(config, section)


def load_config(config: ConfigWrapper) -> AceSetup:
    return AceSetup(config)


def load_config_prefix(config: ConfigWrapper) -> afcACE:
    unit = afcACE(config)
    # Registered before its lanes are built: an AFC lane loads its unit
    # object in its own constructor (AFC_lane._get_steppers), and an
    # unregistered unit would be built a second time
    unit.printer.add_object(config.get_name(), unit)
    unit._create_lanes(config)
    return unit
