# Rust-AFC status — 2026-10-26 (later)
## Goal
AFC (Armored Turtle's Automated Filament Control, the version COSMOS ships, 1.2.7) as a native, built-in
kaliru plugin: a crate a distro compiles into its kaliru binary, enabled by the user's `[AFC...]` config
sections, behaving exactly as the Python AFC (user: "full port", all of AFC including the ACE unit).
Python AFC on kaliru's plugin layer stays the fallback until the port is complete.

## State
- Native: AFC core, Canvas, ACE (6 ACE cases), and since 2026-10-26 milestone M1: COSMOS's complete
  BoxTurtle setup (tests/configs/cosmos/boxturtle.cfg: AFC_stepper lanes, AFC_BoxTurtle (+ NightOwl,
  QuattroBox), espoolers (src/afc/assist.rs), AFC_led (src/afc/led.rs), switched AFC_buffer
  (src/afc/buffer.rs)). Proven native: bt_toolchange runs with libpython unavailable.
- Since then: M2 (AFC_button, AFC_form_tip, AFC_poop) and from M3 EMU, ViViD, Claymore, HTLF (src/afc/
  vivid.rs, htlf.rs: AFC_lane lanes on the unit's drive stepper, the cam on the selector stepper; a drive
  stepper's homing runs the lane's assist, as python swaps assist_move).
- Parity: `tools/afc_cmp.sh` 24/24 identical, three runs; clippy 0 (Rust-AFC and kaliru).
  BoxTurtle cases: bt_basic / bt_toolchange (complete config), btnobuf_toolchange (no buffer: espoolers,
  LEDs), btmin_* (no LEDs, buffer or espoolers: commands, moves, insert, tool load, unload).
- Harness: TEST_BUTTON injects the MCU's buttons_state (test plugin, both sides); `follow_homing` flips the
  sensor a homing move homes to, as it starts; `WAIT n` paces input (tools/refrun.py); test_sensors.cfg
  holds what batch mode needs ([AFC_prep] enable: False, SET_KINEMATIC_POSITION via force_move).
- Fixes found by the new cases (also matter for Canvas/ACE): the sensor hooks' DebounceButton was dropped
  (hook held a Weak) so the toolhead/hub switches never saw a change; prep_callback never released the lane
  mutex; buttons registered after the espooler's pins (MCU oid order); Python int defaults kept
  (empty_spool_weight, full_weight, spool_ratio, timer_delay, delta_movement, assist_max_motor_rpm);
  gcode_get_value returns the default unchanged when the parameter is absent.
- kaliru 702ecb0 (pushed, rustify gate matches): buttons groups API, Endstop::mcu_pin, LEDHelper
  set_color / check_transmit / led_count public, configfile deprecate message "Value '...'".
- EMU, ViViD, Claymore, HTLF configs are made for the tests (tests/configs/*_unit.cfg: none ships with
  COSMOS); their lanes are calibrated (a first ViViD load rewrites config files).

## Queue
- [ ] Toolchanger + FPS_PSF (committed 8593bfa + 9953e5a, NOT pushed; kaliru b6e867d pushed): one full gate
  27/27 identical, clippy 0; two more runs going. Then push.
- [ ] OpenAMS / OAMS (uncommitted: src/afc/oams.rs, openams.rs, unit/lane/functions/scope wiring,
  calibrate_td1 returns a PyVal distance (python's int 0 printed "DISTANCE=0")). Harness: TEST_MCU_MSG
  (tests/plugins/test_mcu_msg.py, src/test_mcu_msg.rs) delivers an MCU message to its handler, DELAY from a
  timer; tests/dicts/oams.dict from tools/make_oams_dict.py (response types assumed); kaliru needs
  Mcu::is_shutdown + SerialReader::response_handler (uncommitted in kaliru). Case: openams_flow.
  Next: afc_cmp.sh copies tests/dicts/*.dict to /ci_build/dict; build; run openams_flow; a runout case.
- [ ] Snapmaker U1 options; klipper-toolchanger `tool` objects (expand kaliru as needed).
- [ ] Rusticc: next image picks up Rust-AFC (user decides when).

## Next
1. Push the toolchanger/FPS commits once the repeat runs match; then the OpenAMS batch above.

## Verify
`bash tools/afc_cmp.sh` (WSL, ~10 min for 14 cases); cargo clippy --release with CARGO_TARGET_DIR=~/rk/target-rafc
under the shared lock /tmp/rustify_gates.lock.

## Known, not changed
- Without native coverage the Python fallback needs kaliru's plugin layer: Python AFC_stepper fails there
  ("klippy.kinematics.extruder.ExtruderStepper.__init__ is not mirrored yet"), so stepper configs outside the
  native scope do not start (kaliru plugin-layer coverage).
- Not native (scope.rs keeps Python): a buffer as tool_start; no pin_tool_start + pin_tool_end + a buffer
  (calibration sets tool_start to "buffer"); LED indexes on Canvas/ACE lanes (untested).
- AFC_ACE _motion_active reads the unit's "busy" as the slot's: with two slots of one ACE moving, a lost
  reply on one is taken as started. Rare in use; kept as the python AFC has it.
- A real-pin [AFC_hub] with runout off: the break check reaches AFC's handle_toolhead_runout, which reads
  fila_tool_end (None without pin_tool_end): logged, no pause. The shipped configs use a virtual hub.
- ACE 2, units: auto on a RESTART may count 0 while the old bus still holds the cable (exclusive open).
- Batch mode: after any extruder move idle_timeout stays "Printing" (no MCU clock), so a spool insert after a
  tool load is refused ("actively moving") and so are buttons; the toolhead retract polls its sensor between
  plain moves, so cases clear it by TEST_BUTTON first; homing always runs its full distance (HTLF's cam
  then "fails to home" on its first step, both sides alike).

## Traps
- No motion on the user's CC1 without the user present; coordinate with the Moonruster session.
- AFC.log frame data `[file:func():line]` comes from the Python sources (afc_codes.py): regenerate, never type.
- SET_ESPOOLER_VALUES (and other AFC commands) with values rewrite the .cfg files in AFC's config dir: never in
  a test case.
- Shell heredocs mangle backslashes in Python edit scripts: write such scripts with the Write tool.
