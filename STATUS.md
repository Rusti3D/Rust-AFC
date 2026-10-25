# Rust-AFC status — 2026-10-25
## Goal
AFC (Armored Turtle's Automated Filament Control, the version COSMOS ships, 1.2.7) as a native, built-in
kaliru plugin: a crate a distro compiles into its kaliru binary, enabled by the user's `[AFC...]` config
sections, behaving exactly as the Python AFC (user: "full port", all of AFC including the ACE unit).
Python AFC on kaliru's plugin layer stays the fallback until the port is complete.

## State
- Native AFC exists inside Rusticc's kaliru patches 0021 (COSMOS's stock Canvas) and 0022 (cosmoace_afc
  ACE units); parity: Canvas 9/9, ACE 4/4 identical to Python AFC (rusticc-distro tools/canvas_cmp.sh,
  tools/ace_cmp.sh). It runs on the user's CC1 in Rusticc 0.0.1.
- kaliru d0f37ad has the native plugin API (docs/PLUGINS.md "Native plugins"); its gate unchanged
  (cases 151/152, suite 87/88 - printers/13 timing as before - plugins 8/8, clippy 0).
- This crate: the native AFC from Rusticc (src/afc), builds against kaliru d0f37ad from GitHub, clippy 0.
  `tools/afc_cmp.sh`: ACE cases 4/4 identical to the python AFC on Kalico (status/dryer, lane moves/reset,
  offline loads/unloads/tool change, spool events + bypass). The Canvas (needs COSMOS's drv8833) is tested
  in Rusticc (canvas_cmp.sh).

- ACE runout (2026-10-25), following tests/reference/AFC_ACE (the python unit, kept here): reload of a paused runout lane (2f2abc9) armed only by a pause and dropped on resume, works
  with unload_on_runout off; eject of a spent spool skips the empty-slot unwind (as OpenAMS); an unloaded
  lane stays loadable; AFC_ACE_STOP keeps another lane's feed assist; the humidity sensor has get_temp
  (TEMPERATURE_WAIT); failed Moonraker requests log before their callback runs (utils.rs). Parity 6/6,
  twice: tools/afc_cmp.sh; ace_runout(_keep) run runout -> pause -> new
  spool -> load -> resume; tools/fake_moonraker.py gives AFC its stats so loads run to the end.
- config/ace: the ACE configs as the user's CC1 runs them (cc1-toolhead.cfg).

## Queue
- [x] 1. kaliru main: native plugin registration (a distro binary = kaliru + chosen plugin crates) and the
  AFC patches' kaliru-side hooks as public extension points. Evidence: kaliru gate unchanged.
- [x] 2. Rust-AFC crate: the existing native AFC (Canvas, ACE) moved here, with its parity harness.
  Evidence: ACE 4/4 here; Canvas 9/9 is re-checked in step 3 (Rusticc).
- [x] 3. Rusticc builds kaliru + Rust-AFC; patches 0021/0022 dropped. Evidence: Rusticc gate unchanged
  (cases 171/172, cc1 1/1, canvas 9/9, ace 4/4, plugin_cmp 8/8); installed on the user's CC1 (Rusticc
  3904616fee, Rust-AFC 3ea26b1): native AFC, 4/4 ACE lanes SPOOL READY, no libpython loaded.
- [ ] 4. The rest of AFC, module by module (docs/PLAN.md), each with parity scenarios against Python AFC.

## Next (step 4)
1. Port module by module from docs/PLAN.md's table, Python AFC 1.2.7 (Originals_Base/addons/afc_extras) as the
   reference: AFC_stepper and the lane / unit / extruder / assist parts the stepper units use first (BoxTurtle
   family), then buffer, led, button, form_tip, poop; then HTLF, Claymore, EMU, ViViD, Toolchanger, OpenAMS.
   native_scope (src/afc/scope.rs) widens with each one; anything outside it keeps running as Python AFC.
2. Each with parity cases in tests/cases (fake hardware where a unit has a serial device, fake_moonraker for
   loads), tools/afc_cmp.sh identical, then Rusticc's gate.
3. kaliru gap to fix on the way: chelper.get_ffi() is not mirrored (AFC_stepper's python path needs it).
4. Untested on hardware: the ACE runout -> pause -> new spool -> load -> resume flow; watch the first one.

## Known, not changed
- AFC_ACE _motion_active reads the unit's "busy" as the slot's: with two slots of one ACE moving, a lost
  reply on one is taken as started. Rare in use; kept as the python AFC has it.
- A real-pin [AFC_hub] with runout off: the break check reaches AFC's handle_toolhead_runout, which reads
  fila_tool_end (None without pin_tool_end): logged, no pause. The shipped configs use a virtual hub.
- ACE 2, units: auto on a RESTART may count 0 while the old bus still holds the cable (exclusive open).

## Traps
- No motion on the user's CC1 without the user present; coordinate with the Moonruster session.
- AFC.log frame data `[file:func():line]` comes from the Python sources (afc_codes.py): regenerate, never type.
