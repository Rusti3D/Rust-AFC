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

## Queue
- [x] 1. kaliru main: native plugin registration (a distro binary = kaliru + chosen plugin crates) and the
  AFC patches' kaliru-side hooks as public extension points. Evidence: kaliru gate unchanged.
- [x] 2. Rust-AFC crate: the existing native AFC (Canvas, ACE) moved here, with its parity harness.
  Evidence: ACE 4/4 here; Canvas 9/9 is re-checked in step 3 (Rusticc).
- [ ] 3. Rusticc builds kaliru + Rust-AFC; patches 0021/0022 dropped. Evidence: Rusticc gate unchanged.
- [ ] 4. The rest of AFC, module by module (docs/PLAN.md), each with parity scenarios against Python AFC.

## Next (step 3)
1. rusticc-distro: a small binary crate (kaliru with Rusticc's patches + rust-afc, Cargo [patch] so
   rust-afc's kaliru is the patched tree); build_cc1.sh / the recipe build it.
2. Rusticc patches: drop 0021/0022 (now kaliru d0f37ad + this crate); new: drv8833 answers call_method
   (drv8833_move, drv8833_set_speed, active) for the Canvas (src/afc/canvas.rs Drv8833); refit 0018/0019
   (queuelogger) on kaliru's send(msg, sync).
3. Evidence: Rusticc gate (cases, cc1, canvas_cmp 9/9, ace_cmp 4/4) unchanged.

## Harness ideas
- A minimal fake Moonraker (HTTP): Python AFC creates afc_stats on moonraker connect; batch runs without
  it stop at the first TOOL_LOAD / UNLOAD's stats (AttributeError), cutting tool-change coverage short.

## Traps
- No motion on the user's CC1 without the user present; coordinate with the Moonruster session.
- AFC.log frame data `[file:func():line]` comes from the Python sources (afc_codes.py): regenerate, never type.
