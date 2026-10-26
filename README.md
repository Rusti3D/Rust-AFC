# Rust-AFC

[AFC](https://github.com/ArmoredTurtle/AFC-Klipper-Add-On) (Armored Turtle's Automated Filament Control) as a
native plugin for [kaliru](https://github.com/Rusti3D/kaliru), the Rust port of Kalico. It is a port, not a
rewrite: the goal is the same behaviour as the Python AFC, checked command by command against it, with
kaliru's lower CPU and memory use (no Python interpreter on the printer for AFC).

Credit where it's due: AFC is Armored Turtle's work. Please support the upstream project.

> **Runs on real printers:** Rust-AFC is built into [Rusticc](https://github.com/Rusti3D/rusticc) (release
> 0.0.2), where it drives an Elegoo Centauri Carbon's ACE Pro natively. Early days: please report issues here.

## Getting it

- **Elegoo Centauri Carbon:** install [Rusticc](https://github.com/Rusti3D/rusticc); Rust-AFC is built in.
- **Other printers:** the [releases](https://github.com/Rusti3D/Rust-AFC/releases) have `kaliru-afc`
  (x86_64 and armv7 Linux, glibc): [kaliru](https://github.com/Rusti3D/kaliru) with AFC built in. Run it in
  place of Kalico's `python klippy.py` (same command line and config), with the AFC add-on installed as usual
  for its macros and configs; AFC then runs natively.

## How it plugs in

kaliru's native plugins are built in: a distribution builds its kaliru binary with the plugins it wants
(kaliru `docs/PLUGINS.md`, "Native plugins"):

```rust
fn main() {
    kaliru::native_plugin::main(&[rust_afc::PLUGIN]);
}
```

Users then use AFC exactly as with the Python add-on: the same `[AFC]`, `[AFC_hub]`, unit and lane config
sections, macros and commands. A config the port does not cover yet keeps running the Python AFC on kaliru's
plugin layer, unchanged.

## Status

| covered natively | still Python AFC |
|---|---|
| AFC core, prep, hubs, extruders, lanes, spool, error, stats; stepper lanes (`AFC_stepper`) of BoxTurtle, NightOwl, QuattroBox, EMU; selector units (ViViD, Claymore, HTLF); Toolchanger (`AFC_Toolchanger`, toolhead LEDs); OpenAMS / OAMS (`AFC_OpenAMS`, `AFC_OAMS`); espoolers (N20 assist motors); `AFC_led`; switched (TurtleNeck) and FPS_PSF `AFC_buffer`; `AFC_button`, `AFC_form_tip`, `AFC_poop`; Elegoo Canvas (`AFC_canvas`, needs COSMOS's drv8833 module); Anycubic ACE Pro / ACE 2 Pro ([cosmoace_afc](https://github.com/Rusti3D)'s `AFC_ACE`) | klipper-toolchanger `tool` objects on extruders; Snapmaker U1 options; a buffer as the toolhead sensor |

Ported against AFC 1.2.7 as COSMOS ships it. The plan and progress are in [STATUS.md](STATUS.md) and
[docs/PLAN.md](docs/PLAN.md).

## ACE configs

`config/ace/` holds the configs of [cosmoace_afc](https://github.com/shawn-makes-stuff/cosmoace_afc) (Centauri
Carbon 1 and 2, runout-sensor or toolhead-sensor path) as they run on a CC1 today. A distribution ships them
next to its AFC base config (Rusticc: `extras-readonly/afc-ace/`); `printer.cfg` includes the entry file for
the machine and path, e.g. `[include extras-readonly/afc-ace/cc1-toolhead.cfg]`.

## Parity testing

`tools/afc_cmp.sh` runs each `tests/cases/*.test` with the Python AFC on Kalico and with `kaliru-afc` (kaliru
plus this plugin, `src/main.rs`) in batch mode and compares them: the messages sent to the MCUs byte for
byte, and klippy.log with only wall-clock and thread-timing values masked (`tools/refdiff.py`). Units with
serial devices run against fakes (`tools/fake_ace.py`, an ACE Pro on a pseudo-terminal), and
`tools/fake_moonraker.py` answers AFC's Moonraker client (AFC creates its stats on connecting, and every load
and unload updates them). Batch mode has no MCU input, so sensors change through a test-only
`TEST_BUTTON PIN= STATE=` (`tests/plugins/test_button.py` / `src/test_button.rs`, the buttons_state message
the MCU would send), optionally following the homing moves (`follow_homing`: the sensor a lane homes to
changes during the move), and `WAIT <secs>` lines pace the input so AFC reacts as on an idle printer.

```sh
cargo build --release
bash tools/afc_cmp.sh            # needs Kalico, a CPython 3.15 venv and the AFC sources: see the script
```

## License

GPL-3.0-only, as AFC and Kalico.
