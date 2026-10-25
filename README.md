# Rust-AFC

[AFC](https://github.com/ArmoredTurtle/AFC-Klipper-Add-On) (Armored Turtle's Automated Filament Control) as a
native plugin for [kaliru](https://github.com/Rusti3D/kaliru), the Rust port of Kalico. It is a port, not a
rewrite: the goal is the same behaviour as the Python AFC, checked command by command against it, with
kaliru's lower CPU and memory use (no Python interpreter on the printer for AFC).

Credit where it's due: AFC is Armored Turtle's work. Please support the upstream project.

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
| AFC core, prep, hubs, extruders (no toolchanger / buffer options), lanes of the units below, spool, error, stats; Elegoo Canvas (`AFC_canvas`, needs COSMOS's drv8833 module); Anycubic ACE Pro / ACE 2 Pro ([cosmoace_afc](https://github.com/Rusti3D)'s `AFC_ACE`) | stepper lanes (BoxTurtle, NightOwl, QuattroBox, HTLF, Claymore, EMU, ViViD), buffers, LEDs, buttons, form_tip, poop, Toolchanger, OpenAMS / OAMS |

Ported against AFC 1.2.7 as COSMOS ships it. The plan and progress are in [STATUS.md](STATUS.md) and
[docs/PLAN.md](docs/PLAN.md).

## Parity testing

`tools/afc_cmp.sh` runs each `tests/cases/*.test` with the Python AFC on Kalico and with `kaliru-afc` (kaliru
plus this plugin, `src/main.rs`) in batch mode and compares them: the messages sent to the MCUs byte for
byte, and klippy.log with only wall-clock and thread-timing values masked (`tools/refdiff.py`). Units with
serial devices run against fakes (`tools/fake_ace.py`, an ACE Pro on a pseudo-terminal). Batch mode has no MCU
replies, so motor moves and sensor changes beyond what fakes provide need real hardware.

```sh
cargo build --release
bash tools/afc_cmp.sh            # needs Kalico, a CPython 3.15 venv and the AFC sources: see the script
```

## License

GPL-3.0-only, as AFC and Kalico.
