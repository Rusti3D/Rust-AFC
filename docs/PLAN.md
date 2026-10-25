# Rust-AFC plan

## How it plugs in (decided with the user, 2026-10-25)
Built-in, configurable plugins: Rust-AFC is a crate depending on kaliru as a library. A distro (Rusticc,
or anyone building on kaliru) makes its binary from kaliru plus the plugin crates it wants; a plugin only
does something when the user's config has its sections, as with Kalico's own modules. No runtime-loaded
native plugins (Rust has no stable ABI; a C / Lua / WASM boundary only if demand appears). Python plugins
on kaliru's plugin layer remain the fallback.

kaliru stays plugin-agnostic: it exposes a registration API (the plugin's printer modules, and which Python
modules a plugin replaces for a given config) and public extension points (filament sensor runout hooks and
the like). Nothing AFC-specific goes into kaliru.

## Working method (Rusti3D manifesto)
Understand the Python module first; parity scenarios against the Python AFC are the baseline; every module
lands with its scenarios identical (klippy.log and AFC.log masked only for wall clock / thread timing, the
serialqueue trace byte-identical); report what was tested and what could not be (batch mode has no MCU
replies: moves on DC motors and sensor changes need fakes or hardware).

## Modules (Python lines, AFC 1.2.7 as COSMOS ships it)
| done in Rusticc (move) | to port |
|---|---|
| AFC 3232, AFC_functions 2162, AFC_lane 2554 (partly), AFC_unit 1156 (partly), AFC_extruder 1237 (partly), AFC_hub 268, AFC_prep 315, AFC_error 400, AFC_spool 981, AFC_stats 483, AFC_assist 923 (stub), AFC_utils 998, AFC_logger 253, AFC_respond 110, AFC_canvas 305, AFC_canvas_lane 614, AFC_ACE 2999 | AFC_stepper 929, AFC_buffer 1593, AFC_led 167, AFC_button 139, AFC_form_tip 225, AFC_poop 114, AFC_BoxTurtle 655, AFC_NightOwl 39, AFC_QuattroBox 60, AFC_HTLF 313, AFC_Claymore 128, AFC_EMU 117, AFC_vivid 391, AFC_Toolchanger 309, AFC_OAMS 1506, AFC_OpenAMS 3852; the parts of lane / unit / extruder / assist the Canvas and ACE scope left out |

Order: shared pieces first (stepper, the rest of lane / unit / extruder / assist, buffer, led, button,
form_tip, poop), then units by how common they are (BoxTurtle family, HTLF, Claymore, EMU, ViViD,
Toolchanger, OpenAMS / OAMS).
