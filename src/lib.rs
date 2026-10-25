// Rust-AFC: AFC (Armored Turtle's Automated Filament Control) as a native
// kaliru plugin. A distribution builds its kaliru binary with PLUGIN; the
// plugin runs for configs whose [AFC...] sections it covers
// (afc/scope.rs), and other configs keep the python AFC.
pub mod afc;

use kaliru::configfile::ConfigWrapper;
use kaliru::error::KResult;
use kaliru::native_plugin::NativePlugin;
use kaliru::printer::PrinterModule;

const fn module(name: &'static str, load_config: Option<kaliru::printer::LoadFn>, load_config_prefix: Option<kaliru::printer::LoadFn>) -> PrinterModule {
    PrinterModule { name, ported: true, load_config, load_config_prefix, register_components: None }
}

const MODULES: &[PrinterModule] = &[
    module("AFC", Some(afc::load_config), None),
    module("AFC_ACE", Some(afc::ace::load_config), Some(afc::ace::load_config_prefix)),
    module("AFC_canvas", None, Some(afc::canvas::load_config_prefix_unit)),
    module("AFC_canvas_lane", None, Some(afc::canvas::load_config_prefix_lane)),
    module("AFC_error", Some(afc::error::load_config), None),
    module("AFC_extruder", None, Some(afc::extruder::load_config_prefix)),
    module("AFC_functions", Some(afc::functions::load_config), None),
    module("AFC_hub", None, Some(afc::hub::load_config_prefix)),
    module("AFC_lane", None, Some(afc::lane::load_config_prefix)),
    module("AFC_prep", Some(afc::prep::load_config), None),
    module("AFC_spool", Some(afc::spool::load_config), None),
];

/// whether the native AFC runs this config (else the python AFC does)
fn active(config: &ConfigWrapper) -> KResult<bool> {
    Ok(afc::native_scope(config)?.is_ok())
}

pub const PLUGIN: NativePlugin = NativePlugin { name: "Rust-AFC", modules: MODULES, active };
