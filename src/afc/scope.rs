// Which AFC runs: the native one when the config is inside what it ports
// (COSMOS's stock Canvas setup, cosmoace_afc's ACE units), else the python
// AFC exactly as before
use kaliru::configfile::ConfigWrapper;
use kaliru::error::KResult;

/// The AFC modules the native AFC replaces (klippy/extras names)
pub const NATIVE_MODULES: [&str; 11] = ["AFC", "AFC_functions", "AFC_spool", "AFC_error", "AFC_prep", "AFC_hub", "AFC_extruder", "AFC_canvas", "AFC_canvas_lane", "AFC_lane", "AFC_ACE"];

/// Ok(()) when every AFC section is one the native AFC ports, else why not
pub fn native_scope(config: &ConfigWrapper) -> KResult<Result<(), String>> {
    let fc = config.fileconfig.clone();
    let sections = fc.borrow().sections();
    // [danger_options] use_python_modules: AFC (or one of its modules)
    if sections.iter().any(|s| s == "danger_options") {
        if let Some(list) = config.getsection("danger_options").get_opt("use_python_modules")? {
            if list.split([',', '\n', ' ']).any(|m| m.trim().starts_with("AFC")) {
                return Ok(Err("use_python_modules".into()));
            }
        }
    }
    let opt = |section: &str, option: &str| -> Option<String> {
        let fc = fc.borrow();
        if fc.has_option(section, option) {
            fc.get(section, option).ok()
        } else {
            None
        }
    };
    let mut afc = false;
    for s in &sections {
        let module = s.split_whitespace().next().unwrap_or("");
        if !module.starts_with("AFC") {
            continue;
        }
        if !NATIVE_MODULES.contains(&module) {
            return Ok(Err(format!("[{}]", s)));
        }
        match module {
            "AFC" => {
                afc = true;
                if opt(s, "form_tip_cmd").as_deref() == Some("AFC") {
                    return Ok(Err("form_tip_cmd AFC".into()));
                }
            }
            "AFC_extruder" => {
                for o in ["toolchanger_unit", "tool", "u1_filament_sensor_name", "u1_park_detector_name", "led_name", "status_led_idx", "nozzle_led_idx", "buffer"] {
                    if opt(s, o).is_some() {
                        return Ok(Err(format!("[{}] {}", s, o)));
                    }
                }
                if let Some(p) = opt(s, "pin_tool_start") {
                    if p == "buffer" || p.to_lowercase() == "virtual" {
                        return Ok(Err(format!("[{}] pin_tool_start {}", s, p)));
                    }
                }
            }
            "AFC_lane" => {
                // an ACE lane: one AFC_ACE builds, or the override of one
                // (no unit line); lanes of other units stay python
                match opt(s, "unit") {
                    None if sections.iter().any(|x| x == "AFC_ACE" || x.starts_with("AFC_ACE ")) => {}
                    None => return Ok(Err(format!("[{}] without unit", s))),
                    Some(u) => {
                        let unit = u.split(':').next().unwrap_or("").trim().to_string();
                        let ace_unit = sections.iter().any(|x| *x == format!("AFC_ACE {}", unit));
                        let auto_unit = sections.iter().any(|x| x == "AFC_ACE") && unit.starts_with("ACE_");
                        if !ace_unit && !auto_unit {
                            return Ok(Err(format!("[{}] unit {}", s, unit)));
                        }
                    }
                }
                for o in ["buffer", "selector", "step_pin", "led_index", "led_spool_index"] {
                    if opt(s, o).is_some() {
                        return Ok(Err(format!("[{}] {}", s, o)));
                    }
                }
            }
            "AFC_canvas" | "AFC_canvas_lane" => {
                for o in ["buffer", "selector", "step_pin", "afc_motor_rwd", "afc_motor_fwd", "afc_motor_enb", "led_index", "led_logo_index"] {
                    if opt(s, o).is_some() {
                        return Ok(Err(format!("[{}] {}", s, o)));
                    }
                }
            }
            _ => {}
        }
    }
    if !afc {
        return Ok(Err("no [AFC]".into()));
    }
    Ok(Ok(()))
}
