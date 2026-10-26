// kaliru-afc: kaliru with Rust-AFC, for the parity runs (tools/afc_cmp.sh); it
// also has the test-only TEST_BUTTON (src/test_button.rs) and TEST_MCU_MSG
// (src/test_mcu_msg.rs)
mod test_button;
mod test_mcu_msg;

fn main() {
    kaliru::native_plugin::main(&[rust_afc::PLUGIN, test_button::PLUGIN, test_mcu_msg::PLUGIN]);
}
