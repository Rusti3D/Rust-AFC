// kaliru-afc: kaliru with Rust-AFC, for the parity runs (tools/afc_cmp.sh); it
// also has the test-only TEST_BUTTON (src/test_button.rs)
mod test_button;

fn main() {
    kaliru::native_plugin::main(&[rust_afc::PLUGIN, test_button::PLUGIN]);
}
