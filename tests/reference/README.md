# Parity reference

`AFC_ACE/` is the Python AFC_ACE unit (cosmoace_afc, GPL-3.0) that the native ACE in `src/afc/ace.rs` and
`ace_transport.rs` follows, line for line: cosmoace_afc auto-units a58d89f with the runout work done here
(reload after a runout pause, empty-slot eject, an unloaded lane stays loadable, AFC_ACE_STOP assist,
humidity get_temp). `tools/afc_cmp.sh` runs it as the reference; `tools/afc_codes.py` reads its AFC.log
frame data from `AFC_ACE.py`. A change to the native ACE is made here first, then ported.
