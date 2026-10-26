#!/usr/bin/env python3
# Writes tests/dicts/oams.dict: the stm32h723 test dictionary plus the
# OpenAMS messages, for the parity runs' [mcu oams] (no OpenAMS firmware
# dictionary is at hand). The commands are exactly the formats AFC_OAMS.py
# looks up and sends; the responses' field names are the ones it reads, their
# types are assumed (both sides of a parity run use the same dictionary, and
# batch mode only ever sees responses that TEST_MCU_MSG delivers).
# usage: make_oams_dict.py [<stm32h723.dict>]
import json
import pathlib
import sys

COMMANDS = [
    "config_oams_buffer upper=%u lower=%u is_reversed=%u",
    "config_oams_f1s_hes on1=%u on2=%u on3=%u on4=%u is_above=%u",
    "config_oams_hub_hes on1=%u on2=%u on3=%u on4=%u is_above=%u",
    "config_oams_pid kp=%u ki=%u kd=%u target=%u",
    "config_oams_ptfe length=%u",
    "config_oams_current_pid kp=%u ki=%u kd=%u target=%u",
    "config_oams_logger idx=%u",
    "oams_cmd_load_spool spool=%c",
    "oams_cmd_unload_spool",
    "oams_cmd_follower enable=%c direction=%c",
    "oams_cmd_calibrate_ptfe_length spool=%c",
    "oams_cmd_calibrate_hub_hes spool=%c",
    "oams_cmd_pid kp=%u ki=%u kd=%u target=%u",
    "oams_set_led_error idx=%c value=%c",
    "oams_cmd_load_spool_cancel",
    "oams_cmd_query_spool",
]
RESPONSES = [
    "oams_query_response_spool spool=%u",
    "oams_action_status action=%c code=%c value=%u",
    "oams_cmd_stats fps_value=%u f1s_hes_value_0=%u f1s_hes_value_1=%u"
    " f1s_hes_value_2=%u f1s_hes_value_3=%u hub_hes_value_0=%u"
    " hub_hes_value_1=%u hub_hes_value_2=%u hub_hes_value_3=%u"
    " encoder_clicks=%i",
    "oams_cmd_current_status current_value=%u",
]


def main():
    src = pathlib.Path(sys.argv[1] if len(sys.argv) > 1
                       else "/ci_build/dict/stm32h723.dict")
    d = json.loads(src.read_text())
    next_id = max(list(d["commands"].values())
                  + list(d["responses"].values())) + 1
    for fmt in COMMANDS:
        d["commands"][fmt] = next_id
        next_id += 1
    for fmt in RESPONSES:
        d["responses"][fmt] = next_id
        next_id += 1
    out = pathlib.Path(__file__).resolve().parent.parent / "tests/dicts/oams.dict"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(d, separators=(",", ":")))
    print(out)


if __name__ == "__main__":
    main()
