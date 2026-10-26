# Test only (Rust-AFC parity runs). Batch mode gets no MCU input:
#   TEST_MCU_MSG MCU=<mcu> NAME=<message> [DELAY=<s>] <field>=<int> ...
# hands the handler registered for an MCU message (oams_cmd_stats,
# oams_action_status, ...) that message's fields, as the MCU would send it;
# DELAY delivers it from a reactor timer that many seconds later, so it
# reaches a command that waits on it (a load waiting for the firmware's
# action status). Field names are given upper-cased by gcode and passed
# lower-cased.
# A query is never answered in batch mode (its send waits for an ack that
# never comes). [test_mcu_msg] answer: lines,
#   <mcu> <query command> -> <response> <field>=<int> ...
# and TEST_MCU_ANSWER MCU= QUERY= RESPONSE= <field>=<int> ... make the query's
# send deliver that response and return.
# The native counterpart is src/test_mcu_msg.rs (kaliru-afc only).
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import mcu

OWN_PARAMS = ("MCU", "NAME", "DELAY", "QUERY", "RESPONSE")


def _fields(items):
    return {key.lower(): int(value) for key, value in items}


class TestMcuMsg:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.answers = {}
        for line in config.get("answer", "").split("\n"):
            if not line.strip():
                continue
            query_part, response_part = line.split("->")
            mcu_name, query = query_part.split()
            words = response_part.split()
            fields = _fields(w.split("=", 1) for w in words[1:])
            self._set_answer(mcu_name, query, words[0], fields)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("TEST_MCU_MSG", self.cmd_TEST_MCU_MSG)
        gcode.register_command("TEST_MCU_ANSWER", self.cmd_TEST_MCU_ANSWER)

    def _set_answer(self, mcu_name, query, response, fields):
        if mcu_name not in self.answers:
            self.answers[mcu_name] = {}
            self._install(mcu_name)
        self.answers[mcu_name][query] = (response, fields)

    def _install(self, mcu_name):
        serial = mcu.get_printer_mcu(self.printer, mcu_name)._serial
        orig = serial.raw_send_wait_ack
        answers = self.answers[mcu_name]

        def raw_send_wait_ack(cmd, minclock, reqclock, cmd_queue):
            for query, (response, fields) in answers.items():
                if list(cmd) == list(serial.msgparser.create_command(query)):
                    handler = serial.handlers.get((response, None))
                    if handler is not None:
                        handler(dict(fields))
                    serial.raw_send(cmd, minclock, reqclock, cmd_queue)
                    return {}
            return orig(cmd, minclock, reqclock, cmd_queue)

        serial.raw_send_wait_ack = raw_send_wait_ack

    def _own_fields(self, gcmd):
        return _fields((k, v) for k, v in gcmd.get_command_parameters().items()
                       if k not in OWN_PARAMS)

    def cmd_TEST_MCU_ANSWER(self, gcmd):
        self._set_answer(gcmd.get("MCU", "mcu"), gcmd.get("QUERY"),
                         gcmd.get("RESPONSE"), self._own_fields(gcmd))

    def cmd_TEST_MCU_MSG(self, gcmd):
        mcu_obj = mcu.get_printer_mcu(self.printer, gcmd.get("MCU", "mcu"))
        name = gcmd.get("NAME")
        delay = gcmd.get_float("DELAY", 0.0, minval=0.0)
        params = self._own_fields(gcmd)
        handler = mcu_obj._serial.handlers.get((name, None))
        if handler is None:
            raise gcmd.error("TEST_MCU_MSG: no handler for %s" % (name,))
        if not delay:
            handler(params)
            return
        reactor = self.printer.get_reactor()

        def deliver(eventtime):
            handler(params)
            return reactor.NEVER

        reactor.register_timer(deliver, reactor.monotonic() + delay)


def load_config(config):
    return TestMcuMsg(config)
