# Test only (Rust-AFC parity runs): TEST_BUTTON PIN=<pin> STATE=0|1 hands
# each button group on the pin's MCU the buttons_state message that MCU sends
# when that pin changes. Batch mode gets no MCU input, so filament sensors
# never change otherwise; from the message on everything runs as on a printer
# (the ack to the MCU, debounce, filament switches, AFC's callbacks).
# PIN is spelled as the config spells it (an alias stays an alias).
#
# With follow_homing: True, each homing move flips the buttons on its
# endstops' pins as it starts: on a printer the sensor the move homes to
# changes state during the move (and in batch every homing move triggers at
# full distance); the hook then waits FOLLOW_TIME so the switch callbacks
# have run by the time the move ends.
# The native counterpart is src/test_button.rs (kaliru-afc only).
#
# This file may be distributed under the terms of the GNU GPLv3 license.


FOLLOW_TIME = 0.1


class TestButton:
    def __init__(self, config):
        self.printer = config.get_printer()
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("TEST_BUTTON", self.cmd_TEST_BUTTON)
        if config.getboolean("follow_homing", False):
            self.printer.register_event_handler(
                "homing:homing_move_begin", self._handle_homing_move_begin
            )

    def _press(self, mcu, pin, state):
        # state None flips the pin's current state
        reactor = self.printer.get_reactor()
        found = False
        groups = [
            cb.__self__
            for (name, oid), cb in mcu._serial.handlers.items()
            if name == "buttons_state"
        ]
        for mb in groups:
            bits = 0
            for i, (p, _) in enumerate(mb.pin_list):
                if p == pin:
                    bits |= 1 << i
            if not bits:
                continue
            found = True
            if state is None:
                on = not (mb.last_button & bits)
            else:
                on = state
            logical = (mb.last_button & ~bits) | (bits if on else 0)
            raw = logical ^ mb.invert
            mb.handle_buttons_state(
                {
                    "ack_count": mb.ack_count & 0xFF,
                    "state": bytes([raw & 0xFF]),
                    "#receive_time": reactor.monotonic(),
                }
            )
        return found

    def _handle_homing_move_begin(self, hmove):
        pressed = False
        for es in hmove.get_mcu_endstops():
            pin = getattr(es, "_pin", None)
            if pin is not None:
                pressed |= self._press(es.get_mcu(), pin, None)
        if pressed:
            # the move's duration on a printer: batch homing never waits, so
            # without it the switch callbacks would only run after the move
            reactor = self.printer.get_reactor()
            reactor.pause(reactor.monotonic() + FOLLOW_TIME)

    def cmd_TEST_BUTTON(self, gcmd):
        pin = gcmd.get("PIN")
        state = gcmd.get_int("STATE", minval=0, maxval=1)
        ppins = self.printer.lookup_object("pins")
        pin_params = ppins.parse_pin(pin, True, True)
        if not self._press(pin_params["chip"], pin_params["pin"], state):
            raise gcmd.error("TEST_BUTTON: no button on %s" % (pin,))


def load_config(config):
    return TestButton(config)
