#!/usr/bin/env python3
"""
attiny_led.py - a small library for controlling the ATtiny412 LED over I2C.

Wraps the raw smbus2 register reads/writes from the console script behind a
high-level interface, so you can just do:

    from attiny_led import ATtinyLED

    led = ATtinyLED()              # opens I2C bus 1, address 0x10
    led.led_control("on")          # "on" | "off" | "toggle"
    print(led.read_state())        # current LED-state byte
    print(led.is_on())             # True / False
    led.blink(5)
    led.close()

Or as a context manager (auto-closes the bus):

    with ATtinyLED() as led:
        led.led_control("toggle")

Needs: pip3 install smbus2
"""

import threading
from smbus2 import SMBus

# ── defaults that must match the firmware ────────────────────────────────
DEFAULT_ADDR = 0x10
DEFAULT_BUS = 1

# register map
REG_CMD, REG_STATE = 0x00, 0x01
REG_PIN, REG_WHO = 0x03, 0x7E

# commands
CMD_OFF, CMD_ON, CMD_TOGGLE, CMD_BLINK = 0x00, 0x01, 0x02, 0x03

# friendly strings used by led_control() -> firmware command
_LED_COMMANDS = {
    "off": CMD_OFF,
    "on": CMD_ON,
    "toggle": CMD_TOGGLE,
}


class ATtinyLED:
    """High-level control of the LED on an ATtiny412 over I2C."""

    def __init__(self, addr=DEFAULT_ADDR, bus=DEFAULT_BUS, verify=False):
        self.addr = addr
        self._bus = SMBus(bus)
        self._lock = threading.Lock()
        if verify:
            self.signature()  # raises OSError if the chip doesn't answer

    # ── low-level helpers ────────────────────────────────────────────────
    def _read(self, reg):
        with self._lock:
            return self._bus.read_byte_data(self.addr, reg)

    def _write(self, reg, value, *extra):
        with self._lock:
            if extra:
                self._bus.write_i2c_block_data(self.addr, reg, [value, *extra])
            else:
                self._bus.write_byte_data(self.addr, reg, value)

    # ── high-level LED control ───────────────────────────────────────────
    def led_control(self, option):
        """Set the LED. `option` is "on", "off", or "toggle".

        Returns the LED-state byte read back after the command.
        Raises ValueError on an unrecognised option.
        """
        try:
            cmd = _LED_COMMANDS[option.strip().lower()]
        except (KeyError, AttributeError):
            raise ValueError(
                f"unknown LED option {option!r}; "
                f"use one of: {', '.join(_LED_COMMANDS)}"
            ) from None
        self._write(REG_CMD, cmd)
        return self.read_state()

    # convenience wrappers, if you prefer named calls over strings
    def on(self):
        return self.led_control("on")

    def off(self):
        return self.led_control("off")

    def toggle(self):
        return self.led_control("toggle")

    def blink(self, times=3):
        """Blink the LED `times` times (clamped to 0-255). Returns the count sent."""
        n = int(times) & 0xFF
        self._write(REG_CMD, CMD_BLINK, n)
        return n

    # ── reads ────────────────────────────────────────────────────────────
    def read_state(self):
        """Return the raw LED-state byte reported by the firmware."""
        return self._read(REG_STATE)

    def is_on(self):
        """True if the LED-state byte is non-zero."""
        return bool(self.read_state())

    def read_pin(self):
        """Return the current level of physical pin 3 as 'HIGH' / 'LOW'."""
        return "HIGH" if self._read(REG_PIN) else "LOW"

    def signature(self):
        """Return the firmware signature byte (REG_WHO)."""
        return self._read(REG_WHO)

    # ── lifecycle ────────────────────────────────────────────────────────
    def close(self):
        self._bus.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


if __name__ == "__main__":
    # quick smoke test: cycle the LED and read back state each time
    with ATtinyLED(verify=True) as led:
        print(f"signature = 0x{led.signature():02X}")
        for opt in ("on", "off", "toggle", "toggle"):
            state = led.led_control(opt)
            print(f"{opt:<6} -> state={state} ({'on' if state else 'off'})")
