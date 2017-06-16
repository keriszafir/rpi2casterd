# -*- coding: utf-8 -*-
"""WiringPi output backend"""

import wiringpi


class WiringPiOutput:
    """A 32-channel control interface based on two MCP23017 chips"""
    name = 'MCP23017 driver using wiringPi2-Python library'
    pin_base = 65

    def __init__(self, signals, mcp0_address, mcp1_address, _):
        # Set up an output interface on two MCP23017 chips
        wiringpi.mcp23017Setup(self.pin_base, mcp0_address)
        wiringpi.mcp23017Setup(self.pin_base + 16, mcp1_address)
        # map signals to outputs
        signal_numbers = [*range(self.pin_base, self.pin_base+32)]
        self.mapping = dict(zip(signals, signal_numbers))
        # update the pin base for possible additional interfaces
        WiringPiOutput.pin_base += 32
        # Set all I/O lines on MCP23017s as outputs - mode=1
        for pin in self.mapping.values():
            wiringpi.pinMode(pin, 1)

    def valves_on(self, signals):
        """Looks a signal up in arrangement and turns it on"""
        for sig in signals:
            pin_number = self.mapping.get(sig)
            if not pin_number:
                continue
            wiringpi.digitalWrite(pin_number, 1)

    def valves_off(self):
        """Looks a signal up in arrangement and turns it off"""
        for pin in self.mapping.values():
            wiringpi.digitalWrite(pin, 0)
