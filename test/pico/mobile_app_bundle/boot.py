"""
Boot file for Pico mobile-app MicroPython testing.

- Keeps USB console/data available when supported.
- Blinks LED quickly at boot so you know firmware started.
"""

import time
import machine

try:
    import usb_cdc  # Available on newer MicroPython builds
    usb_cdc.enable(console=True, data=True)
except Exception:
    # Safe fallback for builds without usb_cdc module
    pass

led = machine.Pin(25, machine.Pin.OUT)
for _ in range(3):
    led.toggle()
    time.sleep_ms(120)
led.value(0)
