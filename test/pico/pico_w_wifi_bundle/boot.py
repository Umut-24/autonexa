"""Boot file for Pico W direct Wi-Fi control."""

import machine
import time

led = machine.Pin(25, machine.Pin.OUT)
for _ in range(3):
    led.toggle()
    time.sleep_ms(120)
led.value(0)
