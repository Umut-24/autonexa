import network
import time

def connect(ssid, password, timeout=15):
    """
    Connect the Pico W to a WiFi network.
    
    Returns the assigned IP address string, or None on failure.
    Blinks the onboard LED during connection attempts.
    """
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if wlan.isconnected():
        ip = wlan.ifconfig()[0]
        print(f"[WiFi] Already connected: {ip}")
        return ip
    
    print(f"[WiFi] Connecting to '{ssid}'...")
    wlan.connect(ssid, password)
    
    start = time.ticks_ms()
    led_state = False
    
    while not wlan.isconnected():
        elapsed = time.ticks_diff(time.ticks_ms(), start)
        if elapsed > timeout * 1000:
            print("[WiFi] Connection timed out!")
            wlan.active(False)
            return None
        
        # Blink LED to show connecting status
        led_state = not led_state
        _set_led(led_state)
        time.sleep_ms(300)
    
    # Solid LED on = connected
    _set_led(True)
    
    ip = wlan.ifconfig()[0]
    print(f"[WiFi] Connected! IP: {ip}")
    print(f"[WiFi] Subnet: {wlan.ifconfig()[1]}")
    print(f"[WiFi] Gateway: {wlan.ifconfig()[2]}")
    return ip


def _set_led(state):
    """Set the onboard LED on Pico W (uses CYW43 driver)."""
    try:
        import machine
        led = machine.Pin("LED", machine.Pin.OUT)
        led.value(1 if state else 0)
    except Exception:
        pass


def blink_ip(ip):
    """
    Blink the last octet of the IP address on the LED.
    E.g., IP 192.168.1.42 → blinks 4 times, pause, 2 times.
    Useful when no serial console is available.
    """
    last_octet = ip.split('.')[-1]
    for digit_char in last_octet:
        digit = int(digit_char)
        if digit == 0:
            digit = 10  # blink 10 times for 0
        for _ in range(digit):
            _set_led(True)
            time.sleep_ms(200)
            _set_led(False)
            time.sleep_ms(200)
        # Pause between digits
        time.sleep_ms(800)
    
    # End pattern: rapid triple blink
    for _ in range(3):
        _set_led(True)
        time.sleep_ms(100)
        _set_led(False)
        time.sleep_ms(100)
    
    _set_led(True)  # Leave LED on
