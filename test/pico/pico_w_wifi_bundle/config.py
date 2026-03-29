"""Configuration for Pico W direct Wi-Fi control firmware."""

# Wi-Fi mode: "AP" (recommended, Pico creates hotspot) or "STA" (join router)
WIFI_MODE = "AP"

# AP mode settings
AP_SSID = "AutoNexa-PicoW"
AP_PASSWORD = "autonexa123"  # 8+ chars required for WPA2
AP_IP = "192.168.4.1"

# STA mode settings (used only when WIFI_MODE == "STA")
STA_SSID = ""
STA_PASSWORD = ""
STA_TIMEOUT_MS = 15000

# HTTP API port used by the mobile app in MicroPython mode
HTTP_PORT = 5001
