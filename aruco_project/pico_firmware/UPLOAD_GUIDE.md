# Pico WH Firmware Upload Guide
## How to Upload the Ackermann Chassis Code to Your Pico WH

---

## What You Need

1. **Raspberry Pi Pico WH** (the one with WiFi + headers pre-soldered)
2. **Micro-USB cable** (data cable, not charge-only!)
3. **PC** with Windows
4. **Thonny IDE** (free, download from [thonny.org](https://thonny.org))

---

## Step 1: Install MicroPython on Pico WH (One-Time Setup)

If your Pico already has MicroPython, skip to Step 2.

1. **Download the MicroPython firmware**:
   - Go to: https://micropython.org/download/RPI_PICO_W/
   - Download the latest `.uf2` file (e.g., `RPI_PICO_W-20241025-v1.24.0.uf2`)

2. **Put Pico into bootloader mode**:
   - **Hold the BOOTSEL button** on the Pico (the small white button)
   - While holding it, **plug the USB cable** into your PC
   - Release the button after plugging in
   - A new drive called **"RPI-RP2"** should appear in File Explorer

3. **Flash MicroPython**:
   - Drag and drop the `.uf2` file onto the **RPI-RP2** drive
   - The Pico will **reboot automatically** — the drive will disappear
   - MicroPython is now installed! ✅

---

## Step 2: Set Up Thonny IDE

1. **Download and install** Thonny from https://thonny.org
2. Open Thonny
3. Go to **Tools → Options → Interpreter**
4. Select **"MicroPython (Raspberry Pi Pico)"** from the dropdown
5. For port, select the COM port that appeared (e.g., `COM3`)
   - If you don't see it, try unplugging and re-plugging the USB cable
6. Click **OK**
7. You should see `>>>` in the **Shell** panel at the bottom — this is the MicroPython REPL

> **Tip:** If Thonny doesn't detect the Pico, make sure you're using a **data** USB cable (not a charge-only cable). Charge-only cables don't have data wires inside.

---

## Step 3: Edit WiFi Credentials

Before uploading, **edit `config.py`** with your WiFi details:

```python
# Open this file in any text editor:
# c:\aruco_project\pico_firmware\config.py

# Change these two lines (lines 8-9):
WIFI_SSID = "YourWiFiName"          # ← Put your WiFi network name
WIFI_PASSWORD = "YourWiFiPassword"   # ← Put your WiFi password
```

Save the file.

---

## Step 4: Upload All Files to the Pico

You need to upload **6 files** to the Pico's root filesystem:

```
Files to upload (from c:\aruco_project\pico_firmware\):
├── main.py            ← Main program (runs on boot)
├── config.py          ← Configuration (WiFi, pins, etc.)
├── wifi_manager.py    ← WiFi connection handler
├── motor.py           ← I2C motor driver interface
├── servo.py           ← Steering servo controller
└── ackermann.py       ← Ackermann steering math
```

### Upload Method: Using Thonny (Recommended)

For **each file**, do this:

1. In Thonny, go to **File → Open**
2. Navigate to `c:\aruco_project\pico_firmware\`
3. Select the file (e.g., `config.py`)
4. Click **Open** — the file opens in the editor
5. Go to **File → Save As...**
6. A dialog box appears asking **"Where to save to?"**
   - Click **"Raspberry Pi Pico"** (NOT "This Computer")
7. Type the **exact same filename** (e.g., `config.py`)
8. Click **OK**

**Repeat for all 6 files.** The order doesn't matter, but make sure all filenames are exactly as listed above.

### Alternative: Using the File Manager in Thonny

1. Go to **View → Files** (this opens a file panel)
2. In the top panel, navigate to `c:\aruco_project\pico_firmware\`
3. Right-click on each `.py` file → **"Upload to /"**
4. Do this for all 6 files

---

## Step 5: Verify Files Are on the Pico

In Thonny's **Shell** (bottom panel), type:

```python
import os
print(os.listdir())
```

You should see:
```
['main.py', 'config.py', 'wifi_manager.py', 'motor.py', 'servo.py', 'ackermann.py']
```

If all 6 files are listed, you're good! ✅

---

## Step 6: First Run (Test)

### Option A: Run manually (recommended for first time)
1. In Thonny, open `main.py` from the Pico
2. Click the **green ▶ Run** button (or press F5)
3. Watch the Shell for output:

```
========================================
 Ackermann Chassis Controller v2.0
 (I2C Motor Driver Edition)
========================================
[WiFi] Connecting to 'YourWiFiName'...
[WiFi] Connected! IP: 192.168.1.42      ← Note this IP!
[WiFi] Subnet: 255.255.255.0
[WiFi] Gateway: 192.168.1.1
[Init] Setting up motor driver (I2C)...
[Motor] Board found at I2C address 0x34  ← Motor board OK!
[Motor] Motor type set to 2 (520 DC gear motor)
[Init] Setting up steering servo (GP15)...
[Init] Setting up Ackermann controller...
[UDP] Starting server on 192.168.1.42:4210
[Ready] Waiting for commands...
[Ready] Send UDP to 192.168.1.42:4210
```

4. **Write down the IP address** — you'll enter it in the phone app

### Option B: Auto-run on boot
Once everything works, `main.py` will **automatically run on every power-up** because MicroPython always executes `main.py` on boot.

Just power the Pico with any USB power source (power bank, etc.) and it will:
1. Connect to WiFi
2. Start the UDP server
3. Wait for joystick commands from the app

---

## Step 7: Connect the Phone App

1. Install the APK on your Android phone:
   ```
   c:\aruco_project\mobile_app\build\app\outputs\flutter-apk\app-release.apk
   ```
2. Open the **AutoNexa** app
3. Go to the **Control** tab (gamepad icon)
4. Enter the **Pico's IP address** (e.g., `192.168.1.42`)
5. Port: `4210` (default)
6. Tap **"Link"**
7. Move the joystick — the car should respond! 🚗

---

## Troubleshooting

### "Board NOT found at 0x34"
- Check I2C wiring: SDA → GP0, SCL → GP1, GND → GND, 5V → VBUS
- Verify the motor driver board is powered
- Run an I2C scan in the Shell:
  ```python
  from machine import Pin, I2C
  i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)
  print([hex(addr) for addr in i2c.scan()])
  ```
  This shows all detected I2C devices

### WiFi won't connect
- Double-check SSID and password in `config.py` (case-sensitive!)
- Make sure the Pico WH is within WiFi range
- The Pico W only supports **2.4 GHz** WiFi (not 5 GHz)

### Car doesn't move when using joystick
- Check motor driver power supply (battery connected?)
- Make sure phone and Pico are on the **same WiFi network**
- Check the servo cable is on **GP15**
- Increase the speed limiter slider in the app

### How to stop the program
- In Thonny: Click the **red ■ Stop** button
- Or press **Ctrl+C** in the Shell
- The motors will stop automatically (safety shutdown)

### How to update a file
Just repeat Step 4 for the specific file — it will overwrite the old version.

---

## Wiring Summary

```
Pico WH Pin     →    Connected To
─────────────────────────────────────
GP0  (I2C0 SDA) →    Motor Driver SDA
GP1  (I2C0 SCL) →    Motor Driver SCL
GP15 (PWM)      →    Steering Servo Signal
GND             →    Motor Driver GND + Servo GND
VBUS (5V)       →    Motor Driver 5V
3V3             →    Servo VCC (if servo is 3.3V tolerant)
                      (or use external 6-8.4V for servo)
```

> ⚠️ **Power Note:** The steering servo (LD-1501MG) needs 6-8.4V for full torque. You may need a separate power supply for the servo — don't power high-torque servos from the Pico's 3.3V pin!
