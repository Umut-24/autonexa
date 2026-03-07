# ============================================================
# Pico WH Ackermann Chassis — Configuration
# ============================================================
# All tunable parameters in one place. Edit this file to match
# your specific hardware wiring and chassis dimensions.

# --- WiFi ---
WIFI_SSID = "24"
WIFI_PASSWORD = "2424uU2424"

# --- UDP ---
UDP_PORT = 4210            # Port to listen for commands
TELEMETRY_INTERVAL_MS = 200  # How often to send telemetry back (ms)
COMMAND_TIMEOUT_MS = 500     # Stop motors if no command received (ms)

# --- GPIO Pin Assignments ---
#
# Motor Driver: Hiwonder 4-channel encoder motor driver board
# Connected via I2C — the board has an onboard MCU that handles
# encoder reading, PID, and motor driving internally.
#
# The Pico only sends speed commands over I2C.
# The board does ALL the hard work: encoder counting, PID, PWM.
#
# I2C connection (4-pin header on driver board):
#   SCL → GP1 (I2C0 SCL)
#   SDA → GP0 (I2C0 SDA)
#   GND → Pico GND
#   5V  → Pico VBUS (5V)

# I2C Bus
I2C_ID = 0                   # I2C0 peripheral
I2C_SDA = 0                  # GP0 = I2C0 SDA
I2C_SCL = 1                  # GP1 = I2C0 SCL
I2C_FREQ = 100_000           # 100kHz (standard I2C speed)

# Motor Driver Board I2C Address
MOTOR_DRIVER_ADDR = 0x34     # Default I2C address of the Hiwonder board

# Motor Driver Board Registers
MOTOR_TYPE_ADDR = 0x14           # Register to set motor type (decimal 20)
MOTOR_ENCODER_POLARITY_ADDR = 0x15  # Register to set encoder polarity
MOTOR_FIXED_SPEED_ADDR = 0x33   # Closed-loop speed register (PID on board)
MOTOR_FIXED_PWM_ADDR = 0x1F     # Open-loop PWM register (no PID)
MOTOR_ENCODER_TOTAL_ADDR = 0x3C  # Read total encoder count

# Motor type configuration
# 0 = TT motor with AB encoder
# 1 = N20 motor with AB encoder
# 2 = JGB37-520 motor with AB encoder (our motor)
MOTOR_TYPE = 2

# Steering servo
SERVO_PIN = 15                # GP15 for servo PWM

# --- Servo ---
SERVO_FREQ = 50              # Hz (standard servo frequency)
SERVO_MIN_US = 500           # Minimum pulse width (μs) → full left
SERVO_MAX_US = 2500          # Maximum pulse width (μs) → full right
SERVO_CENTER_US = 1500       # Center pulse width (μs)
SERVO_CENTER_OFFSET = 0      # Calibration offset in μs (adjust if center is off)
SERVO_MIN_ANGLE = 60         # Minimum steering angle (mechanical limit)
SERVO_MAX_ANGLE = 120        # Maximum steering angle (mechanical limit)
SERVO_CENTER_ANGLE = 90      # Neutral angle

# --- Chassis Dimensions (mm) ---
WHEELBASE = 160              # Distance between front and rear axle centers
TRACK_WIDTH = 140            # Distance between left and right wheel centers
WHEEL_DIAMETER = 65          # Wheel diameter in mm

# --- Speed ---
# The motor driver board accepts speed values in range [-100, 100]
# for closed-loop control (MOTOR_FIXED_SPEED_ADDR).
# Positive = forward, negative = reverse.
MAX_SPEED = 100              # Maximum speed value for the motor driver
DEADZONE = 0.05              # Joystick deadzone (values below this = 0)

# --- Motor Channel Mapping ---
# The 4-channel board has channels 1-4.
# Our motors are physically connected to M2 and M4.
MOTOR_LEFT_CHANNEL = 2       # Channel for left rear motor (M2 port)
MOTOR_RIGHT_CHANNEL = 4      # Channel for right rear motor (M4 port)
