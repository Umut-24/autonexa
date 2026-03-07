from machine import Pin, I2C
import struct
import time
import config


class MotorDriver:
    """
    Controls motors via the Hiwonder 4-channel encoder motor driver board
    over I2C.
    
    ┌─────────────────────────────────────────────────────────────┐
    │  HOW THIS BOARD WORKS                                       │
    │                                                             │
    │  Unlike a simple H-bridge (L298N / TB6612) where the Pico  │
    │  does everything (PWM, encoder reading, PID), this board    │
    │  has its OWN onboard microcontroller that handles:          │
    │                                                             │
    │  1. Reading encoders (AB phase, all 4 channels)             │
    │  2. Running PID control (closed-loop speed regulation)      │
    │  3. Driving the YX-4055AM H-bridge chips (PWM output)       │
    │                                                             │
    │  The Pico just sends I2C commands:                          │
    │  "Motor 1 → speed +50, Motor 2 → speed -30"               │
    │  And the board takes care of everything else!               │
    │                                                             │
    │  ┌─────────┐    I2C     ┌─────────────────────┐            │
    │  │ Pico WH │ ─────────► │  Motor Driver Board  │            │
    │  │ (master)│   GP0/GP1  │  (onboard MCU)       │            │
    │  └─────────┘            │  ├─ Reads encoders   │            │
    │                         │  ├─ Runs PID          │            │
    │                         │  └─ Drives motors     │            │
    │                         └─────────────────────┘            │
    │                                                             │
    │  I2C Address: 0x34 (default)                                │
    │                                                             │
    │  Key Registers:                                             │
    │  ┌────────┬───────────────────────────────────┐            │
    │  │ 0x14   │ Motor type (TT=0, N20=1, 520=2)  │            │
    │  │ 0x33   │ Set speed, closed-loop [-100,100] │            │
    │  │ 0x1F   │ Set PWM, open-loop [-100, 100]    │            │
    │  │ 0x3C   │ Read encoder count (4 × int32)    │            │
    │  └────────┴───────────────────────────────────┘            │
    │                                                             │
    │  Speed data format (0x33 register):                         │
    │  Write 4 signed bytes: [motor1, motor2, motor3, motor4]     │
    │  Range: -100 to +100 per motor                              │
    │  Positive = forward, Negative = reverse, 0 = stop           │
    └─────────────────────────────────────────────────────────────┘
    """
    
    def __init__(self):
        """Initialize I2C connection to the motor driver board."""
        self.i2c = I2C(
            config.I2C_ID,
            sda=Pin(config.I2C_SDA),
            scl=Pin(config.I2C_SCL),
            freq=config.I2C_FREQ,
        )
        self.addr = config.MOTOR_DRIVER_ADDR
        self.left_ch = config.MOTOR_LEFT_CHANNEL
        self.right_ch = config.MOTOR_RIGHT_CHANNEL
        
        # Verify the board is reachable
        devices = self.i2c.scan()
        if self.addr in devices:
            print(f"[Motor] Board found at I2C address 0x{self.addr:02X}")
        else:
            print(f"[Motor] WARNING: Board NOT found at 0x{self.addr:02X}!")
            print(f"[Motor] Devices found: {['0x%02X' % d for d in devices]}")
        
        # Configure motor type
        self._setup_motor_type()
        
        # Stop all motors initially
        self.stop_all()
    
    def _setup_motor_type(self):
        """
        Tell the board what type of motors are connected.
        This affects the internal PID tuning on the board.
        
        Motor types:
            0 = TT motor with encoder
            1 = N20 motor with encoder
            2 = JGB37-520 motor with encoder
        """
        motor_type = config.MOTOR_TYPE
        # Write motor type for all 4 channels
        data = bytes([motor_type, motor_type, motor_type, motor_type])
        try:
            self.i2c.writeto_mem(self.addr, config.MOTOR_TYPE_ADDR, data)
            print(f"[Motor] Motor type set to {motor_type} (520 DC gear motor)")
            time.sleep_ms(100)  # Give board time to reconfigure
        except OSError as e:
            print(f"[Motor] Failed to set motor type: {e}")
    
    def set_speed(self, left_speed, right_speed):
        """
        Set speed for left and right motors using closed-loop control.
        
        The board's onboard MCU will use its PID controller to maintain
        the requested speeds using encoder feedback.
        
        Args:
            left_speed:  -100 to +100 (negative = reverse)
            right_speed: -100 to +100 (negative = reverse)
        """
        left_speed = int(max(-100, min(100, left_speed)))
        right_speed = int(max(-100, min(100, right_speed)))
        
        # Build speed array for all 4 channels (set unused channels to 0)
        speeds = [0, 0, 0, 0]
        speeds[self.left_ch - 1] = left_speed
        speeds[self.right_ch - 1] = right_speed
        
        # Pack as signed bytes
        data = struct.pack('4b', *speeds)
        
        try:
            self.i2c.writeto_mem(self.addr, config.MOTOR_FIXED_SPEED_ADDR, data)
        except OSError:
            pass
    
    def set_pwm(self, left_pwm, right_pwm):
        """
        Set PWM directly (open-loop, no PID).
        Useful for testing or when you want raw control.
        
        Args:
            left_pwm:  -100 to +100
            right_pwm: -100 to +100
        """
        left_pwm = int(max(-100, min(100, left_pwm)))
        right_pwm = int(max(-100, min(100, right_pwm)))
        
        pwms = [0, 0, 0, 0]
        pwms[self.left_ch - 1] = left_pwm
        pwms[self.right_ch - 1] = right_pwm
        
        data = struct.pack('4b', *pwms)
        
        try:
            self.i2c.writeto_mem(self.addr, config.MOTOR_FIXED_PWM_ADDR, data)
        except OSError:
            pass
    
    def read_encoders(self):
        """
        Read encoder counts from all 4 channels.
        
        Returns:
            tuple: (left_count, right_count) as signed 32-bit integers
        """
        try:
            data = self.i2c.readfrom_mem(self.addr, config.MOTOR_ENCODER_TOTAL_ADDR, 16)
            counts = struct.unpack('<4i', data)  # 4 × int32, little-endian
            return (counts[self.left_ch - 1], counts[self.right_ch - 1])
        except OSError:
            return (0, 0)
    
    def stop_all(self):
        """Stop all motors (closed-loop zero speed)."""
        data = struct.pack('4b', 0, 0, 0, 0)
        try:
            self.i2c.writeto_mem(self.addr, config.MOTOR_FIXED_SPEED_ADDR, data)
        except OSError:
            pass
    
    def get_speeds_from_driver(self):
        """
        Read back current motor speeds (if the board supports it).
        Returns the raw data for telemetry.
        """
        left_enc, right_enc = self.read_encoders()
        return (left_enc, right_enc)
