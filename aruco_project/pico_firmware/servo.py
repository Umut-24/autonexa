from machine import Pin, PWM
import config


class Servo:
    """
    Controls the steering servo (e.g., Hiwonder LD-1501MG).
    Connected to GP15.
    
    The servo expects a 50Hz PWM signal where the pulse width
    determines the angle:
        500μs  → 0°   (full left)
        1500μs → 90°  (center)
        2500μs → 180° (full right)
    
    Duty cycle calculation for 50Hz (period = 20ms = 20000μs):
        duty_u16 = (pulse_us / 20000) × 65535
    """
    
    def __init__(self, pin_num=None):
        pin_num = pin_num or config.SERVO_PIN
        self.pwm = PWM(Pin(pin_num))
        self.pwm.freq(config.SERVO_FREQ)
        self._current_angle = config.SERVO_CENTER_ANGLE
        
        # Go to center on init
        self.set_angle(config.SERVO_CENTER_ANGLE)
    
    def set_angle(self, angle):
        """
        Set servo to a specific angle in degrees.
        
        Args:
            angle: 0-180 degrees (clamped to mechanical limits)
        """
        angle = max(config.SERVO_MIN_ANGLE, min(config.SERVO_MAX_ANGLE, angle))
        self._current_angle = angle
        
        # Map angle (0-180) to pulse width (500-2500μs)
        pulse_us = config.SERVO_MIN_US + (angle / 180.0) * (config.SERVO_MAX_US - config.SERVO_MIN_US)
        pulse_us += config.SERVO_CENTER_OFFSET
        pulse_us = max(config.SERVO_MIN_US, min(config.SERVO_MAX_US, pulse_us))
        
        # Convert to 16-bit duty cycle
        period_us = 1_000_000 / config.SERVO_FREQ
        duty = int((pulse_us / period_us) * 65535)
        self.pwm.duty_u16(duty)
    
    def set_steering(self, normalized):
        """
        Set steering from a normalized value.
        
        Args:
            normalized: -1.0 (full left) to +1.0 (full right), 0 = center
        """
        normalized = max(-1.0, min(1.0, normalized))
        center = config.SERVO_CENTER_ANGLE
        half_range = min(
            center - config.SERVO_MIN_ANGLE,
            config.SERVO_MAX_ANGLE - center,
        )
        angle = center + (normalized * half_range)
        self.set_angle(angle)
    
    def center(self):
        """Return servo to center position."""
        self.set_angle(config.SERVO_CENTER_ANGLE)
    
    def get_angle(self):
        """Get current servo angle."""
        return self._current_angle
    
    def deinit(self):
        """Release PWM resources."""
        self.center()
        self.pwm.deinit()
