import math
import config


class AckermannController:
    """
    Converts joystick inputs (steering, throttle) into individual
    wheel speeds and a servo angle using Ackermann steering geometry.
    
    Since the motor driver board handles PID internally, this module
    outputs speed values in the board's range [-100, +100] directly.
    """
    
    def __init__(self):
        self.wheelbase = config.WHEELBASE    # mm
        self.track_width = config.TRACK_WIDTH  # mm
        self.max_speed = config.MAX_SPEED    # board's max speed value (100)
        self.deadzone = config.DEADZONE
    
    def compute(self, steering, throttle):
        """
        Convert joystick inputs to wheel speeds and servo angle.
        
        Args:
            steering: -1.0 (full left) to +1.0 (full right)
            throttle: -1.0 (full reverse) to +1.0 (full forward)
        
        Returns:
            tuple: (left_speed, right_speed, servo_angle)
                - left_speed:  -100 to +100 (for I2C motor driver)
                - right_speed: -100 to +100 (for I2C motor driver)
                - servo_angle: degrees (for servo PWM)
        """
        # Apply deadzone
        if abs(steering) < self.deadzone:
            steering = 0.0
        if abs(throttle) < self.deadzone:
            throttle = 0.0
        
        # Base speed from throttle
        base_speed = throttle * self.max_speed
        
        # Servo angle: map steering to angle range
        center = config.SERVO_CENTER_ANGLE
        half_range = min(
            center - config.SERVO_MIN_ANGLE,
            config.SERVO_MAX_ANGLE - center,
        )
        servo_angle = center + (steering * half_range)
        
        # Calculate differential speeds
        if abs(steering) < self.deadzone:
            # Straight — both wheels same speed
            left_speed = base_speed
            right_speed = base_speed
        else:
            # Turning radius from bicycle model
            steering_angle_rad = math.radians(servo_angle - center)
            
            if abs(steering_angle_rad) < 0.01:
                left_speed = base_speed
                right_speed = base_speed
            else:
                turning_radius = self.wheelbase / math.tan(abs(steering_angle_rad))
                
                inner_factor = (turning_radius - self.track_width / 2) / turning_radius
                outer_factor = (turning_radius + self.track_width / 2) / turning_radius
                
                if steering > 0:
                    # Turning right: right is inner
                    left_speed = base_speed * outer_factor
                    right_speed = base_speed * inner_factor
                else:
                    # Turning left: left is inner
                    left_speed = base_speed * inner_factor
                    right_speed = base_speed * outer_factor
        
        # Clamp to board's range
        left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
        right_speed = max(-self.max_speed, min(self.max_speed, right_speed))
        
        return (left_speed, right_speed, servo_angle)
