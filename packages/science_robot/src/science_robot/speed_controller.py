"""
Closed-loop speed controller using encoder feedback
Adjusts motor speeds to maintain desired velocities
"""
import logging
import time

logger = logging.getLogger(__name__)

class SpeedController:
    """PID-based speed controller for differential drive"""
    
    def __init__(self, kp=1.0, ki=0.1, kd=0.05, max_integral=0.5):
        """
        Initialize speed controller
        
        Args:
            kp: Proportional gain (default: 1.0)
            ki: Integral gain (default: 0.1)
            kd: Derivative gain (default: 0.05)
            max_integral: Maximum integral term to prevent windup (default: 0.5)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        
        # Per-wheel state
        self.left_integral = 0.0
        self.right_integral = 0.0
        self.left_last_error = 0.0
        self.right_last_error = 0.0
        self.last_update_time = time.time()
        
        # Diagnostic logging
        from science_robot import config
        self.diagnostics_enabled = config.ENABLE_MOVEMENT_DIAGNOSTICS
        self.diagnostics_interval = config.MOVEMENT_DIAGNOSTICS_INTERVAL
        self.last_diagnostics_log = time.time()
        
        logger.info(f"Speed controller initialized: kp={kp}, ki={ki}, kd={kd}")
    
    def compute_correction(self, desired_velocity, actual_velocity, 
                          integral, last_error, dt):
        """
        Compute PID correction for one wheel
        
        Args:
            desired_velocity: Target velocity in m/s
            actual_velocity: Measured velocity in m/s
            integral: Current integral term
            last_error: Previous error
            dt: Time delta since last update
            
        Returns:
            (correction, new_integral, new_error) tuple
        """
        error = desired_velocity - actual_velocity
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (with anti-windup)
        integral += error * dt
        integral = max(-self.max_integral, min(self.max_integral, integral))
        i_term = self.ki * integral
        
        # Derivative term
        if dt > 0:
            d_term = self.kd * (error - last_error) / dt
        else:
            d_term = 0.0
        
        correction = p_term + i_term + d_term
        return correction, integral, error
    
    def adjust_speeds(self, desired_left, desired_right, 
                      actual_left, actual_right):
        """
        Adjust motor speeds based on encoder feedback
        
        Args:
            desired_left: Desired left wheel velocity (m/s)
            desired_right: Desired right wheel velocity (m/s)
            actual_left: Actual left wheel velocity (m/s) from encoders
            actual_right: Actual right wheel velocity (m/s) from encoders
            
        Returns:
            (adjusted_left, adjusted_right) normalized speeds (-1.0 to 1.0)
        """
        current_time = time.time()
        dt = current_time - self.last_update_time
        
        if dt < 0.001:
            dt = 0.001
        
        # Compute corrections for each wheel
        left_correction, self.left_integral, self.left_last_error = \
            self.compute_correction(desired_left, actual_left, 
                                  self.left_integral, self.left_last_error, dt)
        
        right_correction, self.right_integral, self.right_last_error = \
            self.compute_correction(desired_right, actual_right,
                                  self.right_integral, self.right_last_error, dt)
        
        # Convert desired velocities to normalized speeds
        # Assuming max speed is from config
        from science_robot import config
        max_speed = config.MOTOR_MAX_SPEED
        
        if max_speed <= 0:
            max_speed = 0.8  # Fallback default
        
        left_normalized = (desired_left / max_speed) if max_speed > 0 else 0.0
        right_normalized = (desired_right / max_speed) if max_speed > 0 else 0.0
        
        # Apply corrections (convert correction from m/s to normalized)
        correction_scale = 1.0 / max_speed if max_speed > 0 else 1.0
        adjusted_left = left_normalized + (left_correction * correction_scale)
        adjusted_right = right_normalized + (right_correction * correction_scale)
        
        # Clamp to valid range
        adjusted_left = max(-1.0, min(1.0, adjusted_left))
        adjusted_right = max(-1.0, min(1.0, adjusted_right))
        
        # Diagnostic logging
        if self.diagnostics_enabled:
            if current_time - self.last_diagnostics_log >= self.diagnostics_interval:
                left_error = desired_left - actual_left
                right_error = desired_right - actual_right
                logger.info(f"PID: Desired L={desired_left:.3f} R={desired_right:.3f} m/s, "
                          f"Actual L={actual_left:.3f} R={actual_right:.3f} m/s, "
                          f"Error L={left_error:.3f} R={right_error:.3f} m/s, "
                          f"Correction L={left_correction:.3f} R={right_correction:.3f} m/s, "
                          f"Adjusted L={adjusted_left:.2f} R={adjusted_right:.2f}")
                self.last_diagnostics_log = current_time
        
        self.last_update_time = current_time
        
        return adjusted_left, adjusted_right
    
    def reset(self):
        """Reset controller state"""
        self.left_integral = 0.0
        self.right_integral = 0.0
        self.left_last_error = 0.0
        self.right_last_error = 0.0
        self.last_update_time = time.time()
        logger.debug("Speed controller reset")

