"""
Navigation controller - steers robot toward detected wave position
"""
from science_robot import config
import numpy as np


class NavigationController:
    """Controls robot navigation based on target position"""
    
    def __init__(self, steering_gain=None, dead_zone=None):
        """
        Initialize navigation controller
        
        Args:
            steering_gain: Proportional gain for steering (default from config)
            dead_zone: Dead zone in normalized coordinates (default from config)
        """
        self.steering_gain = steering_gain or config.STEERING_GAIN
        self.dead_zone = dead_zone or config.STEERING_DEAD_ZONE
        self.max_angle = config.MAX_STEERING_ANGLE
        self.base_speed = config.MOTOR_BASE_SPEED
    
    def calculate_steering(self, target_position):
        """
        Calculate steering commands based on target position in frame
        
        Args:
            target_position: (x, y) tuple in normalized coordinates (0.0 to 1.0)
                           where x=0 is left, x=1 is right, y=0 is top, y=1 is bottom
        
        Returns:
            (left_speed, right_speed) tuple for differential drive motors
            Returns (0, 0) if target is None or in dead zone
        """
        if target_position is None:
            return 0.0, 0.0
        
        target_x, target_y = target_position
        
        # Convert to centered coordinate system (-1.0 to 1.0)
        # x = 0.0 is center, x = -1.0 is far left, x = 1.0 is far right
        centered_x = (target_x - 0.5) * 2.0
        
        # Apply dead zone
        if abs(centered_x) < self.dead_zone:
            # Target is centered, move straight forward
            return self.base_speed, self.base_speed
        
        # Calculate steering angle (proportional control)
        steering_angle = centered_x * self.max_angle
        
        # Apply proportional gain
        turn_rate = steering_angle * self.steering_gain
        
        # Calculate left and right wheel speeds
        # Positive turn_rate means turn right (left wheel faster)
        # Negative turn_rate means turn left (right wheel faster)
        left_speed = self.base_speed + turn_rate
        right_speed = self.base_speed - turn_rate
        
        # Clamp speeds to valid range
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))
        
        # Ensure at least some forward motion
        if left_speed > 0 and right_speed > 0:
            # Both positive, good to go
            pass
        elif left_speed < 0 and right_speed < 0:
            # Both negative, switch to moving backward
            left_speed = -abs(left_speed)
            right_speed = -abs(right_speed)
        else:
            # One positive, one negative - pivot in place
            # This is fine for turning
            pass
        
        return left_speed, right_speed
    
    def get_distance_estimate(self, target_position):
        """
        Estimate distance to target based on y-coordinate
        
        Args:
            target_position: (x, y) tuple in normalized coordinates
        
        Returns:
            Estimated distance (normalized, 0.0 to 1.0)
        """
        if target_position is None:
            return None
        
        _, target_y = target_position
        # Lower y values (closer to top of frame) indicate farther away
        # Higher y values (closer to bottom of frame) indicate closer
        # This is a simplified model
        return target_y
    
    def should_stop(self, target_position):
        """
        Determine if robot should stop (target is close enough)
        
        Args:
            target_position: (x, y) tuple in normalized coordinates
        
        Returns:
            Boolean indicating if robot should stop
        """
        if target_position is None:
            return True
        
        _, target_y = target_position
        # Stop if target is very close (bottom 20% of frame)
        return target_y > 0.8

