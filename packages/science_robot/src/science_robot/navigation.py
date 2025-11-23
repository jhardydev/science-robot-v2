"""
Navigation controller - steers robot toward detected wave position
"""
from science_robot import config
import numpy as np
import time


class NavigationController:
    """Controls robot navigation based on target position"""
    
    def __init__(self, steering_gain=None, dead_zone=None, encoder_reader=None, speed_controller=None, imu_reader=None):
        """
        Initialize navigation controller
        
        Args:
            steering_gain: Proportional gain for steering (default from config)
            dead_zone: Dead zone in normalized coordinates (default from config)
            encoder_reader: Optional EncoderReader instance for velocity feedback
            speed_controller: Optional SpeedController instance for closed-loop control
            imu_reader: Optional IMUReader instance for rotation detection and validation
        """
        self.steering_gain = steering_gain or config.STEERING_GAIN
        self.dead_zone = dead_zone or config.STEERING_DEAD_ZONE
        self.max_angle = config.MAX_STEERING_ANGLE
        self.base_speed = config.MOTOR_BASE_SPEED
        self.smoothing_factor = config.TRACKING_SMOOTHING
        self.speed_by_distance = config.SPEED_BY_DISTANCE
        
        # Encoder feedback
        self.encoder_reader = encoder_reader
        self.speed_controller = speed_controller
        self.use_encoder_feedback = (encoder_reader is not None and 
                                     speed_controller is not None and
                                     encoder_reader.is_available())
        
        # IMU feedback
        self.imu_reader = imu_reader
        self.use_imu_validation = (imu_reader is not None and imu_reader.is_available())
        
        # Track encoder feedback disable state (due to IMU validation failures)
        self.encoder_feedback_disabled = False
        self.encoder_feedback_disable_time = None
        self.encoder_feedback_disable_duration = 2.0  # Re-enable after 2 seconds
        
        import logging
        logger = logging.getLogger(__name__)
        
        if self.use_encoder_feedback:
            logger.info("Encoder feedback enabled for closed-loop speed control")
        else:
            if config.ENCODER_ENABLED:
                logger.warning("Encoder feedback requested but not available - using open-loop control")
        
        if self.use_imu_validation:
            logger.info("IMU validation enabled for rotation detection and encoder validation")
        else:
            if config.IMU_ENABLED:
                logger.warning("IMU validation requested but not available")
        
        # Smoothed position tracking
        self.smoothed_position = None
        self.last_update_time = time.time()
    
    def calculate_steering(self, target_position):
        """
        Calculate steering commands based on target position in frame
        Uses smoothed position tracking for more stable navigation
        
        Args:
            target_position: (x, y) tuple in normalized coordinates (0.0 to 1.0)
                           where x=0 is left, x=1 is right, y=0 is top, y=1 is bottom
        
        Returns:
            (left_speed, right_speed) tuple for differential drive motors
            Returns (0, 0) if target is None or in dead zone
        """
        if target_position is None:
            # Reset smoothed position when target is lost
            self.smoothed_position = None
            return 0.0, 0.0
        
        target_x, target_y = target_position
        
        # Apply smoothing to position for more stable tracking
        if self.smoothed_position is None:
            # Initialize smoothed position
            self.smoothed_position = (target_x, target_y)
        else:
            # Exponential moving average for smooth tracking
            smooth_x = self.smoothed_position[0] * (1.0 - self.smoothing_factor) + target_x * self.smoothing_factor
            smooth_y = self.smoothed_position[1] * (1.0 - self.smoothing_factor) + target_y * self.smoothing_factor
            self.smoothed_position = (smooth_x, smooth_y)
        
        # Use smoothed position for steering
        target_x, target_y = self.smoothed_position
        
        # Convert to centered coordinate system (-1.0 to 1.0)
        # x = 0.0 is center, x = -1.0 is far left, x = 1.0 is far right
        centered_x = (target_x - 0.5) * 2.0
        
        # Calculate speed based on distance (y coordinate)
        # Closer targets (higher y) = slower speed for precision
        # Farther targets (lower y) = faster speed to catch up
        if self.speed_by_distance:
            # Distance factor: 0.0 (top/far) = 1.0 speed, 1.0 (bottom/close) = 0.3 speed
            distance_factor = 0.3 + (1.0 - target_y) * 0.7  # Range: 0.3 to 1.0
            current_speed = self.base_speed * distance_factor
        else:
            current_speed = self.base_speed
        
        # Apply dead zone (smaller dead zone for tighter tracking)
        if abs(centered_x) < self.dead_zone:
            # Target is centered, move straight forward
            return current_speed, current_speed
        
        # Calculate steering angle (proportional control)
        steering_angle = centered_x * self.max_angle
        
        # Apply proportional gain (higher gain = more aggressive steering)
        turn_rate = steering_angle * self.steering_gain
        
        # Calculate left and right wheel speeds
        # Positive turn_rate means turn right (left wheel faster)
        # Negative turn_rate means turn left (right wheel faster)
        left_speed = current_speed + turn_rate
        right_speed = current_speed - turn_rate
        
        # Clamp speeds to valid range
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))
        
        # Ensure at least some forward motion when target is ahead
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
        
        # Check IMU for safety and validation before applying encoder feedback
        import logging
        logger = logging.getLogger(__name__)
        
        # Safety check: Emergency stop if dangerous spinning detected
        if self.use_imu_validation:
            if self.imu_reader.is_dangerous_spinning():
                logger.error("EMERGENCY STOP: Dangerous spinning detected by IMU")
                self.last_update_time = time.time()
                return 0.0, 0.0
        
        # Check if encoder feedback should be temporarily disabled
        current_time = time.time()
        if self.encoder_feedback_disabled:
            if current_time - self.encoder_feedback_disable_time > self.encoder_feedback_disable_duration:
                # Re-enable encoder feedback after timeout
                self.encoder_feedback_disabled = False
                logger.info("Re-enabling encoder feedback after timeout")
        
        # Apply encoder feedback if available and not disabled (closed-loop control)
        if self.use_encoder_feedback and not self.encoder_feedback_disabled:
            # Convert normalized speeds to m/s for controller
            desired_left = left_speed * config.MOTOR_MAX_SPEED
            desired_right = right_speed * config.MOTOR_MAX_SPEED
            
            # Get actual velocities from encoders
            actual_left, actual_right = self.encoder_reader.get_velocities()
            
            # Validate encoder feedback with IMU if available
            if self.use_imu_validation:
                is_valid, expected_yaw, actual_yaw = self.imu_reader.validate_encoder_rotation(
                    actual_left, actual_right, tolerance=config.IMU_VALIDATION_TOLERANCE
                )
                
                # If IMU detects spinning but encoders don't match, disable feedback
                if self.imu_reader.is_spinning_detected() and not is_valid:
                    logger.warning(f"IMU detects spinning (yaw={actual_yaw:.3f} rad/s) but encoders don't match "
                                 f"(expected={expected_yaw:.3f} rad/s) - disabling encoder feedback")
                    self.encoder_feedback_disabled = True
                    self.encoder_feedback_disable_time = current_time
                    # Use open-loop control instead
                    self.last_update_time = time.time()
                    return left_speed, right_speed
                
                # If oscillating, disable encoder feedback
                if self.imu_reader.is_oscillating_detected():
                    logger.warning("Oscillation detected by IMU - disabling encoder feedback to prevent instability")
                    self.encoder_feedback_disabled = True
                    self.encoder_feedback_disable_time = current_time
                    # Use open-loop control instead
                    self.last_update_time = time.time()
                    return left_speed, right_speed
                
                # Log validation status periodically (every 30 frames ~= 1 second at 30fps)
                if hasattr(self, '_validation_log_counter'):
                    self._validation_log_counter += 1
                else:
                    self._validation_log_counter = 0
                
                if self._validation_log_counter % 30 == 0:
                    logger.debug(f"IMU validation: valid={is_valid}, expected_yaw={expected_yaw:.3f}, "
                               f"actual_yaw={actual_yaw:.3f} rad/s")
            
            # Get corrected speeds from PID controller
            left_speed, right_speed = self.speed_controller.adjust_speeds(
                desired_left, desired_right,
                actual_left, actual_right
            )
        
        self.last_update_time = time.time()
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
        
        # Use smoothed position if available
        if self.smoothed_position:
            _, target_y = self.smoothed_position
        else:
            _, target_y = target_position
        
        # Stop if target is very close (bottom 25% of frame)
        # Increased from 20% to 25% for better stopping distance
        return target_y > 0.75
    
    def reset_smoothing(self):
        """Reset smoothed position tracking"""
        self.smoothed_position = None
        self.last_update_time = time.time()

