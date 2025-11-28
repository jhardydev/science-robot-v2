"""
Navigation controller - steers robot toward detected wave position
"""
from science_robot import config
import numpy as np
import time
from collections import deque


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
        
        # Adaptive steering gain
        self.adaptive_gain_enabled = config.STEERING_ADAPTIVE_ENABLED
        self.adaptive_gain_factor = config.STEERING_ADAPTIVE_FACTOR
        self.large_error_threshold = config.STEERING_LARGE_ERROR_THRESHOLD
        self.large_error_reduction = config.STEERING_LARGE_ERROR_REDUCTION
        
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
        
        # Track spinning state for anti-spin control
        self.spinning_start_time = None
        self.spinning_timeout = 0.2  # Stop if spinning for more than 0.2 seconds (reduced from 0.5)
        self.spin_reduction_factor = 0.0  # Completely eliminate steering when spinning detected (changed from 0.3)
        self.max_turn_rate_when_spinning = 0.1  # Maximum turn rate allowed when spinning (even if reduction factor is applied)
        
        # PID steering control state
        self.steering_kp = config.STEERING_KP
        self.steering_ki = config.STEERING_KI
        self.steering_kd = config.STEERING_KD
        self.steering_integral_max = config.STEERING_INTEGRAL_MAX
        self.last_error = 0.0
        self.error_integral = 0.0
        self.last_error_time = time.time()
        
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
        
        # Target velocity estimation for predictive tracking
        self.target_history = deque(maxlen=5)  # Store recent positions and timestamps
        self.target_velocity = (0.0, 0.0)  # (vx, vy) in normalized coordinates per second
        self.lookahead_time = config.TRACKING_LOOKAHEAD_TIME
        
        # Oscillation detection for preventing circular motion
        self.error_history = deque(maxlen=10)  # Store recent error values to detect oscillation
        self.oscillation_detected = False
        self.oscillation_sign_changes = 0  # Count of error sign changes (indicates oscillation)
        self.last_error_sign = 0  # -1, 0, or 1
        self.oscillation_reset_time = 0.5  # Reset oscillation counter after this time
        self.last_oscillation_check_time = time.time()
    
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
        import logging
        logger = logging.getLogger(__name__)
        
        if target_position is None:
            # Reset smoothed position when target is lost
            self.smoothed_position = None
            # Reset PID state when target is lost
            self.error_integral = 0.0
            self.last_error = 0.0
            # Reset velocity estimation when target is lost
            self.target_history.clear()
            self.target_velocity = (0.0, 0.0)
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
        
        # Track position history for velocity estimation
        current_time = time.time()
        self.target_history.append((self.smoothed_position[0], self.smoothed_position[1], current_time))
        
        # Calculate target velocity from position history
        if len(self.target_history) >= 2:
            # Use most recent two points to estimate velocity
            (x1, y1, t1) = self.target_history[-1]
            (x2, y2, t2) = self.target_history[-2]
            dt = t1 - t2
            if dt > 0.001:  # Avoid division by zero
                vx = (x1 - x2) / dt
                vy = (y1 - y2) / dt
                # Smooth velocity estimate (exponential moving average)
                self.target_velocity = (
                    0.7 * self.target_velocity[0] + 0.3 * vx,
                    0.7 * self.target_velocity[1] + 0.3 * vy
                )
        
        # Predict future position using velocity
        predicted_x = self.smoothed_position[0] + self.target_velocity[0] * self.lookahead_time
        predicted_y = self.smoothed_position[1] + self.target_velocity[1] * self.lookahead_time
        
        # Clamp predicted position to valid range [0, 1]
        predicted_x = max(0.0, min(1.0, predicted_x))
        predicted_y = max(0.0, min(1.0, predicted_y))
        
        # Calculate current error to determine if we should use predictive tracking
        # For large errors, disable predictive tracking to prevent overshoot
        current_centered_x = (self.smoothed_position[0] - 0.5) * 2.0
        abs_current_error = abs(current_centered_x)
        
        # Use predicted position for steering (blend with current position for stability)
        # Higher blend factor = more predictive, lower = more reactive
        # Disable predictive tracking for large errors to prevent overshoot
        # Reduced blend factor from 0.6 to 0.3 to prevent overshoot and circular motion
        if abs_current_error > self.large_error_threshold:
            # Large error: use current position only (no prediction)
            blend_factor = 0.0  # 100% current, 0% predicted
        else:
            # Small/medium error: use reduced predictive tracking to prevent overshoot
            blend_factor = 0.3  # Use 30% predicted, 70% current (reduced from 0.6)
        
        target_x = blend_factor * predicted_x + (1.0 - blend_factor) * self.smoothed_position[0]
        target_y = blend_factor * predicted_y + (1.0 - blend_factor) * self.smoothed_position[1]
        
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
        
        # Apply deceleration zone: gradually reduce speed when approaching target
        if target_y > config.STOPPING_DECELERATION_START:
            # Calculate deceleration factor
            # At target_y = STOPPING_DECELERATION_START: factor = 1.0 (no reduction)
            # At target_y = 0.75 (stop threshold): factor = (1.0 - STOPPING_DECELERATION_FACTOR)
            deceleration_range = 0.75 - config.STOPPING_DECELERATION_START
            if deceleration_range > 0:
                deceleration_progress = (target_y - config.STOPPING_DECELERATION_START) / deceleration_range
                deceleration_progress = min(1.0, deceleration_progress)  # Clamp to 1.0
                deceleration_factor = 1.0 - (config.STOPPING_DECELERATION_FACTOR * deceleration_progress)
                current_speed = current_speed * deceleration_factor
        
        # Apply dead zone with hysteresis to prevent oscillation
        # Use a slightly larger dead zone when exiting (prevents rapid re-entry)
        dead_zone_exit = self.dead_zone * 1.5  # 50% larger exit threshold
        
        # Check if we're in dead zone
        in_dead_zone = abs(centered_x) < self.dead_zone
        # Check if we should exit dead zone (use larger threshold to prevent oscillation)
        if hasattr(self, '_was_in_dead_zone') and self._was_in_dead_zone:
            # Was in dead zone, use exit threshold
            in_dead_zone = abs(centered_x) < dead_zone_exit
        
        if in_dead_zone:
            # Target is centered, move straight forward
            # Reset PID integral when in dead zone
            self.error_integral = 0.0
            self.last_error = 0.0
            self._was_in_dead_zone = True
            return current_speed, current_speed
        else:
            self._was_in_dead_zone = False
        
        # Calculate current error (centered_x is the error from center)
        error = centered_x
        
        # Detect oscillation: track error sign changes to identify circular motion
        current_time = time.time()
        error_sign = 1 if error > 0.01 else (-1 if error < -0.01 else 0)
        
        # Reset oscillation counter if too much time has passed
        if current_time - self.last_oscillation_check_time > self.oscillation_reset_time:
            self.oscillation_sign_changes = 0
            self.error_history.clear()
        
        # Track error history for oscillation detection
        self.error_history.append(error)
        
        # Detect sign changes (oscillation indicator)
        # Note: We'll apply oscillation reduction to turn_rate after it's calculated
        if self.last_error_sign != 0 and error_sign != 0 and self.last_error_sign != error_sign:
            self.oscillation_sign_changes += 1
            if self.oscillation_sign_changes >= 4:  # 4+ sign changes = oscillation detected
                self.oscillation_detected = True
                logger.warning(f"OSCILLATION DETECTED: {self.oscillation_sign_changes} sign changes - resetting integral")
                # Reset integral to prevent persistent error
                self.error_integral = 0.0
            else:
                self.oscillation_detected = False
        else:
            # Reset counter if no recent sign changes
            if self.oscillation_sign_changes > 0 and current_time - self.last_oscillation_check_time > self.oscillation_reset_time:
                self.oscillation_sign_changes = 0
                self.oscillation_detected = False
        
        self.last_error_sign = error_sign
        self.last_oscillation_check_time = current_time
        
        # Calculate time delta for PID
        dt = current_time - self.last_error_time
        if dt < 0.001:  # Avoid division by zero
            dt = 0.001
        if dt > 1.0:  # Reset if too much time has passed
            dt = 0.1
            self.error_integral = 0.0
        
        # PID Control
        # Proportional term
        p_term = error * self.steering_kp
        
        # Integral term (with anti-windup)
        self.error_integral += error * dt
        # Clamp integral to prevent windup
        self.error_integral = max(-self.steering_integral_max, 
                                  min(self.steering_integral_max, self.error_integral))
        i_term = self.error_integral * self.steering_ki
        
        # Derivative term (rate of change of error)
        derivative = (error - self.last_error) / dt
        d_term = derivative * self.steering_kd
        
        # Adaptive gain: reduce steering when error is small, large, or target is close
        adaptive_multiplier = 1.0
        if self.adaptive_gain_enabled:
            abs_error = abs(error)
            
            # Reduce gain when error is SMALL (small error = less aggressive correction)
            if abs_error < 0.2:
                # Linear reduction: at error=0, multiplier = (1 - adaptive_factor)
                # at error=0.2, multiplier = 1.0
                error_factor = abs_error / 0.2
                adaptive_multiplier = 1.0 - self.adaptive_gain_factor * (1.0 - error_factor)
            
            # Reduce gain when error is LARGE (large error = prevent overshoot and spinning)
            elif abs_error > self.large_error_threshold:
                # Progressive reduction: at error=large_error_threshold, multiplier = 1.0
                # at error=1.0 (maximum), multiplier = (1 - large_error_reduction)
                # This prevents aggressive turns that cause spinning
                error_range = 1.0 - self.large_error_threshold
                if error_range > 0:
                    error_progress = (abs_error - self.large_error_threshold) / error_range
                    error_progress = min(1.0, error_progress)  # Clamp to 1.0
                    large_error_multiplier = 1.0 - (self.large_error_reduction * error_progress)
                    # Use the more restrictive multiplier (smaller value)
                    adaptive_multiplier = min(adaptive_multiplier, large_error_multiplier)
            
            # Also reduce gain when close to target (target_y > 0.7)
            if target_y > 0.7:
                # Linear reduction: at target_y=0.7, multiplier = 1.0
                # at target_y=1.0, multiplier = (1 - adaptive_factor)
                distance_factor = (target_y - 0.7) / 0.3
                distance_multiplier = 1.0 - self.adaptive_gain_factor * distance_factor
                # Use the more restrictive multiplier (smaller value)
                adaptive_multiplier = min(adaptive_multiplier, distance_multiplier)
        
        # Apply adaptive multiplier to PID terms
        p_term = p_term * adaptive_multiplier
        i_term = i_term * adaptive_multiplier
        d_term = d_term * adaptive_multiplier
        
        # Combined PID output
        turn_rate = p_term + i_term + d_term
        
        # Update for next iteration
        self.last_error = error
        self.last_error_time = current_time
        
        # Scale by max_angle to maintain compatibility
        turn_rate = turn_rate * self.max_angle
        
        # Progressive turn rate limiting: more restrictive as error increases
        # This prevents aggressive turns that cause spinning
        abs_error = abs(error)
        if abs_error > self.large_error_threshold:
            # Large error: very restrictive limit (0.3)
            max_safe_turn_rate = 0.3
        elif abs_error > 0.2:
            # Medium error: moderate limit (0.4)
            max_safe_turn_rate = 0.4
        else:
            # Small error: standard limit (0.5)
            max_safe_turn_rate = 0.5
        
        if abs(turn_rate) > max_safe_turn_rate:
            turn_rate = max_safe_turn_rate if turn_rate > 0 else -max_safe_turn_rate
            logger.debug(f"Progressive limit: turn_rate={turn_rate:.3f} (error={error:.3f}, limit={max_safe_turn_rate:.3f})")
        
        # Phase 2 Option C: Cap turn_rate relative to current_speed to prevent negative speeds
        # This ensures left_speed = current_speed + turn_rate >= 0.1 * current_speed
        # This is the ROOT CAUSE fix - prevents turn_rate from exceeding current_speed
        # Tightened from 90% to 75% for more conservative turning and to prevent circular motion
        max_turn_rate_relative = current_speed * 0.75  # Never exceed 75% of current speed (tightened from 0.9)
        if abs(turn_rate) > max_turn_rate_relative:
            turn_rate = max_turn_rate_relative if turn_rate > 0 else -max_turn_rate_relative
            logger.debug(f"Turn rate capped to {turn_rate:.3f} (current_speed={current_speed:.3f}, "
                        f"relative_limit={max_turn_rate_relative:.3f})")
        
        # Apply oscillation reduction to turn_rate if oscillation detected
        # This reduces steering gain when oscillating to break out of circular motion
        if self.oscillation_detected:
            turn_rate = turn_rate * 0.5  # Cut turn rate in half when oscillating
            logger.debug(f"Oscillation reduction applied: turn_rate={turn_rate:.3f}")
        
        # Check IMU for safety and anti-spin control BEFORE calculating speeds
        
        # Safety check: Emergency stop if dangerous spinning detected
        if self.use_imu_validation:
            if self.imu_reader.is_dangerous_spinning():
                logger.error("EMERGENCY STOP: Dangerous spinning detected by IMU")
                self.last_update_time = time.time()
                self.spinning_start_time = None  # Reset spinning timer
                return 0.0, 0.0
            
            # Anti-spin control: Detect and prevent unintentional spinning
            if self.imu_reader.is_spinning_detected():
                current_time = time.time()
                
                # Track how long we've been spinning
                if self.spinning_start_time is None:
                    self.spinning_start_time = current_time
                    yaw_rate = abs(self.imu_reader.get_yaw_rate())
                    logger.warning(f"SPINNING DETECTED: Yaw rate = {yaw_rate:.3f} rad/s - eliminating steering")
                
                # If spinning for too long, stop completely
                if current_time - self.spinning_start_time > self.spinning_timeout:
                    spin_duration = current_time - self.spinning_start_time
                    logger.error(f"STOPPING: Spinning detected for {spin_duration:.2f}s - stopping motors")
                    self.last_update_time = time.time()
                    # Reset PID state to prevent error accumulation
                    self.error_integral = 0.0
                    self.last_error = 0.0
                    return 0.0, 0.0
                
                # Aggressively reduce steering when spinning is detected
                # Apply reduction factor (0.0 = completely eliminate steering)
                turn_rate = turn_rate * self.spin_reduction_factor
                
                # Also apply maximum turn rate limit (even if reduction factor didn't fully eliminate it)
                if abs(turn_rate) > self.max_turn_rate_when_spinning:
                    turn_rate = self.max_turn_rate_when_spinning if turn_rate > 0 else -self.max_turn_rate_when_spinning
                    logger.debug(f"Limiting turn_rate to {turn_rate:.3f} due to spinning")
                
                # Reset PID integral to prevent error accumulation during spin
                self.error_integral = 0.0
            else:
                # Not spinning - reset timer
                if self.spinning_start_time is not None:
                    logger.info("Spinning stopped - resuming normal operation")
                self.spinning_start_time = None
        
        # Calculate left and right wheel speeds
        # Positive turn_rate means turn right (left wheel faster)
        # Negative turn_rate means turn left (right wheel faster)
        left_speed = current_speed + turn_rate
        right_speed = current_speed - turn_rate
        
        # Clamp speeds to valid range
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))
        
        # Phase 1: Diagnostic logging for debugging spinning issues
        if config.ENABLE_MOVEMENT_DIAGNOSTICS or abs(left_speed) > 0.5 or abs(right_speed) > 0.5 or left_speed < 0 or right_speed < 0:
            logger.info(f"NAV_DIAG: target_y={target_y:.3f}, target_x={target_x:.3f}, "
                        f"error={error:.3f}, current_speed={current_speed:.3f}, "
                        f"turn_rate={turn_rate:.3f}, "
                        f"L={left_speed:.3f} R={right_speed:.3f} (before encoder feedback)")
        
        # Store target_y for use after encoder feedback (safety checks)
        stored_target_y = target_y
        
        # Fix 1 & 3: Prevent spinning when close to target
        # When close to target (target_y > 0.6), prevent negative wheel speeds and ensure forward motion
        if target_y > 0.6:  # Close to target
            # Fix 1: Prevent negative wheel speeds when close
            if left_speed < 0 or right_speed < 0:
                # If one is negative, reduce turn_rate to keep both positive
                min_speed = min(left_speed, right_speed)
                if min_speed < 0:
                    # Adjust to keep both positive
                    adjustment = abs(min_speed)
                    left_speed = left_speed + adjustment
                    right_speed = right_speed + adjustment
                    # Re-clamp
                    left_speed = max(0.0, min(1.0, left_speed))
                    right_speed = max(0.0, min(1.0, right_speed))
                    logger.warning(f"NAV_FIX: Prevented negative speed when close (target_y={target_y:.3f}), "
                                  f"adjusted L={left_speed:.3f} R={right_speed:.3f}")
            
            # Fix 3: Ensure minimum forward speed to prevent pure pivoting
            min_forward_speed = 0.1  # Minimum 10% forward speed when close
            if left_speed < min_forward_speed and right_speed < min_forward_speed:
                # Both too slow, ensure minimum forward motion
                left_speed = max(min_forward_speed, left_speed)
                right_speed = max(min_forward_speed, right_speed)
            elif left_speed < min_forward_speed:
                # Left too slow, ensure minimum forward
                left_speed = max(min_forward_speed, left_speed)
            elif right_speed < min_forward_speed:
                # Right too slow, ensure minimum forward
                right_speed = max(min_forward_speed, right_speed)
        else:
            # Target is far away - allow normal turning behavior
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
                # This is fine for turning when target is far
                pass
        
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
                
                # Log validation status periodically
                if hasattr(self, '_validation_log_counter'):
                    self._validation_log_counter += 1
                else:
                    self._validation_log_counter = 0
                
                # Enhanced diagnostic logging
                if config.ENABLE_MOVEMENT_DIAGNOSTICS:
                    if self._validation_log_counter % 30 == 0:  # Every ~1 second at 30fps
                        logger.info(f"VALIDATION: valid={is_valid}, expected_yaw={expected_yaw:.3f}, "
                                  f"actual_yaw={actual_yaw:.3f} rad/s, "
                                  f"desired L={desired_left:.3f} R={desired_right:.3f} m/s, "
                                  f"actual L={actual_left:.3f} R={actual_right:.3f} m/s")
                else:
                    if self._validation_log_counter % 30 == 0:
                        logger.debug(f"IMU validation: valid={is_valid}, expected_yaw={expected_yaw:.3f}, "
                                   f"actual_yaw={actual_yaw:.3f} rad/s")
            
            # Get corrected speeds from PID controller
            left_speed, right_speed = self.speed_controller.adjust_speeds(
                desired_left, desired_right,
                actual_left, actual_right
            )
            
            # Phase 1: Log speeds after encoder feedback
            if config.ENABLE_MOVEMENT_DIAGNOSTICS or abs(left_speed) > 0.5 or abs(right_speed) > 0.5 or left_speed < 0 or right_speed < 0:
                logger.info(f"NAV_DIAG: After encoder feedback - L={left_speed:.3f} R={right_speed:.3f}")
            
            # CRITICAL FIX: Re-apply safety checks after encoder feedback
            # Encoder feedback can reintroduce negative speeds or cause issues, so we need to validate again
            # Use stored target_y from before encoder feedback
            if stored_target_y > 0.6:  # Close to target - apply safety checks
                # Prevent negative wheel speeds when close
                if left_speed < 0 or right_speed < 0:
                    min_speed = min(left_speed, right_speed)
                    if min_speed < 0:
                        adjustment = abs(min_speed)
                        left_speed = left_speed + adjustment
                        right_speed = right_speed + adjustment
                        left_speed = max(0.0, min(1.0, left_speed))
                        right_speed = max(0.0, min(1.0, right_speed))
                        logger.warning(f"NAV_FIX_POST_ENCODER: Prevented negative speed after encoder feedback, "
                                      f"adjusted L={left_speed:.3f} R={right_speed:.3f}")
                
                # Ensure minimum forward speed to prevent pure pivoting
                min_forward_speed = 0.1
                if left_speed < min_forward_speed and right_speed < min_forward_speed:
                    left_speed = max(min_forward_speed, left_speed)
                    right_speed = max(min_forward_speed, right_speed)
                elif left_speed < min_forward_speed:
                    left_speed = max(min_forward_speed, left_speed)
                elif right_speed < min_forward_speed:
                    right_speed = max(min_forward_speed, right_speed)
        
        # Stability check: Detect circular motion pattern
        # If error is oscillating and we're not making progress, reduce steering
        # This check happens after all speed calculations to catch circular motion
        if len(self.error_history) >= 5:
            # Check if error magnitude is not decreasing (stuck in circle)
            recent_errors = list(self.error_history)[-5:]
            avg_error_magnitude = sum(abs(e) for e in recent_errors) / len(recent_errors)
            current_error_magnitude = abs(error)
            
            # If average error is similar to current and we're oscillating, we might be in a circle
            if self.oscillation_detected and abs(avg_error_magnitude - current_error_magnitude) < 0.1:
                # Calculate a reduced turn rate to break out of circular motion
                # Use the difference between left and right speeds as a proxy for turn rate
                current_turn_rate_estimate = (left_speed - right_speed) / 2.0
                reduced_turn_rate = current_turn_rate_estimate * 0.7  # Reduce by 30%
                logger.warning(f"STABILITY CHECK: Circular motion detected - reducing steering")
                # Apply reduction to both speeds proportionally
                center_speed = (left_speed + right_speed) / 2.0
                left_speed = center_speed + reduced_turn_rate
                right_speed = center_speed - reduced_turn_rate
                left_speed = max(-1.0, min(1.0, left_speed))
                right_speed = max(-1.0, min(1.0, right_speed))
        
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
        Considers target velocity and distance for smoother stopping behavior
        
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
        
        # Base stopping threshold (bottom 25% of frame)
        stop_threshold = 0.75
        
        # Consider target velocity: if target is moving away, don't stop as early
        # Calculate velocity magnitude
        velocity_magnitude = np.sqrt(self.target_velocity[0]**2 + self.target_velocity[1]**2)
        
        # If target is moving away (positive vy = moving down/away in frame coordinates)
        # Adjust stop threshold to be more lenient
        if velocity_magnitude > 0.1:  # Target is moving significantly
            # If moving away (positive vy), increase threshold (stop later)
            if self.target_velocity[1] > 0:
                # Moving away - increase threshold by velocity factor
                stop_threshold = min(0.85, stop_threshold + velocity_magnitude * 0.3)
            # If moving toward (negative vy), decrease threshold (stop earlier)
            elif self.target_velocity[1] < -0.05:
                # Moving toward - decrease threshold slightly
                stop_threshold = max(0.70, stop_threshold - abs(self.target_velocity[1]) * 0.2)
        
        # Stop if target is very close (adjusted threshold)
        return target_y > stop_threshold
    
    def reset_smoothing(self):
        """Reset smoothed position tracking and velocity estimation"""
        self.smoothed_position = None
        self.last_update_time = time.time()
        self.target_history.clear()
        self.target_velocity = (0.0, 0.0)
        self.error_integral = 0.0
        self.last_error = 0.0
        # Reset oscillation detection state
        self.error_history.clear()
        self.oscillation_detected = False
        self.oscillation_sign_changes = 0
        self.last_error_sign = 0
        self.last_oscillation_check_time = time.time()

