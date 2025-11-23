"""
IMU reader for detecting rotation and validating encoder feedback
"""
import rospy
import time
import threading
import logging
from collections import deque

try:
    from sensor_msgs.msg import Imu
    SENSOR_MSGS_AVAILABLE = True
except ImportError:
    SENSOR_MSGS_AVAILABLE = False
    logging.warning("sensor_msgs not available - IMU reading disabled")

logger = logging.getLogger(__name__)

class IMUReader:
    """Reads IMU data to detect rotation and validate encoder feedback"""
    
    def __init__(self, robot_name='robot1', spinning_threshold=0.5, 
                 dangerous_spin_threshold=2.0, wheel_base=0.1):
        """
        Initialize IMU reader
        
        Args:
            robot_name: Robot name for topic namespace
            spinning_threshold: Angular velocity threshold (rad/s) for detecting spinning
            dangerous_spin_threshold: Threshold (rad/s) for emergency stop
            wheel_base: Distance between wheels in meters (for encoder validation)
        """
        self.robot_name = robot_name
        self.spinning_threshold = spinning_threshold
        self.dangerous_spin_threshold = dangerous_spin_threshold
        self.wheel_base = wheel_base
        self.lock = threading.Lock()
        
        # Current IMU state
        self.angular_velocity_z = 0.0  # Yaw rate (rad/s) - most important for spinning
        self.angular_velocity_x = 0.0  # Roll rate
        self.angular_velocity_y = 0.0  # Pitch rate
        self.linear_acceleration_x = 0.0
        self.linear_acceleration_y = 0.0
        self.linear_acceleration_z = 0.0
        self.last_update_time = None
        
        # Rotation detection
        self.is_spinning = False
        self.spin_direction = 0  # -1 = left, +1 = right, 0 = not spinning
        
        # Oscillation detection
        self.angular_velocity_history = deque(maxlen=10)
        self.is_oscillating = False
        
        # Subscribe to IMU topic
        self.imu_topic = f'/{robot_name}/imu_node/raw'
        if SENSOR_MSGS_AVAILABLE:
            try:
                self.imu_sub = rospy.Subscriber(
                    self.imu_topic,
                    Imu,
                    self._imu_callback,
                    queue_size=10
                )
                logger.info(f"IMU reader initialized: {self.imu_topic}")
                logger.info(f"  Spinning threshold: {spinning_threshold} rad/s")
                logger.info(f"  Dangerous spin threshold: {dangerous_spin_threshold} rad/s")
            except Exception as e:
                logger.error(f"Failed to subscribe to IMU topic: {e}")
                self.imu_sub = None
        else:
            logger.warning("sensor_msgs not available - IMU reading disabled")
            self.imu_sub = None
    
    def _imu_callback(self, msg):
        """Callback for IMU data"""
        with self.lock:
            # Extract angular velocity (rotation rates)
            self.angular_velocity_x = msg.angular_velocity.x
            self.angular_velocity_y = msg.angular_velocity.y
            self.angular_velocity_z = msg.angular_velocity.z  # Yaw rate (most important)
            
            # Extract linear acceleration
            self.linear_acceleration_x = msg.linear_acceleration.x
            self.linear_acceleration_y = msg.linear_acceleration.y
            self.linear_acceleration_z = msg.linear_acceleration.z
            
            self.last_update_time = time.time()
            
            # Update rotation detection
            self._update_rotation_detection()
    
    def _update_rotation_detection(self):
        """Update spinning and oscillation detection"""
        # Check for spinning
        abs_yaw_rate = abs(self.angular_velocity_z)
        if abs_yaw_rate > self.spinning_threshold:
            self.is_spinning = True
            self.spin_direction = 1 if self.angular_velocity_z > 0 else -1
        else:
            self.is_spinning = False
            self.spin_direction = 0
        
        # Check for dangerous spinning (emergency stop)
        if abs_yaw_rate > self.dangerous_spin_threshold:
            logger.warning(f"DANGEROUS SPINNING DETECTED: {abs_yaw_rate:.2f} rad/s (threshold: {self.dangerous_spin_threshold} rad/s)")
        
        # Update history for oscillation detection
        self.angular_velocity_history.append(self.angular_velocity_z)
        
        # Detect oscillation (rapid sign changes)
        if len(self.angular_velocity_history) >= 5:
            sign_changes = sum(
                1 for i in range(1, len(self.angular_velocity_history))
                if (self.angular_velocity_history[i] > 0) != (self.angular_velocity_history[i-1] > 0)
            )
            # If 3+ sign changes in recent history, likely oscillating
            self.is_oscillating = sign_changes >= 3
        else:
            self.is_oscillating = False
    
    def is_spinning_detected(self):
        """Check if robot is spinning unintentionally"""
        with self.lock:
            return self.is_spinning
    
    def get_yaw_rate(self):
        """Get current yaw rate (angular velocity around z-axis) in rad/s"""
        with self.lock:
            return self.angular_velocity_z
    
    def get_angular_velocity(self):
        """Get all angular velocities (x, y, z) in rad/s"""
        with self.lock:
            return (self.angular_velocity_x, self.angular_velocity_y, self.angular_velocity_z)
    
    def is_oscillating_detected(self):
        """Check if robot is oscillating (back and forth rotation)"""
        with self.lock:
            return self.is_oscillating
    
    def is_dangerous_spinning(self):
        """Check if rotation rate exceeds dangerous threshold"""
        with self.lock:
            return abs(self.angular_velocity_z) > self.dangerous_spin_threshold
    
    def validate_encoder_rotation(self, left_velocity, right_velocity, tolerance=0.3):
        """
        Validate encoder velocities against IMU rotation
        
        Args:
            left_velocity: Left wheel velocity (m/s)
            right_velocity: Right wheel velocity (m/s)
            tolerance: Tolerance for validation (rad/s)
            
        Returns:
            (is_valid, expected_yaw_rate, actual_yaw_rate) tuple
        """
        with self.lock:
            # Calculate expected yaw rate from wheel velocities
            # For differential drive: ω = (v_r - v_l) / wheel_base
            # Positive yaw rate = turning right (counter-clockwise from top)
            expected_yaw_rate = (right_velocity - left_velocity) / self.wheel_base
            
            # Get actual yaw rate from IMU
            actual_yaw_rate = self.angular_velocity_z
            
            # Check if they agree (within tolerance)
            is_valid = abs(expected_yaw_rate - actual_yaw_rate) < tolerance
            
            return is_valid, expected_yaw_rate, actual_yaw_rate
    
    def is_available(self):
        """Check if IMU is available"""
        return self.imu_sub is not None and self.last_update_time is not None
    
    def is_data_fresh(self, max_age=0.5):
        """Check if IMU data is recent"""
        if self.last_update_time is None:
            return False
        return (time.time() - self.last_update_time) < max_age
    
    def get_status(self):
        """Get current IMU status for diagnostics"""
        with self.lock:
            return {
                'available': self.is_available(),
                'data_fresh': self.is_data_fresh(),
                'yaw_rate': self.angular_velocity_z,
                'is_spinning': self.is_spinning,
                'is_oscillating': self.is_oscillating,
                'is_dangerous': self.is_dangerous_spinning(),
                'spin_direction': self.spin_direction
            }

