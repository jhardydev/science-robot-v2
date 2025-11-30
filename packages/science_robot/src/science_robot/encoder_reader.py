"""
Wheel encoder reader via ROS topics
Subscribes to encoder tick topics published by encoder driver nodes
"""
import rospy
import time
import threading
import logging
from collections import deque

try:
    from duckietown_msgs.msg import WheelEncoderStamped
    DUCKIE_MSGS_AVAILABLE = True
except ImportError:
    DUCKIE_MSGS_AVAILABLE = False
    logging.warning("duckietown_msgs not available - encoder reading disabled")

logger = logging.getLogger(__name__)

class EncoderReader:
    """Reads wheel encoder data from ROS topics"""
    
    def __init__(self, robot_name='robot1', encoder_ppr=137, wheel_diameter=0.0664):
        """
        Initialize encoder reader
        
        Args:
            robot_name: Robot name for topic namespace
            encoder_ppr: Pulses per revolution (default: 137)
            wheel_diameter: Wheel diameter in meters (default: 0.0664m = 66.40mm)
        """
        self.robot_name = robot_name
        
        # Encoder state
        self.left_tick_count = 0
        self.right_tick_count = 0
        self.last_left_tick = 0
        self.last_right_tick = 0
        self.last_left_time = None
        self.last_right_time = None
        
        # Velocity tracking (m/s)
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        
        # Encoder parameters
        self.encoder_ppr = encoder_ppr
        self.wheel_diameter = wheel_diameter
        self.wheel_circumference = 3.14159 * wheel_diameter
        
        # Thread safety
        self.lock = threading.Lock()
        
        # Topic names
        self.left_topic = f'/{robot_name}/left_wheel_encoder_driver_node/tick'
        self.right_topic = f'/{robot_name}/right_wheel_encoder_driver_node/tick'
        
        # Subscribe to encoder topics
        if DUCKIE_MSGS_AVAILABLE:
            try:
                self.left_sub = rospy.Subscriber(
                    self.left_topic,
                    WheelEncoderStamped,
                    self._left_callback,
                    queue_size=10
                )
                self.right_sub = rospy.Subscriber(
                    self.right_topic,
                    WheelEncoderStamped,
                    self._right_callback,
                    queue_size=10
                )
                logger.info(f"Encoder reader initialized:")
                logger.info(f"  Left: {self.left_topic}")
                logger.info(f"  Right: {self.right_topic}")
                logger.info(f"  PPR: {self.encoder_ppr}, Wheel diameter: {self.wheel_diameter}m")
            except Exception as e:
                logger.error(f"Failed to subscribe to encoder topics: {e}")
                self.left_sub = None
                self.right_sub = None
        else:
            logger.warning("duckietown_msgs not available - encoder feedback disabled")
            self.left_sub = None
            self.right_sub = None
        
        # Velocity calculation window (for smoothing)
        self.velocity_window_size = 5
        self.left_velocity_history = deque(maxlen=self.velocity_window_size)
        self.right_velocity_history = deque(maxlen=self.velocity_window_size)
        
        # Diagnostic logging
        from science_robot import config
        self.diagnostics_enabled = config.ENABLE_MOVEMENT_DIAGNOSTICS
        self.diagnostics_interval = config.MOVEMENT_DIAGNOSTICS_INTERVAL
        self.last_diagnostics_log = time.time()
    
    def _left_callback(self, msg):
        """Callback for left encoder ticks"""
        with self.lock:
            tick = int(msg.data)
            timestamp = msg.header.stamp.to_sec()
            
            self.left_tick_count = tick
            self._update_velocity('left', tick, timestamp)
    
    def _right_callback(self, msg):
        """Callback for right encoder ticks"""
        with self.lock:
            tick = int(msg.data)
            timestamp = msg.header.stamp.to_sec()
            
            self.right_tick_count = tick
            self._update_velocity('right', tick, timestamp)
    
    def _update_velocity(self, wheel, current_tick, timestamp):
        """Update velocity calculation for a wheel"""
        # Get last values
        if wheel == 'left':
            last_tick = self.last_left_tick
            last_time = self.last_left_time
        else:
            last_tick = self.last_right_tick
            last_time = self.last_right_time
        
        # Skip if this is the first reading
        if last_time is None:
            if wheel == 'left':
                self.last_left_tick = current_tick
                self.last_left_time = timestamp
            else:
                self.last_right_tick = current_tick
                self.last_right_time = timestamp
            return
        
        # Calculate time delta
        dt = timestamp - last_time
        
        if dt < 0.001:  # Avoid division by zero or very small deltas
            return
        
        # Calculate tick delta
        delta_ticks = current_tick - last_tick
        
        # Handle wraparound (if encoder resets or overflows)
        # For large jumps, assume wraparound occurred
        if abs(delta_ticks) > 1000000:  # Unrealistic jump = wraparound
            logger.debug(f"{wheel} encoder wraparound detected, ignoring delta")
            delta_ticks = 0
        
        # Convert ticks to distance (meters)
        # distance = (ticks / ticks_per_rev) * circumference
        distance = (delta_ticks / self.encoder_ppr) * self.wheel_circumference
        
        # Calculate velocity (m/s)
        velocity = distance / dt if dt > 0 else 0.0
        
        # Store in history for smoothing
        if wheel == 'left':
            self.left_velocity_history.append(velocity)
            # Use average for smoother velocity
            if len(self.left_velocity_history) > 0:
                self.left_velocity = sum(self.left_velocity_history) / len(self.left_velocity_history)
            self.last_left_tick = current_tick
            self.last_left_time = timestamp
        else:
            self.right_velocity_history.append(velocity)
            if len(self.right_velocity_history) > 0:
                self.right_velocity = sum(self.right_velocity_history) / len(self.right_velocity_history)
            self.last_right_tick = current_tick
            self.last_right_time = timestamp
        
        # Diagnostic logging - only log if sensor reading logs are enabled (reduces noise)
        # Movement diagnostics handle navigation-specific logs, not raw sensor readings
        from science_robot import config
        if config.ENABLE_SENSOR_READING_LOGS:
            current_time = time.time()
            if current_time - self.last_diagnostics_log >= self.diagnostics_interval:
                logger.info(f"ENCODER: L={self.left_velocity:.3f} m/s (ticks={self.left_tick_count}), "
                          f"R={self.right_velocity:.3f} m/s (ticks={self.right_tick_count})")
                self.last_diagnostics_log = current_time
    
    def get_velocities(self):
        """
        Get current wheel velocities
        
        Returns:
            (left_velocity, right_velocity) tuple in m/s
        """
        with self.lock:
            return (self.left_velocity, self.right_velocity)
    
    def get_tick_counts(self):
        """
        Get current encoder tick counts
        
        Returns:
            (left_ticks, right_ticks) tuple
        """
        with self.lock:
            return (self.left_tick_count, self.right_tick_count)
    
    def is_available(self):
        """Check if encoder reading is available"""
        return (self.left_sub is not None and self.right_sub is not None)
    
    def reset(self):
        """Reset encoder counts"""
        with self.lock:
            self.left_tick_count = 0
            self.right_tick_count = 0
            self.last_left_tick = 0
            self.last_right_tick = 0
            self.last_left_time = None
            self.last_right_time = None
            self.left_velocity = 0.0
            self.right_velocity = 0.0
            self.left_velocity_history.clear()
            self.right_velocity_history.clear()

