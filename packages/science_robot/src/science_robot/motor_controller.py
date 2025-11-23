"""
Motor controller for differential drive Duckiebot using ROS
Publishes wheel commands to Duckiebot's motor control node
"""
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Header
from science_robot import config


class MotorController:
    """Differential drive motor control via ROS"""
    
    def __init__(self):
        """Initialize ROS publisher for motor control"""
        # Note: rospy.init_node should be called in main(), not here
        # But we check if it's already initialized
        if rospy.get_node_uri() is None:
            rospy.logwarn("ROS node not initialized. Call rospy.init_node() in main() first.")
        
        # Publisher for wheel commands
        self.wheels_pub = rospy.Publisher(
            config.MOTOR_TOPIC,
            WheelsCmdStamped,
            queue_size=1,
            latch=False  # Don't latch - send fresh commands each time
        )
        
        # Wait a moment for publisher to advertise
        rospy.sleep(0.2)
        
        # Publish an initial zero-speed command to establish connection
        # ROS uses "lazy connections" - subscribers only connect when first message is published
        # This triggers the connection with any existing subscribers (wheels_driver_node)
        zero_msg = WheelsCmdStamped()
        zero_msg.header = Header()
        zero_msg.header.stamp = rospy.Time.now()
        zero_msg.vel_left = 0.0
        zero_msg.vel_right = 0.0
        
        # Publish multiple times with delays to ensure connection is established
        # Sometimes ROS needs a few messages to establish the connection
        for attempt in range(3):
            self.wheels_pub.publish(zero_msg)
            rospy.sleep(0.15)  # Give ROS time between publishes
        
        # Give ROS additional time to establish the connection after publishing
        rospy.sleep(0.3)
        
        # Check for subscribers multiple times (ROS connection can be delayed)
        subscriber_count = 0
        max_retries = 5
        for retry in range(max_retries):
            subscriber_count = self.wheels_pub.get_num_connections()
            if subscriber_count > 0:
                break
            if retry < max_retries - 1:
                # Publish again and wait
                self.wheels_pub.publish(zero_msg)
                rospy.sleep(0.2)
        
        if subscriber_count == 0:
            rospy.logwarn(f"Motor command topic '{config.MOTOR_TOPIC}' has no subscribers!")
            rospy.logwarn("  Make sure the wheels_driver_node is running")
            rospy.logwarn("  Note: Connection is established when first message is published")
            # Try to diagnose - check if topic exists at all
            try:
                topics = rospy.get_published_topics()
                topic_found = any(config.MOTOR_TOPIC in topic[0] for topic in topics)
                if not topic_found:
                    rospy.logwarn(f"  Topic '{config.MOTOR_TOPIC}' not found in published topics list")
            except Exception as e:
                rospy.logdebug(f"Could not check published topics: {e}")
        else:
            rospy.loginfo(f"Motor command topic has {subscriber_count} subscriber(s)")
        
        self.is_stopped = True
        self._last_log_time = 0
        self._log_interval = 1.0  # Log every 1 second
        self._emergency_stop_active = False  # Emergency stop flag
        rospy.loginfo(f"Motor controller initialized (ROS) - Topic: {config.MOTOR_TOPIC}")
    
    def _publish_wheel_command(self, left_speed, right_speed):
        """
        Publish wheel command to ROS topic
        
        Args:
            left_speed: Left wheel speed (normalized -1.0 to 1.0)
            right_speed: Right wheel speed (normalized -1.0 to 1.0)
        """
        # Block commands if emergency stop is active
        if self._emergency_stop_active:
            rospy.logdebug("Motor command blocked: Emergency stop is active")
            return
        # Create wheel command message
        msg = WheelsCmdStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        
        # Duckiebot wheels_driver expects speeds in m/s
        # Duckiebot typical max speed is around 0.5-1.0 m/s
        # Scale normalized speeds (-1.0 to 1.0) to actual velocities
        # Using MOTOR_MAX_SPEED as scaling factor (in m/s, not normalized)
        # If MOTOR_MAX_SPEED is 0.8, a normalized speed of 1.0 becomes 0.8 m/s
        
        # Clamp normalized speeds first
        left_speed = max(-1.0, min(1.0, float(left_speed)))
        right_speed = max(-1.0, min(1.0, float(right_speed)))
        
        # Convert to m/s (Duckiebot expects velocities in m/s)
        # Note: MOTOR_MAX_SPEED is already in m/s, so multiply by it
        msg.vel_left = left_speed * config.MOTOR_MAX_SPEED
        msg.vel_right = right_speed * config.MOTOR_MAX_SPEED
        
        # Store last speeds for status reporting
        self._last_left_speed = msg.vel_left
        self._last_right_speed = msg.vel_right
        
        # Log motor commands for debugging (but throttle to avoid spam)
        if not hasattr(self, '_last_log_time'):
            self._last_log_time = 0
            self._log_interval = 1.0  # Log every 1 second
        
        current_time = rospy.get_time()
        if current_time - self._last_log_time >= self._log_interval:
            rospy.logdebug(f"Motor command: left={msg.vel_left:.3f} m/s, right={msg.vel_right:.3f} m/s "
                          f"(normalized: left={left_speed:.2f}, right={right_speed:.2f})")
            self._last_log_time = current_time
        
        self.wheels_pub.publish(msg)
        self.is_stopped = (left_speed == 0.0 and right_speed == 0.0)
    
    def clear_emergency_stop(self):
        """
        Clear emergency stop flag to allow normal operation
        
        Note: This should only be called after ensuring it's safe to resume
        """
        if self._emergency_stop_active:
            rospy.loginfo("Clearing emergency stop - resuming normal operation")
            self._emergency_stop_active = False
    
    def is_emergency_stop_active(self):
        """Check if emergency stop is currently active"""
        return getattr(self, '_emergency_stop_active', False)
    
    def move_forward(self, speed=None):
        """
        Move forward
        
        Args:
            speed: Speed from 0.0 to 1.0 (default from config)
        """
        if speed is None:
            speed = config.MOTOR_BASE_SPEED
        
        self._publish_wheel_command(speed, speed)
    
    def move_backward(self, speed=None):
        """
        Move backward
        
        Args:
            speed: Speed from 0.0 to 1.0 (default from config)
        """
        if speed is None:
            speed = config.MOTOR_BASE_SPEED
        
        self._publish_wheel_command(-speed, -speed)
    
    def turn_left(self, speed=None):
        """
        Turn left (left wheel backward, right wheel forward)
        
        Args:
            speed: Speed from 0.0 to 1.0 (default from config)
        """
        if speed is None:
            speed = config.MOTOR_TURN_SPEED
        
        self._publish_wheel_command(-speed, speed)
    
    def turn_right(self, speed=None):
        """
        Turn right (right wheel backward, left wheel forward)
        
        Args:
            speed: Speed from 0.0 to 1.0 (default from config)
        """
        if speed is None:
            speed = config.MOTOR_TURN_SPEED
        
        self._publish_wheel_command(speed, -speed)
    
    def set_differential_speed(self, left_speed, right_speed):
        """
        Set individual speeds for left and right wheels
        
        Args:
            left_speed: Left wheel speed from -1.0 to 1.0
            right_speed: Right wheel speed from -1.0 to 1.0
        """
        # Clamp speeds to [-1, 1]
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))
        
        self._publish_wheel_command(left_speed, right_speed)
    
    def stop(self):
        """Stop all motors"""
        self._publish_wheel_command(0.0, 0.0)
    
    def emergency_stop(self):
        """
        Emergency stop - immediately stop all motors and publish to emergency stop topic
        
        This function:
        1. Stops motors immediately by publishing zero-speed commands
        2. Publishes to Duckietown's emergency stop topic (if available)
        3. Sets emergency stop flag to prevent further movement
        """
        # First, stop motors immediately
        self.stop()
        self.is_stopped = True
        
        # Publish to Duckietown emergency stop topic
        # This is a direct command to the wheels_driver_node to halt immediately
        try:
            emergency_pub = rospy.Publisher(
                config.EMERGENCY_STOP_TOPIC,
                Header,
                queue_size=1,
                latch=True  # Latch so the emergency stop persists
            )
            
            # Give publisher a moment to advertise
            rospy.sleep(0.05)
            
            # Publish emergency stop message
            emergency_msg = Header()
            emergency_msg.stamp = rospy.Time.now()
            emergency_pub.publish(emergency_msg)
            
            # Publish multiple times to ensure it's received
            for _ in range(3):
                emergency_pub.publish(emergency_msg)
                rospy.sleep(0.05)
            
            rospy.logwarn("EMERGENCY STOP activated (via dedicated topic)")
        except Exception as e:
            rospy.logerr(f"Failed to publish emergency stop to topic: {e}")
            rospy.logwarn("EMERGENCY STOP activated (via normal wheel command only)")
        
        # Set emergency stop flag
        self._emergency_stop_active = True
        rospy.logwarn("EMERGENCY STOP: All motors stopped")
    
    def get_last_speeds(self):
        """
        Get the last commanded wheel speeds in m/s.
        Returns:
            (left_speed, right_speed) tuple in m/s
        """
        return (getattr(self, '_last_left_speed', 0.0), getattr(self, '_last_right_speed', 0.0))
    
    def cleanup(self):
        """Clean up ROS resources"""
        self.stop()
        rospy.sleep(0.1)
        rospy.loginfo("Motor controller cleaned up")

