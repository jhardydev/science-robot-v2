"""
Collision avoidance module using ToF sensors and video-based obstacle detection
Combines multiple sensor inputs for robust collision prevention
"""
import rospy
import cv2
import numpy as np
import threading
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from science_robot import config
import logging

logger = logging.getLogger(__name__)


class CollisionAvoidance:
    """
    Collision avoidance system using ToF sensors and video analysis
    
    Features:
    - ToF sensor integration (front, left, right sensors)
    - Video-based obstacle detection (edge detection, depth estimation)
    - Multi-sensor fusion for robust obstacle detection
    - Configurable safety distances and thresholds
    """
    
    def __init__(self):
        """Initialize collision avoidance system"""
        # ToF sensor data (distance in meters)
        self.tof_front = None
        self.tof_left = None
        self.tof_right = None
        self.tof_lock = threading.Lock()
        self.tof_available = False
        
        # ToF floor filtering
        self.tof_min_valid_distance = config.TOF_MIN_VALID_DISTANCE
        self.tof_floor_filter_enabled = config.TOF_FLOOR_FILTER_ENABLED
        self.tof_stability_threshold = config.TOF_STABILITY_THRESHOLD
        self.tof_stability_samples = config.TOF_STABILITY_SAMPLES
        self.tof_history = []  # Store recent readings for stability check
        self.tof_max_history = max(self.tof_stability_samples, 20)  # Keep enough history
        
        # ToF cliff/edge detection (prevents driving off ledges)
        self.tof_cliff_detection_enabled = config.TOF_CLIFF_DETECTION_ENABLED
        self.tof_cliff_distance_threshold = config.TOF_CLIFF_DISTANCE_THRESHOLD
        self.tof_cliff_increase_threshold = config.TOF_CLIFF_INCREASE_THRESHOLD
        self.tof_cliff_history_samples = config.TOF_CLIFF_HISTORY_SAMPLES
        self.tof_cliff_history = []  # Store recent valid readings for cliff detection
        self.tof_cliff_max_history = max(self.tof_cliff_history_samples, 10)  # Keep enough history for cliff detection
        
        # Video-based obstacle detection
        self.obstacle_detected = False
        self.obstacle_distance_estimate = None  # Estimated distance in meters
        self.obstacle_position = None  # (x, y) position in frame (normalized 0-1)
        self.frame_history = []  # For motion-based depth estimation
        self.max_history = 5
        
        # Safety zones
        self.emergency_zone = config.COLLISION_EMERGENCY_DISTANCE  # Immediate stop
        self.warning_zone = config.COLLISION_WARNING_DISTANCE  # Slow down
        self.safe_zone = config.COLLISION_SAFE_DISTANCE  # Normal operation
        
        # Initialize ToF subscribers
        self._init_tof_subscribers()
        
        logger.info("Collision avoidance system initialized")
        logger.info(f"  Emergency zone: {self.emergency_zone}m")
        logger.info(f"  Warning zone: {self.warning_zone}m")
        logger.info(f"  Safe zone: {self.safe_zone}m")
        if self.tof_floor_filter_enabled:
            logger.info(f"  ToF floor filter: ENABLED (min valid: {self.tof_min_valid_distance}m)")
        else:
            logger.info(f"  ToF floor filter: DISABLED")
        if self.tof_cliff_detection_enabled:
            logger.info(f"  ToF cliff detection: ENABLED (threshold: {self.tof_cliff_distance_threshold}m, increase: {self.tof_cliff_increase_threshold}m)")
        else:
            logger.info(f"  ToF cliff detection: DISABLED")
    
    def _init_tof_subscribers(self):
        """Initialize ToF sensor subscribers"""
        # Common Duckiebot ToF topic patterns
        # Try multiple possible topic names (including I2C address-based names)
        tof_topics = [
            f'/{config.ROBOT_NAME}/front_center_tof_driver_node/range',  # Duckietown front center ToF
            f'/{config.ROBOT_NAME}/tof_node/distance',  # Standard Duckiebot ToF
            f'/{config.ROBOT_NAME}/tof_node/front/distance',
            f'/{config.ROBOT_NAME}/tof_node/left/distance',
            f'/{config.ROBOT_NAME}/tof_node/right/distance',
            f'/{config.ROBOT_NAME}/tof/distance',
            f'/{config.ROBOT_NAME}/tof_node/range',  # Alternative message type
            f'/{config.ROBOT_NAME}/tof_0x29/distance',  # I2C address-based naming
            f'/{config.ROBOT_NAME}/tof_0x29/range',
            f'/{config.ROBOT_NAME}/vl53l0x/distance',  # VL53L0X sensor name
            f'/{config.ROBOT_NAME}/vl53l0x/range',
            '/tof_node/distance',  # Fallback without robot name
            '/tof_node/range',
            '/tof_0x29/distance',
            '/vl53l0x/distance',
        ]
        
        # Subscribe to front ToF sensor
        self.tof_front_sub = None
        
        # Check for manual topic override first
        if config.TOF_TOPIC_OVERRIDE:
            try:
                self.tof_front_sub = rospy.Subscriber(
                    config.TOF_TOPIC_OVERRIDE,
                    Range,
                    self._tof_front_callback,
                    queue_size=1
                )
                logger.info(f"✓ Subscribed to ToF sensor (manual override): {config.TOF_TOPIC_OVERRIDE}")
                self.tof_available = True
                return  # Skip auto-detection if manual override is set
            except Exception as e:
                logger.warning(f"Failed to subscribe to manual ToF topic {config.TOF_TOPIC_OVERRIDE}: {e}")
        
        # First, get list of all published topics for better matching
        try:
            all_topics = rospy.get_published_topics()
            topic_names = [t[0] for t in all_topics]
            logger.debug(f"Available ROS topics: {len(topic_names)} topics found")
            
            # Look for any topic containing 'tof', 'distance', 'range', '0x29', or 'vl53'
            tof_related_topics = [t for t in topic_names if any(
                keyword in t.lower() for keyword in ['tof', 'distance', 'range', '0x29', 'vl53']
            )]
            
            if tof_related_topics:
                logger.info(f"Found ToF-related topics: {tof_related_topics}")
        except Exception as e:
            logger.debug(f"Could not get topic list: {e}")
            all_topics = []
            topic_names = []
            tof_related_topics = []
        
        # Try exact topic matches first
        for topic in tof_topics:
            try:
                # Check if topic exists (exact match or contains)
                if any(topic == t[0] or topic in t[0] for t in all_topics if all_topics):
                    self.tof_front_sub = rospy.Subscriber(
                        topic,
                        Range,
                        self._tof_front_callback,
                        queue_size=1
                    )
                    logger.info(f"✓ Subscribed to ToF front sensor: {topic}")
                    self.tof_available = True
                    break
            except Exception as e:
                logger.debug(f"Could not subscribe to {topic}: {e}")
                continue
        
        # If no exact match, try any ToF-related topic we found
        if not self.tof_available and tof_related_topics:
            # Prioritize topics with "range" in the name (more common for ToF sensors)
            sorted_topics = sorted(tof_related_topics, key=lambda x: 'range' in x.lower(), reverse=True)
            
            for topic in sorted_topics:
                try:
                    # Try to get topic type
                    topic_type = None
                    for t in all_topics:
                        if t[0] == topic:
                            topic_type = t[1]
                            break
                    
                    # If it's a Range message or we can't determine, try subscribing
                    if topic_type is None or 'Range' in topic_type or 'sensor_msgs/Range' in topic_type:
                        self.tof_front_sub = rospy.Subscriber(
                            topic,
                            Range,
                            self._tof_front_callback,
                            queue_size=1
                        )
                        logger.info(f"✓ Subscribed to ToF sensor (auto-detected): {topic}")
                        self.tof_available = True
                        break
                except Exception as e:
                    logger.debug(f"Could not subscribe to auto-detected topic {topic}: {e}")
                    continue
        
        # Try to find left/right ToF sensors
        left_topics = [
            f'/{config.ROBOT_NAME}/tof_node/left/distance',
            f'/{config.ROBOT_NAME}/tof/left/distance',
        ]
        right_topics = [
            f'/{config.ROBOT_NAME}/tof_node/right/distance',
            f'/{config.ROBOT_NAME}/tof/right/distance',
        ]
        
        for topic in left_topics:
            try:
                topics = rospy.get_published_topics()
                if any(topic in t[0] for t in topics):
                    self.tof_left_sub = rospy.Subscriber(
                        topic,
                        Range,
                        self._tof_left_callback,
                        queue_size=1
                    )
                    logger.info(f"Subscribed to ToF left sensor: {topic}")
                    break
            except Exception:
                continue
        
        for topic in right_topics:
            try:
                topics = rospy.get_published_topics()
                if any(topic in t[0] for t in topics):
                    self.tof_right_sub = rospy.Subscriber(
                        topic,
                        Range,
                        self._tof_right_callback,
                        queue_size=1
                    )
                    logger.info(f"Subscribed to ToF right sensor: {topic}")
                    break
            except Exception:
                continue
        
        if not self.tof_available:
            logger.warning("No ToF sensors found - using video-based detection only")
            logger.warning("  ToF sensors provide more accurate distance measurements")
            logger.warning("  To enable ToF sensor:")
            logger.warning("    1. Ensure ToF sensor node is running (e.g., roslaunch duckietown tof_node.launch)")
            logger.warning("    2. Check available topics: rostopic list | grep tof")
            logger.warning("    3. Verify sensor is publishing: rostopic echo /robot1/tof_node/distance")
            logger.warning(f"    4. Your ToF sensor is at I2C address 0x29 (Bus 1, Channel 6)")
    
    def _tof_front_callback(self, msg):
        """Callback for front ToF sensor"""
        with self.tof_lock:
            # Range message contains range field (distance in meters)
            raw_distance = None
            if hasattr(msg, 'range'):
                raw_distance = msg.range
            elif hasattr(msg, 'data'):
                # Some ToF sensors publish Float32 directly
                raw_distance = msg.data
            else:
                logger.debug("Unknown ToF message format")
                return
            
            # Add to history for stability checking
            if raw_distance is not None and raw_distance > 0:
                self.tof_history.append(raw_distance)
                if len(self.tof_history) > self.tof_max_history:
                    self.tof_history.pop(0)
            
            # Apply floor filtering if enabled
            if self.tof_floor_filter_enabled:
                filtered_distance = self._filter_floor_reading(raw_distance)
                self.tof_front = filtered_distance
            else:
                self.tof_front = raw_distance
            
            # Add valid readings to cliff detection history (for detecting sudden distance increases)
            # Only add readings that passed floor filtering (or all readings if floor filter disabled)
            if self.tof_cliff_detection_enabled:
                valid_reading = self.tof_front if self.tof_front is not None else raw_distance
                if valid_reading is not None and valid_reading > 0:
                    self.tof_cliff_history.append(valid_reading)
                    if len(self.tof_cliff_history) > self.tof_cliff_max_history:
                        self.tof_cliff_history.pop(0)
    
    def _detect_cliff(self):
        """
        Detect if robot is approaching a cliff/edge by monitoring sudden increases in ToF distance
        
        A cliff is detected when:
        1. Current distance is above threshold (e.g., 1.0m - floor dropped away)
        2. Distance increased significantly from recent history (e.g., >0.5m increase)
        
        This indicates the floor dropped away (cliff/edge/table edge)
        
        Returns:
            (cliff_detected, current_distance, previous_avg_distance) tuple
        """
        if not self.tof_cliff_detection_enabled:
            return False, None, None
        
        with self.tof_lock:
            current_distance = self.tof_front
            
            # Need valid current reading and enough history
            if current_distance is None or current_distance <= 0:
                return False, None, None
            
            if len(self.tof_cliff_history) < self.tof_cliff_history_samples:
                return False, current_distance, None
            
            # Get recent history (excluding current reading)
            recent_history = self.tof_cliff_history[:-1]  # Exclude most recent (current)
            if len(recent_history) < self.tof_cliff_history_samples:
                return False, current_distance, None
            
            # Calculate average of recent readings (before current)
            recent_avg = sum(recent_history[-self.tof_cliff_history_samples:]) / len(recent_history[-self.tof_cliff_history_samples:])
            
            # Check for cliff: current distance must be:
            # 1. Above distance threshold (floor dropped away)
            # 2. Increased significantly from recent average
            distance_increase = current_distance - recent_avg
            
            if (current_distance >= self.tof_cliff_distance_threshold and 
                distance_increase >= self.tof_cliff_increase_threshold):
                logger.warning(f"CLIFF DETECTED: ToF distance jumped from {recent_avg:.3f}m to {current_distance:.3f}m "
                             f"(increase: {distance_increase:.3f}m) - floor dropped away!")
                return True, current_distance, recent_avg
        
        return False, current_distance, None
    
    def _filter_floor_reading(self, distance):
        """
        Filter out floor readings from ToF sensor
        
        Floor readings are typically:
        - Very close (below minimum valid distance)
        - Very stable (little variation over time)
        
        Args:
            distance: Raw ToF reading in meters
            
        Returns:
            Filtered distance (None if likely floor, original distance otherwise)
        """
        if distance is None or distance <= 0:
            return None
        
        # Rule 1: Ignore readings below minimum valid distance (likely floor)
        if distance < self.tof_min_valid_distance:
            if len(self.tof_history) >= self.tof_stability_samples:
                # Check if reading is stable (likely floor)
                recent_readings = self.tof_history[-self.tof_stability_samples:]
                if len(recent_readings) >= self.tof_stability_samples:
                    min_reading = min(recent_readings)
                    max_reading = max(recent_readings)
                    variation = max_reading - min_reading
                    
                    # If readings are very stable (low variation) and all below threshold,
                    # it's likely the floor
                    if variation < self.tof_stability_threshold and max_reading < self.tof_min_valid_distance:
                        logger.info(f"Filtering ToF reading {distance:.3f}m (likely floor - stable, below threshold {self.tof_min_valid_distance}m)")
                        return None  # Filter out as floor
            
            # If not stable yet, still filter if below threshold
            logger.info(f"Filtering ToF reading {distance:.3f}m (below minimum valid distance {self.tof_min_valid_distance}m - likely floor)")
            return None
        
        # Rule 2: If reading is above threshold, it's valid
        return distance
    
    def _tof_left_callback(self, msg):
        """Callback for left ToF sensor"""
        with self.tof_lock:
            raw_distance = None
            if hasattr(msg, 'range'):
                raw_distance = msg.range
            elif hasattr(msg, 'data'):
                raw_distance = msg.data
            
            # Apply floor filtering if enabled
            if self.tof_floor_filter_enabled and raw_distance is not None:
                self.tof_left = self._filter_floor_reading(raw_distance)
            else:
                self.tof_left = raw_distance
    
    def _tof_right_callback(self, msg):
        """Callback for right ToF sensor"""
        with self.tof_lock:
            raw_distance = None
            if hasattr(msg, 'range'):
                raw_distance = msg.range
            elif hasattr(msg, 'data'):
                raw_distance = msg.data
            
            # Apply floor filtering if enabled
            if self.tof_floor_filter_enabled and raw_distance is not None:
                self.tof_right = self._filter_floor_reading(raw_distance)
            else:
                self.tof_right = raw_distance
    
    def detect_obstacles_video(self, frame):
        """
        Detect obstacles using video analysis
        
        Uses:
        - Edge detection to find vertical edges (potential obstacles)
        - Depth estimation from motion (if frame history available)
        - Lower region analysis (ground obstacles)
        
        Args:
            frame: Current camera frame (BGR format)
        
        Returns:
            (obstacle_detected, distance_estimate, position)
            - obstacle_detected: bool
            - distance_estimate: float (meters) or None
            - position: (x, y) normalized coordinates or None
        """
        if frame is None:
            return False, None, None
        
        height, width = frame.shape[:2]
        
        # Focus on lower 40% of frame (ground obstacles)
        roi_bottom = int(height * 0.6)
        roi = frame[roi_bottom:, :]
        
        # Convert to grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Edge detection (Canny)
        edges = cv2.Canny(blurred, 50, 150)
        
        # Find vertical lines (potential obstacles)
        # Use HoughLinesP for line detection
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50,
                              minLineLength=30, maxLineGap=10)
        
        obstacle_detected = False
        obstacle_distance = None
        obstacle_x = None
        
        if lines is not None and len(lines) > 0:
            # Count vertical lines (obstacles)
            vertical_lines = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Check if line is roughly vertical (within 20 degrees)
                if abs(x2 - x1) < 10:  # Nearly vertical
                    vertical_lines.append((x1, y1, x2, y2))
            
            if len(vertical_lines) > config.COLLISION_EDGE_THRESHOLD:
                obstacle_detected = True
                
                # Estimate obstacle position (center of vertical lines)
                if vertical_lines:
                    x_coords = [l[0] for l in vertical_lines] + [l[2] for l in vertical_lines]
                    obstacle_x = np.mean(x_coords) / width  # Normalize to 0-1
                    
                    # Estimate distance based on line position in frame
                    # Lines closer to bottom = closer obstacle
                    y_coords = [l[1] for l in vertical_lines] + [l[3] for l in vertical_lines]
                    avg_y = np.mean(y_coords)
                    # Convert y position to distance estimate (heuristic)
                    # Closer to bottom (higher y) = closer obstacle
                    normalized_y = avg_y / roi.shape[0]  # 0-1 in ROI
                    # Rough distance estimate: closer objects appear larger
                    # This is a simplified model - would need calibration for accuracy
                    obstacle_distance = config.COLLISION_MAX_DISTANCE * (1.0 - normalized_y * 0.7)
        
        # Also check for large dark regions (potential obstacles)
        # Convert to HSV and look for dark regions in lower frame
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        dark_mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 50))  # Dark regions
        dark_pixels = cv2.countNonZero(dark_mask)
        dark_ratio = dark_pixels / (roi.shape[0] * roi.shape[1])
        
        if dark_ratio > config.COLLISION_DARK_REGION_THRESHOLD:
            obstacle_detected = True
            if obstacle_distance is None:
                # Estimate distance from dark region size
                obstacle_distance = config.COLLISION_MAX_DISTANCE * 0.5
        
        # Store frame for motion-based depth estimation
        self.frame_history.append(gray)
        if len(self.frame_history) > self.max_history:
            self.frame_history.pop(0)
        
        position = (obstacle_x, 0.7) if obstacle_x is not None else None
        
        return obstacle_detected, obstacle_distance, position
    
    def check_collision_risk(self, frame=None):
        """
        Check for collision risk using all available sensors
        
        Args:
            frame: Optional camera frame for video-based detection
        
        Returns:
            dict with:
            - 'risk_level': 'none', 'warning', 'emergency'
            - 'distance': float (meters) or None
            - 'position': (x, y) normalized or None
            - 'sensor': 'tof', 'video', or 'both'
            - 'should_stop': bool
            - 'should_slow': bool
        """
        result = {
            'risk_level': 'none',
            'distance': None,
            'position': None,
            'sensor': 'none',
            'should_stop': False,
            'should_slow': False
        }
        
        # Check ToF sensors (with floor filtering applied)
        tof_distance = None
        tof_available = False
        with self.tof_lock:
            if self.tof_available:
                tof_distance = self.tof_front
                # Only consider valid if distance is not None and above minimum threshold
                tof_available = (tof_distance is not None and tof_distance > 0 and 
                               tof_distance >= self.tof_min_valid_distance)
                if not tof_available and tof_distance is not None:
                    # Log why ToF reading was rejected (for debugging)
                    logger.debug(f"ToF reading {tof_distance:.3f}m rejected (below minimum valid distance {self.tof_min_valid_distance}m)")
        
        # Check video-based detection
        video_obstacle = False
        video_distance = None
        video_position = None
        
        if frame is not None:
            video_obstacle, video_distance, video_position = self.detect_obstacles_video(frame)
        
        # Fuse sensor data
        if tof_available:
            result['distance'] = tof_distance
            result['sensor'] = 'tof'
            
            if tof_distance < self.emergency_zone:
                result['risk_level'] = 'emergency'
                result['should_stop'] = True
            elif tof_distance < self.warning_zone:
                result['risk_level'] = 'warning'
                result['should_slow'] = True
        elif video_obstacle and video_distance is not None:
            result['distance'] = video_distance
            result['position'] = video_position
            result['sensor'] = 'video'
            
            if video_distance < self.emergency_zone:
                result['risk_level'] = 'emergency'
                result['should_stop'] = True
            elif video_distance < self.warning_zone:
                result['risk_level'] = 'warning'
                result['should_slow'] = True
        
        # If both sensors agree, increase confidence
        if tof_available and video_obstacle:
            # Fuse distances (weighted average, ToF is more accurate)
            fused_distance = (tof_distance * 0.7 + video_distance * 0.3) if video_distance else tof_distance
            result['distance'] = fused_distance
            result['sensor'] = 'both'
            
            if fused_distance < self.emergency_zone:
                result['risk_level'] = 'emergency'
                result['should_stop'] = True
            elif fused_distance < self.warning_zone:
                result['risk_level'] = 'warning'
                result['should_slow'] = True
        
        # Check side sensors for lateral obstacles
        with self.tof_lock:
            if self.tof_left is not None and self.tof_left < self.warning_zone:
                result['risk_level'] = 'warning'
                result['should_slow'] = True
            if self.tof_right is not None and self.tof_right < self.warning_zone:
                result['risk_level'] = 'warning'
                result['should_slow'] = True
        
        # CRITICAL SAFETY: Check for cliff/edge detection (prevents driving off ledges)
        # This takes absolute priority - if cliff detected, force emergency stop
        cliff_detected, cliff_distance, previous_distance = self._detect_cliff()
        if cliff_detected:
            result['risk_level'] = 'emergency'
            result['should_stop'] = True
            result['distance'] = cliff_distance
            result['sensor'] = 'tof_cliff'
            logger.error(f"EMERGENCY STOP: Cliff/edge detected! Distance jumped from {previous_distance:.3f}m to {cliff_distance:.3f}m")
        
        return result
    
    def get_safe_speed(self, base_speed, collision_risk):
        """
        Calculate safe speed based on collision risk
        
        Args:
            base_speed: Desired base speed (normalized 0-1)
            collision_risk: Result from check_collision_risk()
        
        Returns:
            Safe speed (normalized 0-1)
        """
        if collision_risk['should_stop']:
            return 0.0
        
        if collision_risk['should_slow']:
            # Reduce speed proportionally to distance
            if collision_risk['distance'] is not None:
                # Linear reduction: full speed at safe_zone, zero at emergency_zone
                distance = collision_risk['distance']
                if distance < self.warning_zone:
                    # Scale between warning_zone and emergency_zone
                    ratio = (distance - self.emergency_zone) / (self.warning_zone - self.emergency_zone)
                    ratio = max(0.0, min(1.0, ratio))  # Clamp to 0-1
                    return base_speed * ratio * config.COLLISION_SPEED_REDUCTION
            return base_speed * config.COLLISION_SPEED_REDUCTION
        
        return base_speed
    
    def draw_overlay(self, frame, collision_risk):
        """
        Draw collision avoidance information on frame
        
        Args:
            frame: Frame to draw on
            collision_risk: Result from check_collision_risk()
        """
        if frame is None:
            return
        
        height, width = frame.shape[:2]
        
        # Draw ToF sensor readings
        y_offset = 120
        if collision_risk['sensor'] in ['tof', 'both']:
            distance_text = f"ToF: {collision_risk['distance']:.2f}m" if collision_risk['distance'] else "ToF: N/A"
            color = (0, 255, 255)  # Cyan
            cv2.putText(frame, distance_text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            y_offset += 25
        
        if collision_risk['sensor'] in ['video', 'both']:
            video_text = f"Video: {collision_risk['distance']:.2f}m" if collision_risk['distance'] else "Video: Detected"
            color = (255, 255, 0)  # Yellow
            cv2.putText(frame, video_text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            y_offset += 25
        
        # Draw risk level
        risk_colors = {
            'none': (0, 255, 0),      # Green
            'warning': (0, 165, 255),  # Orange
            'emergency': (0, 0, 255)   # Red
        }
        risk_color = risk_colors.get(collision_risk['risk_level'], (255, 255, 255))
        risk_text = f"Collision Risk: {collision_risk['risk_level'].upper()}"
        cv2.putText(frame, risk_text, (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, risk_color, 2)
        
        # Draw obstacle position if available
        if collision_risk['position']:
            x, y = collision_risk['position']
            pixel_x = int(x * width)
            pixel_y = int(y * height)
            cv2.circle(frame, (pixel_x, pixel_y), 30, risk_color, 3)
            cv2.putText(frame, "OBSTACLE", (pixel_x - 50, pixel_y - 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, risk_color, 2)
        
        # Draw safety zones (visual indicator)
        if collision_risk['distance'] is not None:
            # Draw distance bar
            bar_width = 200
            bar_height = 20
            bar_x = width - bar_width - 10
            bar_y = height - 50
            
            # Background
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height),
                         (50, 50, 50), -1)
            
            # Color based on risk
            if collision_risk['risk_level'] == 'emergency':
                bar_color = (0, 0, 255)  # Red
            elif collision_risk['risk_level'] == 'warning':
                bar_color = (0, 165, 255)  # Orange
            else:
                bar_color = (0, 255, 0)  # Green
            
            # Fill based on distance (closer = more filled)
            fill_ratio = 1.0 - (collision_risk['distance'] / self.safe_zone)
            fill_ratio = max(0.0, min(1.0, fill_ratio))
            fill_width = int(bar_width * fill_ratio)
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + fill_width, bar_y + bar_height),
                         bar_color, -1)
            
            # Border
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height),
                         (255, 255, 255), 2)
    
    def cleanup(self):
        """Clean up resources"""
        if hasattr(self, 'tof_front_sub') and self.tof_front_sub is not None:
            try:
                self.tof_front_sub.unregister()
            except Exception:
                pass
        if hasattr(self, 'tof_left_sub') and self.tof_left_sub is not None:
            try:
                self.tof_left_sub.unregister()
            except Exception:
                pass
        if hasattr(self, 'tof_right_sub') and self.tof_right_sub is not None:
            try:
                self.tof_right_sub.unregister()
            except Exception:
                pass
        logger.info("Collision avoidance system cleaned up")

