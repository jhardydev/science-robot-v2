"""
Wave detection module - tracks hand motion to detect waving gestures
"""
import numpy as np
import time
from science_robot import config
from collections import deque


class WaveDetector:
    """Detects waving motion patterns from hand tracking"""
    
    def __init__(self, history_size=None, motion_threshold=None):
        """
        Initialize wave detector
        
        Args:
            history_size: Number of frames to track (default from config)
            motion_threshold: Minimum motion to consider waving (default from config)
        """
        self.history_size = history_size or config.WAVE_DETECTION_FRAMES
        self.motion_threshold = motion_threshold or config.WAVE_MOTION_THRESHOLD
        self.min_duration = config.WAVE_MIN_DURATION
        self.sensitivity = config.WAVE_SENSITIVITY
        
        # Store hand positions over time
        self.hand_history = deque(maxlen=self.history_size)
        self.time_history = deque(maxlen=self.history_size)
        
        # Wave detection state
        self.wave_detected = False
        self.wave_position = None  # (x, y) in normalized coordinates (hand position)
        self.face_position = None  # (x, y) in normalized coordinates (associated face position)
        self.wave_start_time = None
        
        # Face-hand association parameters
        self.max_face_distance = 0.4  # Maximum normalized distance to associate hand with face (increased from 0.3 for better association)
        
        # Face locking mechanism - maintains face lock across frames
        self.locked_face_position = None  # Persists across frames once locked
        self.locked_face_time = None  # When face was locked
        self.face_lock_timeout = 3.0  # Keep lock for 3 seconds even if face not detected
        self.face_lock_distance = 0.15  # Max distance to re-associate with same face (position matching)
    
    def associate_hand_with_face(self, hand_position, faces_data):
        """
        Find the face closest to a waving hand
        
        Args:
            hand_position: (x, y) normalized coordinates of hand
            faces_data: List of face detection dicts with 'center' key
            
        Returns:
            Face center position (x, y) if found within max_face_distance, None otherwise
        """
        if not faces_data or hand_position is None:
            return None
        
        hand_x, hand_y = hand_position
        closest_face = None
        min_distance = float('inf')
        
        for face in faces_data:
            face_x, face_y = face['center']
            # Calculate Euclidean distance in normalized coordinates
            distance = np.sqrt((hand_x - face_x)**2 + (hand_y - face_y)**2)
            
            if distance < self.max_face_distance and distance < min_distance:
                min_distance = distance
                closest_face = face['center']
        
        return closest_face
    
    def _find_locked_face(self, faces_data):
        """
        Find the locked face in current detections by position matching
        Uses position proximity to maintain lock even if association fails temporarily
        
        Args:
            faces_data: List of face detection dicts with 'center' key
            
        Returns:
            Face center position (x, y) if found within lock_distance, None otherwise
        """
        if not self.locked_face_position or not faces_data:
            return None
        
        locked_x, locked_y = self.locked_face_position
        for face in faces_data:
            face_x, face_y = face['center']
            distance = np.sqrt((locked_x - face_x)**2 + (locked_y - face_y)**2)
            if distance < self.face_lock_distance:
                return face['center']
        return None
    
    def update(self, hand_landmarks_list, faces_data=None):
        """
        Update detector with new hand position data
        
        Args:
            hand_landmarks_list: List of hand landmark arrays from gesture detector
            faces_data: Optional list of face detection dicts with 'center' key
            
        Returns:
            (is_waving, target_position, face_position) tuple where:
            - is_waving: bool indicating if waving is detected
            - target_position: (x, y) normalized coordinates to track (face if available, else hand)
            - face_position: (x, y) normalized coordinates of associated face, or None
        """
        current_time = time.time()
        
        # Get the first hand (primary hand)
        if not hand_landmarks_list:
            # No hands detected - but maintain face lock if we have one
            self.hand_history.append(None)
            self.time_history.append(current_time)
            self.wave_detected = False
            
            # Maintain face lock even without hands
            if self.locked_face_position:
                locked_face_found = self._find_locked_face(faces_data) if faces_data else None
                if locked_face_found:
                    self.locked_face_position = locked_face_found
                    self.locked_face_time = current_time
                    self.face_position = locked_face_found
                elif self.locked_face_time and (current_time - self.locked_face_time) < self.face_lock_timeout:
                    # Keep using locked face even if not detected this frame
                    self.face_position = self.locked_face_position
                else:
                    # Lock expired
                    self.locked_face_position = None
                    self.locked_face_time = None
                    self.face_position = None
            else:
                self.face_position = None
            
            return False, self.face_position if self.face_position else None, self.face_position
        
        # Use the first detected hand
        primary_hand = hand_landmarks_list[0]
        hand_center_x, hand_center_y = self._get_hand_center(primary_hand)
        
        # Add to history
        self.hand_history.append((hand_center_x, hand_center_y))
        self.time_history.append(current_time)
        
        # Analyze motion if we have enough history
        if len(self.hand_history) < self.history_size:
            self.wave_detected = False
            self.face_position = None
            return False, None, None
        
        # Check for waving motion
        is_waving, confidence = self._detect_wave_motion()
        
        if is_waving:
            # Calculate average position during wave
            valid_positions = [pos for pos in self.hand_history if pos is not None]
            if valid_positions:
                avg_x = np.mean([pos[0] for pos in valid_positions])
                avg_y = np.mean([pos[1] for pos in valid_positions])
                self.wave_position = (avg_x, avg_y)
                
                # Try to associate hand with a face (new association attempt)
                new_face_position = None
                if faces_data:
                    new_face_position = self.associate_hand_with_face(
                        self.wave_position, faces_data
                    )
                
                # Face locking logic: maintain lock if we have one, otherwise establish new lock
                if self.locked_face_position:
                    # We have a locked face - try to maintain it
                    locked_face_found = self._find_locked_face(faces_data) if faces_data else None
                    if locked_face_found:
                        # Found the locked face - update position and refresh lock
                        self.locked_face_position = locked_face_found
                        self.locked_face_time = current_time
                        self.face_position = locked_face_found
                        import logging
                        logger = logging.getLogger(__name__)
                        logger.debug(f"Maintained face lock: {self.face_position}")
                    elif (current_time - self.locked_face_time) < self.face_lock_timeout:
                        # Keep using locked face even if not detected this frame
                        self.face_position = self.locked_face_position
                        import logging
                        logger = logging.getLogger(__name__)
                        logger.debug(f"Using locked face (not detected this frame): {self.face_position}")
                    else:
                        # Lock expired - try new association or clear
                        self.locked_face_position = None
                        self.locked_face_time = None
                        if new_face_position:
                            # Establish new lock
                            self.locked_face_position = new_face_position
                            self.locked_face_time = current_time
                            self.face_position = new_face_position
                            import logging
                            logger = logging.getLogger(__name__)
                            logger.info(f"Face lock expired, established new lock: {self.face_position}")
                        else:
                            self.face_position = None
                else:
                    # No locked face - establish lock if we found one
                    if new_face_position:
                        self.locked_face_position = new_face_position
                        self.locked_face_time = current_time
                        self.face_position = new_face_position
                        import logging
                        logger = logging.getLogger(__name__)
                        logger.info(f"Established face lock: {self.face_position} (hand was at {self.wave_position})")
                    else:
                        self.face_position = None
                        import logging
                        logger = logging.getLogger(__name__)
                        logger.debug(f"No face associated with waving hand (hand={self.wave_position}, {len(faces_data) if faces_data else 0} faces detected)")
                
                # Use face position as target if available, otherwise use hand position
                target_position = self.face_position if self.face_position else self.wave_position
                
                # Log which target we're using
                import logging
                logger = logging.getLogger(__name__)
                if self.face_position:
                    logger.info(f"TRACKING FACE at {target_position} (hand was at {self.wave_position})")
                else:
                    logger.debug(f"Tracking hand at {target_position} (no face associated)")
                
                # Check if wave has been sustained long enough
                if self.wave_start_time is None:
                    self.wave_start_time = current_time
                
                wave_duration = current_time - self.wave_start_time
                if wave_duration >= self.min_duration:
                    self.wave_detected = True
                    return True, target_position, self.face_position
        else:
            # Not actively waving - but maintain face lock if we have one
            self.wave_start_time = None
            self.wave_detected = False
            
            if self.locked_face_position:
                # Try to maintain face lock even when not waving
                locked_face_found = self._find_locked_face(faces_data) if faces_data else None
                if locked_face_found:
                    # Found the locked face - update position and refresh lock
                    self.locked_face_position = locked_face_found
                    self.locked_face_time = current_time
                    self.face_position = locked_face_found
                elif (current_time - self.locked_face_time) < self.face_lock_timeout:
                    # Keep using locked face even if not detected this frame
                    self.face_position = self.locked_face_position
                else:
                    # Lock expired
                    self.locked_face_position = None
                    self.locked_face_time = None
                    self.face_position = None
            else:
                self.face_position = None
        
        # Return last known position if still in tracking state
        if self.wave_detected:
            target_position = self.face_position if self.face_position else self.wave_position
            return False, target_position, self.face_position
        
        return False, None, None
    
    def _get_hand_center(self, landmarks):
        """
        Get hand center from landmarks (uses average of wrist, palm, and finger base)
        More stable than just wrist position for better tracking
        
        Args:
            landmarks: Array of hand landmarks
            
        Returns:
            (x, y) tuple in normalized coordinates
        """
        # Use wrist (0), middle finger base (9 - palm center), and index finger base (5)
        # Average of these three points provides more stable tracking
        wrist = landmarks[0]
        palm = landmarks[9]  # Middle finger base (palm center)
        index_base = landmarks[5]  # Index finger base
        
        # Average of these three points for more stable tracking
        center_x = (wrist[0] + palm[0] + index_base[0]) / 3.0
        center_y = (wrist[1] + palm[1] + index_base[1]) / 3.0
        
        return center_x, center_y
    
    def _detect_wave_motion(self):
        """
        Analyze hand history to detect waving motion pattern
        
        Returns:
            (is_waving, confidence) tuple
        """
        # Filter out None values
        valid_positions = [(pos, t) for pos, t in zip(self.hand_history, self.time_history) if pos is not None]
        
        if len(valid_positions) < 3:
            return False, 0.0
        
        # Extract x and y coordinates
        positions = [pos for pos, _ in valid_positions]
        x_coords = [pos[0] for pos in positions]
        y_coords = [pos[1] for pos in positions]
        
        # Calculate horizontal (lateral) movement
        x_range = max(x_coords) - min(x_coords)
        y_range = max(y_coords) - min(y_coords)
        
        # Waving is primarily horizontal movement
        # Convert to pixel space (assuming 640px width)
        x_movement_pixels = x_range * 640
        
        # Check for oscillatory pattern (sign changes in velocity)
        if len(x_coords) >= 3:
            velocities = [x_coords[i] - x_coords[i-1] for i in range(1, len(x_coords))]
            sign_changes = sum(1 for i in range(1, len(velocities)) 
                             if (velocities[i] > 0) != (velocities[i-1] > 0))
            
            # Waving should have direction changes (oscillation)
            # Reduced from >= 2 to >= 1 for more lenient detection
            oscillatory = sign_changes >= 1
        else:
            oscillatory = False
        
        # Check motion threshold
        has_motion = x_movement_pixels >= self.motion_threshold
        
        # Waving requires both significant horizontal motion and oscillatory pattern
        # Reduced horizontal/vertical ratio from 1.5 to 1.2 for more lenient detection
        is_waving = has_motion and oscillatory and (x_range > y_range * 1.2)
        
        # Calculate confidence
        if is_waving:
            confidence = min(1.0, (x_movement_pixels / self.motion_threshold) * self.sensitivity)
        else:
            confidence = 0.0
        
        return is_waving, confidence
    
    def update_parameters(self, history_size=None, motion_threshold=None,
                         min_duration=None, sensitivity=None):
        """
        Update wave detection parameters in real-time
        
        Args:
            history_size: Number of frames to track
            motion_threshold: Minimum pixel movement to consider as waving
            min_duration: Minimum seconds of waving to trigger
            sensitivity: Sensitivity multiplier for wave detection (0.0-1.0)
        """
        import logging
        logger = logging.getLogger(__name__)
        
        if history_size is not None:
            self.history_size = max(3, int(history_size))  # Minimum 3 frames
            # Resize history deques
            old_history = list(self.hand_history)
            old_time_history = list(self.time_history)
            self.hand_history = deque(old_history, maxlen=self.history_size)
            self.time_history = deque(old_time_history, maxlen=self.history_size)
            logger.info(f"Updated wave detection frames: {self.history_size}")
        
        if motion_threshold is not None:
            self.motion_threshold = max(1, int(motion_threshold))
            logger.info(f"Updated wave motion threshold: {self.motion_threshold} pixels")
        
        if min_duration is not None:
            self.min_duration = max(0.1, float(min_duration))
            logger.info(f"Updated wave min duration: {self.min_duration}s")
        
        if sensitivity is not None:
            self.sensitivity = max(0.0, min(1.0, float(sensitivity)))
            logger.info(f"Updated wave sensitivity: {self.sensitivity}")

    def get_parameters(self):
        """
        Get current wave detection parameters
        
        Returns:
            dict with current parameter values
        """
        return {
            'history_size': self.history_size,
            'motion_threshold': self.motion_threshold,
            'min_duration': self.min_duration,
            'sensitivity': self.sensitivity
        }
    
    def reset(self):
        """Reset detector state"""
        self.hand_history.clear()
        self.time_history.clear()
        self.wave_detected = False
        self.wave_position = None
        self.face_position = None
        self.wave_start_time = None
        # Clear face lock on reset
        self.locked_face_position = None
        self.locked_face_time = None

