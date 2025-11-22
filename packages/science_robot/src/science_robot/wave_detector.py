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
        self.wave_position = None  # (x, y) in normalized coordinates
        self.wave_start_time = None
    
    def update(self, hand_landmarks_list):
        """
        Update detector with new hand position data
        
        Args:
            hand_landmarks_list: List of hand landmark arrays from gesture detector
            
        Returns:
            (is_waving, position) tuple where is_waving is bool and position is (x, y)
        """
        current_time = time.time()
        
        # Get the first hand (primary hand)
        if not hand_landmarks_list:
            # No hands detected
            self.hand_history.append(None)
            self.time_history.append(current_time)
            self.wave_detected = False
            return False, None
        
        # Use the first detected hand
        primary_hand = hand_landmarks_list[0]
        hand_center_x, hand_center_y = self._get_hand_center(primary_hand)
        
        # Add to history
        self.hand_history.append((hand_center_x, hand_center_y))
        self.time_history.append(current_time)
        
        # Analyze motion if we have enough history
        if len(self.hand_history) < self.history_size:
            self.wave_detected = False
            return False, None
        
        # Check for waving motion
        is_waving, confidence = self._detect_wave_motion()
        
        if is_waving:
            # Calculate average position during wave
            valid_positions = [pos for pos in self.hand_history if pos is not None]
            if valid_positions:
                avg_x = np.mean([pos[0] for pos in valid_positions])
                avg_y = np.mean([pos[1] for pos in valid_positions])
                self.wave_position = (avg_x, avg_y)
                
                # Check if wave has been sustained long enough
                if self.wave_start_time is None:
                    self.wave_start_time = current_time
                
                wave_duration = current_time - self.wave_start_time
                if wave_duration >= self.min_duration:
                    self.wave_detected = True
                    return True, self.wave_position
        else:
            self.wave_start_time = None
            self.wave_detected = False
        
        return False, self.wave_position if self.wave_detected else None
    
    def _get_hand_center(self, landmarks):
        """
        Get hand center from landmarks (uses wrist, index 0)
        
        Args:
            landmarks: Array of hand landmarks
            
        Returns:
            (x, y) tuple in normalized coordinates
        """
        return landmarks[0][0], landmarks[0][1]
    
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
            
            # Waving should have multiple direction changes (oscillation)
            oscillatory = sign_changes >= 2
        else:
            oscillatory = False
        
        # Check motion threshold
        has_motion = x_movement_pixels >= self.motion_threshold
        
        # Waving requires both significant horizontal motion and oscillatory pattern
        is_waving = has_motion and oscillatory and (x_range > y_range * 1.5)
        
        # Calculate confidence
        if is_waving:
            confidence = min(1.0, (x_movement_pixels / self.motion_threshold) * self.sensitivity)
        else:
            confidence = 0.0
        
        return is_waving, confidence
    
    def reset(self):
        """Reset detector state"""
        self.hand_history.clear()
        self.time_history.clear()
        self.wave_detected = False
        self.wave_position = None
        self.wave_start_time = None

