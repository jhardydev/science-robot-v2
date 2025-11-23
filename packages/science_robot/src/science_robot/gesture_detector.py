"""
Gesture detector using MediaPipe for hand detection and gesture recognition
"""
import cv2
import numpy as np
from science_robot import config

# Try to import MediaPipe - handle gracefully if not available (common on ARM64)
try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False
    mp = None
    import logging
    logger = logging.getLogger(__name__)
    logger.warning("MediaPipe not available - gesture detection will be disabled")
    logger.warning("Install with: pip3 install mediapipe (may require building from source on ARM64)")


class GestureDetector:
    """Hand gesture detection using MediaPipe"""
    
    # Finger landmark indices for MediaPipe hands
    FINGER_TIPS = [4, 8, 12, 16, 20]  # Thumb, Index, Middle, Ring, Pinky
    FINGER_PIPS = [3, 6, 10, 14, 18]  # Joints before tips
    
    def __init__(self, min_detection_confidence=0.5, min_tracking_confidence=0.5):
        """
        Initialize MediaPipe hands detector
        
        Args:
            min_detection_confidence: Minimum confidence for hand detection
            min_tracking_confidence: Minimum confidence for hand tracking
        """
        if not MEDIAPIPE_AVAILABLE:
            self.mp_hands = None
            self.mp_drawing = None
            self.hands = None
            self.min_detection_confidence = min_detection_confidence
            self.min_tracking_confidence = min_tracking_confidence
            import logging
            logger = logging.getLogger(__name__)
            logger.error("MediaPipe not available - GestureDetector cannot be used")
            return
        
        # Store confidence values for runtime updates
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence = min_tracking_confidence
        
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )
    
    def detect_hands(self, frame):
        """
        Detect hands in a frame
        
        Args:
            frame: BGR image frame
            
        Returns:
            List of hand landmarks (each is a list of 21 landmark points)
        """
        if not MEDIAPIPE_AVAILABLE or self.hands is None:
            return [], None
        
        # Convert BGR to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process frame
        results = self.hands.process(rgb_frame)
        
        hands_data = []
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Convert landmarks to numpy array
                landmarks = []
                for lm in hand_landmarks.landmark:
                    landmarks.append([lm.x, lm.y, lm.z])
                hands_data.append(np.array(landmarks))
        
        return hands_data, results
    
    def get_finger_states(self, landmarks):
        """
        Determine which fingers are extended
        
        Args:
            landmarks: Array of 21 hand landmarks
            
        Returns:
            List of 5 booleans [thumb, index, middle, ring, pinky]
        """
        finger_states = []
        
        # For each finger (except thumb)
        for i in range(1, 5):
            # Check if tip is above the pip joint (finger extended)
            tip_y = landmarks[self.FINGER_TIPS[i]][1]
            pip_y = landmarks[self.FINGER_PIPS[i]][1]
            finger_states.append(tip_y < pip_y)
        
        # Thumb is different - check x coordinate
        thumb_tip_x = landmarks[self.FINGER_TIPS[0]][0]
        thumb_pip_x = landmarks[self.FINGER_PIPS[0]][0]
        thumb_extended = thumb_tip_x > thumb_pip_x
        finger_states.insert(0, thumb_extended)
        
        return finger_states
    
    def classify_gesture(self, hands):
        """
        Classify gesture based on one or more detected hands
        
        Args:
            hands: List of arrays of 21 hand landmarks
            
        Returns:
            String indicating gesture type: 'wave', 'dance', 'treat', or None
        """
        if not hands:
            return None
        
        # Check for two-hand clapping gesture (duck bill)
        if len(hands) >= 2:
            for i in range(len(hands)):
                for j in range(i + 1, len(hands)):
                    if self._is_duck_bill_clap(hands[i], hands[j]):
                        return 'dance'
        
        # Check for treat gesture on any hand
        for hand in hands:
            if self._is_treat_gesture(hand):
                return 'treat'
        
        # Wave detection is handled separately by tracking motion
        # This method focuses on static gestures
        return None
    
    def _is_duck_bill_clap(self, hand_a, hand_b):
        """
        Determine if two hands are in a clapping "duck bill" pose
        
        Args:
            hand_a: Landmarks for first hand
            hand_b: Landmarks for second hand
            
        Returns:
            True if hands form the clap gesture, False otherwise
        """
        # Require palms close together
        palm_distance = self._landmark_distance(hand_a, hand_b, 9)
        wrist_distance = self._landmark_distance(hand_a, hand_b, 0)
        if palm_distance > config.DANCE_CLAP_PALM_THRESHOLD and wrist_distance > config.DANCE_CLAP_PALM_THRESHOLD:
            return False
        
        # Require fingertips to be close (thumbs and index fingers)
        thumb_distance = self._landmark_distance(hand_a, hand_b, self.FINGER_TIPS[0])
        index_distance = self._landmark_distance(hand_a, hand_b, self.FINGER_TIPS[1])
        
        if thumb_distance is None or index_distance is None:
            return False
        
        if thumb_distance > config.DANCE_CLAP_FINGER_THRESHOLD:
            return False
        
        if index_distance > config.DANCE_CLAP_FINGER_THRESHOLD:
            return False
        
        return True
    
    def _is_treat_gesture(self, landmarks):
        """
        Determine if landmarks represent the treat gesture (thumb and pinky extended)
        """
        finger_states = self.get_finger_states(landmarks)
        thumb, index, middle, ring, pinky = finger_states
        return thumb and not index and not middle and not ring and pinky
    
    def _landmark_distance(self, hand_a, hand_b, landmark_index):
        """
        Calculate Euclidean distance between a specific landmark on two hands
        using 2D normalized coordinates.
        """
        try:
            point_a = np.array(hand_a[landmark_index][:2])
            point_b = np.array(hand_b[landmark_index][:2])
        except (IndexError, TypeError):
            return None
        
        if np.any(np.isnan(point_a)) or np.any(np.isnan(point_b)):
            return None
        
        return np.linalg.norm(point_a - point_b)
    
    def get_hand_center(self, landmarks):
        """
        Get center position of hand in normalized coordinates
        
        Args:
            landmarks: Array of 21 hand landmarks
            
        Returns:
            (x, y) tuple in normalized coordinates (0.0 to 1.0)
        """
        # Use wrist landmark (index 0) as hand center
        return landmarks[0][0], landmarks[0][1]
    
    def get_bounding_box(self, landmarks, frame_width, frame_height):
        """
        Calculate bounding box for hand landmarks
        
        Args:
            landmarks: List or numpy array of 21 hand landmarks (normalized 0.0-1.0)
            frame_width: Frame width in pixels
            frame_height: Frame height in pixels
            
        Returns:
            (x, y, width, height) bounding box in pixel coordinates, or None
        """
        if landmarks is None:
            return None
        
        # Handle numpy arrays - check length properly
        try:
            if len(landmarks) == 0:
                return None
        except (TypeError, AttributeError):
            return None
        
        # Get min/max x and y coordinates
        x_coords = [lm[0] for lm in landmarks]
        y_coords = [lm[1] for lm in landmarks]
        
        min_x = min(x_coords)
        max_x = max(x_coords)
        min_y = min(y_coords)
        max_y = max(y_coords)
        
        # Convert normalized coordinates to pixel coordinates
        x = int(min_x * frame_width)
        y = int(min_y * frame_height)
        width = int((max_x - min_x) * frame_width)
        height = int((max_y - min_y) * frame_height)
        
        # Add padding (10% on each side)
        padding_x = int(width * 0.1)
        padding_y = int(height * 0.1)
        x = max(0, x - padding_x)
        y = max(0, y - padding_y)
        width = min(frame_width - x, width + 2 * padding_x)
        height = min(frame_height - y, height + 2 * padding_y)
        
        return (x, y, width, height)
    
    def draw_landmarks(self, frame, results, hands_data=None, draw_bbox=True, 
                       is_waving=False, current_gesture=None):
        """
        Draw hand landmarks and bounding boxes on frame (for visualization)
        
        Args:
            frame: BGR image frame
            results: MediaPipe results object
            hands_data: Optional list of hand landmark arrays (for bounding box calculation)
            draw_bbox: Whether to draw bounding boxes around detected hands
            is_waving: Whether waving motion is detected
            current_gesture: Current detected gesture ('dance', 'treat', or None)
        """
        if not MEDIAPIPE_AVAILABLE or self.mp_drawing is None or results is None:
            return
        
        height, width = frame.shape[:2]
        
        if results.multi_hand_landmarks:
            for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                # Draw landmarks
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                    self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2)
                )
                
                # Draw bounding box if requested
                if draw_bbox:
                    # Get landmarks as array for bounding box calculation
                    if hands_data and idx < len(hands_data):
                        landmarks = hands_data[idx]
                    else:
                        # Convert MediaPipe landmarks to array format
                        landmarks = []
                        for lm in hand_landmarks.landmark:
                            landmarks.append([lm.x, lm.y, lm.z])
                    
                    bbox = self.get_bounding_box(landmarks, width, height)
                    if bbox:
                        x, y, w, h = bbox
                        
                        # Determine box color and label based on detected motion/gesture
                        box_color = (0, 255, 0)  # Default green
                        label_parts = []
                        
                        # Check for static gestures first (they take priority)
                        if hands_data:
                            gesture = self.classify_gesture([landmarks])
                            if gesture:
                                if gesture == 'dance':
                                    box_color = (255, 165, 0)  # Orange for dance
                                    label_parts.append("DANCE")
                                elif gesture == 'treat':
                                    box_color = (255, 0, 255)  # Magenta for treat
                                    label_parts.append("TREAT")
                        
                        # Add waving status (can be combined with gestures)
                        if is_waving:
                            label_parts.append("WAVING")
                            # Use yellow for waving if no gesture detected
                            if not current_gesture:
                                box_color = (0, 255, 255)  # Yellow/Cyan for waving
                        
                        # Default label if nothing detected
                        if not label_parts:
                            label_parts.append("HAND")
                        
                        # Draw rectangle with appropriate color
                        cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
                        
                        # Draw label above bounding box with background for readability
                        label_text = " | ".join(label_parts)
                        label_y = max(20, y - 5)
                        
                        # Calculate text size for background
                        (text_width, text_height), baseline = cv2.getTextSize(
                            label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
                        )
                        
                        # Draw background rectangle for text
                        cv2.rectangle(frame, 
                                    (x, label_y - text_height - 5), 
                                    (x + text_width + 4, label_y + baseline), 
                                    (0, 0, 0), -1)  # Black background
                        
                        # Draw text
                        cv2.putText(frame, label_text, (x + 2, label_y),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
    
    def update_parameters(self, min_detection_confidence=None, min_tracking_confidence=None,
                         gesture_confidence_threshold=None, dance_hold_time=None,
                         treat_hold_time=None, clap_finger_threshold=None,
                         clap_palm_threshold=None):
        """
        Update gesture detection parameters in real-time
        
        Args:
            min_detection_confidence: Minimum confidence for hand detection (0.0-1.0)
            min_tracking_confidence: Minimum confidence for hand tracking (0.0-1.0)
            gesture_confidence_threshold: Overall confidence threshold for gesture classification
            dance_hold_time: How long dance gesture must be held (seconds)
            treat_hold_time: How long treat gesture must be held (seconds)
            clap_finger_threshold: Max distance between fingertips for clap (normalized)
            clap_palm_threshold: Max distance between palms for clap (normalized)
        """
        import logging
        logger = logging.getLogger(__name__)
        
        # If MediaPipe confidence changes, recreate the Hands object
        if min_detection_confidence is not None or min_tracking_confidence is not None:
            if not MEDIAPIPE_AVAILABLE:
                logger.warning("MediaPipe not available, cannot update detection/tracking confidence")
            else:
                if self.hands is not None:
                    self.hands.close()
                
                new_detection = min_detection_confidence if min_detection_confidence is not None else self.min_detection_confidence
                new_tracking = min_tracking_confidence if min_tracking_confidence is not None else self.min_tracking_confidence
                
                # Clamp values to valid range
                new_detection = max(0.0, min(1.0, float(new_detection)))
                new_tracking = max(0.0, min(1.0, float(new_tracking)))
                
                self.min_detection_confidence = new_detection
                self.min_tracking_confidence = new_tracking
                
                self.hands = self.mp_hands.Hands(
                    static_image_mode=False,
                    max_num_hands=2,
                    min_detection_confidence=self.min_detection_confidence,
                    min_tracking_confidence=self.min_tracking_confidence
                )
                logger.info(f"Updated MediaPipe confidence: detection={self.min_detection_confidence:.2f}, tracking={self.min_tracking_confidence:.2f}")
        
        # Update config values (these are used by gesture classification)
        if gesture_confidence_threshold is not None:
            config.GESTURE_CONFIDENCE_THRESHOLD = max(0.0, min(1.0, float(gesture_confidence_threshold)))
            logger.info(f"Updated gesture confidence threshold: {config.GESTURE_CONFIDENCE_THRESHOLD:.2f}")
        
        if dance_hold_time is not None:
            config.DANCE_GESTURE_HOLD_TIME = max(0.1, float(dance_hold_time))
            logger.info(f"Updated dance hold time: {config.DANCE_GESTURE_HOLD_TIME:.2f}s")
        
        if treat_hold_time is not None:
            config.TREAT_GESTURE_HOLD_TIME = max(0.1, float(treat_hold_time))
            logger.info(f"Updated treat hold time: {config.TREAT_GESTURE_HOLD_TIME:.2f}s")
        
        if clap_finger_threshold is not None:
            config.DANCE_CLAP_FINGER_THRESHOLD = max(0.01, min(1.0, float(clap_finger_threshold)))
            logger.info(f"Updated clap finger threshold: {config.DANCE_CLAP_FINGER_THRESHOLD:.3f}")
        
        if clap_palm_threshold is not None:
            config.DANCE_CLAP_PALM_THRESHOLD = max(0.01, min(1.0, float(clap_palm_threshold)))
            logger.info(f"Updated clap palm threshold: {config.DANCE_CLAP_PALM_THRESHOLD:.3f}")

    def get_parameters(self):
        """
        Get current gesture detection parameters
        
        Returns:
            dict with current parameter values
        """
        return {
            'min_detection_confidence': self.min_detection_confidence,
            'min_tracking_confidence': self.min_tracking_confidence,
            'gesture_confidence_threshold': config.GESTURE_CONFIDENCE_THRESHOLD,
            'dance_hold_time': config.DANCE_GESTURE_HOLD_TIME,
            'treat_hold_time': config.TREAT_GESTURE_HOLD_TIME,
            'clap_finger_threshold': config.DANCE_CLAP_FINGER_THRESHOLD,
            'clap_palm_threshold': config.DANCE_CLAP_PALM_THRESHOLD
        }
    
    def close(self):
        """Clean up resources"""
        if self.hands is not None:
            self.hands.close()

