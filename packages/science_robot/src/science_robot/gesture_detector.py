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
    
    def __init__(self, min_detection_confidence=0.5, min_tracking_confidence=0.5, model_complexity=None):
        """
        Initialize MediaPipe hands detector
        
        Args:
            min_detection_confidence: Minimum confidence for hand detection
            min_tracking_confidence: Minimum confidence for hand tracking
            model_complexity: MediaPipe model complexity (0=fastest, 1=balanced, 2=most accurate)
                            Default from config.MEDIAPIPE_MODEL_COMPLEXITY
        """
        if not MEDIAPIPE_AVAILABLE:
            self.mp_hands = None
            self.mp_drawing = None
            self.hands = None
            self.mp_face_detection = None
            self.face_detection = None
            self.min_detection_confidence = min_detection_confidence
            self.min_tracking_confidence = min_tracking_confidence
            self.model_complexity = model_complexity or config.MEDIAPIPE_MODEL_COMPLEXITY
            import logging
            logger = logging.getLogger(__name__)
            logger.error("MediaPipe not available - GestureDetector cannot be used")
            return
        
        # Store confidence values for runtime updates
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence = min_tracking_confidence
        self.model_complexity = model_complexity if model_complexity is not None else config.MEDIAPIPE_MODEL_COMPLEXITY
        
        # Face detection parameters (tunable)
        self.face_min_detection_confidence = min_detection_confidence * 0.7  # Default: 70% of hand detection
        self.face_model_selection = 1  # 0 for short-range (2m), 1 for full-range (5m)
        
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_face_detection = mp.solutions.face_detection
        
        # Build Hands arguments
        hands_args = {
            'static_image_mode': False,
            'max_num_hands': 2,
            'min_detection_confidence': min_detection_confidence,
            'min_tracking_confidence': min_tracking_confidence
        }
        
        # Add model_complexity if MediaPipe supports it (available in newer versions)
        try:
            # Check if model_complexity parameter is supported
            import inspect
            hands_signature = inspect.signature(self.mp_hands.Hands.__init__)
            if 'model_complexity' in hands_signature.parameters:
                hands_args['model_complexity'] = self.model_complexity
                import logging
                logger = logging.getLogger(__name__)
                logger.info(f"MediaPipe model_complexity set to {self.model_complexity} (0=fastest, 1=balanced, 2=most accurate)")
        except (AttributeError, TypeError):
            # Older MediaPipe versions may not support model_complexity
            pass
        
        self.hands = self.mp_hands.Hands(**hands_args)
        
        # Initialize face detection with stored parameters
        try:
            self.face_detection = self.mp_face_detection.FaceDetection(
                model_selection=self.face_model_selection,
                min_detection_confidence=self.face_min_detection_confidence
            )
            import logging
            logger = logging.getLogger(__name__)
            logger.info(f"Face detection initialized: confidence={self.face_min_detection_confidence:.2f}, model={self.face_model_selection}")
        except Exception as e:
            import logging
            logger = logging.getLogger(__name__)
            logger.error(f"Failed to initialize face detection: {e}")
            self.face_detection = None
    
    def detect_hands(self, frame, faces_data=None):
        """
        Detect hands in a frame with validation to filter out false positives (like feet)
        
        Args:
            frame: BGR image frame
            faces_data: Optional list of face detection dicts for context-aware filtering
            
        Returns:
            List of hand landmarks (each is a list of 21 landmark points), filtered to remove false positives
        """
        if not MEDIAPIPE_AVAILABLE or self.hands is None:
            return [], None
        
        # Convert BGR to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process frame
        results = self.hands.process(rgb_frame)
        
        hands_data = []
        frame_height = frame.shape[0] if frame is not None else None
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Convert landmarks to numpy array
                landmarks = []
                for lm in hand_landmarks.landmark:
                    landmarks.append([lm.x, lm.y, lm.z])
                landmarks_array = np.array(landmarks)
                
                # Validate that this is actually a hand (not feet or false positive)
                if self._is_valid_hand(landmarks_array, faces_data, frame_height):
                    hands_data.append(landmarks_array)
                else:
                    import logging
                    logger = logging.getLogger(__name__)
                    logger.debug("Filtered out false hand detection (likely feet)")
        
        return hands_data, results
    
    def _is_valid_hand(self, landmarks, faces_data=None, frame_height=None):
        """
        Validate that detected landmarks represent a real hand (not feet or false positive)
        
        Args:
            landmarks: Array of 21 hand landmarks
            faces_data: Optional list of face detection dicts for context-aware filtering
            frame_height: Optional frame height for position-based filtering
            
        Returns:
            True if landmarks appear to be a valid hand, False otherwise
        """
        # Get hand center (wrist position, index 0)
        wrist_x, wrist_y = landmarks[0][0], landmarks[0][1]
        
        # Filter 1: Position-based filtering - hands are typically in upper portion of frame
        # Feet are usually in bottom 30% of frame (y > 0.7 in normalized coordinates)
        # Hands are typically in upper 70% of frame (y < 0.7)
        if wrist_y > 0.7:  # Bottom 30% of frame - likely feet
            import logging
            logger = logging.getLogger(__name__)
            logger.debug(f"Filtered out false hand detection: y={wrist_y:.3f} (too low, likely feet)")
            return False
        
        # Filter 2: Face context - if faces are detected, hands should be relatively near faces
        # Feet would be much further below faces
        if faces_data:
            # Find closest face to this "hand"
            min_face_distance = float('inf')
            for face in faces_data:
                face_x, face_y = face['center']
                # Calculate vertical distance (feet would be much lower than faces)
                vertical_distance = abs(wrist_y - face_y)
                # Calculate horizontal distance
                horizontal_distance = abs(wrist_x - face_x)
                # Total distance (weighted toward vertical - feet are primarily below faces)
                distance = vertical_distance * 1.5 + horizontal_distance
                if distance < min_face_distance:
                    min_face_distance = distance
            
            # If closest face is very far vertically (more than 0.4 normalized units),
            # it's likely feet (feet are typically 0.5+ units below faces in normalized coordinates)
            if min_face_distance > 0.5:
                import logging
                logger = logging.getLogger(__name__)
                logger.debug(f"Filtered out false hand detection: too far from faces (distance={min_face_distance:.3f})")
                return False
        
        # Filter 3: Hand geometry validation
        # Hands have certain proportions - check finger-to-palm ratios
        # Wrist (0), Middle finger MCP (9 - palm center), Middle finger tip (12)
        wrist = landmarks[0]
        palm_center = landmarks[9]  # Middle finger base (palm center)
        middle_tip = landmarks[12]   # Middle finger tip
        
        # Calculate finger length (from palm to tip)
        finger_length = np.sqrt(
            (middle_tip[0] - palm_center[0])**2 + 
            (middle_tip[1] - palm_center[1])**2
        )
        
        # Calculate palm size (from wrist to palm center)
        palm_size = np.sqrt(
            (palm_center[0] - wrist[0])**2 + 
            (palm_center[1] - wrist[1])**2
        )
        
        # Hand-like proportions: finger should be 2-4x longer than palm width
        # Feet have different proportions (toes are shorter relative to foot size)
        if palm_size > 0.001:  # Avoid division by zero
            finger_to_palm_ratio = finger_length / palm_size
            # Valid hands: finger is 1.5-5x the palm size
            # Feet typically have shorter toe-to-foot ratios
            if finger_to_palm_ratio < 1.0 or finger_to_palm_ratio > 6.0:
                import logging
                logger = logging.getLogger(__name__)
                logger.debug(f"Filtered out false hand detection: invalid finger-to-palm ratio={finger_to_palm_ratio:.2f}")
                return False
        
        return True
    
    def detect_faces(self, frame):
        """
        Detect faces in a frame
        
        Args:
            frame: BGR image frame
            
        Returns:
            (faces_data, results) tuple where faces_data is a list of face dicts
            Each face dict contains: 'center' (x, y), 'bbox' (x, y, w, h), 'confidence'
        """
        if not MEDIAPIPE_AVAILABLE:
            return [], None
        
        if self.face_detection is None:
            import logging
            logger = logging.getLogger(__name__)
            logger.warning("Face detection not initialized - MediaPipe available but face_detection is None")
            return [], None
        
        try:
            # Convert BGR to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process frame
            results = self.face_detection.process(rgb_frame)
        except Exception as e:
            import logging
            logger = logging.getLogger(__name__)
            logger.error(f"Error during face detection: {e}")
            return [], None
        
        faces_data = []
        if results and results.detections:
            height, width = frame.shape[:2]
            for detection in results.detections:
                try:
                    # Get bounding box (normalized coordinates)
                    bbox = detection.location_data.relative_bounding_box
                    center_x = bbox.xmin + bbox.width / 2.0
                    center_y = bbox.ymin + bbox.height / 2.0
                    
                    # Convert bbox to pixel coordinates
                    bbox_px = (
                        int(bbox.xmin * width),
                        int(bbox.ymin * height),
                        int(bbox.width * width),
                        int(bbox.height * height)
                    )
                    
                    confidence = detection.score[0] if detection.score else 0.0
                    
                    faces_data.append({
                        'center': (center_x, center_y),  # Normalized coordinates
                        'bbox': bbox_px,  # Pixel coordinates
                        'confidence': confidence
                    })
                except Exception as e:
                    import logging
                    logger = logging.getLogger(__name__)
                    logger.warning(f"Error processing face detection: {e}")
                    continue
        
        return faces_data, results
    
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
            String indicating gesture type: 'thumbs_up', 'stop', 'treat', or None (dance disabled)
        """
        if not hands:
            return None
        
        # Dance gesture detection disabled - focusing on wave detection and tracking
        # Check for two-hand clapping gesture (duck bill) - DISABLED
        # if len(hands) >= 2:
        #     for i in range(len(hands)):
        #         for j in range(i + 1, len(hands)):
        #             if self._is_duck_bill_clap(hands[i], hands[j]):
        #                 return 'dance'
        
        # Check for thumbs up gesture (thumb extended, others closed) - PRIMARY TRIGGER
        for hand in hands:
            if self._is_thumbs_up_gesture(hand):
                return 'thumbs_up'
        
        # Stop gesture detection DISABLED - was causing false positives with thumbs up
        # The stop gesture (all 5 fingers extended) is too easily confused with thumbs up
        # for hand in hands:
        #     if self._is_stop_gesture(hand):
        #         return 'stop'
        
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
    
    def _is_stop_gesture(self, landmarks):
        """
        Determine if landmarks represent the stop gesture (all 5 fingers extended)
        This is a more reliable alternative to wave detection
        """
        finger_states = self.get_finger_states(landmarks)
        thumb, index, middle, ring, pinky = finger_states
        # All 5 fingers must be extended
        return thumb and index and middle and ring and pinky
    
    def _is_thumbs_up_gesture(self, landmarks):
        """
        Determine if landmarks represent the thumbs up gesture: FIST with thumb sticking STRAIGHT UP
        This is the trigger for face tracking and forward movement
        
        Detection criteria:
        1. All four fingers (index, middle, ring, pinky) must be closed in a fist
           - Finger tips must be below MCP joints (base of fingers)
           - Fingers should be curled tightly
        2. Thumb must be extended STRAIGHT UP (vertically)
           - Thumb tip must be significantly above thumb base
           - Thumb should be pointing upward (not sideways)
        """
        # MediaPipe hand landmark indices:
        # WRIST = 0
        # THUMB: CMC=1, MCP=2, IP=3, TIP=4
        # INDEX: MCP=5, PIP=6, DIP=7, TIP=8
        # MIDDLE: MCP=9, PIP=10, DIP=11, TIP=12
        # RING: MCP=13, PIP=14, DIP=15, TIP=16
        # PINKY: MCP=17, PIP=18, DIP=19, TIP=20
        
        wrist = np.array(landmarks[0][:2])
        thumb_tip = np.array(landmarks[4][:2])
        thumb_mcp = np.array(landmarks[2][:2])  # Thumb base
        
        # Check 1: All four fingers must be closed in a fist
        # For each finger, check that tip is below MCP (base joint) - indicates fist
        index_tip = np.array(landmarks[8][:2])
        index_mcp = np.array(landmarks[5][:2])
        
        middle_tip = np.array(landmarks[12][:2])
        middle_mcp = np.array(landmarks[9][:2])
        
        ring_tip = np.array(landmarks[16][:2])
        ring_mcp = np.array(landmarks[13][:2])
        
        pinky_tip = np.array(landmarks[20][:2])
        pinky_mcp = np.array(landmarks[17][:2])
        
        # All finger tips must be below their MCP joints (fist position)
        # In normalized coordinates, Y increases downward, so tip_y > mcp_y means finger is closed
        index_closed = index_tip[1] > index_mcp[1]  # Tip below base = closed
        middle_closed = middle_tip[1] > middle_mcp[1]
        ring_closed = ring_tip[1] > ring_mcp[1]
        pinky_closed = pinky_tip[1] > pinky_mcp[1]
        
        # All four fingers must be closed for a fist
        if not (index_closed and middle_closed and ring_closed and pinky_closed):
            return False
        
        # Check 2: Thumb must be extended STRAIGHT UP (vertically)
        # Thumb tip should be significantly above thumb base
        # Check vertical distance (Y coordinate - remember Y decreases upward)
        thumb_vertical_distance = thumb_mcp[1] - thumb_tip[1]  # Positive if thumb is above base
        
        # Thumb should be at least 0.08 normalized units above base (significant upward extension)
        if thumb_vertical_distance < 0.08:
            return False
        
        # Check 3: Thumb should be pointing upward (vertical orientation preferred)
        # Calculate horizontal vs vertical distance
        thumb_horizontal_distance = abs(thumb_tip[0] - thumb_mcp[0])
        
        # For thumbs up, vertical distance should be much greater than horizontal
        # This ensures thumb is pointing up, not sideways
        # Allow some horizontal movement but vertical should dominate
        is_mostly_vertical = thumb_vertical_distance > thumb_horizontal_distance * 1.5
        
        # Check 4: Thumb tip should be above wrist (for proper thumbs up)
        thumb_above_wrist = thumb_tip[1] < wrist[1]  # Thumb tip Y < wrist Y means thumb is above
        
        # Also check thumb tip is well above wrist (significant upward extension)
        wrist_to_thumb_height = wrist[1] - thumb_tip[1]  # Positive if thumb is above wrist
        significant_extension_above_wrist = wrist_to_thumb_height > 0.05
        
        return is_mostly_vertical and thumb_above_wrist and significant_extension_above_wrist
    
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
                        # Classify gesture directly from this hand's landmarks
                        gesture = None
                        if hands_data:
                            gesture = self.classify_gesture([landmarks])
                            
                            if gesture == 'thumbs_up':
                                box_color = (0, 255, 0)  # Green for thumbs up
                                label_parts.append("THUMBS UP")
                            elif gesture == 'treat':
                                box_color = (255, 0, 255)  # Magenta for treat
                                label_parts.append("TREAT")
                            elif gesture == 'stop':
                                # Stop gesture should not be detected, but handle if it is
                                box_color = (0, 0, 255)  # Red for stop
                                label_parts.append("STOP")
                        
                        # Add waving status (can be combined with gestures)
                        # Only show "WAVING" if gesture detection mode allows it (not in 'gesture' mode)
                        if is_waving and config.GESTURE_DETECTION_MODE != 'gesture' and not gesture:
                            label_parts.append("WAVING")
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
    
    def draw_faces(self, frame, faces_data, target_face_center=None):
        """
        Draw face bounding boxes on frame
        
        Args:
            frame: BGR image frame
            faces_data: List of face detection dicts with 'bbox' and 'center' keys
            target_face_center: Optional (x, y) normalized coordinates of target face to highlight
        """
        if not faces_data:
            return
        
        # Debug: Log face drawing
        import logging
        logger = logging.getLogger(__name__)
        if len(faces_data) > 0:
            logger.info(f"Drawing {len(faces_data)} face bounding box(es) on frame")
        
        height, width = frame.shape[:2]
        
        for face in faces_data:
            bbox = face['bbox']
            center = face['center']
            confidence = face.get('confidence', 0.0)
            
            x, y, w, h = bbox
            
            # Clamp coordinates to frame bounds
            x = max(0, min(x, width - 1))
            y = max(0, min(y, height - 1))
            w = max(1, min(w, width - x))
            h = max(1, min(h, height - y))
            
            # Determine if this is the target face
            is_target = False
            if target_face_center:
                target_x, target_y = target_face_center
                face_x, face_y = center
                # Check if centers are close (within 0.05 normalized distance)
                distance = np.sqrt((target_x - face_x)**2 + (target_y - face_y)**2)
                is_target = distance < 0.05
            
            # Use green for target face, bright blue for others (BGR format)
            # Make boxes more visible with thicker lines
            color = (0, 255, 0) if is_target else (255, 100, 0)  # Bright cyan-blue for visibility
            thickness = 4 if is_target else 3  # Thicker lines for better visibility
            
            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, thickness)
            
            # Debug: Log first face bbox coordinates occasionally
            if len(faces_data) > 0 and face == faces_data[0]:
                import logging
                logger = logging.getLogger(__name__)
                logger.debug(f"Face bbox: x={x}, y={y}, w={w}, h={h}, frame_size={width}x{height}, color={color}")
            
            # Draw label
            label_text = f"FACE {confidence:.2f}"
            if is_target:
                label_text = "TARGET " + label_text
            
            label_y = max(20, y - 5)
            (text_width, text_height), baseline = cv2.getTextSize(
                label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2
            )
            
            # Draw background rectangle for text
            cv2.rectangle(frame, 
                        (x, label_y - text_height - 5), 
                        (x + text_width + 4, label_y + baseline), 
                        (0, 0, 0), -1)  # Black background
            
            # Draw text
            cv2.putText(frame, label_text, (x + 2, label_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    def update_parameters(self, min_detection_confidence=None, min_tracking_confidence=None,
                         gesture_confidence_threshold=None, dance_hold_time=None,
                         treat_hold_time=None, clap_finger_threshold=None,
                         clap_palm_threshold=None, model_complexity=None,
                         face_min_detection_confidence=None, face_model_selection=None):
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
        
        # If MediaPipe confidence or model complexity changes, recreate the Hands object
        if min_detection_confidence is not None or min_tracking_confidence is not None or model_complexity is not None:
            if not MEDIAPIPE_AVAILABLE:
                logger.warning("MediaPipe not available, cannot update detection/tracking confidence")
            else:
                if self.hands is not None:
                    self.hands.close()
                
                new_detection = min_detection_confidence if min_detection_confidence is not None else self.min_detection_confidence
                new_tracking = min_tracking_confidence if min_tracking_confidence is not None else self.min_tracking_confidence
                new_complexity = model_complexity if model_complexity is not None else self.model_complexity
                
                # Clamp values to valid range
                new_detection = max(0.0, min(1.0, float(new_detection)))
                new_tracking = max(0.0, min(1.0, float(new_tracking)))
                new_complexity = max(0, min(2, int(new_complexity)))
                
                self.min_detection_confidence = new_detection
                self.min_tracking_confidence = new_tracking
                self.model_complexity = new_complexity
                
                # Build Hands arguments
                hands_args = {
                    'static_image_mode': False,
                    'max_num_hands': 2,
                    'min_detection_confidence': self.min_detection_confidence,
                    'min_tracking_confidence': self.min_tracking_confidence
                }
                
                # Add model_complexity if supported
                try:
                    import inspect
                    hands_signature = inspect.signature(self.mp_hands.Hands.__init__)
                    if 'model_complexity' in hands_signature.parameters:
                        hands_args['model_complexity'] = self.model_complexity
                except (AttributeError, TypeError):
                    pass
                
                self.hands = self.mp_hands.Hands(**hands_args)
                
                logger.info(f"Updated MediaPipe: detection={self.min_detection_confidence:.2f}, tracking={self.min_tracking_confidence:.2f}, complexity={self.model_complexity}")
        
        # Update face detection parameters
        face_detection_updated = False
        if face_min_detection_confidence is not None:
            self.face_min_detection_confidence = max(0.0, min(1.0, float(face_min_detection_confidence)))
            face_detection_updated = True
            logger.info(f"Updated face detection confidence: {self.face_min_detection_confidence:.2f}")
        
        if face_model_selection is not None:
            self.face_model_selection = max(0, min(1, int(face_model_selection)))
            face_detection_updated = True
            logger.info(f"Updated face model selection: {self.face_model_selection} (0=short-range, 1=full-range)")
        
        # Recreate face detection if parameters changed
        if face_detection_updated and self.face_detection is not None:
            try:
                self.face_detection.close()
                self.face_detection = self.mp_face_detection.FaceDetection(
                    model_selection=self.face_model_selection,
                    min_detection_confidence=self.face_min_detection_confidence
                )
                logger.info(f"Face detection reinitialized: confidence={self.face_min_detection_confidence:.2f}, model={self.face_model_selection}")
            except Exception as e:
                logger.error(f"Failed to update face detection: {e}")
        
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
            'model_complexity': self.model_complexity,
            'gesture_confidence_threshold': config.GESTURE_CONFIDENCE_THRESHOLD,
            'dance_hold_time': config.DANCE_GESTURE_HOLD_TIME,
            'treat_hold_time': config.TREAT_GESTURE_HOLD_TIME,
            'clap_finger_threshold': config.DANCE_CLAP_FINGER_THRESHOLD,
            'clap_palm_threshold': config.DANCE_CLAP_PALM_THRESHOLD,
            'face_min_detection_confidence': self.face_min_detection_confidence,
            'face_model_selection': self.face_model_selection
        }
    
    def close(self):
        """Clean up resources"""
        if self.hands is not None:
            self.hands.close()
        if self.face_detection is not None:
            self.face_detection.close()

