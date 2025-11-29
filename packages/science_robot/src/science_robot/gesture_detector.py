"""
Gesture detector using MediaPipe for hand detection and gesture recognition
"""
import cv2
import numpy as np
import time
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

# Try to import MediaPipe Gesture Recognizer (Tasks API)
try:
    from mediapipe.tasks import python as mp_tasks
    from mediapipe.tasks.python import vision
    from mediapipe.framework.formats import landmark_pb2
    # Image class is in the main mediapipe module, not in vision
    # We already have mp imported above, so we can use mp.Image
    GESTURE_RECOGNIZER_AVAILABLE = True
except ImportError:
    GESTURE_RECOGNIZER_AVAILABLE = False
    mp_tasks = None
    vision = None
    landmark_pb2 = None
    import logging
    logger = logging.getLogger(__name__)
    logger.warning("MediaPipe Gesture Recognizer not available - using custom detection only")


class GestureDetector:
    """Hand gesture detection using MediaPipe"""
    
    # Finger landmark indices for MediaPipe hands
    FINGER_TIPS = [4, 8, 12, 16, 20]  # Thumb, Index, Middle, Ring, Pinky
    FINGER_PIPS = [3, 6, 10, 14, 18]  # Joints before tips
    
    def __init__(self, min_detection_confidence=None, min_tracking_confidence=None, model_complexity=None):
        """
        Initialize MediaPipe hands detector
        
        Args:
            min_detection_confidence: Minimum confidence for hand detection
            min_tracking_confidence: Minimum confidence for hand tracking
            model_complexity: MediaPipe model complexity (0=fastest, 1=balanced)
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
        # Use config defaults if not provided (lowered for better distance detection)
        self.min_detection_confidence = min_detection_confidence if min_detection_confidence is not None else getattr(config, 'MEDIAPIPE_HAND_DETECTION_CONFIDENCE', 0.35)
        self.min_tracking_confidence = min_tracking_confidence if min_tracking_confidence is not None else getattr(config, 'MEDIAPIPE_HAND_TRACKING_CONFIDENCE', 0.35)
        self.model_complexity = model_complexity if model_complexity is not None else config.MEDIAPIPE_MODEL_COMPLEXITY
        
        # Face detection parameters (tunable)
        # Use config default for face detection confidence (lowered for distance detection)
        self.face_min_detection_confidence = getattr(config, 'MEDIAPIPE_FACE_DETECTION_CONFIDENCE', 0.30)
        self.face_model_selection = 1  # 0 for short-range (2m), 1 for full-range (5m)
        
        # Gesture Recognizer instance variables
        self.gesture_recognizer = None
        self.gesture_recognizer_enabled = False
        self.last_frame_timestamp = 0
        self.frame_timestamp_counter = 0  # Counter for monotonically increasing timestamps
        self.running_mode = None
        
        # Hand Landmarker instance variables (Tasks API)
        self.hand_landmarker = None
        self.hand_landmarker_enabled = False
        self.hand_landmarker_running_mode = None
        
        # Logging frame counter for landmark logging (to reduce spam)
        self._landmark_log_frame_counter = 0
        
        # Performance optimization: Cache Gesture Recognizer results to avoid duplicate processing
        # When Gesture Recognizer is enabled, it already detects hands, so we skip Hand Landmarker
        self._cached_gesture_result = None  # Full Gesture Recognizer result
        self._cached_hands_data = None  # Extracted hand landmarks from Gesture Recognizer
        self._cached_hands_result = None  # Compatible result object for drawing
        self._cache_frame_id = None  # Track which frame the cache is for (to detect stale cache)
        
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_face_detection = mp.solutions.face_detection
        
        # PERFORMANCE OPTIMIZATION: Skip Hand Landmarker when Gesture Recognizer is enabled
        # Gesture Recognizer already includes hand detection, so Hand Landmarker is redundant
        # This prevents processing the same frame twice (major performance improvement)
        should_use_hand_landmarker = config.HAND_LANDMARKER_ENABLED and not config.GESTURE_RECOGNIZER_ENABLED
        
        # Initialize Hand Landmarker (Tasks API) if enabled and Gesture Recognizer is not enabled
        if should_use_hand_landmarker:
            try:
                success = self._initialize_hand_landmarker()
                if not success:
                    logger.warning("Hand Landmarker initialization returned False - falling back to Solutions API")
                    self.hand_landmarker = None
                    self.hand_landmarker_enabled = False
                    # Fall through to initialize Solutions API as fallback
            except (Exception, SystemExit, KeyboardInterrupt) as e:
                import logging
                logger = logging.getLogger(__name__)
                logger.error(f"Hand Landmarker initialization failed with exception (will use Solutions API fallback): {type(e).__name__}: {e}")
                import traceback
                logger.debug(traceback.format_exc())
                self.hand_landmarker = None
                self.hand_landmarker_enabled = False
                logger.warning("Continuing with Solutions API fallback for hand detection")
        
        # Initialize Solutions API as fallback (or primary if Hand Landmarker disabled)
        # PERFORMANCE: Also skip Solutions API when Gesture Recognizer is enabled (it already detects hands)
        if not self.hand_landmarker_enabled and not config.GESTURE_RECOGNIZER_ENABLED:
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
                    logger.info(f"MediaPipe model_complexity set to {self.model_complexity} (0=fastest, 1=balanced)")
            except (AttributeError, TypeError):
                # Older MediaPipe versions may not support model_complexity
                pass
            
            self.hands = self.mp_hands.Hands(**hands_args)
            
            # Log detection settings for distance optimization
            import logging
            logger = logging.getLogger(__name__)
            logger.info(f"Hand detection initialized (Solutions API fallback): detection_confidence={self.min_detection_confidence:.2f}, tracking_confidence={self.min_tracking_confidence:.2f}, model_complexity={self.model_complexity} (optimized for distance detection)")
        elif self.hand_landmarker_enabled:
            # Hand Landmarker is enabled, set hands to None
            self.hands = None
            import logging
            logger = logging.getLogger(__name__)
            logger.info(f"Hand detection initialized (Hand Landmarker Tasks API): detection_confidence={config.HAND_LANDMARKER_MIN_DETECTION_CONFIDENCE:.2f}, tracking_confidence={config.HAND_LANDMARKER_MIN_TRACKING_CONFIDENCE:.2f} (optimized for distance detection)")
        elif config.GESTURE_RECOGNIZER_ENABLED:
            # Gesture Recognizer is enabled - skip hand detection initialization entirely
            # Gesture Recognizer already includes hand detection, so we don't need Hand Landmarker or Solutions API
            self.hands = None
            import logging
            logger = logging.getLogger(__name__)
            logger.info(f"Hand detection skipped - Gesture Recognizer enabled (includes hand detection)")
        
        # Initialize face detection with stored parameters
        try:
            self.face_detection = self.mp_face_detection.FaceDetection(
                model_selection=self.face_model_selection,
                min_detection_confidence=self.face_min_detection_confidence
            )
            logger.info(f"Face detection initialized: confidence={self.face_min_detection_confidence:.2f}, model={self.face_model_selection} (0=short-range/2m, 1=full-range/5m - optimized for distance detection)")
        except Exception as e:
            import logging
            logger = logging.getLogger(__name__)
            logger.error(f"Failed to initialize face detection: {e}")
            self.face_detection = None
        
        # Initialize Gesture Recognizer if enabled
        # Note: MediaPipe Gesture Recognizer can have C++ fatal errors that crash the process
        # If you see "Failed to get tag C2__PACKET" errors, try setting GESTURE_RECOGNIZER_ENABLED=False
        # or check MediaPipe version compatibility with the model file
        if config.GESTURE_RECOGNIZER_ENABLED:
            try:
                success = self._initialize_gesture_recognizer()
                if not success:
                    logger.warning("Gesture Recognizer initialization returned False - continuing without it")
                    self.gesture_recognizer = None
                    self.gesture_recognizer_enabled = False
            except (Exception, SystemExit, KeyboardInterrupt) as e:
                import logging
                logger = logging.getLogger(__name__)
                logger.error(f"Gesture Recognizer initialization failed with exception (will continue without it): {type(e).__name__}: {e}")
                import traceback
                logger.debug(traceback.format_exc())
                # Ensure recognizer is disabled
                self.gesture_recognizer = None
                self.gesture_recognizer_enabled = False
                logger.warning("Continuing without Gesture Recognizer - using fallback hand detection only")
                logger.warning("If you see C++ fatal errors, try setting GESTURE_RECOGNIZER_ENABLED=False in config")
            except BaseException as e:
                # Catch even base exceptions (though C++ fatal errors may still crash)
                import logging
                logger = logging.getLogger(__name__)
                logger.error(f"Gesture Recognizer initialization failed with base exception: {type(e).__name__}: {e}")
                self.gesture_recognizer = None
                self.gesture_recognizer_enabled = False
                logger.warning("Continuing without Gesture Recognizer")
    
    def _initialize_hand_landmarker(self):
        """
        Initialize MediaPipe Hand Landmarker (Tasks API)
        
        Returns:
            True if initialization successful, False otherwise
        """
        if not GESTURE_RECOGNIZER_AVAILABLE:  # Uses same Tasks API imports
            import logging
            logger = logging.getLogger(__name__)
            logger.warning("Hand Landmarker not available - MediaPipe Tasks API not found")
            logger.warning("MediaPipe Hand Landmarker Tasks API requires MediaPipe >=0.10.8")
            logger.warning("Current MediaPipe version may be too old. Try: pip install 'mediapipe>=0.10.8'")
            return False
        
        import os
        import logging
        logger = logging.getLogger(__name__)
        
        # Check MediaPipe version - Hand Landmarker Tasks API requires >=0.10.8
        try:
            import mediapipe as mp
            mp_version = getattr(mp, '__version__', 'unknown')
            logger.info(f"MediaPipe version: {mp_version}")
            # Simple version check: if version starts with "0.10.0" through "0.10.7", warn
            if mp_version.startswith('0.10.0') or mp_version.startswith('0.10.1') or \
               mp_version.startswith('0.10.2') or mp_version.startswith('0.10.3') or \
               mp_version.startswith('0.10.4') or mp_version.startswith('0.10.5') or \
               mp_version.startswith('0.10.6') or mp_version.startswith('0.10.7'):
                logger.error(f"MediaPipe version {mp_version} is too old for Hand Landmarker Tasks API")
                logger.error("MediaPipe >=0.10.8 is required. Version 0.10.0-0.10.7 may cause errors.")
                logger.error("Please upgrade: pip install 'mediapipe>=0.10.8'")
                logger.error("Disabling Hand Landmarker to prevent crashes")
                return False
        except Exception as e:
            logger.debug(f"Could not check MediaPipe version: {e}")
        
        # Check if model file exists
        if not os.path.exists(config.HAND_LANDMARKER_MODEL_PATH):
            logger.warning(f"Hand Landmarker model file not found: {config.HAND_LANDMARKER_MODEL_PATH}")
            logger.warning("Hand Landmarker will be disabled. Model file should be in repository at models/hand_landmarker.task")
            return False
        
        try:
            # Determine running mode - VIDEO mode is required when using detect_for_video()
            running_mode_str = config.HAND_LANDMARKER_RUNNING_MODE
            if running_mode_str == 'VIDEO':
                running_mode = vision.RunningMode.VIDEO
            elif running_mode_str == 'LIVE_STREAM':
                # LIVE_STREAM requires async callback - we're using sync detect_for_video(), so switch to VIDEO
                logger.warning(f"LIVE_STREAM mode requires async callback, but we're using detect_for_video(). Switching to VIDEO mode.")
                running_mode = vision.RunningMode.VIDEO
            else:
                logger.warning(f"Invalid running mode '{running_mode_str}', defaulting to VIDEO")
                running_mode = vision.RunningMode.VIDEO
            
            self.hand_landmarker_running_mode = running_mode
            
            # Create base options
            base_options = mp_tasks.BaseOptions(
                model_asset_path=config.HAND_LANDMARKER_MODEL_PATH
            )
            
            # Create hand landmarker options
            options = vision.HandLandmarkerOptions(
                base_options=base_options,
                running_mode=running_mode,
                num_hands=config.HAND_LANDMARKER_NUM_HANDS,
                min_hand_detection_confidence=config.HAND_LANDMARKER_MIN_DETECTION_CONFIDENCE,
                min_hand_presence_confidence=config.HAND_LANDMARKER_MIN_DETECTION_CONFIDENCE,
                min_tracking_confidence=config.HAND_LANDMARKER_MIN_TRACKING_CONFIDENCE
            )
            
            # Create landmarker
            try:
                self.hand_landmarker = vision.HandLandmarker.create_from_options(options)
                self.hand_landmarker_enabled = True
            except Exception as create_error:
                logger.error(f"Failed to create Hand Landmarker from options: {create_error}")
                import traceback
                logger.debug(traceback.format_exc())
                raise  # Re-raise to be handled by outer try-except
            
            # Reset timestamp counter for VIDEO mode
            self.frame_timestamp_counter = 0
            
            logger.info(f"MediaPipe Hand Landmarker initialized successfully (mode: VIDEO - using detect_for_video() for synchronous processing)")
            logger.info(f"  - Detection confidence threshold: {config.HAND_LANDMARKER_MIN_DETECTION_CONFIDENCE:.2f}")
            logger.info(f"  - Tracking confidence threshold: {config.HAND_LANDMARKER_MIN_TRACKING_CONFIDENCE:.2f}")
            logger.info(f"  - Model path: {config.HAND_LANDMARKER_MODEL_PATH}")
            return True
            
        except Exception as e:
            import logging
            logger = logging.getLogger(__name__)
            logger.error(f"Failed to initialize Hand Landmarker: {e}")
            self.hand_landmarker = None
            self.hand_landmarker_enabled = False
            return False
    
    def _initialize_gesture_recognizer(self):
        """
        Initialize MediaPipe Gesture Recognizer (Tasks API)
        
        Returns:
            True if initialization successful, False otherwise
        """
        if not GESTURE_RECOGNIZER_AVAILABLE:
            import logging
            logger = logging.getLogger(__name__)
            logger.warning("Gesture Recognizer not available - MediaPipe Tasks API not found")
            logger.warning("MediaPipe Gesture Recognizer Tasks API requires MediaPipe >=0.10.8")
            logger.warning("Current MediaPipe version may be too old. Try: pip install 'mediapipe>=0.10.8'")
            return False
        
        import os
        import logging
        logger = logging.getLogger(__name__)
        
        # Check MediaPipe version - Gesture Recognizer Tasks API requires >=0.10.8
        # Version 0.10.0 is too old and causes "C2__PACKET" fatal errors
        try:
            import mediapipe as mp
            mp_version = getattr(mp, '__version__', 'unknown')
            logger.info(f"MediaPipe version: {mp_version}")
            # Simple version check: if version starts with "0.10.0" through "0.10.7", warn
            if mp_version.startswith('0.10.0') or mp_version.startswith('0.10.1') or \
               mp_version.startswith('0.10.2') or mp_version.startswith('0.10.3') or \
               mp_version.startswith('0.10.4') or mp_version.startswith('0.10.5') or \
               mp_version.startswith('0.10.6') or mp_version.startswith('0.10.7'):
                logger.error(f"MediaPipe version {mp_version} is too old for Gesture Recognizer Tasks API")
                logger.error("MediaPipe >=0.10.8 is required. Version 0.10.0-0.10.7 cause 'C2__PACKET' errors.")
                logger.error("Please upgrade: pip install 'mediapipe>=0.10.8'")
                logger.error("Disabling Gesture Recognizer to prevent crashes")
                return False
        except Exception as e:
            logger.debug(f"Could not check MediaPipe version: {e}")
        
        # Check if model file exists
        if not os.path.exists(config.GESTURE_RECOGNIZER_MODEL_PATH):
            logger.warning(f"Gesture Recognizer model file not found: {config.GESTURE_RECOGNIZER_MODEL_PATH}")
            logger.warning("Gesture Recognizer will be disabled. Model file should be in repository at models/gesture_recognizer.task")
            return False
        
        try:
            # Determine running mode - VIDEO mode is required when using recognize_for_video()
            # LIVE_STREAM mode requires a callback function (recognize_async), which we're not using
            # Since we're using recognize_for_video() synchronously, we must use VIDEO mode
            running_mode_str = config.GESTURE_RECOGNIZER_RUNNING_MODE
            if running_mode_str == 'VIDEO':
                running_mode = vision.RunningMode.VIDEO
            elif running_mode_str == 'LIVE_STREAM':
                # LIVE_STREAM requires async callback - we're using sync recognize_for_video(), so switch to VIDEO
                logger.warning(f"LIVE_STREAM mode requires async callback, but we're using recognize_for_video(). Switching to VIDEO mode.")
                running_mode = vision.RunningMode.VIDEO
            else:
                logger.warning(f"Invalid running mode '{running_mode_str}', defaulting to VIDEO")
                running_mode = vision.RunningMode.VIDEO
            
            self.running_mode = running_mode
            
            # Create base options
            base_options = mp_tasks.BaseOptions(
                model_asset_path=config.GESTURE_RECOGNIZER_MODEL_PATH
            )
            
            # Create gesture recognizer options
            options = vision.GestureRecognizerOptions(
                base_options=base_options,
                running_mode=running_mode,
                min_hand_detection_confidence=config.GESTURE_RECOGNIZER_MIN_DETECTION_CONFIDENCE,
                min_hand_presence_confidence=config.GESTURE_RECOGNIZER_MIN_DETECTION_CONFIDENCE,
                min_tracking_confidence=config.GESTURE_RECOGNIZER_MIN_DETECTION_CONFIDENCE,
                num_hands=2
            )
            
            # Create recognizer - this can fail with C++ fatal errors in some MediaPipe versions
            # Wrap in additional try-except to catch any initialization errors
            try:
                self.gesture_recognizer = vision.GestureRecognizer.create_from_options(options)
                self.gesture_recognizer_enabled = True
            except Exception as create_error:
                # Re-raise to be caught by outer exception handler
                logger.error(f"Failed to create Gesture Recognizer from options: {create_error}")
                import traceback
                logger.debug(traceback.format_exc())
                raise  # Re-raise to be handled by outer try-except
            
            # Reset timestamp counter for VIDEO mode and last timestamp for LIVE_STREAM mode
            self.frame_timestamp_counter = 0
            self.last_frame_timestamp = 0
            
            logger.info(f"MediaPipe Gesture Recognizer initialized successfully (mode: VIDEO - using recognize_for_video() for synchronous processing)")
            logger.info(f"  - Detection confidence threshold: {config.GESTURE_RECOGNIZER_MIN_DETECTION_CONFIDENCE:.2f}")
            logger.info(f"  - Gesture confidence threshold: {config.GESTURE_RECOGNIZER_MIN_GESTURE_CONFIDENCE:.2f}")
            logger.info(f"  - Model path: {config.GESTURE_RECOGNIZER_MODEL_PATH}")
            return True
            
        except Exception as e:
            import logging
            logger = logging.getLogger(__name__)
            logger.error(f"Failed to initialize Gesture Recognizer: {e}")
            self.gesture_recognizer = None
            self.gesture_recognizer_enabled = False
            return False
    
    def clear_cache(self):
        """
        Clear cached Gesture Recognizer results - should be called at start of each frame
        to prevent stale data from previous frames
        """
        self._cached_gesture_result = None
        self._cached_hands_data = None
        self._cached_hands_result = None
        self._cache_frame_id = None
    
    def detect_hands(self, frame, faces_data=None):
        """
        Detect hands in a frame with validation to filter out false positives (like feet)
        Uses Hand Landmarker Tasks API if enabled, otherwise falls back to Solutions API
        
        PERFORMANCE OPTIMIZATION: When Gesture Recognizer is enabled, skip Hand Landmarker
        and return cached hand landmarks from Gesture Recognizer instead (avoids processing frame twice)
        
        Args:
            frame: BGR image frame
            faces_data: Optional list of face detection dicts for context-aware filtering
            
        Returns:
            (hands_data, results) tuple where:
            - hands_data: List of hand landmarks (each is a list of 21 landmark points), filtered to remove false positives
            - results: Hand Landmarker result object (Tasks API) or Solutions API results object
        """
        if not MEDIAPIPE_AVAILABLE:
            return [], None
        
        # PERFORMANCE FIX: When Gesture Recognizer is enabled, skip hand detection here
        # Gesture Recognizer will process the frame in classify_gesture() and cache the results
        # Return empty list for now - the cache will be populated by classify_gesture() after this call
        # Note: The main loop should call classify_gesture() first when Gesture Recognizer is enabled
        if self.gesture_recognizer_enabled:
            # Check if cache already populated (from previous call to classify_gesture() in same frame)
            if self._cached_hands_data is not None:
                import logging
                logger = logging.getLogger(__name__)
                logger.debug(f"Using cached hand landmarks from Gesture Recognizer ({len(self._cached_hands_data)} hands)")
                return self._cached_hands_data, self._cached_hands_result
            # Otherwise, return empty - classify_gesture() will populate cache
            return [], None
        
        # Use Hand Landmarker Tasks API if enabled (only when Gesture Recognizer is disabled)
        if self.hand_landmarker_enabled and self.hand_landmarker is not None:
            return self._detect_hands_with_landmarker(frame, faces_data)
        
        # Fallback to Solutions API
        if self.hands is None:
            return [], None
        
        # Convert BGR to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process frame with Solutions API
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
    
    def _detect_hands_with_landmarker(self, frame, faces_data=None):
        """
        Detect hands using Hand Landmarker Tasks API
        
        Args:
            frame: BGR image frame
            faces_data: Optional list of face detection dicts for context-aware filtering
            
        Returns:
            (hands_data, results) tuple where:
            - hands_data: List of hand landmarks (each is a list of 21 landmark points), filtered to remove false positives
            - results: HandLandmarkerResult object
        """
        if not GESTURE_RECOGNIZER_AVAILABLE or self.hand_landmarker is None:
            return [], None
        
        try:
            # Convert BGR to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Convert to MediaPipe Image
            # Image class is in the main mediapipe module (mp.Image)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            
            # Update timestamp for VIDEO mode (monotonically increasing)
            timestamp_ms = int(self.frame_timestamp_counter * 1000)  # Convert to milliseconds
            self.frame_timestamp_counter += 1
            
            # Detect hands
            detection_result = self.hand_landmarker.detect_for_video(mp_image, timestamp_ms)
            
            hands_data = []
            frame_height = frame.shape[0] if frame is not None else None
            
            # Process detected hands
            if detection_result.hand_landmarks:
                for hand_landmarks in detection_result.hand_landmarks:
                    # Convert landmarks to numpy array
                    # Hand Landmarker provides landmarks as list of landmark_pb2.NormalizedLandmark
                    landmarks = []
                    for landmark in hand_landmarks:
                        landmarks.append([landmark.x, landmark.y, landmark.z])
                    landmarks_array = np.array(landmarks)
                    
                    # Validate that this is actually a hand (not feet or false positive)
                    if self._is_valid_hand(landmarks_array, faces_data, frame_height):
                        hands_data.append(landmarks_array)
                    else:
                        import logging
                        logger = logging.getLogger(__name__)
                        logger.debug("Filtered out false hand detection (likely feet)")
            
            return hands_data, detection_result
            
        except Exception as e:
            import logging
            logger = logging.getLogger(__name__)
            logger.error(f"Error during hand detection with Hand Landmarker: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            return [], None
    
    def _is_valid_hand(self, landmarks, faces_data=None, frame_height=None):
        """
        Validate that detected landmarks represent a real hand (not feet or false positive)
        Optimized for distance detection - more lenient filters for distant objects
        
        Args:
            landmarks: Array of 21 hand landmarks
            faces_data: Optional list of face detection dicts for context-aware filtering
            frame_height: Optional frame height for position-based filtering
            
        Returns:
            True if landmarks appear to be a valid hand, False otherwise
        """
        # Get hand center (wrist position, index 0)
        wrist_x, wrist_y = landmarks[0][0], landmarks[0][1]
        
        # Estimate hand size to determine if it's distant (smaller hands = further away)
        # Use bounding box of landmarks to estimate size
        x_coords = [lm[0] for lm in landmarks]
        y_coords = [lm[1] for lm in landmarks]
        hand_size_x = max(x_coords) - min(x_coords)
        hand_size_y = max(y_coords) - min(y_coords)
        hand_size = max(hand_size_x, hand_size_y)  # Use max dimension as hand size estimate
        
        # Determine if this is likely a distant hand (smaller than typical close-up hand)
        # Typical close-up hand is ~0.2-0.3 normalized units, distant might be <0.1
        is_likely_distant = hand_size < 0.12  # More lenient threshold for distance detection
        
        # Filter 1: Position-based filtering - relaxed for distance detection
        # For distant objects, be more lenient (hands can be lower in frame)
        # Feet are usually in bottom 20% of frame (y > 0.8 in normalized coordinates)
        # Relax threshold to 0.85 for distant objects, 0.75 for close objects
        position_threshold = 0.85 if is_likely_distant else 0.75
        if wrist_y > position_threshold:  # Bottom portion of frame - likely feet
            import logging
            logger = logging.getLogger(__name__)
            logger.debug(f"Filtered out false hand detection: y={wrist_y:.3f} (too low, likely feet, threshold={position_threshold:.2f})")
            return False
        
        # Filter 2: Face context - if faces are detected, hands should be relatively near faces
        # Relaxed for distance detection - allow larger distances
        if faces_data:
            # Find closest face to this "hand"
            min_face_distance = float('inf')
            closest_face_size = None
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
                    # Estimate face size from bbox
                    if 'bbox' in face:
                        bbox = face['bbox']
                        face_w = bbox[2] / (frame_height if frame_height else 720)  # Normalize by frame height
                        face_h = bbox[3] / (frame_height if frame_height else 720)
                        closest_face_size = max(face_w, face_h)
            
            # Distance threshold: more lenient for distant/small faces
            # If face is small (distant), allow larger hand-to-face distance
            if closest_face_size is not None:
                # Distant face (<0.15 normalized) - allow up to 0.7 distance
                # Close face (>0.15 normalized) - allow up to 0.6 distance
                max_distance_threshold = 0.7 if closest_face_size < 0.15 else 0.6
            else:
                max_distance_threshold = 0.65  # Default threshold increased for distance detection
            
            # If closest face is very far, it's likely feet
            if min_face_distance > max_distance_threshold:
                import logging
                logger = logging.getLogger(__name__)
                logger.debug(f"Filtered out false hand detection: too far from faces (distance={min_face_distance:.3f}, threshold={max_distance_threshold:.2f})")
                return False
        
        # Filter 3: Hand geometry validation - relaxed for distance detection
        # Distant hands have less accurate landmark detection, so be more lenient
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
        
        # Hand-like proportions: more lenient for distant/small hands
        # Distant hands may have less accurate landmark detection
        if palm_size > 0.001:  # Avoid division by zero
            finger_to_palm_ratio = finger_length / palm_size
            # Valid hands: more lenient ratio for distant objects
            # Distant: 0.8-7.0, Close: 1.0-6.0
            min_ratio = 0.8 if is_likely_distant else 1.0
            max_ratio = 7.0 if is_likely_distant else 6.0
            
            if finger_to_palm_ratio < min_ratio or finger_to_palm_ratio > max_ratio:
                import logging
                logger = logging.getLogger(__name__)
                logger.debug(f"Filtered out false hand detection: invalid finger-to-palm ratio={finger_to_palm_ratio:.2f} (range: {min_ratio:.1f}-{max_ratio:.1f})")
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

                    # Filter out false positives (e.g., hands detected as faces)
                    # Validate face detection using geometric properties
                    # Faces are typically:
                    # - Roughly square (aspect ratio close to 1.0)
                    # - In upper portion of frame (y < 0.8 typically, but kids may be lower)
                    # - Have reasonable size (not too small, not too large)
                    
                    # Calculate aspect ratio (width/height)
                    aspect_ratio = bbox.width / bbox.height if bbox.height > 0 else 1.0
                    
                    # Face size validation: faces are typically 0.05 to 0.4 normalized height
                    # Very small (< 0.02) or very large (> 0.5) detections are likely false positives
                    # Kids' faces can be smaller, so we allow down to 0.02
                    face_height_norm = max(0.0, min(1.0, float(bbox.height)))
                    is_small_face = face_height_norm < 0.08  # Likely a child's face
                    
                    # Faces are roughly square (0.5 to 1.6 aspect ratio typically, more lenient for kids)
                    # Hands are more elongated (often > 1.8 or < 0.4)
                    # More lenient aspect ratio for small faces (kids)
                    if is_small_face:
                        is_valid_aspect_ratio = 0.5 <= aspect_ratio <= 1.6  # More lenient for kids
                    else:
                        is_valid_aspect_ratio = 0.6 <= aspect_ratio <= 1.5  # Standard for adults
                    
                    # Faces are typically in upper 80% of frame (y < 0.8)
                    # Kids may be shorter, so allow lower in frame (y < 0.85 for small faces)
                    # Hands can be anywhere but often lower
                    if is_small_face:
                        is_in_face_region = center_y < 0.85  # More lenient for kids (they're shorter)
                    else:
                        is_in_face_region = center_y < 0.8  # Standard for adults
                    
                    # Size validation: more lenient for small faces (kids)
                    if is_small_face:
                        is_valid_size = 0.02 <= face_height_norm <= 0.5  # Allow smaller faces (kids)
                    else:
                        is_valid_size = 0.03 <= face_height_norm <= 0.5  # Standard for adults
                    
                    # Require at least 1 out of 3 validations for small faces (kids), 2 out of 3 for normal faces
                    # This is more lenient for kids' faces which may not pass all validations
                    validation_score = sum([is_valid_aspect_ratio, is_in_face_region, is_valid_size])
                    required_score = 1 if is_small_face else 2
                    if validation_score < required_score:
                        import logging
                        logger = logging.getLogger(__name__)
                        logger.debug(f"Filtered out false face detection: aspect={aspect_ratio:.2f}, y={center_y:.2f}, size={face_height_norm:.3f}, score={validation_score}/3 (required: {required_score}, small_face={is_small_face})")
                        continue  # Skip this detection
                    
                    # --- Height-aware course plotting support ------------------------------------
                    # Estimate normalized face size using the relative bounding box height.
                    # This stays purely image-based and does NOT require camera calibration.
                    #
                    # face_height_norm:
                    #   - ~0.40–0.50 when very close to the camera
                    #   - ~0.05–0.15 when further away
                    # approx_distance:
                    #   - Simple inverse proxy for "how far away" the face is
                    #   - Larger faces (bigger height) => smaller approx_distance (closer)
                    #   - Smaller faces => larger approx_distance (farther)
                    #
                    # These values are used downstream for logging and for plotting a virtual
                    # ground target under the face. All behavior is gated by config flags so it
                    # can be rolled back easily if it behaves poorly.
                    face_height_norm = max(0.0, min(1.0, float(bbox.height)))
                    # Prevent division by very small faces so the estimate stays bounded.
                    safe_face_size = max(face_height_norm, config.FACE_DISTANCE_MIN_FACE_SIZE)
                    approx_distance = config.FACE_DISTANCE_CALIBRATION_K / safe_face_size
                    
                    faces_data.append({
                        'center': (center_x, center_y),          # Normalized coordinates
                        'bbox': bbox_px,                         # Pixel coordinates
                        'confidence': confidence,
                        'height_norm': face_height_norm,         # Normalized face height (0–1)
                        'approx_distance': approx_distance       # Image-based distance proxy
                    })
                except Exception as e:
                    import logging
                    logger = logging.getLogger(__name__)
                    logger.warning(f"Error processing face detection: {e}")
                    continue
        
        return faces_data, results
    
    def associate_gesture_with_closest_face(self, hand_position, faces_data):
        """
        Find the face closest to a gesture (hand position)
        Optimized for distance detection - increased distance threshold
        
        Args:
            hand_position: (x, y) normalized coordinates of hand center
            faces_data: List of face detection dicts with 'center' key
            
        Returns:
            Face dict with 'center', 'bbox', 'confidence', and 'distance' if found within threshold, None otherwise
        """
        if not faces_data or hand_position is None:
            return None
        
        hand_x, hand_y = hand_position
        closest_face = None
        min_distance = float('inf')
        
        # Maximum distance threshold increased from 0.4 to 0.7 for better distance detection
        # This allows gestures further from faces to still be associated (important for distant objects)
        max_distance = 0.7
        
        for face in faces_data:
            face_x, face_y = face['center']
            # Calculate Euclidean distance in normalized coordinates
            distance = np.sqrt((hand_x - face_x)**2 + (hand_y - face_y)**2)
            
            # Also consider face size for distance-aware threshold
            # Smaller faces (distant) should allow larger hand-to-face distances
            face_size = None
            if 'bbox' in face:
                bbox = face['bbox']
                # Estimate face size from bbox (normalized, approximate)
                face_size = max(bbox[2], bbox[3]) / 720.0  # Normalize by typical frame height
            
            # Adjust threshold based on face size (distant faces = more lenient)
            adjusted_max_distance = max_distance
            if face_size is not None and face_size < 0.15:
                # Small face (distant) - allow even larger distance
                adjusted_max_distance = 0.8
            
            if distance < adjusted_max_distance and distance < min_distance:
                min_distance = distance
                closest_face = face.copy()  # Make a copy
                closest_face['distance'] = distance  # Add distance to face dict
        
        return closest_face
    
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
    
    def classify_gesture(self, hands, frame=None, faces_data=None):
        """
        Classify gesture based on detected hands
        PRIORITY ORDER (absolute priority - if Gesture Recognizer returns a result, that's used):
        1. MediaPipe Gesture Recognizer (Tasks API) - takes ABSOLUTE PRIORITY if enabled
        2. Custom thumbs-up detection (fallback only if Gesture Recognizer unavailable/disabled)
        
        Args:
            hands: List of arrays of 21 hand landmarks (for backward compatibility - only used as fallback)
            frame: Optional BGR image frame (REQUIRED for Gesture Recognizer - if provided, Gesture Recognizer takes priority)
            faces_data: Optional list of face detection dicts for face association
            
        Returns:
            (gesture_type, hand_position, associated_face) tuple or (None, None, None)
            gesture_type: 'thumbs_up', 'stop', or None (only these two gestures supported)
            hand_position: (x, y) normalized coordinates of hand center, or None
            associated_face: Face dict with 'center', 'bbox', 'confidence', 'distance', or None
        """
        # PRIORITY 1: MediaPipe Gesture Recognizer takes ABSOLUTE PRIORITY
        # If Gesture Recognizer is enabled and frame is provided, use it exclusively
        # This ensures robust, pre-trained gesture classification takes precedence over custom detection
        if self.gesture_recognizer_enabled and frame is not None:
            gesture_type, hand_position, hand_landmarks = self.classify_gesture_with_recognizer(frame)
            
            if gesture_type:
                # Find closest face to gesture if faces_data provided
                associated_face = None
                if hand_position and faces_data:
                    associated_face = self.associate_gesture_with_closest_face(hand_position, faces_data)
                
                return gesture_type, hand_position, associated_face
        
        # Fallback to custom detection only for thumbs-up (stop gesture handled by recognizer)
        # Custom detection doesn't provide hand position or face association
        if not hands:
            return None, None, None
        
        # Check for thumbs up gesture using custom detection (fallback)
        for hand in hands:
            if self._is_thumbs_up_gesture(hand):
                # Get hand center for face association
                hand_position = self.get_hand_center(hand)
                associated_face = None
                if faces_data:
                    associated_face = self.associate_gesture_with_closest_face(hand_position, faces_data)
                return 'thumbs_up', hand_position, associated_face
        
        # No gesture detected
        return None, None, None
    
    def classify_gesture_with_recognizer(self, frame):
        """
        Classify gesture using MediaPipe Gesture Recognizer (Tasks API)
        Only returns 'thumbs_up' or 'stop' gestures
        
        Args:
            frame: BGR image frame
            
        Returns:
            (gesture_type, hand_position, hand_landmarks) tuple or (None, None, None)
            gesture_type: 'thumbs_up', 'stop', or None
            hand_position: (x, y) normalized coordinates of hand center, or None
            hand_landmarks: List of hand landmarks, or None
        """
        if not self.gesture_recognizer_enabled or self.gesture_recognizer is None:
            return None, None, None
        
        try:
            import logging
            logger = logging.getLogger(__name__)
            
            # Convert BGR to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Convert to MediaPipe Image
            # Image class is in the main mediapipe module (mp.Image)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            
            # Generate monotonically increasing timestamp based on running mode
            if self.running_mode == vision.RunningMode.LIVE_STREAM:
                # For LIVE_STREAM mode, use real-time timestamps (milliseconds since epoch)
                # This is appropriate for continuous live camera streams
                timestamp_ms = int(time.time() * 1000)
                # Ensure timestamp is always increasing (handle clock adjustments)
                if timestamp_ms <= self.last_frame_timestamp:
                    timestamp_ms = self.last_frame_timestamp + 1
                self.last_frame_timestamp = timestamp_ms
            else:  # VIDEO mode
                # For VIDEO mode with continuous camera stream, use frame count * milliseconds per frame
                # This represents the time position in the video stream
                self.frame_timestamp_counter += 1
                # Calculate milliseconds based on camera FPS (default 30 fps = 33.33 ms per frame)
                fps = getattr(config, 'CAMERA_FPS', 30)  # Default to 30 fps if not available
                ms_per_frame = 1000.0 / fps
                timestamp_ms = int(self.frame_timestamp_counter * ms_per_frame)
            
            # Use recognize_for_video for both VIDEO and LIVE_STREAM modes
            # This is the proper synchronous method for continuous video stream from camera
            result = self.gesture_recognizer.recognize_for_video(mp_image, timestamp_ms)
            
            # PERFORMANCE OPTIMIZATION: Cache full result and extract ALL hand landmarks
            # This allows detect_hands() to reuse the same result instead of processing the frame again
            self._cached_gesture_result = result
            
            # Extract ALL hand landmarks from Gesture Recognizer result (not just gesture hand)
            # This provides hand detection for the rest of the pipeline without running Hand Landmarker
            hands_data = []
            frame_height = frame.shape[0] if frame is not None else None
            
            if result.hand_landmarks and len(result.hand_landmarks) > 0:
                for hand_landmarks_mp in result.hand_landmarks:
                    if hand_landmarks_mp and len(hand_landmarks_mp) > 0:
                        # Convert landmarks to our format (numpy array)
                        landmarks = []
                        for lm in hand_landmarks_mp:
                            landmarks.append([lm.x, lm.y, lm.z])
                        landmarks_array = np.array(landmarks)
                        
                        # Validate that this is actually a hand (not feet or false positive)
                        # Use the same validation as Hand Landmarker for consistency
                        if self._is_valid_hand(landmarks_array, None, frame_height):  # faces_data not needed for validation
                            hands_data.append(landmarks_array)
            
            # Cache hand landmarks and create a compatible result object for drawing
            self._cached_hands_data = hands_data
            # Create a simple result-like object that draw_landmarks() can use
            # Gesture Recognizer result has hand_landmarks attribute, which is what we need
            self._cached_hands_result = result  # Gesture Recognizer result has hand_landmarks
            
            # Always cache hand landmarks (even if no gestures detected)
            # This allows detect_hands() to reuse the same result instead of processing the frame again
            logger.debug(f"Gesture Recognizer: Cached {len(hands_data)} validated hand(s) from result")
            
            if not result or not result.gestures or len(result.gestures) == 0:
                logger.debug("Gesture Recognizer: No gestures detected in frame, but hand landmarks cached")
                return None, None, None
            
            logger.debug(f"Gesture Recognizer: Found {len(result.gestures)} hand(s) with gesture results")
            
            # Process first hand's gestures (filter for thumbs_up or stop only)
            for gesture_list in result.gestures:
                if gesture_list and len(gesture_list) > 0:
                    # Get highest confidence gesture
                    best_gesture = max(gesture_list, key=lambda g: g.score)
                    
                    category_name = best_gesture.category_name
                    gesture_score = best_gesture.score
                    
                    logger.debug(f"Gesture Recognizer: Detected gesture '{category_name}' with score {gesture_score:.3f} (threshold: {config.GESTURE_RECOGNIZER_MIN_GESTURE_CONFIDENCE:.3f})")
                    
                    # Check confidence threshold
                    if gesture_score < config.GESTURE_RECOGNIZER_MIN_GESTURE_CONFIDENCE:
                        logger.debug(f"Gesture Recognizer: Gesture '{category_name}' below threshold ({gesture_score:.3f} < {config.GESTURE_RECOGNIZER_MIN_GESTURE_CONFIDENCE:.3f})")
                        continue
                    
                    # Filter: Only return thumbs_up or stop gestures
                    if category_name == 'Thumb_Up':
                        logger.info(f"Gesture Recognizer: THUMBS UP detected! Score: {gesture_score:.3f}")
                        # Extract hand position from landmarks
                        hand_position = None
                        hand_landmarks = None
                        
                        if result.hand_landmarks and len(result.hand_landmarks) > 0:
                            # Get hand center from first hand landmarks
                            hand_landmarks_mp = result.hand_landmarks[0]
                            if hand_landmarks_mp and len(hand_landmarks_mp) > 0:
                                # Use wrist (first landmark) as hand center
                                wrist = hand_landmarks_mp[0]
                                hand_position = (wrist.x, wrist.y)
                                
                                # Convert landmarks to our format
                                hand_landmarks = []
                                for lm in hand_landmarks_mp:
                                    hand_landmarks.append([lm.x, lm.y, lm.z])
                        
                        return 'thumbs_up', hand_position, hand_landmarks
                    
                    elif category_name == 'Open_Palm':
                        logger.info(f"Gesture Recognizer: STOP (Open_Palm) detected! Score: {gesture_score:.3f}")
                        # Extract hand position from landmarks
                        hand_position = None
                        hand_landmarks = None
                        
                        if result.hand_landmarks and len(result.hand_landmarks) > 0:
                            hand_landmarks_mp = result.hand_landmarks[0]
                            if hand_landmarks_mp and len(hand_landmarks_mp) > 0:
                                wrist = hand_landmarks_mp[0]
                                hand_position = (wrist.x, wrist.y)
                                
                                hand_landmarks = []
                                for lm in hand_landmarks_mp:
                                    hand_landmarks.append([lm.x, lm.y, lm.z])
                        
                        return 'stop', hand_position, hand_landmarks
                    
                    # Ignore other gestures - only return thumbs_up or stop
                    logger.debug(f"Gesture Recognizer: Ignoring gesture '{category_name}' (not thumbs_up or stop)")
                    continue
            
            logger.debug("Gesture Recognizer: No thumbs_up or stop gesture found after filtering")
            return None, None, None
            
        except Exception as e:
            import logging
            logger = logging.getLogger(__name__)
            logger.debug(f"Error in Gesture Recognizer: {e}")
            return None, None, None
    
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
        
        DEPRECATED: Treat gesture removed - only thumbs-up and stop gestures are supported
        """
        # Treat gesture disabled - only thumbs-up and stop gestures supported
        return False
    
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
        Determine if landmarks represent the thumbs up gesture: thumb sticking UP (with relaxed finger requirements)
        This is the trigger for face tracking and forward movement
        
        Detection criteria (size-aware for small hands like children's):
        1. Finger closure: At least 2 out of 4 fingers for small hands, 3 out of 4 for normal hands
           - OR thumb must be clearly extended upward (threshold scales with hand size)
           - Finger tips below PIP joints indicates closed finger
        2. Thumb must be extended upward with generous tilt tolerance
           - Thumb extension thresholds scale with hand size (smaller hands = smaller thresholds)
           - Thumb can be tilted up to 80° from vertical (much more forgiving for wrist rotation)
           - Uses angle calculation: atan2(|horizontal|, vertical) ≤ 80°
           - Accounts for natural wrist rotation and hand orientation
        3. Thumb tip must be above wrist (threshold scales with hand size)
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
        
        # Calculate middle finger length (MCP to tip) - most reliable size indicator
        # This is more consistent than wrist-to-tip distance and works better for calibration
        middle_mcp = np.array(landmarks[9][:2])
        middle_tip = np.array(landmarks[12][:2])
        middle_length = np.linalg.norm(middle_tip - middle_mcp)
        
        # Also calculate wrist to middle tip for fallback size detection
        wrist_to_middle_tip = np.linalg.norm(wrist - middle_tip)
        hand_size = wrist_to_middle_tip  # Hand size based on actual hand dimensions
        
        # Use reference finger length for calibration if provided
        # This allows personalized calibration for specific children's hand sizes
        if config.CHILD_REFERENCE_FINGER_ENABLED:
            # Calculate scale factor: detected_length / reference_length
            # If detected is smaller than reference, scale < 1.0 (more lenient)
            # If detected is larger than reference, scale > 1.0 (less lenient)
            size_scale = middle_length / config.CHILD_REFERENCE_FINGER_LENGTH
            
            # Clamp scale to reasonable range (0.3 to 1.5) to prevent extreme values
            size_scale = max(0.3, min(1.5, size_scale))
            
            # Determine if this is a small hand based on reference
            # If detected finger is within 30% of reference, treat as small hand (child)
            # If much larger, treat as normal hand (adult) - this ensures adults aren't affected
            reference_tolerance = 0.3  # 30% tolerance around reference
            is_small_hand = (middle_length <= config.CHILD_REFERENCE_FINGER_LENGTH * (1.0 + reference_tolerance))
            
            # For hands much larger than reference (adults), use normal thresholds
            if not is_small_hand:
                size_scale = 1.0  # Adult hands use full thresholds
        else:
            # Fallback to original method when reference not set
            # Determine if this is a small hand (like a child's)
            # Typical adult hand: wrist to middle tip ~0.12-0.18 normalized units
            # Child hand: wrist to middle tip ~0.08-0.12 normalized units
            is_small_hand = hand_size < 0.08
            
            # Scale thresholds based on hand size
            # For small hands, use proportionally smaller thresholds
            # Reference size: 0.13 (typical adult hand wrist-to-middle-tip distance)
            if is_small_hand:
                size_scale = max(0.5, hand_size / 0.13)  # Scale between 0.5x and 1.0x
                # Even more lenient for very small hands
                if hand_size < 0.07:
                    size_scale = 0.4  # Very small hands get 40% of thresholds
            else:
                size_scale = 1.0  # Normal hands use full thresholds (original values)
        
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
        # Also check that tips are below PIP joints for tighter fist detection
        index_pip = np.array(landmarks[6][:2])
        middle_pip = np.array(landmarks[10][:2])
        ring_pip = np.array(landmarks[14][:2])
        pinky_pip = np.array(landmarks[18][:2])
        
        # Finger is closed if tip is below PIP joint (more reliable than MCP for fist)
        index_closed = index_tip[1] > index_pip[1]  # Tip below PIP = closed
        middle_closed = middle_tip[1] > middle_pip[1]
        ring_closed = ring_tip[1] > ring_pip[1]
        pinky_closed = pinky_tip[1] > pinky_pip[1]
        
        # Check 2: Thumb must be extended upward - be more forgiving about finger closure
        # For thumbs up, we primarily care about thumb position, fingers can be partially extended
        # Require fewer closed fingers for small hands (2 out of 4 for small, 3 out of 4 for normal)
        fingers_closed_count = sum([index_closed, middle_closed, ring_closed, pinky_closed])
        required_closed_fingers = 1 if is_small_hand else 3  # More lenient for small hands
        enough_fingers_closed = fingers_closed_count >= required_closed_fingers
        
        # Calculate thumb vertical distance
        thumb_vertical_distance = thumb_mcp[1] - thumb_tip[1]  # Positive if thumb is above base
        
        # Scale thumb extension thresholds based on hand size
        # Base thresholds optimized for children's hands (more lenient than original)
        thumb_clearly_extended_threshold = 0.08 * size_scale  # Scaled for small hands
        # For normal hands, use 0.02 threshold (more lenient than original 0.03 for better detection); for small hands, scale down
        thumb_min_extension_threshold = (0.02 if size_scale >= 1.0 else 0.02 * size_scale)
        
        # If not enough fingers closed, still allow if thumb is clearly extended upward
        # This makes detection more forgiving for natural hand positions
        thumb_clearly_extended = thumb_vertical_distance > thumb_clearly_extended_threshold
        
        if not enough_fingers_closed and not thumb_clearly_extended:
            return False
        
        # Check 3: Thumb must be extended upward with tolerance for tilt and wrist rotation
        # Thumb tip should be above thumb base (threshold scales with hand size)
        if thumb_vertical_distance < thumb_min_extension_threshold:
            return False
        
        # Calculate thumb vector from MCP to tip
        thumb_vector = thumb_tip - thumb_mcp
        thumb_horizontal = thumb_vector[0]  # X component (positive = right, negative = left)
        thumb_vertical_raw = thumb_vector[1]  # Y component (Y increases downward, so negative = up)
        thumb_vertical = -thumb_vertical_raw  # Negate so positive = up
        
        # Calculate angle from vertical accounting for wrist rotation
        # Use atan2: angle = atan2(|horizontal|, vertical) gives angle from vertical
        # Calculate angle from vertical (0 degrees = straight up)
        if thumb_vertical > 0:  # Only check if thumb is pointing upward at all
            thumb_angle_rad = np.arctan2(abs(thumb_horizontal), thumb_vertical)
            thumb_angle_deg = np.degrees(thumb_angle_rad)
            
            # Allow up to 80 degrees tilt tolerance (much more forgiving for wrist rotation)
            # This allows thumbs up even when hand/wrist is rotated significantly
            max_tilt_deg = 80.0
            max_tilt_rad = np.radians(max_tilt_deg)
            is_within_tilt_tolerance = thumb_angle_rad <= max_tilt_rad
        else:
            # Thumb is pointing down, not up
            is_within_tilt_tolerance = False
        
        # Check 4: Thumb tip should be above wrist (for proper thumbs up)
        thumb_above_wrist = thumb_tip[1] < wrist[1]  # Thumb tip Y < wrist Y means thumb is above
        
        # More lenient: thumb should be at least slightly above wrist (threshold scales with hand size)
        wrist_to_thumb_height = wrist[1] - thumb_tip[1]  # Positive if thumb is above wrist
        # For normal hands, use original 0.02 threshold; for small hands, scale down
        wrist_extension_threshold = (0.02 if size_scale >= 1.0 else 0.015 * size_scale)
        significant_extension_above_wrist = wrist_to_thumb_height > wrist_extension_threshold
        
        return is_within_tilt_tolerance and thumb_above_wrist and significant_extension_above_wrist
    
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
                       is_waving=False, current_gesture=None, faces_data=None):
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
        
        # Handle both Hand Landmarker (Tasks API) and Solutions API results
        # Hand Landmarker has 'hand_landmarks' attribute (list)
        # Solutions API has 'multi_hand_landmarks' attribute
        hand_landmarks_list = None
        if hasattr(results, 'hand_landmarks'):
            # Hand Landmarker Tasks API result
            hand_landmarks_list = results.hand_landmarks
        elif hasattr(results, 'multi_hand_landmarks'):
            # Solutions API result
            hand_landmarks_list = results.multi_hand_landmarks
        
        if hand_landmarks_list:
            for idx, hand_landmarks in enumerate(hand_landmarks_list):
                # Draw landmarks - only for Solutions API (Hand Landmarker doesn't have drawing utils)
                # For Hand Landmarker, we'll just draw bounding boxes
                if hasattr(results, 'multi_hand_landmarks') and self.mp_hands is not None:
                    # Solutions API - use drawing utils
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
                        # Hand Landmarker: landmarks are already in list format (landmark_pb2.NormalizedLandmark)
                        # Solutions API: landmarks have .landmark attribute
                        landmarks = []
                        if hasattr(hand_landmarks, 'landmark'):
                            # Solutions API format
                            for lm in hand_landmarks.landmark:
                                landmarks.append([lm.x, lm.y, lm.z])
                        else:
                            # Hand Landmarker Tasks API format (list of landmark_pb2.NormalizedLandmark)
                            for lm in hand_landmarks:
                                landmarks.append([lm.x, lm.y, lm.z])
                    
                    bbox = self.get_bounding_box(landmarks, width, height)
                    if bbox:
                        x, y, w, h = bbox
                        
                        # Determine box color and label based on detected motion/gesture
                        box_color = (0, 255, 0)  # Default green
                        label_parts = []
                        
                        # Import logging at function level
                        import logging
                        logger = logging.getLogger(__name__)
                        
                        # Log hand landmark details for fine-tuning (log every 30 frames to reduce spam)
                        self._landmark_log_frame_counter += 1
                        
                        if self._landmark_log_frame_counter % 30 == 0:
                            # Log key landmark positions for thumbs-up detection
                            wrist = landmarks[0]
                            thumb_mcp = landmarks[2]
                            thumb_tip = landmarks[4]
                            index_mcp = landmarks[5]
                            index_pip = landmarks[6]
                            index_tip = landmarks[8]
                            middle_mcp = landmarks[9]
                            middle_pip = landmarks[10]
                            middle_tip = landmarks[12]
                            
                            # Calculate finger lengths (MCP to tip) for reference determination
                            index_length = np.linalg.norm(np.array(index_tip[:2]) - np.array(index_mcp[:2]))
                            middle_length = np.linalg.norm(np.array(middle_tip[:2]) - np.array(middle_mcp[:2]))
                            
                            logger.info(f"Hand landmarks (hand {idx}):")
                            logger.info(f"  Wrist: ({wrist[0]:.3f}, {wrist[1]:.3f}, {wrist[2]:.3f})")
                            logger.info(f"  Thumb MCP: ({thumb_mcp[0]:.3f}, {thumb_mcp[1]:.3f}, {thumb_mcp[2]:.3f})")
                            logger.info(f"  Thumb Tip: ({thumb_tip[0]:.3f}, {thumb_tip[1]:.3f}, {thumb_tip[2]:.3f})")
                            
                            # Calculate thumb vector and angle
                            thumb_vector = np.array(thumb_tip[:2]) - np.array(thumb_mcp[:2])
                            thumb_horizontal = thumb_vector[0]
                            thumb_vertical = -thumb_vector[1]  # Negate because Y increases downward
                            if thumb_vertical > 0:  # Only calculate if thumb is pointing up
                                thumb_angle_rad = np.arctan2(abs(thumb_horizontal), thumb_vertical)
                                thumb_angle_deg = np.degrees(thumb_angle_rad)
                                logger.info(f"  Thumb angle from vertical: {thumb_angle_deg:.1f}°")
                            
                            logger.info(f"  Index PIP: ({index_pip[0]:.3f}, {index_pip[1]:.3f}), Tip: ({index_tip[0]:.3f}, {index_tip[1]:.3f})")
                            logger.info(f"  Middle PIP: ({middle_pip[0]:.3f}, {middle_pip[1]:.3f}), Tip: ({middle_tip[0]:.3f}, {middle_tip[1]:.3f})")
                            
                            # Log finger lengths for calibration
                            logger.info(f"  Finger lengths (normalized, MCP to tip): Index={index_length:.4f}, Middle={middle_length:.4f}")
                            
                            # Show calibration info if reference is set
                            if config.CHILD_REFERENCE_FINGER_ENABLED:
                                scale_factor = middle_length / config.CHILD_REFERENCE_FINGER_LENGTH
                                logger.info(f"  Calibration: ref={config.CHILD_REFERENCE_FINGER_LENGTH:.4f}, detected={middle_length:.4f}, scale={scale_factor:.2f}x")
                            else:
                                logger.info(f"  Calibration: Set CHILD_REFERENCE_FINGER_LENGTH={middle_length:.4f} to use this as baseline")
                            
                            # Check finger closed state
                            index_closed = index_tip[1] > index_pip[1]
                            middle_closed = middle_tip[1] > middle_pip[1]
                            logger.info(f"  Fingers closed: Index={index_closed}, Middle={middle_closed}")
                        
                        # Check for static gestures first (they take priority)
                        # Classify gesture using recognizer if enabled, or custom detection
                        # Always classify gesture, even if hands_data is None (we have landmarks from MediaPipe)
                        gesture = None
                        try:
                            # Pass frame for Gesture Recognizer, use landmarks for custom fallback
                            # Also pass faces_data for face association
                            gesture, hand_pos, associated_face = self.classify_gesture([landmarks], frame=frame, faces_data=faces_data)
                            
                            # Log gesture detection for diagnostics
                            if gesture:
                                logger.debug(f"Gesture detected in draw_landmarks: {gesture}")
                            else:
                                logger.debug(f"No gesture detected in draw_landmarks - will show HAND label")
                            
                            if gesture == 'thumbs_up':
                                box_color = (0, 255, 0)  # Green for thumbs up
                                label_parts.append("THUMBS UP")
                            elif gesture == 'stop':
                                box_color = (0, 0, 255)  # Red for stop
                                label_parts.append("STOP")
                            # Treat gesture removed - no longer supported
                        except Exception as e:
                            # Log error but continue with default label
                            logger.debug(f"Error classifying gesture: {e}")
                        
                        # Add waving status (can be combined with gestures)
                        # Only show "WAVING" if gesture detection mode allows it (not in 'gesture' mode)
                        if is_waving and config.GESTURE_DETECTION_MODE != 'gesture' and not gesture:
                            label_parts.append("WAVING")
                            box_color = (0, 255, 255)  # Yellow/Cyan for waving
                        
                        # Default label if nothing detected
                        if not label_parts:
                            label_parts.append("HAND")
                        
                        # Log bounding box appearance (similar to face detection logging)
                        label_text = " | ".join(label_parts)
                        logger.info(f"Drawing {label_text} bounding box: x={x}, y={y}, w={w}, h={h}, frame_size={width}x{height}")
                        
                        # Draw rectangle with appropriate color
                        cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
                        
                        # Draw label above bounding box with background for readability
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
                         clap_finger_threshold=None, clap_palm_threshold=None, 
                         model_complexity=None, face_min_detection_confidence=None, 
                         face_model_selection=None):
        """
        Update gesture detection parameters in real-time
        
        Args:
            min_detection_confidence: Minimum confidence for hand detection (0.0-1.0)
            min_tracking_confidence: Minimum confidence for hand tracking (0.0-1.0)
            gesture_confidence_threshold: Overall confidence threshold for gesture classification
            dance_hold_time: How long dance gesture must be held (seconds)
            clap_finger_threshold: Max distance between fingertips for clap (normalized)
            clap_palm_threshold: Max distance between palms for clap (normalized)
            model_complexity: MediaPipe model complexity (0, 1, or 2)
            face_min_detection_confidence: Face detection confidence threshold (0.0-1.0)
            face_model_selection: Face model selection (0=short-range, 1=full-range)
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
        
        # Treat gesture removed - only thumbs-up and stop gestures are supported
        # No treat_hold_time parameter needed
        
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
            # Treat gesture removed - only thumbs-up and stop gestures are supported
            'clap_finger_threshold': config.DANCE_CLAP_FINGER_THRESHOLD,
            'clap_palm_threshold': config.DANCE_CLAP_PALM_THRESHOLD,
            'face_min_detection_confidence': self.face_min_detection_confidence,
            'face_model_selection': self.face_model_selection
        }
    
    def close(self):
        """Clean up resources"""
        if self.hands is not None:
            self.hands.close()
        if self.hand_landmarker is not None:
            self.hand_landmarker.close()
        if self.face_detection is not None:
            self.face_detection.close()
        if self.gesture_recognizer is not None:
            self.gesture_recognizer.close()

