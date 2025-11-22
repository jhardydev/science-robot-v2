"""
Gesture detector using MediaPipe for hand detection and gesture recognition
"""
import cv2
import mediapipe as mp
import numpy as np
from science_robot import config


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
    
    def draw_landmarks(self, frame, results):
        """
        Draw hand landmarks on frame (for visualization)
        
        Args:
            frame: BGR image frame
            results: MediaPipe results object
        """
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                    self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2)
                )
    
    def close(self):
        """Clean up resources"""
        self.hands.close()

