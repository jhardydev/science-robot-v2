"""
Configuration parameters for Duckiebot Science Fair Robot
Updated for Ubuntu 22.04 and Catkin package structure
"""
import os
import rospkg

# Get package path
try:
    rospack = rospkg.RosPack()
    PACKAGE_PATH = rospack.get_path('science_robot')
except (rospkg.ResourceNotFound, rospkg.ROSException):
    # Fallback if ROS package not found (e.g., during development)
    PACKAGE_PATH = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ROS configuration
ROBOT_NAME = os.getenv('VEHICLE_NAME', 'robot1')  # Can be overridden by env var
MOTOR_TOPIC = f'/{ROBOT_NAME}/wheels_driver_node/wheels_cmd'
CAMERA_TOPIC = f'/{ROBOT_NAME}/camera_node/image/compressed'
EMERGENCY_STOP_TOPIC = f'/{ROBOT_NAME}/wheels_driver_node/emergency_stop'

# Camera settings optimized for IMX219 sensor
# IMX219 native capabilities (per Arducam documentation):
# - 3280x2464 @ 21 fps (8MP native)
# - 3280x1848 @ 28 fps
# - 1920x1080 @ 30 fps (recommended for video/gesture detection)
# - 1640x1232 @ 30 fps (2MP binning, good balance)
# - 1280x720 @ 30 fps
# - 640x480 @ various fps
# 
# Using 1920x1080 @ 30 fps for optimal gesture detection and tracking
# This is a native sensor mode that provides excellent quality while maintaining
# smooth frame rates for real-time processing
CAMERA_WIDTH = int(os.getenv('CAMERA_WIDTH', '1920'))  # IMX219 native 1920x1080 mode
CAMERA_HEIGHT = int(os.getenv('CAMERA_HEIGHT', '1080'))  # 16:9 aspect ratio
CAMERA_FPS = int(os.getenv('CAMERA_FPS', '30'))  # Native 30 fps for 1920x1080 mode

# Processing resolution (can be different from capture resolution)
# MediaPipe works well at 640x480, but higher resolution improves accuracy
# Set to None to use same as CAMERA_WIDTH/HEIGHT (no downscaling)
# Using 1280x720 for good balance between quality and performance
PROCESSING_WIDTH = int(os.getenv('PROCESSING_WIDTH', '1280'))  # Process at 1280x720 for balance
PROCESSING_HEIGHT = int(os.getenv('PROCESSING_HEIGHT', '720'))  # Good performance/quality tradeoff

# Motor speeds
MOTOR_BASE_SPEED = 0.5  # Normalized speed (0.0 to 1.0)
MOTOR_TURN_SPEED = 0.6  # Normalized speed (0.0 to 1.0)
MOTOR_MAX_SPEED = 0.8   # Maximum speed in m/s
MOTOR_DANCE_SPEED = 0.7  # Normalized speed (0.0 to 1.0)

# Wave detection parameters
# Lowered thresholds for more sensitive wave detection
WAVE_DETECTION_FRAMES = int(os.getenv('WAVE_DETECTION_FRAMES', '10'))  # Reduced from 15 (faster response)
WAVE_MOTION_THRESHOLD = int(os.getenv('WAVE_MOTION_THRESHOLD', '20'))  # Reduced from 30 (detects smaller waves)
WAVE_MIN_DURATION = float(os.getenv('WAVE_MIN_DURATION', '0.3'))  # Reduced from 0.5 (faster trigger)
WAVE_SENSITIVITY = float(os.getenv('WAVE_SENSITIVITY', '0.5'))  # Increased from 0.3 (more sensitive)

# Gesture detection mode
# Options: 'wave', 'gesture', 'both'
# 'wave' = only motion-based wave detection
# 'gesture' = only thumbs up gesture detection (triggers tracking)
# 'both' = either wave OR thumbs up gesture triggers tracking
GESTURE_DETECTION_MODE = os.getenv('GESTURE_DETECTION_MODE', 'gesture').lower()  # Default to thumbs up only

# Thumbs up gesture detection parameters (trigger for face tracking)
THUMBS_UP_MIN_DURATION = float(os.getenv('THUMBS_UP_MIN_DURATION', '0.1'))  # Minimum seconds to hold thumbs up (lowered from 0.2 for distance detection)
THUMBS_UP_GRACE_PERIOD = float(os.getenv('THUMBS_UP_GRACE_PERIOD', '0.5'))  # Grace period for intermittent detection (allow brief gaps)
THUMBS_UP_REQUIRE_FACE = os.getenv('THUMBS_UP_REQUIRE_FACE', 'False').lower() == 'true'  # Require face detection for thumbs up

# Navigation parameters
STEERING_GAIN = float(os.getenv('STEERING_GAIN', '1.8'))  # Reduced from 2.5 for smoother, less aggressive control (works with PID)
STEERING_DEAD_ZONE = float(os.getenv('STEERING_DEAD_ZONE', '0.05'))  # Reduced from 0.1 for tighter tracking
MAX_STEERING_ANGLE = float(os.getenv('MAX_STEERING_ANGLE', '1.0'))
TRACKING_SMOOTHING = float(os.getenv('TRACKING_SMOOTHING', '0.7'))  # Position smoothing factor (0.0-1.0, higher = smoother)
TRACKING_TIMEOUT = float(os.getenv('TRACKING_TIMEOUT', '1.0'))  # Seconds to keep tracking after wave stops
TRACKING_LOOKAHEAD_TIME = float(os.getenv('TRACKING_LOOKAHEAD_TIME', '0.2'))  # Seconds to predict ahead for velocity-based tracking
SPEED_BY_DISTANCE = os.getenv('SPEED_BY_DISTANCE', 'True').lower() == 'true'  # Adjust speed based on distance

# PID steering control parameters
STEERING_KP = float(os.getenv('STEERING_KP', '1.2'))  # Reduced from 1.8 to 1.2 for less aggressive control (prevents spinning)
STEERING_KI = float(os.getenv('STEERING_KI', '0.1'))  # Integral gain (eliminates steady-state error)
STEERING_KD = float(os.getenv('STEERING_KD', '0.3'))  # Derivative gain (reduces overshoot and oscillation)
STEERING_INTEGRAL_MAX = float(os.getenv('STEERING_INTEGRAL_MAX', '0.5'))  # Maximum integral term to prevent windup

# Adaptive steering gain
STEERING_ADAPTIVE_ENABLED = os.getenv('STEERING_ADAPTIVE_ENABLED', 'True').lower() == 'true'  # Enable adaptive gain
STEERING_ADAPTIVE_FACTOR = float(os.getenv('STEERING_ADAPTIVE_FACTOR', '0.5'))  # Reduction factor for adaptive gain
STEERING_LARGE_ERROR_THRESHOLD = float(os.getenv('STEERING_LARGE_ERROR_THRESHOLD', '0.4'))  # Error threshold for large error reduction
STEERING_LARGE_ERROR_REDUCTION = float(os.getenv('STEERING_LARGE_ERROR_REDUCTION', '0.6'))  # Reduction factor for large errors (0.6 = reduce by 40%)

# Stopping and deceleration parameters
STOPPING_DECELERATION_START = float(os.getenv('STOPPING_DECELERATION_START', '0.6'))  # Start decelerating when target_y > this value
STOPPING_DECELERATION_FACTOR = float(os.getenv('STOPPING_DECELERATION_FACTOR', '0.7'))  # Speed reduction factor in deceleration zone

# Gesture recognition thresholds
GESTURE_CONFIDENCE_THRESHOLD = 0.3
DANCE_GESTURE_HOLD_TIME = 1.0
# TREAT_GESTURE_HOLD_TIME deprecated - treat gesture removed, using only thumbs-up and stop
# TREAT_GESTURE_HOLD_TIME = 2.0
DANCE_CLAP_FINGER_THRESHOLD = 0.12
DANCE_CLAP_PALM_THRESHOLD = 0.18

# MediaPipe performance settings
# model_complexity: 0=fastest (lower accuracy), 1=balanced (default), 2=slowest (highest accuracy)
# Lower values increase FPS but may reduce detection accuracy slightly
MEDIAPIPE_MODEL_COMPLEXITY = int(os.getenv('MEDIAPIPE_MODEL_COMPLEXITY', '1'))  # 1 for balanced accuracy/FPS (better for distance detection)

# MediaPipe detection confidence thresholds (lowered for better distance detection)
# Lower thresholds allow detection of smaller/distant objects but may increase false positives
# These settings optimize for detecting gestures and faces at greater distances (3-5 meters)
MEDIAPIPE_HAND_DETECTION_CONFIDENCE = float(os.getenv('MEDIAPIPE_HAND_DETECTION_CONFIDENCE', '0.35'))  # Lowered from 0.5 for distance detection
MEDIAPIPE_HAND_TRACKING_CONFIDENCE = float(os.getenv('MEDIAPIPE_HAND_TRACKING_CONFIDENCE', '0.35'))  # Lowered from 0.5 for distance detection  
MEDIAPIPE_FACE_DETECTION_CONFIDENCE = float(os.getenv('MEDIAPIPE_FACE_DETECTION_CONFIDENCE', '0.30'))  # Lowered for better distance face detection

# Face navigation / course plotting settings
# These settings make it easy to experiment with height-aware, face-based navigation and to roll back if needed.
# Set FACE_NAV_USE_GROUND_TARGET=False to completely disable the new behavior and revert to original center-of-face targeting.
FACE_NAV_USE_GROUND_TARGET = os.getenv('FACE_NAV_USE_GROUND_TARGET', 'True').lower() == 'true'
# Normalized y-position (0=top, 1=bottom) of the "virtual ground point" under the face that the robot should drive toward.
# Example: 0.75 means aim at a point 75% down from the top of the frame (near the bottom, but not at the edge).
FACE_NAV_GROUND_TARGET_Y = float(os.getenv('FACE_NAV_GROUND_TARGET_Y', '0.75'))

# Approximate face-distance estimation (purely image-based, for tuning and future use)
# We treat the normalized face height as an inverse proxy for distance: larger face => closer, smaller => farther.
# This keeps all logic image-based (no calibration required) and makes it easy to adjust or disable.
FACE_DISTANCE_CALIBRATION_K = float(os.getenv('FACE_DISTANCE_CALIBRATION_K', '1.0'))
FACE_DISTANCE_MIN_FACE_SIZE = float(os.getenv('FACE_DISTANCE_MIN_FACE_SIZE', '0.05'))  # Prevent division by very small faces

# Dance routine settings
DANCE_DURATION = 5.0
DANCE_MOVE_DURATION = 1.0

# Main loop settings
# Higher FPS improves gesture detection accuracy by providing more samples
# For IMX219 @ 1920x1080 30fps, processing at 30-60 FPS is optimal
# Processing faster than camera FPS doesn't help (camera is the bottleneck)
# Set to match or slightly exceed camera FPS for best results
# Main loop settings optimized for IMX219
# For IMX219 @ 1920x1080 30fps, processing at 30 FPS is optimal
# Processing faster than camera FPS doesn't help (camera is the bottleneck)
# Set to match camera FPS for best efficiency
MAIN_LOOP_FPS = int(os.getenv('MAIN_LOOP_FPS', '30'))  # Match IMX219 camera FPS (30 fps for 1920x1080 mode)
DISPLAY_OUTPUT = os.getenv('DISPLAY_OUTPUT', 'False').lower() == 'true'

# Virtual display settings
USE_VIRTUAL_DISPLAY = os.getenv('USE_VIRTUAL_DISPLAY', 'False').lower() == 'true'
VIRTUAL_DISPLAY_NUM = int(os.getenv('VIRTUAL_DISPLAY_NUM', '99'))
VIRTUAL_DISPLAY_SIZE = os.getenv('VIRTUAL_DISPLAY_SIZE', '1024x768x24')

# Debug settings
DEBUG_MODE = os.getenv('DEBUG_MODE', 'False').lower() == 'true'
LOG_GESTURES = True

# Logging settings
LOG_DIR = os.getenv('LOG_DIR', os.path.join(PACKAGE_PATH, 'logs'))
LOG_LEVEL = os.getenv('LOG_LEVEL', 'INFO').upper()
ENABLE_FILE_LOGGING = os.getenv('ENABLE_FILE_LOGGING', 'True').lower() == 'true'

# NVIDIA acceleration settings
USE_CUDA_ACCELERATION = True
USE_VPI_ACCELERATION = True
VPI_BACKEND = 'CUDA'
GPU_PREPROCESSING = True

# Performance monitoring
ENABLE_PERFORMANCE_MONITORING = False

# Web server settings
ENABLE_WEB_SERVER = os.getenv('ENABLE_WEB_SERVER', 'False').lower() == 'true'
WEB_SERVER_PORT = int(os.getenv('WEB_SERVER_PORT', '5000'))
WEB_SERVER_HOST = os.getenv('WEB_SERVER_HOST', '0.0.0.0')

# Collision avoidance settings
ENABLE_COLLISION_AVOIDANCE = os.getenv('ENABLE_COLLISION_AVOIDANCE', 'True').lower() == 'true'
COLLISION_EMERGENCY_DISTANCE = float(os.getenv('COLLISION_EMERGENCY_DISTANCE', '0.15'))  # meters - immediate stop
COLLISION_WARNING_DISTANCE = float(os.getenv('COLLISION_WARNING_DISTANCE', '0.30'))  # meters - slow down
COLLISION_SAFE_DISTANCE = float(os.getenv('COLLISION_SAFE_DISTANCE', '0.35'))  # meters - normal operation
COLLISION_MAX_DISTANCE = float(os.getenv('COLLISION_MAX_DISTANCE', '3.0'))  # meters - max detection range
COLLISION_SPEED_REDUCTION = float(os.getenv('COLLISION_SPEED_REDUCTION', '0.5'))  # Speed reduction factor in warning zone
COLLISION_EDGE_THRESHOLD = int(os.getenv('COLLISION_EDGE_THRESHOLD', '5'))  # Minimum vertical edges to detect obstacle
COLLISION_DARK_REGION_THRESHOLD = float(os.getenv('COLLISION_DARK_REGION_THRESHOLD', '0.15'))  # Ratio of dark pixels to detect obstacle

# ToF sensor filtering (to ignore floor readings)
# Increased to 0.25m to filter out floor readings (typical floor distance is 0.06-0.20m)
TOF_MIN_VALID_DISTANCE = float(os.getenv('TOF_MIN_VALID_DISTANCE', '0.25'))  # meters - ignore readings below this (likely floor)
TOF_FLOOR_FILTER_ENABLED = os.getenv('TOF_FLOOR_FILTER_ENABLED', 'True').lower() == 'true'  # Enable floor filtering
TOF_STABILITY_THRESHOLD = float(os.getenv('TOF_STABILITY_THRESHOLD', '0.02'))  # meters - max variation for "stable" reading (likely floor)
TOF_STABILITY_SAMPLES = int(os.getenv('TOF_STABILITY_SAMPLES', '10'))  # Number of samples to check for stability

# ToF cliff/edge detection (to prevent driving off ledges)
# Detects sudden increases in ToF distance indicating the floor dropped away
TOF_CLIFF_DETECTION_ENABLED = os.getenv('TOF_CLIFF_DETECTION_ENABLED', 'True').lower() == 'true'  # Enable cliff detection
TOF_CLIFF_DISTANCE_THRESHOLD = float(os.getenv('TOF_CLIFF_DISTANCE_THRESHOLD', '1.0'))  # meters - sudden jump to this distance indicates cliff
TOF_CLIFF_INCREASE_THRESHOLD = float(os.getenv('TOF_CLIFF_INCREASE_THRESHOLD', '0.5'))  # meters - minimum increase to trigger cliff detection
TOF_CLIFF_HISTORY_SAMPLES = int(os.getenv('TOF_CLIFF_HISTORY_SAMPLES', '5'))  # Number of recent readings to compare against

# ToF sensor configuration (I2C address 0x29, Bus 1, Channel 6)
TOF_I2C_BUS = int(os.getenv('TOF_I2C_BUS', '1'))
TOF_I2C_CHANNEL = int(os.getenv('TOF_I2C_CHANNEL', '6'))
TOF_I2C_ADDRESS = os.getenv('TOF_I2C_ADDRESS', '0x29')
TOF_TOPIC_OVERRIDE = os.getenv('TOF_TOPIC_OVERRIDE', '')  # Manually specify ToF topic if auto-detection fails

# Encoder configuration
ENCODER_ENABLED = os.getenv('ENCODER_ENABLED', 'True').lower() == 'true'  # Enable encoder feedback
ENCODER_PPR = int(os.getenv('ENCODER_PPR', '137'))  # Pulses per revolution
WHEEL_DIAMETER = float(os.getenv('WHEEL_DIAMETER', '0.0664'))  # Meters (66.40mm)

# Speed controller PID gains
SPEED_CONTROLLER_KP = float(os.getenv('SPEED_CONTROLLER_KP', '1.0'))  # Proportional gain
SPEED_CONTROLLER_KI = float(os.getenv('SPEED_CONTROLLER_KI', '0.1'))  # Integral gain
SPEED_CONTROLLER_KD = float(os.getenv('SPEED_CONTROLLER_KD', '0.05'))  # Derivative gain
SPEED_CONTROLLER_MAX_INTEGRAL = float(os.getenv('SPEED_CONTROLLER_MAX_INTEGRAL', '0.5'))  # Max integral term to prevent windup

# IMU configuration
IMU_ENABLED = os.getenv('IMU_ENABLED', 'True').lower() == 'true'  # Enable IMU monitoring
IMU_SPINNING_THRESHOLD = float(os.getenv('IMU_SPINNING_THRESHOLD', '0.05'))  # rad/s - threshold for detecting spinning (lowered from 0.5 to catch actual rotation)
IMU_DANGEROUS_SPIN_THRESHOLD = float(os.getenv('IMU_DANGEROUS_SPIN_THRESHOLD', '0.5'))  # rad/s - emergency stop threshold (lowered from 2.0)
IMU_VALIDATION_TOLERANCE = float(os.getenv('IMU_VALIDATION_TOLERANCE', '0.3'))  # rad/s - tolerance for encoder validation
WHEEL_BASE = float(os.getenv('WHEEL_BASE', '0.1'))  # meters - distance between wheels (adjust to your robot)

# Diagnostic logging for movement control
ENABLE_MOVEMENT_DIAGNOSTICS = os.getenv('ENABLE_MOVEMENT_DIAGNOSTICS', 'True').lower() == 'true'  # Enable detailed movement logging
MOVEMENT_DIAGNOSTICS_INTERVAL = float(os.getenv('MOVEMENT_DIAGNOSTICS_INTERVAL', '1.0'))  # seconds - how often to log diagnostics

# MediaPipe Gesture Recognizer configuration
GESTURE_RECOGNIZER_MODEL_PATH = os.path.join(
    PACKAGE_PATH, 'models', 'gesture_recognizer.task'
)
GESTURE_RECOGNIZER_ENABLED = os.getenv('GESTURE_RECOGNIZER_ENABLED', 'True').lower() == 'true'  # Default True
GESTURE_RECOGNIZER_MIN_DETECTION_CONFIDENCE = float(
    os.getenv('GESTURE_RECOGNIZER_MIN_DETECTION_CONFIDENCE', '0.3')
)
GESTURE_RECOGNIZER_MIN_GESTURE_CONFIDENCE = float(
    os.getenv('GESTURE_RECOGNIZER_MIN_GESTURE_CONFIDENCE', '0.3')
)
GESTURE_RECOGNIZER_RUNNING_MODE = os.getenv('GESTURE_RECOGNIZER_RUNNING_MODE', 'VIDEO').upper()
# Options: 'VIDEO' (synchronous, uses recognize_for_video()) or 'LIVE_STREAM' (async, requires callback)
# Default to VIDEO since we're using recognize_for_video() synchronously
