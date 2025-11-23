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

# Navigation parameters
STEERING_GAIN = float(os.getenv('STEERING_GAIN', '2.5'))  # Increased from 1.5 for more responsive tracking
STEERING_DEAD_ZONE = float(os.getenv('STEERING_DEAD_ZONE', '0.05'))  # Reduced from 0.1 for tighter tracking
MAX_STEERING_ANGLE = float(os.getenv('MAX_STEERING_ANGLE', '1.0'))
TRACKING_SMOOTHING = float(os.getenv('TRACKING_SMOOTHING', '0.7'))  # Position smoothing factor (0.0-1.0, higher = smoother)
TRACKING_TIMEOUT = float(os.getenv('TRACKING_TIMEOUT', '1.0'))  # Seconds to keep tracking after wave stops
SPEED_BY_DISTANCE = os.getenv('SPEED_BY_DISTANCE', 'True').lower() == 'true'  # Adjust speed based on distance

# Gesture recognition thresholds
GESTURE_CONFIDENCE_THRESHOLD = 0.5
DANCE_GESTURE_HOLD_TIME = 1.0
TREAT_GESTURE_HOLD_TIME = 2.0
DANCE_CLAP_FINGER_THRESHOLD = 0.12
DANCE_CLAP_PALM_THRESHOLD = 0.18

# MediaPipe performance settings
# model_complexity: 0=fastest (lower accuracy), 1=balanced (default), 2=slowest (highest accuracy)
# Lower values increase FPS but may reduce detection accuracy slightly
MEDIAPIPE_MODEL_COMPLEXITY = int(os.getenv('MEDIAPIPE_MODEL_COMPLEXITY', '0'))  # 0 for maximum FPS

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

# ToF sensor configuration (I2C address 0x29, Bus 1, Channel 6)
TOF_I2C_BUS = int(os.getenv('TOF_I2C_BUS', '1'))
TOF_I2C_CHANNEL = int(os.getenv('TOF_I2C_CHANNEL', '6'))
TOF_I2C_ADDRESS = os.getenv('TOF_I2C_ADDRESS', '0x29')
TOF_TOPIC_OVERRIDE = os.getenv('TOF_TOPIC_OVERRIDE', '')  # Manually specify ToF topic if auto-detection fails

