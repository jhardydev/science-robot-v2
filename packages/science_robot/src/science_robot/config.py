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

# Camera settings (for resizing/processing, actual source is ROS topic)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 60

# Motor speeds
MOTOR_BASE_SPEED = 0.5  # Normalized speed (0.0 to 1.0)
MOTOR_TURN_SPEED = 0.6  # Normalized speed (0.0 to 1.0)
MOTOR_MAX_SPEED = 0.8   # Maximum speed in m/s
MOTOR_DANCE_SPEED = 0.7  # Normalized speed (0.0 to 1.0)

# Wave detection parameters
WAVE_DETECTION_FRAMES = 15
WAVE_MOTION_THRESHOLD = 30
WAVE_MIN_DURATION = 0.5
WAVE_SENSITIVITY = 0.3

# Navigation parameters
STEERING_GAIN = 1.5
STEERING_DEAD_ZONE = 0.1
MAX_STEERING_ANGLE = 1.0

# Gesture recognition thresholds
GESTURE_CONFIDENCE_THRESHOLD = 0.7
DANCE_GESTURE_HOLD_TIME = 1.0
TREAT_GESTURE_HOLD_TIME = 2.0
DANCE_CLAP_FINGER_THRESHOLD = 0.12
DANCE_CLAP_PALM_THRESHOLD = 0.18

# Dance routine settings
DANCE_DURATION = 5.0
DANCE_MOVE_DURATION = 1.0

# Main loop settings
MAIN_LOOP_FPS = 60
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
COLLISION_SAFE_DISTANCE = float(os.getenv('COLLISION_SAFE_DISTANCE', '0.50'))  # meters - normal operation
COLLISION_MAX_DISTANCE = float(os.getenv('COLLISION_MAX_DISTANCE', '2.0'))  # meters - max detection range
COLLISION_SPEED_REDUCTION = float(os.getenv('COLLISION_SPEED_REDUCTION', '0.5'))  # Speed reduction factor in warning zone
COLLISION_EDGE_THRESHOLD = int(os.getenv('COLLISION_EDGE_THRESHOLD', '5'))  # Minimum vertical edges to detect obstacle
COLLISION_DARK_REGION_THRESHOLD = float(os.getenv('COLLISION_DARK_REGION_THRESHOLD', '0.15'))  # Ratio of dark pixels to detect obstacle

