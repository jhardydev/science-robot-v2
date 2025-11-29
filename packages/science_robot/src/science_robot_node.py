#!/usr/bin/env python3
"""
Duckiebot Science Fair Robot - Main ROS Node
Ubuntu 22.04 compatible with Catkin package structure
"""
# Suppress fontconfig warnings during all imports
import os
import sys
_fontconfig_fd = os.open('/dev/null', os.O_WRONLY)
_fontconfig_old_stderr = os.dup(2)
os.dup2(_fontconfig_fd, 2)
os.close(_fontconfig_fd)

import matplotlib
matplotlib.use('Agg')

os.environ['FONTCONFIG_FILE'] = os.environ.get('FONTCONFIG_FILE', '/etc/fonts/fonts.conf')
os.environ['FC_DEBUG'] = '0'

# Import from Catkin package
from science_robot import config

import rospy
import cv2
import numpy as np
import time
import signal
import logging
from datetime import datetime
import traceback
import threading
import queue
import select

# Import from Catkin package
from science_robot.camera import Camera
from science_robot.gesture_detector import GestureDetector
from science_robot.wave_detector import WaveDetector
from science_robot.motor_controller import MotorController
from science_robot.navigation import NavigationController
from science_robot.dance import DanceController
from science_robot.treat_dispenser import TreatDispenser

# Conditionally import encoder feedback
encoder_feedback_available = False
if config.ENCODER_ENABLED:
    try:
        from science_robot.encoder_reader import EncoderReader
        from science_robot.speed_controller import SpeedController
        encoder_feedback_available = True
    except ImportError as e:
        encoder_feedback_available = False

# Conditionally import IMU feedback
imu_feedback_available = False
if config.IMU_ENABLED:
    try:
        from science_robot.imu_reader import IMUReader
        imu_feedback_available = True
    except ImportError as e:
        imu_feedback_available = False

# Conditionally import collision avoidance
if config.ENABLE_COLLISION_AVOIDANCE:
    try:
        from science_robot.collision_avoidance import CollisionAvoidance
        collision_avoidance_available = True
    except ImportError as e:
        collision_avoidance_available = False
        print(f"Warning: Collision avoidance not available: {e}")
else:
    collision_avoidance_available = False

# Conditionally import VPI processor
if config.USE_VPI_ACCELERATION:
    try:
        from science_robot.vpi_processor import VPIProcessor
        vpi_processor = VPIProcessor(backend=config.VPI_BACKEND)
    except ImportError:
        vpi_processor = None
else:
    vpi_processor = None

# Conditionally import web server
web_server_available = False
if config.ENABLE_WEB_SERVER:
    try:
        from science_robot.web_server import (
            start_web_server, update_frame, update_status,
            set_initialization_status, set_robot_initialized
        )
        web_server_available = True
    except ImportError as e:
        web_server_available = False
        # Logger not initialized yet, use print
        print(f"Warning: Web server not available: {e}")

# Restore stderr
os.dup2(_fontconfig_old_stderr, 2)
os.close(_fontconfig_old_stderr)
del _fontconfig_fd, _fontconfig_old_stderr

# Setup logging
os.makedirs(config.LOG_DIR, exist_ok=True)
log_file = os.path.join(config.LOG_DIR, f'science_robot_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')

log_level = getattr(logging, config.LOG_LEVEL, logging.INFO)
handlers = [logging.StreamHandler(sys.stdout)]
if config.ENABLE_FILE_LOGGING:
    handlers.append(logging.FileHandler(log_file))

logging.basicConfig(
    level=log_level,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=handlers
)

logger = logging.getLogger(__name__)
logger.info(f"Logging initialized. Log file: {log_file}")
logger.info(f"Log level: {config.LOG_LEVEL}")

if config.USE_VPI_ACCELERATION and vpi_processor is None:
    logger.warning("VPI not available, continuing without GPU preprocessing")

# Log encoder feedback availability
if config.ENCODER_ENABLED:
    if encoder_feedback_available:
        logger.info("Encoder feedback modules: AVAILABLE")
    else:
        logger.warning("Encoder feedback requested but modules not available - will use open-loop control")
else:
    logger.info("Encoder feedback: DISABLED")

# Log IMU feedback availability
if config.IMU_ENABLED:
    if imu_feedback_available:
        logger.info("IMU validation modules: AVAILABLE")
    else:
        logger.warning("IMU validation requested but modules not available")
else:
    logger.info("IMU validation: DISABLED")


class RobotController:
    """Main robot control system"""
    
    def __init__(self):
        """Initialize all robot subsystems"""
        logger.info("Initializing Duckiebot Science Fair Robot v2.0 (Ubuntu 22.04 + Catkin)...")
        if config.USE_CUDA_ACCELERATION or config.USE_VPI_ACCELERATION:
            logger.info("NVIDIA acceleration: ENABLED")
        else:
            logger.info("NVIDIA acceleration: DISABLED")
        
        try:
            # Initialize components
            self.camera = Camera()
            # Use new config parameters for better distance detection (defaults to lower confidence thresholds)
            self.gesture_detector = GestureDetector(
                min_detection_confidence=getattr(config, 'MEDIAPIPE_HAND_DETECTION_CONFIDENCE', None),
                min_tracking_confidence=getattr(config, 'MEDIAPIPE_HAND_TRACKING_CONFIDENCE', None),
                model_complexity=config.MEDIAPIPE_MODEL_COMPLEXITY
            )
            self.wave_detector = WaveDetector(gesture_detector=self.gesture_detector)
            self.motor_controller = MotorController()
            # Initialize encoder feedback if enabled
            encoder_reader = None
            speed_controller = None
            if config.ENCODER_ENABLED and encoder_feedback_available:
                try:
                    encoder_reader = EncoderReader(
                        robot_name=config.ROBOT_NAME,
                        encoder_ppr=config.ENCODER_PPR,
                        wheel_diameter=config.WHEEL_DIAMETER
                    )
                    speed_controller = SpeedController(
                        kp=config.SPEED_CONTROLLER_KP,
                        ki=config.SPEED_CONTROLLER_KI,
                        kd=config.SPEED_CONTROLLER_KD,
                        max_integral=config.SPEED_CONTROLLER_MAX_INTEGRAL
                    )
                    logger.info("Encoder feedback initialized successfully")
                except Exception as e:
                    logger.warning(f"Failed to initialize encoder feedback: {e}")
                    logger.warning("Continuing with open-loop control")
                    encoder_reader = None
                    speed_controller = None
            else:
                if config.ENCODER_ENABLED:
                    logger.warning("Encoder feedback requested but modules not available")
            
            # Initialize IMU reader if enabled
            imu_reader = None
            if config.IMU_ENABLED and imu_feedback_available:
                try:
                    imu_reader = IMUReader(
                        robot_name=config.ROBOT_NAME,
                        spinning_threshold=config.IMU_SPINNING_THRESHOLD,
                        dangerous_spin_threshold=config.IMU_DANGEROUS_SPIN_THRESHOLD,
                        wheel_base=config.WHEEL_BASE
                    )
                    logger.info("IMU reader initialized successfully")
                except Exception as e:
                    logger.warning(f"Failed to initialize IMU reader: {e}")
                    imu_reader = None
            else:
                if config.IMU_ENABLED:
                    logger.warning("IMU validation requested but modules not available")
            
            self.navigation = NavigationController(
                encoder_reader=encoder_reader,
                speed_controller=speed_controller,
                imu_reader=imu_reader
            )
            self.dance_controller = DanceController(self.motor_controller)
            self.treat_dispenser = TreatDispenser()
            
            # Initialize collision avoidance if enabled
            if collision_avoidance_available:
                self.collision_avoidance = CollisionAvoidance()
                logger.info("Collision avoidance: ENABLED")
            else:
                self.collision_avoidance = None
                logger.info("Collision avoidance: DISABLED")
            
            self.vpi_processor = vpi_processor
            
            # State management
            self.running = False
            self.state = 'idle'
            self.frame_count = 0
            
            # Performance monitoring
            self.frame_times = []
            self.processing_times = []
            
            # Gesture tracking
            self.last_dance_gesture_time = 0
            self.last_treat_gesture_time = 0
            self.current_gesture_hold_time = 0
            self.current_gesture = None
            
            # Tracking state management
            self.last_wave_time = 0
            self.last_wave_position = None
            self.tracking_timeout = config.TRACKING_TIMEOUT
            
            # Manual target tracking (secret feature - click to track)
            self.manual_target_position = None
            self.manual_target_time = 0
            self.manual_target_timeout = 5.0  # Track manual target for 5 seconds
            
            # Face tracking data
            self.current_faces_data = []
            self.current_face_position = None
            
            # Display window state
            self.display_window_created = False
            self.display_init_retries = 0
            
            # Keyboard input handling
            self.input_queue = queue.Queue()
            self.input_thread = None
            self.original_terminal_settings = None
            
            logger.info("Robot initialized successfully!")
            if self.camera.has_cuda():
                logger.info("  - CUDA acceleration: Available")
            if self.vpi_processor and self.vpi_processor.is_available():
                logger.info("  - VPI acceleration: Available")
            
            # Mark robot as initialized in web server
            if config.ENABLE_WEB_SERVER and web_server_available:
                set_robot_initialized()
                logger.info(f"  - Web server: Enabled on port {config.WEB_SERVER_PORT}")
            
        except Exception as e:
            logger.error(f"Failed to initialize robot: {e}")
            logger.error(traceback.format_exc())
            raise
    
    def initialize(self):
        """Initialize camera and hardware"""
        try:
            if config.ENABLE_WEB_SERVER and web_server_available:
                set_initialization_status("Initializing camera vision system...")
            
            if not self.camera.initialize():
                logger.error("Failed to initialize camera")
                if config.ENABLE_WEB_SERVER and web_server_available:
                    set_initialization_status("Camera initialization failed", error=True)
                return False
            
            if config.ENABLE_WEB_SERVER and web_server_available:
                set_initialization_status("Preparing treat dispenser...")
            
            if not self.treat_dispenser.initialize():
                logger.warning("Treat dispenser initialization failed")
            
            if config.ENABLE_WEB_SERVER and web_server_available:
                set_initialization_status("Robot ready! 🎉", success=True)
            
            logger.info("All subsystems initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Initialization error: {e}")
            logger.error(traceback.format_exc())
            if config.ENABLE_WEB_SERVER and web_server_available:
                set_initialization_status(f"Initialization error: {e}")
            return False
    
    def _preprocess_frame(self, frame):
        """Preprocess frame using GPU if available"""
        if config.GPU_PREPROCESSING and self.vpi_processor and self.vpi_processor.is_available():
            pass  # Preprocessing can be added here if needed
        return frame
    
    def _input_thread_func(self):
        """Thread function to read keyboard input from stdin"""
        try:
            if not sys.stdin.isatty():
                return
            
            while self.running:
                try:
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        char = sys.stdin.read(1)
                        if char:
                            self.input_queue.put(char.lower())
                except (IOError, OSError):
                    break
                except Exception as e:
                    logger.debug(f"Input thread error: {e}")
                    break
        except Exception as e:
            logger.debug(f"Input thread exception: {e}")
    
    def _start_input_thread(self):
        """Start the keyboard input thread"""
        try:
            if sys.stdin.isatty():
                self.input_thread = threading.Thread(target=self._input_thread_func, daemon=True)
                self.input_thread.start()
                logger.debug("Keyboard input thread started")
        except Exception as e:
            logger.debug(f"Could not start input thread: {e}")
    
    def _stop_input_thread(self):
        """Stop the keyboard input thread"""
        if self.input_thread and self.input_thread.is_alive():
            self.running = False
            self.input_thread.join(timeout=0.5)
    
    def _check_keyboard_input(self):
        """Check for keyboard input from stdin (non-blocking)"""
        try:
            while not self.input_queue.empty():
                char = self.input_queue.get_nowait()
                if char == 'q':
                    logger.info("Quit requested via keyboard (stdin)")
                    self.running = False
                    return 'quit'
                elif char == 's':
                    logger.warning("Emergency stop requested via keyboard (stdin)!")
                    self.motor_controller.emergency_stop()
                    self.state = 'idle'
                    return 'stop'
        except queue.Empty:
            pass
        except Exception as e:
            logger.debug(f"Error checking keyboard input: {e}")
        return None
    
    def run(self):
        """Main control loop"""
        self.running = True
        frame_time_target = 1.0 / config.MAIN_LOOP_FPS
        
        logger.info("Starting main control loop...")
        if config.DISPLAY_OUTPUT:
            logger.info("Display output: ENABLED - Press 'q' to quit, 's' to emergency stop")
        else:
            logger.info("Display output: DISABLED - Running in headless mode")
            logger.info("Press 'q' to quit, 's' to emergency stop (via terminal)")
        
        self._start_input_thread()
        
        try:
            while self.running:
                input_result = self._check_keyboard_input()
                if input_result == 'quit':
                    break
                loop_start = time.time()
                
                # Capture frame
                success, frame = self.camera.read_frame()
                if not success:
                    logger.warning("Failed to read frame")
                    time.sleep(0.1)
                    continue
                
                self.frame_count += 1
                
                # GPU preprocessing (if enabled)
                if config.GPU_PREPROCESSING:
                    frame = self._preprocess_frame(frame)
                
                # Detect faces first (for hand validation context), then hands with face data for filtering
                detection_start = time.time()
                faces_data, face_results = self.gesture_detector.detect_faces(frame)
                hands_data, mp_results = self.gesture_detector.detect_hands(frame, faces_data=faces_data)
                detection_time = time.time() - detection_start
                
                # Debug: Log face detection
                if self.frame_count % 60 == 0:
                    if faces_data:
                        logger.info(f"Face detection: Found {len(faces_data)} face(s) at {[f['center'] for f in faces_data]}")
                    else:
                        logger.debug("Face detection: No faces found in frame")
                
                # Update wave detector with both hand and face data
                # Returns: (is_waving, target_position, face_position)
                # Note: In 'gesture' mode, is_waving should always be False - only thumbs_up triggers tracking
                is_waving, target_position, face_position = self.wave_detector.update(hands_data, faces_data)
                
                # In gesture mode, ignore is_waving and only use thumbs_up_detected
                # This prevents any wave motion from triggering tracking
                if config.GESTURE_DETECTION_MODE == 'gesture':
                    is_waving = False  # Disable wave triggering in gesture mode
                    # Even if thumbs_up_detected is False, preserve face_position if it exists
                    # This allows face lock to persist even during intermittent gesture detection
                    if not self.wave_detector.thumbs_up_detected:
                        # If we have a face lock, preserve it for tracking continuity
                        # Otherwise clear target_position
                        if face_position:
                            # Preserve face position for tracking even if thumbs_up not fully detected yet
                            target_position = face_position
                        else:
                            target_position = None  # No trigger if thumbs up not detected and no face
                
                # Use target_position (which is face if available, else hand) as wave_position for compatibility
                wave_position = target_position
                
                # Store face data for overlay drawing
                # Always preserve face_position from wave_detector for tracking continuity
                self.current_faces_data = faces_data
                self.current_face_position = face_position if face_position else self.current_face_position
                
                # Check for collision risk
                collision_risk = None
                if self.collision_avoidance:
                    collision_risk = self.collision_avoidance.check_collision_risk(frame)
                    
                    # Emergency stop if collision risk is critical
                    if collision_risk['should_stop']:
                        logger.warning(f"EMERGENCY STOP: Collision detected at {collision_risk['distance']:.2f}m")
                        self.motor_controller.emergency_stop()
                        self.state = 'EMERGENCY_STOP'
                    elif self.motor_controller.is_emergency_stop_active():
                        # Clear emergency stop if collision risk is gone
                        if collision_risk['risk_level'] == 'none' or collision_risk['distance'] is None:
                            logger.info("Collision risk cleared - clearing emergency stop")
                            self.motor_controller.clear_emergency_stop()
                            self.state = 'idle'
                        elif collision_risk['risk_level'] == 'warning':
                            # Still in warning zone, but not emergency - clear stop but slow down
                            logger.info("Collision risk reduced to warning - clearing emergency stop")
                            self.motor_controller.clear_emergency_stop()
                            self.state = 'idle'
                
                # Skip state machine update if emergency stop is still active
                if self.motor_controller.is_emergency_stop_active():
                    # Still draw overlay and update web server
                    display_frame = frame.copy()
                    # In gesture mode, never show "WAVING" label
                    display_is_waving = is_waving if config.GESTURE_DETECTION_MODE != 'gesture' else False
                    self._draw_overlay(display_frame, mp_results, display_is_waving, wave_position, 
                                     hands_data=hands_data, faces_data=faces_data, face_position=face_position)
                    if self.collision_avoidance and collision_risk:
                        self.collision_avoidance.draw_overlay(display_frame, collision_risk)
                    
                    if config.ENABLE_WEB_SERVER and web_server_available:
                        update_frame(display_frame)
                        update_status(
                            state='EMERGENCY_STOP',
                            fps=1.0 / (time.time() - loop_start) if (time.time() - loop_start) > 0 else 0.0,
                            frame_count=self.frame_count,
                            is_waving=is_waving,
                            gesture=None,
                            wave_position=wave_position,
                            motor_speed_left=0.0,
                            motor_speed_right=0.0,
                            collision_risk=collision_risk
                        )
                    
                    if config.DISPLAY_OUTPUT:
                        cv2.imshow('Duckiebot Science Fair Robot v2.0', display_frame)
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q'):
                            break
                    
                    time.sleep(0.1)
                    continue
                
                # Handle state machine (updates self.state)
                self._update_state(is_waving, wave_position, hands_data, frame, collision_risk)
                
                # Draw overlay on frame for display/web
                display_frame = frame.copy()
                # In gesture mode, never show "WAVING" label - only show thumbs up
                display_is_waving = is_waving if config.GESTURE_DETECTION_MODE != 'gesture' else False
                self._draw_overlay(display_frame, mp_results, display_is_waving, wave_position, 
                                 hands_data=hands_data, faces_data=faces_data, face_position=face_position)
                
                # Draw collision avoidance overlay if enabled
                if self.collision_avoidance and collision_risk:
                    self.collision_avoidance.draw_overlay(display_frame, collision_risk)
                
                # Update web server with overlay frame and status
                if config.ENABLE_WEB_SERVER and web_server_available:
                    # Get current gesture (using new signature)
                    current_gesture = None
                    if hands_data or frame is not None:
                        gesture_result = self.gesture_detector.classify_gesture(
                            hands_data if hands_data else [], 
                            frame=frame, 
                            faces_data=faces_data
                        )
                        current_gesture = gesture_result[0] if gesture_result[0] else None
                    
                    # Calculate FPS
                    loop_time = time.time() - loop_start
                    current_fps = 1.0 / loop_time if loop_time > 0 else 0.0
                    
                    # Get motor speeds
                    motor_left, motor_right = self.motor_controller.get_last_speeds()
                    
                    # Update web server with frame that has overlay drawn
                    update_frame(display_frame)
                    update_status(
                        state=self.state,
                        fps=current_fps,
                        frame_count=self.frame_count,
                        is_waving=is_waving,
                        gesture=current_gesture,
                        wave_position=wave_position,
                        motor_speed_left=motor_left,
                        motor_speed_right=motor_right,
                        collision_risk=collision_risk
                    )
                
                # Performance monitoring
                if config.ENABLE_PERFORMANCE_MONITORING:
                    loop_time = time.time() - loop_start
                    self.frame_times.append(loop_time)
                    self.processing_times.append(detection_time)
                    
                    if self.frame_count % 30 == 0:
                        avg_frame_time = sum(self.frame_times[-30:]) / len(self.frame_times[-30:])
                        avg_detection_time = sum(self.processing_times[-30:]) / len(self.processing_times[-30:])
                        logger.debug(f"Performance: {1.0/avg_frame_time:.1f} FPS, Detection: {avg_detection_time*1000:.1f}ms")
                
                # Display output if enabled (use display_frame with overlay already drawn)
                if config.DISPLAY_OUTPUT:
                    try:
                        if not self.display_window_created:
                            if self.frame_count == 1:
                                logger.debug("Waiting for X11 display to initialize...")
                                time.sleep(0.3)
                            
                            try:
                                cv2.imshow('Duckiebot Science Fair Robot v2.0', display_frame)
                                test_key = cv2.waitKey(1)
                                if test_key != -1 or cv2.getWindowProperty('Duckiebot Science Fair Robot v2.0', cv2.WND_PROP_VISIBLE) >= 0:
                                    self.display_window_created = True
                                    logger.debug("Display window created successfully")
                                else:
                                    if self.display_init_retries < 10:
                                        self.display_init_retries += 1
                                        logger.debug(f"Window not ready yet, retry {self.display_init_retries}/10")
                            except Exception as window_error:
                                if self.display_init_retries < 10:
                                    self.display_init_retries += 1
                                    logger.debug(f"Window creation failed (retry {self.display_init_retries}/10): {window_error}")
                                    time.sleep(0.1)
                                else:
                                    raise
                        else:
                            cv2.imshow('Duckiebot Science Fair Robot v2.0', display_frame)
                        
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q'):
                            logger.info("Quit requested via keyboard (OpenCV window)")
                            break
                        elif key == ord('s'):
                            logger.warning("Emergency stop requested via keyboard (OpenCV window)!")
                            self.motor_controller.emergency_stop()
                            self.state = 'idle'
                    except Exception as e:
                        logger.warning(f"Display output failed: {e}, disabling display")
                        logger.debug(traceback.format_exc())
                        config.DISPLAY_OUTPUT = False
                        self.display_window_created = False
                
                # Maintain target FPS
                elapsed = time.time() - loop_start
                if elapsed < frame_time_target:
                    time.sleep(frame_time_target - elapsed)
                    
        except KeyboardInterrupt:
            logger.info("Interrupted by user (Ctrl+C)")
        except rospy.ROSInterruptException:
            logger.info("ROS interrupted")
        except Exception as e:
            logger.error(f"Error in main loop: {e}")
            logger.error(traceback.format_exc())
        finally:
            self._stop_input_thread()
            self.shutdown()
    
    def _update_state(self, is_waving, wave_position, hands_data, frame, collision_risk=None):
        """Update robot state based on sensor input and collision avoidance"""
        
        # Get current gesture using Gesture Recognizer or custom detection
        # Returns: (gesture_type, hand_position, associated_face)
        current_gesture = None
        gesture_hand_position = None
        gesture_associated_face = None
        
        # Get faces_data for gesture classification
        faces_data = self.current_faces_data if hasattr(self, 'current_faces_data') else None
        
        if hands_data or frame is not None:
            current_gesture, gesture_hand_position, gesture_associated_face = self.gesture_detector.classify_gesture(
                hands_data if hands_data else [], frame=frame, faces_data=faces_data
            )
        
        # Store current gesture for display/logging
        self.current_gesture = current_gesture
        
        # Stop gesture handling - highest priority, immediate action
        if current_gesture == 'stop':
            if config.LOG_GESTURES:
                logger.info("Stop gesture detected - immediately stopping robot")
            self.motor_controller.stop()
            self.motor_controller.emergency_stop()  # Use emergency stop for immediate halt
            self.state = 'idle'
            self.navigation.reset_smoothing()
            self.current_gesture = None
            self.current_gesture_hold_time = 0
            return  # Exit early to prevent any further tracking
        
        # Treat gesture removed - no longer supported
        # Only thumbs-up and stop gestures are supported
        
        # Reset gesture hold time (not used for thumbs-up/stop, but maintain for display)
        self.current_gesture_hold_time = 0
        # Dance gesture detection disabled
        # elif dance_gesture_detected:
        #     self.state = 'dancing'
        #     self.motor_controller.stop()
        #     self.dance_controller.execute_dance()
        #     self.current_gesture = None
        #     self.current_gesture_hold_time = 0
        # Update tracking state - continue tracking even if wave briefly stops
        # Manual target takes priority over wave detection (secret feature)
        current_time = time.time()
        target_position = None
        tracking_source = None
        
        # Check for manual target (secret click-to-track feature)
        if self.manual_target_position and (current_time - self.manual_target_time) < self.manual_target_timeout:
            target_position = self.manual_target_position
            tracking_source = 'manual'
            self.state = 'tracking'
            if self.frame_count % 30 == 0:
                logger.debug(f"Tracking manual target: {target_position}")
        elif (is_waving or self.wave_detector.thumbs_up_detected or current_gesture == 'thumbs_up') and (wave_position or self.current_face_position):
            # Trigger detected (thumbs up in gesture mode, or wave in wave/both mode)
            # Prioritize associated face from gesture classification if available
            self.last_wave_time = current_time
            
            # Use associated face from gesture classification if thumbs-up detected
            if current_gesture == 'thumbs_up' and gesture_associated_face:
                # Lock onto the closest face to the thumbs-up gesture
                target_position = gesture_associated_face['center']
                self.current_face_position = target_position
                tracking_source = 'thumbs_up_face'
                logger.info(f"TRACKING STARTED: Thumbs-up with associated face at {target_position}")
            elif current_gesture == 'thumbs_up' and gesture_hand_position:
                # Thumbs-up detected but no face - track hand position
                target_position = gesture_hand_position
                self.current_face_position = None
                tracking_source = 'thumbs_up'
                logger.info(f"TRACKING STARTED: Thumbs-up hand at {target_position} (no face associated)")
            elif self.current_face_position:
                # Use existing face position (established by wave_detector)
                target_position = self.current_face_position
                tracking_source = 'face'
                if self.frame_count % 30 == 0:
                    logger.info(f"TRACKING: Using locked face at {target_position} (thumbs_up_detected={self.wave_detector.thumbs_up_detected}, current_gesture={current_gesture})")
            elif wave_position:
                # Fallback to wave_position (hand position)
                target_position = wave_position
                tracking_source = 'wave'
                logger.info(f"TRACKING STARTED: {tracking_source} at {target_position}")
            else:
                # This shouldn't happen but log it
                logger.warning(f"Tracking condition met but no target! wave_position={wave_position}, current_face_position={self.current_face_position}")
                target_position = None
            
            self.last_wave_position = target_position
            self.state = 'tracking'
            if self.frame_count % 30 == 0:
                if self.current_face_position:
                    logger.debug(f"Tracking FACE at {target_position} (trigger: {tracking_source})")
                else:
                    logger.debug(f"Tracking HAND at {target_position} (trigger: {tracking_source}, no face associated)")
        elif self.state == 'tracking' and (current_time - self.last_wave_time) < self.tracking_timeout:
            # Continue tracking for a short time after wave stops (smooth tracking)
            # Prioritize locked face if available, otherwise use last known position
            if self.current_face_position:
                # We have a locked face - use it for tracking
                target_position = self.current_face_position
                tracking_source = 'face'
                if self.frame_count % 30 == 0:
                    logger.debug(f"Continuing tracking with LOCKED FACE at {target_position} (timeout: {self.tracking_timeout - (current_time - self.last_wave_time):.1f}s)")
            else:
                # No face lock, use last known position
                target_position = self.last_wave_position
                tracking_source = 'wave'
                if self.frame_count % 30 == 0:
                    logger.debug(f"Continuing tracking after wave stopped (timeout: {self.tracking_timeout - (current_time - self.last_wave_time):.1f}s)")
        else:
            # No wave and tracking timeout expired, and no manual target
            if self.state == 'tracking':
                logger.info("Tracking timeout - returning to idle")
                self.navigation.reset_smoothing()
                self.manual_target_position = None  # Clear manual target
            self.state = 'idle'
            self.motor_controller.stop()
            return
        
        # Execute tracking behavior
        if target_position:
            left_speed, right_speed = self.navigation.calculate_steering(target_position)
            
            # Apply collision avoidance speed reduction if needed
            if collision_risk and self.collision_avoidance:
                if collision_risk['should_slow']:
                    # Reduce speed based on collision risk
                    safe_left = self.collision_avoidance.get_safe_speed(left_speed, collision_risk)
                    safe_right = self.collision_avoidance.get_safe_speed(right_speed, collision_risk)
                    left_speed = safe_left
                    right_speed = safe_right
                    if self.frame_count % 30 == 0:
                        logger.debug(f"Collision warning: Reduced speed to {left_speed:.2f}, {right_speed:.2f}")
            
            should_stop = self.navigation.should_stop(target_position)
            if should_stop:
                self.motor_controller.stop()
                if self.frame_count % 30 == 0:
                    logger.info(f"Target close, stopping. Position: {target_position} (source: {tracking_source})")
            else:
                # Always log motor commands when tracking (throttled)
                self.motor_controller.set_differential_speed(left_speed, right_speed)
                if self.frame_count % 30 == 0:
                    logger.info(f"MOVING: Tracking {tracking_source} - steering: left={left_speed:.3f}, right={right_speed:.3f}, position={target_position}")
        else:
            # No target position - this shouldn't happen but log it with more detail
            logger.warning(f"Tracking state but no target_position! state={self.state}, thumbs_up_detected={self.wave_detector.thumbs_up_detected}, "
                          f"current_gesture={current_gesture}, wave_position={wave_position}, current_face_position={self.current_face_position}")
    
    def _draw_overlay(self, frame, mp_results, is_waving, wave_position, hands_data=None, 
                     faces_data=None, face_position=None):
        """Draw overlay information on frame"""
        # Get current gesture for labeling (using new signature)
        current_gesture = None
        if hands_data or frame is not None:
            gesture_result = self.gesture_detector.classify_gesture(
                hands_data if hands_data else [], 
                frame=frame, 
                faces_data=faces_data
            )
            current_gesture = gesture_result[0] if gesture_result[0] else None
        
        # Draw hand landmarks and bounding boxes
        # Pass faces_data so gesture classification can associate gestures with faces
        self.gesture_detector.draw_landmarks(
            frame, mp_results, 
            hands_data=hands_data, 
            draw_bbox=True,
            is_waving=is_waving,
            current_gesture=current_gesture,
            faces_data=faces_data
        )
        
        # Draw face bounding boxes
        if faces_data:
            self.gesture_detector.draw_faces(frame, faces_data, target_face_center=face_position)
        elif self.frame_count % 60 == 0:
            # Debug: Log when no faces detected
            import logging
            logger = logging.getLogger(__name__)
            logger.debug("No faces detected in frame (faces_data is empty or None)")
        
        height, width = frame.shape[:2]
        
        state_colors = {
            'idle': (128, 128, 128),
            'tracking': (0, 255, 0),
            'EMERGENCY_STOP': (0, 0, 255)  # Red for emergency stop
            # 'dancing': (255, 165, 0)  # Dance disabled
        }
        state_color = state_colors.get(self.state, (255, 255, 255))
        cv2.putText(frame, f"State: {self.state.upper()}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
        
        if self.camera.has_cuda():
            cv2.putText(frame, "CUDA", (width - 200, height - 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        if self.vpi_processor and self.vpi_processor.is_available():
            cv2.putText(frame, "VPI", (width - 200, height - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Show tracking status (thumbs up trigger or manual target)
        tracking_position = self.manual_target_position if self.manual_target_position else wave_position
        is_triggered = is_waving or (hasattr(self, 'wave_detector') and self.wave_detector.thumbs_up_detected)
        if is_triggered or (self.state == 'tracking' and tracking_position):
            if self.manual_target_position:
                status_text = "TRACKING (MANUAL)"
                status_color = (255, 255, 0)  # Yellow for manual tracking
            else:
                # Check if thumbs up triggered (in gesture mode) or wave (in wave mode)
                is_thumbs_up_trigger = hasattr(self, 'wave_detector') and self.wave_detector.thumbs_up_detected
                if config.GESTURE_DETECTION_MODE == 'gesture' and is_thumbs_up_trigger:
                    # Thumbs up mode - show thumbs up status
                    if face_position:
                        status_text = "THUMBS UP - TRACKING FACE!" if is_triggered else "THUMBS UP - TRACKING FACE..."
                        status_color = (0, 255, 255)  # Cyan for face tracking
                    else:
                        status_text = "THUMBS UP DETECTED!" if is_triggered else "TRACKING..."
                        status_color = (0, 255, 0)  # Green for hand tracking
                else:
                    # Wave mode - show waving status
                    if face_position:
                        status_text = "TRACKING FACE!" if is_waving else "TRACKING FACE..."
                        status_color = (0, 255, 255)  # Cyan for face tracking
                    else:
                        status_text = "WAVING DETECTED!" if is_waving else "TRACKING..."
                        status_color = (0, 255, 0)  # Green for hand tracking
            cv2.putText(frame, status_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            if tracking_position:
                x, y = tracking_position
                pixel_x = int(x * width)
                pixel_y = int(y * height)
                
                # Draw target circle (different color for manual vs wave)
                if self.manual_target_position:
                    circle_color = (255, 255, 0)  # Yellow for manual target
                    circle_thickness = 4
                else:
                    circle_color = (0, 255, 0)  # Green for wave target
                    circle_thickness = 3
                cv2.circle(frame, (pixel_x, pixel_y), 20, circle_color, circle_thickness)
                
                # Draw direction arrow from center to target
                center_x = width // 2
                center_y = height // 2
                arrow_length = min(100, int(np.sqrt((pixel_x - center_x)**2 + (pixel_y - center_y)**2) * 0.5))
                if arrow_length > 10:
                    angle = np.arctan2(pixel_y - center_y, pixel_x - center_x)
                    arrow_end_x = int(center_x + arrow_length * np.cos(angle))
                    arrow_end_y = int(center_y + arrow_length * np.sin(angle))
                    arrow_color = (255, 255, 0) if self.manual_target_position else (0, 255, 255)
                    cv2.arrowedLine(frame, (center_x, center_y), (arrow_end_x, arrow_end_y),
                                  arrow_color, 3, tipLength=0.3)
                
                # Draw distance indicator (using navigation's distance estimate)
                # Note: This is frame-based distance (y-coordinate), not actual physical distance
                # Higher y = closer to bottom of frame = appears closer visually
                distance_estimate = self.navigation.get_distance_estimate(tracking_position)
                if distance_estimate is not None:
                    # Convert to percentage for display (0% = far/top, 100% = close/bottom)
                    distance_pct = int(distance_estimate * 100)
                    distance_text = f"Frame Pos: {distance_pct}%"
                else:
                    distance_text = "Frame Pos: N/A"
                cv2.putText(frame, distance_text, (pixel_x - 50, pixel_y - 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        if self.current_gesture:
            gesture_text = f"Gesture: {self.current_gesture.upper()} ({self.current_gesture_hold_time:.1f}s)"
            cv2.putText(frame, gesture_text, (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        cv2.putText(frame, f"Frame: {self.frame_count}", (width - 150, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        if not config.DISPLAY_OUTPUT:
            cv2.putText(frame, "Press 'q' to quit, 's' to stop", (10, height - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def shutdown(self):
        """Gracefully shutdown robot"""
        logger.info("Shutting down robot...")
        self.running = False
        try:
            self.motor_controller.stop()
            time.sleep(0.2)
            self.motor_controller.cleanup()
            self.camera.release()
            self.gesture_detector.close()
            self.treat_dispenser.cleanup()
            if self.collision_avoidance:
                self.collision_avoidance.cleanup()
            if config.DISPLAY_OUTPUT:
                cv2.destroyAllWindows()
            logger.info("Shutdown complete")
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
            logger.error(traceback.format_exc())


def signal_handler(sig, frame):
    """Handle interrupt signals"""
    logger.info("Received interrupt signal")
    rospy.signal_shutdown("Interrupted by user")
    sys.exit(0)


def main():
    """Main entry point"""
    # Start web server FIRST (before anything else) to show loading page immediately
    if config.ENABLE_WEB_SERVER and web_server_available:
        logger.info("Starting web server...")
        set_initialization_status("Starting web server...")
        # Start with None, will update with robot controller later
        start_web_server(None, port=config.WEB_SERVER_PORT, host=config.WEB_SERVER_HOST)
        # Give server a moment to start and be accessible
        time.sleep(1.5)
        set_initialization_status("Connecting to robot brain (ROS)...")
        logger.info(f"Web server started on http://{config.WEB_SERVER_HOST}:{config.WEB_SERVER_PORT}")
        logger.info("Loading page is now visible - robot is initializing...")
    
    # Initialize ROS node
    if config.ENABLE_WEB_SERVER and web_server_available:
        set_initialization_status("Initializing robot systems...")
    rospy.init_node('science_robot_controller', anonymous=True)
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and run robot controller
    if config.ENABLE_WEB_SERVER and web_server_available:
        set_initialization_status("Loading AI brain and sensors...")
    robot = RobotController()
    
    # Update web server with robot controller reference
    if config.ENABLE_WEB_SERVER and web_server_available:
        from science_robot.web_server import set_robot_controller
        set_robot_controller(robot)
        logger.info("Robot controller linked to web server")
    
    if not robot.initialize():
        rospy.logerr("Failed to initialize robot")
        if config.ENABLE_WEB_SERVER and web_server_available:
            set_initialization_status("Robot initialization FAILED", error=True)
        return 1
    
    try:
        robot.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupted")
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        logger.error(traceback.format_exc())
        rospy.logerr(f"Fatal error: {e}")
        if config.ENABLE_WEB_SERVER and web_server_available:
            set_initialization_status("Robot offline", error=True)
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

