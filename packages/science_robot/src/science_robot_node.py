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
else:
    web_server_available = False

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
            self.gesture_detector = GestureDetector(
                min_detection_confidence=config.GESTURE_CONFIDENCE_THRESHOLD,
                min_tracking_confidence=config.GESTURE_CONFIDENCE_THRESHOLD
            )
            self.wave_detector = WaveDetector()
            self.motor_controller = MotorController()
            self.navigation = NavigationController()
            self.dance_controller = DanceController(self.motor_controller)
            self.treat_dispenser = TreatDispenser()
            
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
            if not self.camera.initialize():
                logger.error("Failed to initialize camera")
                return False
            
            if not self.treat_dispenser.initialize():
                logger.warning("Treat dispenser initialization failed")
            
            logger.info("All subsystems initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Initialization error: {e}")
            logger.error(traceback.format_exc())
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
                
                # Detect hands and gestures
                detection_start = time.time()
                hands_data, mp_results = self.gesture_detector.detect_hands(frame)
                detection_time = time.time() - detection_start
                
                # Update wave detector
                is_waving, wave_position = self.wave_detector.update(hands_data)
                
                # Handle state machine (updates self.state)
                self._update_state(is_waving, wave_position, hands_data, frame)
                
                # Draw overlay on frame for display/web
                display_frame = frame.copy()
                self._draw_overlay(display_frame, mp_results, is_waving, wave_position)
                
                # Update web server with overlay frame and status
                if config.ENABLE_WEB_SERVER and web_server_available:
                    # Get current gesture
                    current_gesture = None
                    if hands_data:
                        current_gesture = self.gesture_detector.classify_gesture(hands_data)
                    
                    # Calculate FPS
                    loop_time = time.time() - loop_start
                    current_fps = 1.0 / loop_time if loop_time > 0 else 0.0
                    
                    # Get motor speeds (if available)
                    motor_left = 0.0
                    motor_right = 0.0
                    # Note: Motor speeds would need to be tracked in motor_controller
                    
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
                        motor_speed_right=motor_right
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
    
    def _update_state(self, is_waving, wave_position, hands_data, frame):
        """Update robot state based on sensor input"""
        
        dance_gesture_detected = False
        treat_gesture_detected = False
        
        if hands_data:
            gesture = self.gesture_detector.classify_gesture(hands_data)
            
            if gesture == 'dance':
                if self.current_gesture == 'dance':
                    self.current_gesture_hold_time += (1.0 / config.MAIN_LOOP_FPS)
                    if self.current_gesture_hold_time >= config.DANCE_GESTURE_HOLD_TIME:
                        dance_gesture_detected = True
                else:
                    self.current_gesture = 'dance'
                    self.current_gesture_hold_time = 0
            elif gesture == 'treat':
                if self.current_gesture == 'treat':
                    self.current_gesture_hold_time += (1.0 / config.MAIN_LOOP_FPS)
                    if self.current_gesture_hold_time >= config.TREAT_GESTURE_HOLD_TIME:
                        treat_gesture_detected = True
                else:
                    self.current_gesture = 'treat'
                    self.current_gesture_hold_time = 0
            else:
                self.current_gesture = None
                self.current_gesture_hold_time = 0
        else:
            self.current_gesture = None
            self.current_gesture_hold_time = 0
        
        # State machine logic
        if self.state == 'dancing':
            if not self.dance_controller.is_dance_in_progress():
                self.state = 'idle'
        elif treat_gesture_detected:
            if config.LOG_GESTURES:
                logger.info("Secret treat gesture detected!")
            self.treat_dispenser.dispense_treat()
            self.current_gesture = None
            self.current_gesture_hold_time = 0
        elif dance_gesture_detected:
            self.state = 'dancing'
            self.motor_controller.stop()
            self.dance_controller.execute_dance()
            self.current_gesture = None
            self.current_gesture_hold_time = 0
        elif is_waving and wave_position:
            self.state = 'tracking'
            left_speed, right_speed = self.navigation.calculate_steering(wave_position)
            
            if self.navigation.should_stop(wave_position):
                self.motor_controller.stop()
                if self.frame_count % 30 == 0:
                    logger.debug(f"Target close, stopping. Position: {wave_position}")
            else:
                self.motor_controller.set_differential_speed(left_speed, right_speed)
                if self.frame_count % 30 == 0:
                    logger.debug(f"Tracking wave - steering: left={left_speed:.2f}, right={right_speed:.2f}, position={wave_position}")
        else:
            self.state = 'idle'
            self.motor_controller.stop()
    
    def _draw_overlay(self, frame, mp_results, is_waving, wave_position):
        """Draw overlay information on frame"""
        self.gesture_detector.draw_landmarks(frame, mp_results)
        
        height, width = frame.shape[:2]
        
        state_colors = {
            'idle': (128, 128, 128),
            'tracking': (0, 255, 0),
            'dancing': (255, 165, 0)
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
        
        if is_waving:
            cv2.putText(frame, "WAVING DETECTED!", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            if wave_position:
                x, y = wave_position
                pixel_x = int(x * width)
                pixel_y = int(y * height)
                cv2.circle(frame, (pixel_x, pixel_y), 20, (0, 255, 0), 3)
        
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
    # Initialize ROS node
    rospy.init_node('science_robot_controller', anonymous=True)
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and run robot controller
    robot = RobotController()
    
    if not robot.initialize():
        rospy.logerr("Failed to initialize robot")
        return 1
    
    try:
        robot.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupted")
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        logger.error(traceback.format_exc())
        rospy.logerr(f"Fatal error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

