"""
Camera module for Duckiebot using ROS
Subscribes to Duckiebot's camera topic
"""
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from science_robot import config
import threading


class Camera:
    """Camera interface subscribing to ROS camera topic"""
    
    def __init__(self, camera_index=None, width=None, height=None, fps=None):
        """
        Initialize camera subscriber
        
        Args:
            camera_index: Not used (for compatibility with old interface)
            width: Desired width for resizing (default from config)
            height: Desired height for resizing (default from config)
            fps: Not used (for compatibility with old interface)
        """
        # Capture resolution (what camera provides)
        self.capture_width = width or config.CAMERA_WIDTH
        self.capture_height = height or config.CAMERA_HEIGHT
        self.fps = fps or config.CAMERA_FPS
        
        # Processing resolution (what we process at, can be lower for performance)
        # If PROCESSING_WIDTH/HEIGHT not set, use capture resolution
        self.width = config.PROCESSING_WIDTH if hasattr(config, 'PROCESSING_WIDTH') and config.PROCESSING_WIDTH else self.capture_width
        self.height = config.PROCESSING_HEIGHT if hasattr(config, 'PROCESSING_HEIGHT') and config.PROCESSING_HEIGHT else self.capture_height
        
        # ROS setup
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.frame_received = False
        
        # Subscribe to camera topic
        self.image_sub = rospy.Subscriber(
            config.CAMERA_TOPIC,
            CompressedImage,
            self._image_callback,
            queue_size=1
        )
        
        # NVIDIA GPU acceleration (still supported for processing)
        self.cuda_available = False
        self.use_cuda = config.USE_CUDA_ACCELERATION
        
        # Initialize CUDA if available
        if self.use_cuda:
            self._check_cuda_support()
        
        # VPI processor for GPU-accelerated image processing
        # Initialize here (not at module level) so PYTHONPATH is set by entrypoint
        self.vpi_processor = None
        if config.USE_VPI_ACCELERATION:
            try:
                from science_robot.vpi_processor import VPIProcessor
                self.vpi_processor = VPIProcessor(backend=config.VPI_BACKEND)
            except (ImportError, Exception) as e:
                rospy.logdebug(f"VPI processor not available: {e}")
                self.vpi_processor = None
        
        if self.vpi_processor and self.vpi_processor.is_available():
            rospy.loginfo(f"Camera subscriber initialized (ROS topic: {config.CAMERA_TOPIC}) - VPI acceleration available")
        else:
            rospy.loginfo(f"Camera subscriber initialized (ROS topic: {config.CAMERA_TOPIC})")
    
    def _image_callback(self, msg):
        """Callback for receiving camera images"""
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                # Store original capture resolution for reference
                original_height, original_width = frame.shape[:2]
                
                # Resize to processing resolution if different from capture resolution
                # This allows capturing at high resolution (e.g., 1920x1080) but processing
                # at a lower resolution (e.g., 1280x720) for better performance
                if frame.shape[1] != self.width or frame.shape[0] != self.height:
                    if self.vpi_processor and self.vpi_processor.is_available():
                        try:
                            frame = self.vpi_processor.resize_gpu(frame, self.width, self.height)
                        except Exception as e:
                            # Fall back to CPU resize if VPI fails
                            rospy.logdebug(f"VPI resize failed, using CPU: {e}")
                            frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)
                    else:
                        # Use INTER_AREA for downscaling (better quality than default)
                        frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)
                
                with self.frame_lock:
                    self.latest_frame = frame
                    self.frame_received = True
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Error decoding image: {e}")
    
    def _check_cuda_support(self):
        """Check if OpenCV has CUDA support"""
        try:
            if hasattr(cv2, 'cuda') and cv2.cuda.getCudaEnabledDeviceCount() > 0:
                self.cuda_available = True
                rospy.loginfo(f"CUDA acceleration enabled ({cv2.cuda.getCudaEnabledDeviceCount()} device(s))")
            else:
                self.cuda_available = False
                rospy.loginfo("CUDA acceleration requested but not available")
        except Exception as e:
            self.cuda_available = False
            rospy.logwarn(f"CUDA check failed: {e}")
    
    def initialize(self):
        """Initialize camera (wait for first frame)"""
        timeout = 5.0
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < timeout and not rospy.is_shutdown():
            with self.frame_lock:
                if self.latest_frame is not None:
                    height, width = self.latest_frame.shape[:2]
                    # Determine acceleration status
                    if self.vpi_processor and self.vpi_processor.is_available():
                        accel_status = "VPI"
                    elif self.cuda_available:
                        accel_status = "CUDA"
                    else:
                        accel_status = "CPU"
                    
                    # Log both capture and processing resolutions
                    if self.width != self.capture_width or self.height != self.capture_height:
                        rospy.loginfo(f"Camera initialized: {self.capture_width}x{self.capture_height} capture -> {width}x{height} processing @ {self.fps} FPS ({accel_status} acceleration)")
                    else:
                        rospy.loginfo(f"Camera initialized: {width}x{height} @ {self.fps} FPS ({accel_status} acceleration)")
                    return True
            rospy.sleep(0.1)
        
        rospy.logerr("Camera initialization timeout - no frames received")
        return False
    
    def read_frame(self):
        """
        Read a frame from the camera
        
        Returns:
            (success, frame) tuple where success is bool and frame is numpy array
        """
        with self.frame_lock:
            if self.latest_frame is not None:
                frame = self.latest_frame.copy()
                return True, frame
        return False, None
    
    def read_frame_gpu(self):
        """
        Read a frame and upload to GPU memory (if CUDA available)
        
        Returns:
            (success, frame) tuple where frame is either numpy array or cv2.cuda_GpuMat
        """
        success, frame = self.read_frame()
        if not success:
            return False, None
        
        if self.cuda_available and self.use_cuda:
            try:
                gpu_frame = cv2.cuda_GpuMat()
                gpu_frame.upload(frame)
                return True, gpu_frame
            except Exception as e:
                rospy.logwarn(f"GPU upload failed: {e}, using CPU frame")
                return success, frame
        
        return success, frame
    
    def has_cuda(self):
        """Check if CUDA acceleration is available"""
        return self.cuda_available
    
    def release(self):
        """Release camera resources"""
        if self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None
        rospy.loginfo("Camera released")
    
    def __enter__(self):
        """Context manager entry"""
        self.initialize()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.release()

