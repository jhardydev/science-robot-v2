#!/usr/bin/env python3
"""
Display Test Pattern Generator
Creates test patterns to understand display positioning and regions
"""
import rospy
import numpy as np
import cv2
from duckietown_msgs.msg import DisplayFragment
from sensor_msgs.msg import Image, RegionOfInterest
from std_msgs.msg import Header
from science_robot import config

class DisplayTestPattern:
    """Generate test patterns for display positioning"""
    
    REGION_FULL = 0
    REGION_HEADER = 1
    REGION_BODY = 2
    REGION_FOOTER = 3
    
    def __init__(self):
        """Initialize test pattern generator"""
        self.display_pub = rospy.Publisher(
            f'/{config.ROBOT_NAME}/display_driver_node/fragments',
            DisplayFragment,
            queue_size=10
        )
        rospy.loginfo("Display test pattern generator initialized")
    
    def create_test_image(self, width, height, pattern_type="grid"):
        """
        Create a test pattern image
        
        Args:
            width: Image width
            height: Image height
            pattern_type: "grid", "corners", "numbers", "lines", "position"
        
        Returns:
            numpy array (grayscale image, 0-255)
        """
        img = np.zeros((height, width), dtype=np.uint8)
        font = cv2.FONT_HERSHEY_PLAIN
        scale = 0.3
        
        if pattern_type == "grid":
            # Draw grid with numbers
            # Draw border
            cv2.rectangle(img, (0, 0), (width-1, height-1), 255, 1)
            # Draw center lines
            cv2.line(img, (width//2, 0), (width//2, height-1), 255, 1)
            cv2.line(img, (0, height//2), (width-1, height//2), 255, 1)
            # Draw quarter markers
            cv2.circle(img, (width//4, height//4), 2, 255, -1)
            cv2.circle(img, (3*width//4, height//4), 2, 255, -1)
            cv2.circle(img, (width//4, 3*height//4), 2, 255, -1)
            cv2.circle(img, (3*width//4, 3*height//4), 2, 255, -1)
            # Add text labels
            cv2.putText(img, "0,0", (2, 8), font, scale, 255, 1)
            if width > 30 and height > 10:
                cv2.putText(img, f"{width-1},{height-1}", (max(2, width-30), height-2), font, scale, 255, 1)
            if width > 40:
                cv2.putText(img, f"{width}x{height}", (width//2-20, height//2+5), font, scale, 255, 1)
        
        elif pattern_type == "corners":
            # Draw markers at corners
            marker_size = min(5, width//4, height//4)
            # Top-left
            cv2.rectangle(img, (0, 0), (marker_size, marker_size), 255, -1)
            cv2.putText(img, "TL", (0, min(marker_size+8, height-2)), font, scale, 255, 1)
            # Top-right
            cv2.rectangle(img, (width-marker_size, 0), (width-1, marker_size), 255, -1)
            cv2.putText(img, "TR", (max(0, width-15), min(marker_size+8, height-2)), font, scale, 255, 1)
            # Bottom-left
            cv2.rectangle(img, (0, height-marker_size), (marker_size, height-1), 255, -1)
            cv2.putText(img, "BL", (0, height-2), font, scale, 255, 1)
            # Bottom-right
            cv2.rectangle(img, (width-marker_size, height-marker_size), (width-1, height-1), 255, -1)
            cv2.putText(img, "BR", (max(0, width-15), height-2), font, scale, 255, 1)
        
        elif pattern_type == "numbers":
            # Draw numbered grid
            for y in range(0, height, 8):
                for x in range(0, width, 16):
                    if x + 20 < width and y + 8 < height:
                        num = f"{x},{y}"
                        cv2.putText(img, num, (x, y+6), font, scale, 255, 1)
        
        elif pattern_type == "lines":
            # Draw horizontal and vertical lines with labels
            # Horizontal lines every 8 pixels
            for y in range(0, height, 8):
                cv2.line(img, (0, y), (width-1, y), 255, 1)
                if width > 20:
                    cv2.putText(img, str(y), (2, min(y+6, height-2)), font, scale, 255, 1)
            # Vertical lines every 16 pixels
            for x in range(0, width, 16):
                cv2.line(img, (x, 0), (x, height-1), 255, 1)
                if height > 8:
                    cv2.putText(img, str(x), (min(x+2, width-10), 8), font, scale, 255, 1)
        
        elif pattern_type == "position":
            # Simple pattern showing position
            cv2.rectangle(img, (0, 0), (width-1, height-1), 255, 1)
            cv2.putText(img, "POS", (width//2-10, height//2), font, 0.4, 255, 1)
            if width > 30:
                cv2.putText(img, f"{width}x{height}", (2, height-2), font, scale, 255, 1)
        
        return img
    
    def image_to_ros_image(self, img_array):
        """Convert numpy array to sensor_msgs/Image"""
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = rospy.Time.now()
        img_msg.header.frame_id = "display"
        
        height, width = img_array.shape
        img_msg.height = height
        img_msg.width = width
        img_msg.encoding = "mono8"
        img_msg.is_bigendian = 0
        img_msg.step = width
        img_msg.data = img_array.tobytes()
        
        return img_msg
    
    def publish_test_pattern(self, pattern_type="grid", region=REGION_FULL, 
                           x_offset=0, y_offset=0, width=128, height=32, z=0, 
                           fragment_id="test"):
        """Publish a test pattern fragment"""
        try:
            # Create test image
            img_array = self.create_test_image(width, height, pattern_type)
            
            # Convert to ROS Image
            img_msg = self.image_to_ros_image(img_array)
            
            # Create ROI
            roi = RegionOfInterest()
            roi.x_offset = x_offset
            roi.y_offset = y_offset
            roi.height = height
            roi.width = width
            roi.do_rectify = False
            
            # Create fragment
            fragment = DisplayFragment()
            fragment.header = Header()
            fragment.header.stamp = rospy.Time.now()
            fragment.header.frame_id = "display"
            fragment.id = fragment_id
            fragment.region = region
            fragment.page = 255
            fragment.data = img_msg
            fragment.location = roi
            fragment.z = z
            fragment.ttl = -1
            
            self.display_pub.publish(fragment)
            rospy.loginfo(f"Published test pattern: {pattern_type}, region={region}, "
                         f"offset=({x_offset},{y_offset}), size={width}x{height}, id={fragment_id}")
        except Exception as e:
            rospy.logerr(f"Failed to publish test pattern: {e}")
    
    def test_all_regions(self):
        """Test all regions with different patterns"""
        rospy.loginfo("Publishing test patterns for all regions...")
        
        # Clear any existing patterns first (publish empty/black fragments)
        # Actually, let's just publish new ones with higher z-order
        
        # Test 1: REGION_FULL - full display grid (128x32 or 128x64)
        rospy.loginfo("Test 1: REGION_FULL - Full display grid")
        self.publish_test_pattern("grid", self.REGION_FULL, 0, 0, 128, 32, 0, "test_full_32")
        rospy.sleep(1)
        
        # Test 2: REGION_FULL - Try 128x64
        rospy.loginfo("Test 2: REGION_FULL - Full display grid (64px height)")
        self.publish_test_pattern("grid", self.REGION_FULL, 0, 0, 128, 64, 0, "test_full_64")
        rospy.sleep(1)
        
        # Test 3: REGION_HEADER - top area
        rospy.loginfo("Test 3: REGION_HEADER - Top area")
        self.publish_test_pattern("lines", self.REGION_HEADER, 0, 0, 128, 8, 1, "test_header")
        rospy.sleep(1)
        
        # Test 4: REGION_BODY - middle area
        rospy.loginfo("Test 4: REGION_BODY - Middle area")
        self.publish_test_pattern("numbers", self.REGION_BODY, 0, 8, 128, 16, 2, "test_body")
        rospy.sleep(1)
        
        # Test 5: REGION_FOOTER - bottom area
        rospy.loginfo("Test 5: REGION_FOOTER - Bottom area")
        self.publish_test_pattern("corners", self.REGION_FOOTER, 0, 24, 128, 8, 3, "test_footer")
        rospy.sleep(1)
        
        # Test 6: Positioned fragment (between icons) - small at top
        rospy.loginfo("Test 6: Positioned fragment - Small at top (between icons)")
        self.publish_test_pattern("position", self.REGION_FULL, 14, 0, 100, 6, 10, "test_positioned_1")
        rospy.sleep(1)
        
        # Test 7: Positioned fragment - different position
        rospy.loginfo("Test 7: Positioned fragment - Different position")
        self.publish_test_pattern("position", self.REGION_FULL, 20, 2, 90, 6, 10, "test_positioned_2")
        rospy.sleep(1)
        
        # Test 8: Positioned fragment - using REGION_HEADER
        rospy.loginfo("Test 8: REGION_HEADER with offset")
        self.publish_test_pattern("position", self.REGION_HEADER, 14, 0, 100, 6, 10, "test_header_offset")
        rospy.sleep(1)
        
        rospy.loginfo("All test patterns published. Check display and take photo.")
        rospy.loginfo("The patterns should help us understand:")
        rospy.loginfo("  1. Actual display resolution (32px or 64px height)")
        rospy.loginfo("  2. How regions work")
        rospy.loginfo("  3. How x_offset and y_offset work")
        rospy.loginfo("  4. Best position for network info")
    
    def test_single_pattern(self, pattern_type="grid", region=REGION_FULL, 
                          x_offset=0, y_offset=0, width=128, height=32):
        """Test a single pattern with specified parameters"""
        self.publish_test_pattern(pattern_type, region, x_offset, y_offset, width, height, 10, "single_test")
        rospy.loginfo(f"Published single test pattern: {pattern_type}, region={region}, "
                     f"offset=({x_offset},{y_offset}), size={width}x{height}")


