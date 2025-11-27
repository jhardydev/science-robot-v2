#!/usr/bin/env python3
"""
Display Controller for OLED/LCD Display
Publishes network information (IP address and WiFi SSID) to the display driver node
Adds network info without replacing existing display content

TO REMOVE: Delete this file and remove the integration code from science_robot_node.py
"""
import rospy
import subprocess
import threading
import time
import socket
import os
import numpy as np
import cv2
from duckietown_msgs.msg import DisplayFragment
from sensor_msgs.msg import Image, RegionOfInterest
from std_msgs.msg import Header
from science_robot import config

# Try to import netifaces for better network interface access
try:
    import netifaces
    NETIFACES_AVAILABLE = True
except ImportError:
    NETIFACES_AVAILABLE = False

class DisplayController:
    """Controller for publishing display fragments to the OLED/LCD display"""
    
    # DisplayFragment region constants
    REGION_FULL = 0
    REGION_HEADER = 1
    REGION_BODY = 2
    REGION_FOOTER = 3
    
    def __init__(self):
        """Initialize display controller"""
        # ROS publisher for display fragments
        self.display_pub = rospy.Publisher(
            f'/{config.ROBOT_NAME}/display_driver_node/fragments',
            DisplayFragment,
            queue_size=1
        )
        
        # Network info cache
        self.cached_ip = "No IP"
        self.cached_ssid = "No WiFi"
        self.network_update_interval = config.DISPLAY_NETWORK_UPDATE_INTERVAL
        self.last_network_update = 0
        
        # Scrolling state
        self.scroll_position = 0
        self.scroll_update_interval = config.DISPLAY_SCROLL_INTERVAL
        self.last_scroll_update = 0
        
        # Display settings (from config)
        self.display_width = config.DISPLAY_WIDTH
        self.display_height = config.DISPLAY_HEIGHT
        self.text_height = config.DISPLAY_TEXT_HEIGHT
        self.font_scale = config.DISPLAY_FONT_SCALE
        self.font_thickness = config.DISPLAY_FONT_THICKNESS
        
        # Display update thread
        self.running = False
        self.update_thread = None
        
        # Test mode - show test patterns instead of network info
        self.test_mode = config.DISPLAY_TEST_MODE
        self.test_pattern_index = 0
        self.test_pattern_timer = 0
        self.test_pattern_interval = 3.0  # Show each pattern for 3 seconds
        
        if self.test_mode:
            rospy.loginfo("Display controller initialized in TEST MODE - will show test patterns")
        else:
            rospy.loginfo("Display controller initialized")
    
    def get_wlan0_ip(self):
        """Get IP address from wlan0 interface"""
        try:
            # Method 1: Using netifaces (if available, most reliable and works in Docker)
            if NETIFACES_AVAILABLE:
                try:
                    if 'wlan0' in netifaces.interfaces():
                        addrs = netifaces.ifaddresses('wlan0')
                        if netifaces.AF_INET in addrs:
                            for addr_info in addrs[netifaces.AF_INET]:
                                ip = addr_info.get('addr', '')
                                if ip and not ip.startswith('127.'):
                                    return ip
                except Exception:
                    pass
            
            # Method 2: Read from /proc/net/fib_trie (works in Docker, no commands needed)
            # This file contains IP addresses for all interfaces
            try:
                with open('/proc/net/fib_trie', 'r') as f:
                    content = f.read()
                    # Look for wlan0 section and extract IP addresses
                    # Format is complex, but we can look for "wlan0" followed by IP patterns
                    import re
                    # Look for lines with wlan0 and IP addresses
                    lines = content.split('\n')
                    in_wlan0_section = False
                    for line in lines:
                        if 'wlan0' in line.lower():
                            in_wlan0_section = True
                        if in_wlan0_section:
                            # Look for IP address pattern (x.x.x.x format)
                            ip_match = re.search(r'\b(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})\b', line)
                            if ip_match:
                                ip = ip_match.group(1)
                                if not ip.startswith('127.') and not ip.startswith('0.'):
                                    return ip
            except (FileNotFoundError, IOError, PermissionError):
                pass
            
            # Method 3: Read from /sys/class/net/wlan0/ (works in Docker)
            # Check if wlan0 exists and try to get IP from address files
            try:
                wlan0_path = '/sys/class/net/wlan0'
                if os.path.exists(wlan0_path):
                    # Try reading address info from various sysfs files
                    # Unfortunately, /sys doesn't directly give us IP, but we can check if interface is up
                    # For IP, we need to use other methods
                    pass
            except (IOError, PermissionError):
                pass
            
            # Method 4: Try subprocess commands (fallback if file methods fail)
            # Try 'ip' command first
            try:
                result = subprocess.run(
                    ['ip', 'addr', 'show', 'wlan0'],
                    capture_output=True,
                    text=True,
                    timeout=2,
                    stderr=subprocess.DEVNULL
                )
                if result.returncode == 0:
                    for line in result.stdout.split('\n'):
                        if 'inet ' in line and '127.0.0.1' not in line:
                            ip = line.split()[1].split('/')[0]
                            if ip:
                                return ip
            except (FileNotFoundError, subprocess.TimeoutExpired):
                pass
            
            # Method 5: Try 'ifconfig' command
            try:
                result = subprocess.run(
                    ['ifconfig', 'wlan0'],
                    capture_output=True,
                    text=True,
                    timeout=2,
                    stderr=subprocess.DEVNULL
                )
                if result.returncode == 0:
                    for line in result.stdout.split('\n'):
                        if 'inet ' in line:
                            parts = line.split()
                            for i, part in enumerate(parts):
                                if part == 'inet' and i + 1 < len(parts):
                                    ip = parts[i + 1]
                                    if ip and not ip.startswith('127.'):
                                        return ip
            except (FileNotFoundError, subprocess.TimeoutExpired):
                pass
            
        except Exception as e:
            # Only log at debug level - expected failures when commands/files not available
            rospy.logdebug(f"Failed to get IP address from wlan0: {e}")
        return "No IP"
    
    def get_wifi_ssid(self):
        """Get WiFi SSID (network name)"""
        try:
            # Method 1: Read from /proc/net/wireless (works in Docker, no commands needed)
            try:
                with open('/proc/net/wireless', 'r') as f:
                    lines = f.readlines()
                    # Look for wlan0 in the wireless interfaces
                    for line in lines:
                        if 'wlan0' in line:
                            # Found wlan0, but SSID is not in /proc/net/wireless
                            # We need to read from /sys/class/net/wlan0/wireless/ssid or use iw
                            break
            except (FileNotFoundError, IOError, PermissionError):
                pass
            
            # Method 2: Read from /sys/class/net/wlan0/wireless/ssid (if available)
            try:
                ssid_path = '/sys/class/net/wlan0/wireless/ssid'
                if os.path.exists(ssid_path):
                    with open(ssid_path, 'r') as f:
                        ssid = f.read().strip()
                        if ssid:
                            return ssid
            except (IOError, PermissionError):
                pass
            
            # Method 3: Try iwgetid command (if available)
            try:
                result = subprocess.run(
                    ['iwgetid', '-r'],
                    capture_output=True,
                    text=True,
                    timeout=2,
                    stderr=subprocess.DEVNULL
                )
                if result.returncode == 0 and result.stdout.strip():
                    return result.stdout.strip()
            except (FileNotFoundError, subprocess.TimeoutExpired):
                pass
            
            # Method 4: Try iwconfig command
            try:
                result = subprocess.run(
                    ['iwconfig', 'wlan0'],
                    capture_output=True,
                    text=True,
                    timeout=2,
                    stderr=subprocess.DEVNULL
                )
                if result.returncode == 0:
                    for line in result.stdout.split('\n'):
                        if 'ESSID:' in line:
                            ssid = line.split('ESSID:')[1].strip().strip('"')
                            if ssid and ssid != 'off/any':
                                return ssid
            except (FileNotFoundError, subprocess.TimeoutExpired):
                pass
            
            # Method 5: Try iw command (modern alternative to iwconfig)
            try:
                result = subprocess.run(
                    ['iw', 'dev', 'wlan0', 'info'],
                    capture_output=True,
                    text=True,
                    timeout=2,
                    stderr=subprocess.DEVNULL
                )
                if result.returncode == 0:
                    for line in result.stdout.split('\n'):
                        if 'ssid' in line.lower():
                            parts = line.split()
                            for i, part in enumerate(parts):
                                if part.lower() == 'ssid' and i + 1 < len(parts):
                                    ssid = parts[i + 1].strip()
                                    if ssid:
                                        return ssid
            except (FileNotFoundError, subprocess.TimeoutExpired):
                pass
            
            # Method 6: Try nmcli (if NetworkManager is used)
            try:
                result = subprocess.run(
                    ['nmcli', '-t', '-f', 'active,ssid', 'dev', 'wifi'],
                    capture_output=True,
                    text=True,
                    timeout=2,
                    stderr=subprocess.DEVNULL
                )
                if result.returncode == 0:
                    for line in result.stdout.split('\n'):
                        if ':yes:' in line:
                            ssid = line.split(':yes:')[1]
                            if ssid:
                                return ssid
            except (FileNotFoundError, subprocess.TimeoutExpired):
                pass
                
        except Exception as e:
            # Only log at debug level - expected failures when commands/files not available
            rospy.logdebug(f"Failed to get WiFi SSID: {e}")
        return "No WiFi"
    
    def format_scrolling_text(self, ssid, ip, scroll_pos, max_chars=16):
        """
        Format network info with scrolling
        
        Args:
            ssid: WiFi network name
            ip: IP address
            scroll_pos: Current scroll position
            max_chars: Maximum characters to display
        
        Returns:
            Formatted text string
        """
        # Format: "SSID: IP" (e.g., "MyNetwork: 192.168.1.162")
        combined_text = f"{ssid}: {ip}"
        
        # If text fits, no scrolling needed
        if len(combined_text) <= max_chars:
            return combined_text
        
        # Add spacing for smooth scrolling
        padded_text = combined_text + " " * 3
        
        # Scroll the text
        scrolled_text = padded_text[scroll_pos:] + padded_text[:scroll_pos]
        return scrolled_text[:max_chars]
    
    def create_text_image(self, text, width, height):
        """
        Create an image with text rendered on it
        
        Args:
            text: Text to render
            width: Image width in pixels
            height: Image height in pixels
        
        Returns:
            numpy array (grayscale image, 0-255)
        """
        # Create black image (grayscale)
        img = np.zeros((height, width), dtype=np.uint8)
        
        # Use smaller font for OLED displays (FONT_HERSHEY_PLAIN is smaller than SIMPLEX)
        # FONT_HERSHEY_PLAIN = 1, FONT_HERSHEY_SIMPLEX = 0
        font = cv2.FONT_HERSHEY_PLAIN  # Smaller, more compact font for small displays
        (text_width, text_height), baseline = cv2.getTextSize(
            text, font, self.font_scale, self.font_thickness
        )
        
        # Position text (left-aligned, vertically centered)
        x = 0
        y = (height + text_height) // 2
        
        # Draw white text on black background
        cv2.putText(
            img, text, (x, y),
            font, self.font_scale,
            (255,),  # White color (grayscale)
            self.font_thickness,
            cv2.LINE_AA
        )
        
        return img
    
    def image_to_ros_image(self, img_array):
        """
        Convert numpy array to sensor_msgs/Image
        
        Args:
            img_array: numpy array (grayscale, uint8)
        
        Returns:
            sensor_msgs/Image message
        """
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = rospy.Time.now()
        img_msg.header.frame_id = "display"
        
        height, width = img_array.shape
        img_msg.height = height
        img_msg.width = width
        img_msg.encoding = "mono8"  # Grayscale
        img_msg.is_bigendian = 0
        img_msg.step = width  # Bytes per row for mono8
        img_msg.data = img_array.tobytes()
        
        return img_msg
    
    def update_network_info(self):
        """Update cached network information"""
        current_time = time.time()
        if current_time - self.last_network_update >= self.network_update_interval:
            self.cached_ip = self.get_wlan0_ip()
            self.cached_ssid = self.get_wifi_ssid()
            self.last_network_update = current_time
            rospy.logdebug(f"Network info updated: SSID={self.cached_ssid}, IP={self.cached_ip}")
    
    def publish_display_fragment(self, text, region=REGION_FOOTER, page=255, x_offset=0, y_offset=0, z=0):
        """
        Publish a display fragment message
        
        Args:
            text: Text to display
            region: Display region (REGION_FULL, REGION_HEADER, REGION_BODY, REGION_FOOTER)
            page: Page number (255 = PAGE_ALL)
            x_offset: X offset in pixels
            y_offset: Y offset in pixels
            z: Z-order (higher values appear on top)
        """
        try:
            # Create image with text
            img_array = self.create_text_image(text, self.display_width, self.text_height)
            
            # Convert to ROS Image message
            img_msg = self.image_to_ros_image(img_array)
            
            # Create RegionOfInterest
            roi = RegionOfInterest()
            roi.x_offset = x_offset
            roi.y_offset = y_offset
            roi.height = self.text_height
            roi.width = self.display_width
            roi.do_rectify = False
            
            # Create DisplayFragment
            fragment = DisplayFragment()
            fragment.header = Header()
            fragment.header.stamp = rospy.Time.now()
            fragment.header.frame_id = "display"
            fragment.id = "network_info"  # Unique ID for this fragment
            fragment.region = region
            fragment.page = page
            fragment.data = img_msg
            fragment.location = roi
            fragment.z = z  # Z-order (higher = on top)
            fragment.ttl = -1  # -1 = persistent (doesn't expire)
            
            self.display_pub.publish(fragment)
        except Exception as e:
            rospy.logwarn(f"Failed to publish display fragment: {e}")
    
    def create_test_image(self, width, height, pattern_type="grid", label=""):
        """Create a test pattern image with clear labels"""
        img = np.zeros((height, width), dtype=np.uint8)
        font = cv2.FONT_HERSHEY_PLAIN
        scale = 0.25  # Smaller scale for clearer text
        
        if pattern_type == "grid":
            # Draw grid with numbers - very clear borders
            cv2.rectangle(img, (0, 0), (width-1, height-1), 255, 2)  # Thicker border
            cv2.line(img, (width//2, 0), (width//2, height-1), 255, 1)
            cv2.line(img, (0, height//2), (width-1, height//2), 255, 1)
            # Corner markers
            cv2.circle(img, (0, 0), 3, 255, -1)  # Top-left
            cv2.circle(img, (width-1, 0), 3, 255, -1)  # Top-right
            cv2.circle(img, (0, height-1), 3, 255, -1)  # Bottom-left
            cv2.circle(img, (width-1, height-1), 3, 255, -1)  # Bottom-right
            # Labels
            if label:
                cv2.putText(img, label, (2, 7), font, scale, 255, 1)
            cv2.putText(img, f"{width}x{height}", (2, height-2), font, scale, 255, 1)
        elif pattern_type == "lines":
            # Draw horizontal lines with Y coordinates
            for y in range(0, height, 8):
                cv2.line(img, (0, y), (width-1, y), 255, 1)
                if width > 25:
                    cv2.putText(img, f"Y{y}", (2, min(y+6, height-2)), font, scale, 255, 1)
            # Draw vertical lines with X coordinates
            for x in range(0, width, 16):
                cv2.line(img, (x, 0), (x, height-1), 255, 1)
                if height > 8 and x < width-15:
                    cv2.putText(img, str(x), (min(x+2, width-12), 7), font, scale, 255, 1)
        elif pattern_type == "position":
            # Simple pattern with clear label showing position info
            cv2.rectangle(img, (0, 0), (width-1, height-1), 255, 2)
            if label:
                # Show label in center
                text_y = height // 2
                cv2.putText(img, label, (max(2, width//2-15), text_y), font, 0.3, 255, 1)
            if width > 30:
                cv2.putText(img, f"{width}x{height}", (2, height-2), font, scale, 255, 1)
        elif pattern_type == "text":
            # Simple text pattern for testing text rendering
            cv2.rectangle(img, (0, 0), (width-1, height-1), 255, 1)
            if label:
                cv2.putText(img, label, (2, height//2+3), font, 0.3, 255, 1)
        
        return img
    
    def clear_display(self):
        """Clear the entire display by publishing black fragments covering all regions"""
        try:
            # Clear with multiple overlapping black fragments to ensure complete coverage
            # Use REGION_FULL with different sizes to cover all possibilities
            clear_fragments = [
                # Full screen clears (try both common resolutions) - these should cover everything
                (self.REGION_FULL, 0, 0, 128, 64, "clear_full_64"),
                (self.REGION_FULL, 0, 0, 128, 32, "clear_full_32"),
                # Overlapping clears in strips to ensure no gaps, especially for header
                (self.REGION_FULL, 0, 0, 128, 8, "clear_strip_0"),
                (self.REGION_FULL, 0, 8, 128, 8, "clear_strip_8"),
                (self.REGION_FULL, 0, 16, 128, 8, "clear_strip_16"),
                (self.REGION_FULL, 0, 24, 128, 8, "clear_strip_24"),
                (self.REGION_FULL, 0, 32, 128, 8, "clear_strip_32"),
                (self.REGION_FULL, 0, 40, 128, 8, "clear_strip_40"),
                (self.REGION_FULL, 0, 48, 128, 8, "clear_strip_48"),
                (self.REGION_FULL, 0, 56, 128, 8, "clear_strip_56"),
                # Region-specific clears with maximum coverage
                (self.REGION_HEADER, 0, 0, 128, 64, "clear_header_full"),
                (self.REGION_BODY, 0, 0, 128, 64, "clear_body_full"),
                (self.REGION_FOOTER, 0, 0, 128, 64, "clear_footer_full"),
            ]
            
            for region, x_off, y_off, w, h, frag_id in clear_fragments:
                # Create black image (all zeros = black)
                black_img = np.zeros((h, w), dtype=np.uint8)
                img_msg = self.image_to_ros_image(black_img)
                
                # Create ROI covering the entire area
                roi = RegionOfInterest()
                roi.x_offset = x_off
                roi.y_offset = y_off
                roi.height = h
                roi.width = w
                roi.do_rectify = False
                
                # Create clear fragment with high z-order to cover everything
                fragment = DisplayFragment()
                fragment.header = Header()
                fragment.header.stamp = rospy.Time.now()
                fragment.header.frame_id = "display"
                fragment.id = frag_id
                fragment.region = region
                fragment.page = 255  # All pages
                fragment.data = img_msg
                fragment.location = roi
                fragment.z = 250  # Very high z-order to cover system icons, but lower than test patterns (z=255)
                fragment.ttl = -1  # Persistent until replaced
                
                self.display_pub.publish(fragment)
                # Small delay between publishes to ensure they're processed
                rospy.sleep(0.02)
            
            rospy.loginfo("Display cleared - published black fragments covering entire display")
        except Exception as e:
            rospy.logwarn(f"Failed to clear display: {e}")
    
    def update_display(self):
        """Update display with network information or test patterns"""
        if self.test_mode:
            # Test mode - cycle through test patterns
            current_time = time.time()
            
            # Check if it's time to change patterns
            pattern_changed = False
            
            # Initialize timer on first run
            if self.test_pattern_timer == 0:
                self.test_pattern_index = 0
                self.test_pattern_timer = current_time
                pattern_changed = True
                rospy.loginfo(f"[TEST MODE] Starting - showing pattern {self.test_pattern_index + 1}/8")
            else:
                # Calculate elapsed time
                elapsed = current_time - self.test_pattern_timer
                
                # Check if interval has passed
                if elapsed >= self.test_pattern_interval:
                    # Time to change pattern
                    old_index = self.test_pattern_index
                    self.test_pattern_index = (self.test_pattern_index + 1) % 8
                    self.test_pattern_timer = current_time
                    pattern_changed = True
                    rospy.loginfo(f"[TEST MODE] Pattern change: {old_index + 1} -> {self.test_pattern_index + 1}/8 (elapsed={elapsed:.2f}s)")
                else:
                    # Still showing current pattern
                    if int(elapsed) % 1 == 0 and elapsed > 0.1:  # Log every second
                        rospy.logdebug(f"[TEST MODE] Pattern {self.test_pattern_index + 1}/8, elapsed={elapsed:.1f}s/{self.test_pattern_interval}s")
            
            # Define test patterns to cycle through with descriptive labels
            # Format: (pattern_type, region, x_offset, y_offset, width, height, fragment_id, label)
            # Start with smaller, simpler patterns to understand behavior
            test_patterns = [
                # Test 1: Small pattern at top-left corner (should be easy to see)
                ("position", self.REGION_FULL, 0, 0, 40, 8, "test_tl", "TL 40x8"),
                # Test 2: Small pattern at top-right corner
                ("position", self.REGION_FULL, 88, 0, 40, 8, "test_tr", "TR 40x8"),
                # Test 3: Full width, small height at top (for header area)
                ("lines", self.REGION_FULL, 0, 0, 128, 8, "test_top", "TOP 128x8"),
                # Test 4: Full display grid 32px
                ("grid", self.REGION_FULL, 0, 0, 128, 32, "test_full_32", "FULL 32px"),
                # Test 5: Full display grid 64px
                ("grid", self.REGION_FULL, 0, 0, 128, 64, "test_full_64", "FULL 64px"),
                # Test 6: Positioned between icons (target location)
                ("text", self.REGION_FULL, 14, 0, 100, 6, "test_between", "BETWEEN"),
                # Test 7: Using REGION_HEADER
                ("position", self.REGION_HEADER, 14, 0, 100, 6, "test_hdr", "HDR X14"),
                # Test 8: Small text test
                ("text", self.REGION_FULL, 14, 0, 90, 6, "test_text", "SSID:IP"),
            ]
            
            pattern_type, region, x_off, y_off, w, h, frag_id, label = test_patterns[self.test_pattern_index]
            
            # Clear display only when pattern actually changes (not every cycle)
            # This prevents unnecessary clearing and ensures patterns stay visible
            if pattern_changed:
                rospy.loginfo(f"[TEST MODE] Clearing display before showing pattern {self.test_pattern_index + 1}/8...")
                self.clear_display()
                rospy.sleep(0.3)  # Give clearing fragments time to publish
                rospy.loginfo(f"[TEST MODE] Display cleared, publishing pattern {self.test_pattern_index + 1}/8...")
            
            # Create test image with label
            img_array = self.create_test_image(w, h, pattern_type, label)
            img_msg = self.image_to_ros_image(img_array)
            
            # Create ROI
            roi = RegionOfInterest()
            roi.x_offset = x_off
            roi.y_offset = y_off
            roi.height = h
            roi.width = w
            roi.do_rectify = False
            
            # Create fragment
            fragment = DisplayFragment()
            fragment.header = Header()
            fragment.header.stamp = rospy.Time.now()
            fragment.header.frame_id = "display"
            fragment.id = frag_id
            fragment.region = region
            fragment.page = 255
            fragment.data = img_msg
            fragment.location = roi
            fragment.z = 255  # Maximum z-order to show on top of everything including clears (z=250)
            fragment.ttl = -1
            
            # Publish the test pattern
            self.display_pub.publish(fragment)
            # Small delay to ensure publish completes
            rospy.sleep(0.1)
            
            if pattern_changed:
                rospy.loginfo(f"[TEST MODE] Published pattern {self.test_pattern_index + 1}/8: {label} at ({x_off},{y_off}) size {w}x{h}, region={region}, z={fragment.z}")
            else:
                # Refresh pattern every cycle to keep it visible (but don't clear)
                rospy.logdebug(f"[TEST MODE] Refreshing pattern {self.test_pattern_index + 1}/8: {label}")
            return
        
        # Normal mode - show network info
        # Update network info if needed
        self.update_network_info()
        
        # Update scroll position
        current_time = time.time()
        if current_time - self.last_scroll_update >= self.scroll_update_interval:
            # Calculate combined text length for scrolling
            # With FONT_HERSHEY_PLAIN at scale 0.25, each character is approximately 4-5 pixels wide
            combined_text = f"{self.cached_ssid}: {self.cached_ip}"
            max_chars = int(self.display_width / 4.5)  # More accurate for PLAIN font at small scale
            if len(combined_text) > max_chars:
                self.scroll_position = (self.scroll_position + 1) % (len(combined_text) + 3)
            else:
                self.scroll_position = 0
            self.last_scroll_update = current_time
        
        # Format scrolling text
        # With FONT_HERSHEY_PLAIN at scale 0.25, each character is approximately 4-5 pixels wide
        max_chars = int(self.display_width / 4.5)  # More accurate for PLAIN font at small scale
        display_text = self.format_scrolling_text(
            self.cached_ssid,
            self.cached_ip,
            self.scroll_position,
            max_chars=max_chars
        )
        
        # Publish to display header region (adds to existing content at top)
        # Position between the yellow status icons at the top
        # y_offset positions it below the top status bar (typically 8-10 pixels for status icons)
        self.publish_display_fragment(
            display_text,
            region=self.REGION_HEADER,  # Add to header region (top of display)
            page=255,  # PAGE_ALL - show on all pages
            x_offset=0,
            y_offset=10,  # Position below the yellow status icons (WiFi, battery, NoBT)
            z=10  # Higher z-order to appear on top of other content
        )
    
    def start(self):
        """Start the display update thread"""
        if self.running:
            return
        
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()
        rospy.loginfo("Display controller started")
    
    def stop(self):
        """Stop the display update thread"""
        self.running = False
        if self.update_thread:
            self.update_thread.join(timeout=2.0)
        rospy.loginfo("Display controller stopped")
    
    def _update_loop(self):
        """Main update loop for display (runs in separate thread)"""
        # For test mode, use faster update rate to ensure patterns cycle properly
        if self.test_mode:
            update_rate = 10.0  # 10 Hz = every 0.1 seconds for more responsive pattern change detection
            rospy.loginfo(f"[TEST MODE] Update loop started at {update_rate}Hz (pattern interval={self.test_pattern_interval}s)")
        else:
            update_rate = config.DISPLAY_UPDATE_RATE
        
        rate = rospy.Rate(update_rate)
        cycle_count = 0
        
        while self.running and not rospy.is_shutdown():
            try:
                cycle_count += 1
                if self.test_mode and cycle_count % 10 == 0:  # Log every 10 cycles (every second at 10Hz)
                    rospy.logdebug(f"[TEST MODE] Update cycle {cycle_count}, current pattern={self.test_pattern_index + 1}/8")
                self.update_display()
            except Exception as e:
                rospy.logwarn(f"Display update error: {e}")
            
            rate.sleep()

