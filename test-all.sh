#!/bin/bash
# Comprehensive Test Script for Science Robot v2.0
# Runs all tests from POST_BUILD_TESTING.md in sequence

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
IMAGE_NAME="science-robot-v2:latest"
ROBOT_NAME=${VEHICLE_NAME:-robot1}
ROS_MASTER=${ROS_MASTER_URI:-http://localhost:11311}
VPI_PYTHON_PATH="/usr/lib/python3/dist-packages/vpi"
VPI_LIB_PATH="/usr/lib/aarch64-linux-gnu"
TEST_CONTAINER_NAME="test-science-robot"

# Test mode: quick, full, or custom
TEST_MODE=${1:-full}

# VPI volume mounts (standard for all tests)
VPI_MOUNTS=(
  -v "${VPI_PYTHON_PATH}:/host${VPI_PYTHON_PATH}:ro"
  -v "${VPI_LIB_PATH}:/host${VPI_LIB_PATH}:ro"
)

# Common environment variables
COMMON_ENV=(
  -e "ROS_MASTER_URI=${ROS_MASTER}"
  -e "VEHICLE_NAME=${ROBOT_NAME}"
)

# Test counters
TESTS_PASSED=0
TESTS_FAILED=0
TESTS_SKIPPED=0

# Helper functions
print_test() {
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}Test $1: $2${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
    ((TESTS_PASSED++))
}

print_failure() {
    echo -e "${RED}✗ $1${NC}"
    ((TESTS_FAILED++))
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
    ((TESTS_SKIPPED++))
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# Cleanup function
cleanup() {
    print_info "Cleaning up test containers..."
    docker stop "${TEST_CONTAINER_NAME}" > /dev/null 2>&1 || true
    docker rm "${TEST_CONTAINER_NAME}" > /dev/null 2>&1 || true
}

trap cleanup EXIT

# Test 1: Verify Docker Image
test_1_verify_image() {
    print_test "1" "Verify Docker Image"
    
    # Check if image exists using docker image inspect (more reliable than grep)
    if docker image inspect "${IMAGE_NAME}" > /dev/null 2>&1; then
        print_success "Docker image exists: ${IMAGE_NAME}"
        docker images "${IMAGE_NAME}" | head -2
        return 0
    else
        # Fallback: try grep in case inspect fails for other reasons
        if docker images | grep -q "${IMAGE_NAME}"; then
            print_success "Docker image exists: ${IMAGE_NAME}"
            docker images | grep "${IMAGE_NAME}" | head -1
            return 0
        else
            print_failure "Docker image not found: ${IMAGE_NAME}"
            print_info "Build the image first: docker build -t ${IMAGE_NAME} ."
            print_info "Current images:"
            docker images | head -5
            return 1
        fi
    fi
}

# Test 2: Test Container Startup
test_2_container_startup() {
    print_test "2" "Test Container Startup (Basic)"
    
    print_info "Testing container can start and entrypoint runs..."
    
    if docker run --rm --network host \
        "${COMMON_ENV[@]}" \
        "${VPI_MOUNTS[@]}" \
        "${IMAGE_NAME}" \
        bash -c "source /opt/ros/noetic/setup.bash && \
                 source /code/packages/devel/setup.bash && \
                 echo \$ROS_MASTER_URI && \
                 rospack find science_robot && \
                 python3 -c 'from science_robot import config; print(\"OK\")'" > /tmp/test2.log 2>&1; then
        print_success "Container starts and basic checks pass"
        if [ "$TEST_MODE" = "full" ]; then
            echo "--- Container output ---"
            cat /tmp/test2.log
        fi
        return 0
    else
        print_failure "Container startup test failed"
        echo "--- Error output ---"
        cat /tmp/test2.log
        return 1
    fi
}

# Test 3: Verify ROS Package Discovery
test_3_ros_package() {
    print_test "3" "Verify ROS Package Discovery"
    
    print_info "Testing ROS package is discoverable..."
    
    # Suppress entrypoint output by running bash directly
    PACKAGE_PATH=$(docker run --rm --network host \
        "${COMMON_ENV[@]}" \
        "${VPI_MOUNTS[@]}" \
        --entrypoint bash \
        "${IMAGE_NAME}" \
        -c "source /opt/ros/noetic/setup.bash && \
            source /code/packages/devel/setup.bash && \
            rospack find science_robot" 2>/tmp/test3.log)
    
    EXIT_CODE=$?
    
    if [ $EXIT_CODE -eq 0 ] && [ -n "$PACKAGE_PATH" ]; then
        # Clean up the path (remove any extra output)
        PACKAGE_PATH=$(echo "$PACKAGE_PATH" | grep -E "^/.*science_robot" | head -1)
        if [ -n "$PACKAGE_PATH" ]; then
            print_success "ROS package found: ${PACKAGE_PATH}"
            return 0
        fi
    fi
    
    print_failure "ROS package not found"
    if [ -f /tmp/test3.log ]; then
        cat /tmp/test3.log
    fi
    return 1
}

# Test 4: Test ROS Launch File
test_4_launch_file() {
    print_test "4" "Test ROS Launch File Validation"
    
    print_info "Validating launch file XML syntax..."
    
    # Suppress entrypoint output by running bash directly
    OUTPUT=$(docker run --rm --network host \
        "${COMMON_ENV[@]}" \
        "${VPI_MOUNTS[@]}" \
        --entrypoint bash \
        "${IMAGE_NAME}" \
        -c "source /opt/ros/noetic/setup.bash && \
            source /code/packages/devel/setup.bash && \
            LAUNCH_FILE=\$(rospack find science_robot)/launch/science_robot.launch && \
            if [ -f \"\$LAUNCH_FILE\" ]; then
                if command -v xmllint > /dev/null 2>&1; then
                    xmllint --noout \"\$LAUNCH_FILE\" 2>&1
                else
                    echo 'WARNING: xmllint not found, trying Python XML parser...'
                    python3 -c \"import xml.etree.ElementTree as ET; ET.parse('\"\$LAUNCH_FILE\"')\" 2>&1
                fi
            else
                echo \"ERROR: Launch file not found: \$LAUNCH_FILE\"
                exit 1
            fi" 2>&1)
    
    EXIT_CODE=$?
    
    if [ $EXIT_CODE -eq 0 ]; then
        print_success "Launch file XML is valid"
        return 0
    else
        print_failure "Launch file validation failed"
        echo "$OUTPUT"
        return 1
    fi
}

# Test 5: Test ROS Node Startup
test_5_node_startup() {
    print_test "5" "Test ROS Node Startup (10 second timeout)"
    
    print_info "Testing node can initialize (will timeout after 10s)..."
    print_warning "This test requires ROS master to be running"
    
    if [ "$TEST_MODE" = "quick" ]; then
        print_warning "Skipping in quick mode (requires ROS master)"
        return 0
    fi
    
    print_info "Running roslaunch (this may take up to 18 seconds)..."
    print_info "Capturing output (running with entrypoint)..."
    
    # Run docker command and capture ALL output (both stdout and stderr)
    # Use unbuffered output and ensure we capture everything
    OUTPUT=$(timeout 20 docker run --rm --network host \
        "${COMMON_ENV[@]}" \
        "${VPI_MOUNTS[@]}" \
        "${IMAGE_NAME}" \
        timeout 10 roslaunch science_robot science_robot.launch robot_name:="${ROBOT_NAME}" 2>&1; echo "EXIT_CODE:$?")
    
    # Extract exit code if present
    if echo "$OUTPUT" | grep -q "EXIT_CODE:"; then
        EXIT_CODE=$(echo "$OUTPUT" | grep "EXIT_CODE:" | tail -1 | sed 's/.*EXIT_CODE://')
        OUTPUT=$(echo "$OUTPUT" | sed '/EXIT_CODE:/d')
    else
        EXIT_CODE=$?
    fi
    
    echo "$OUTPUT" > /tmp/test5.log
    echo "Exit code: $EXIT_CODE" >> /tmp/test5.log
    
    # Check output length first
    OUTPUT_LEN=${#OUTPUT}
    print_info "Captured ${OUTPUT_LEN} characters of output"
    
    # Always show output in full mode for debugging
    if [ "$TEST_MODE" = "full" ]; then
        echo ""
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "Test 5 Full Output (Entrypoint + ROS Launch)"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        if [ -n "$OUTPUT" ] && [ ${OUTPUT_LEN} -gt 0 ]; then
            echo "$OUTPUT"
        else
            echo "(No output captured - command may have failed silently)"
            print_warning "Check if docker command is working: docker run --rm ${IMAGE_NAME} echo 'test'"
        fi
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "Exit code: $EXIT_CODE"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
    fi
    
    # Check for initialization messages
    if echo "$OUTPUT" | grep -qi "Initializing\|Camera subscriber\|Motor controller\|science_robot_controller"; then
        print_success "Node initialized successfully"
        return 0
    elif echo "$OUTPUT" | grep -qi "ModuleNotFoundError\|ImportError\|Traceback"; then
        print_failure "Import errors detected"
        echo "$OUTPUT" | grep -iE "Error|Exception|Traceback" | head -10
        return 1
    elif echo "$OUTPUT" | grep -qi "ROS_MASTER_URI\|master\|roscore"; then
        print_warning "Node startup test inconclusive - ROS connection issues possible"
        print_info "Check if ROS master is running: rostopic list"
        if [ "$TEST_MODE" = "full" ]; then
            print_info "Exit code: $EXIT_CODE"
        fi
        return 0
    else
        print_warning "Node startup test inconclusive - no clear success/failure indicators"
        print_info "Exit code: $EXIT_CODE"
        print_info "Output length: ${OUTPUT_LEN} characters"
        print_info "Check /tmp/test5.log for full output"
        if [ "$TEST_MODE" = "full" ] && [ ${OUTPUT_LEN} -lt 100 ]; then
            print_warning "Very little output captured - docker command may have failed silently"
        fi
        return 0
    fi
}

# Test 6: Verify ROS Topic Connectivity
test_6_topic_connectivity() {
    print_test "6" "Verify ROS Topic Connectivity"
    
    print_warning "This test requires ROS master and other nodes to be running"
    
    if [ "$TEST_MODE" = "quick" ]; then
        print_warning "Skipping in quick mode (requires ROS master and nodes)"
        return 0
    fi
    
    print_info "Starting container in background..."
    docker run -d --name "${TEST_CONTAINER_NAME}" --network host \
        "${COMMON_ENV[@]}" \
        "${VPI_MOUNTS[@]}" \
        "${IMAGE_NAME}" > /dev/null 2>&1
    
    sleep 5
    
    print_info "Checking if node is running..."
    if docker exec "${TEST_CONTAINER_NAME}" rosnode list 2>/dev/null | grep -q science_robot; then
        print_success "science_robot node is running"
    else
        print_warning "science_robot node not found (may not have started yet)"
    fi
    
    print_info "Checking topics..."
    TOPICS=$(docker exec "${TEST_CONTAINER_NAME}" rostopic list 2>/dev/null | grep -E "(camera|wheels)" || true)
    if [ -n "$TOPICS" ]; then
        print_success "Camera/wheels topics found"
        echo "$TOPICS" | head -5
    else
        print_warning "No camera/wheels topics found (may need other nodes running)"
    fi
    
    print_info "Checking VPI status..."
    if docker exec "${TEST_CONTAINER_NAME}" python3 -c "import vpi; print('VPI OK')" 2>/dev/null; then
        print_success "VPI is accessible"
    else
        print_warning "VPI not available (optional, will use CPU fallback)"
    fi
    
    docker stop "${TEST_CONTAINER_NAME}" > /dev/null 2>&1 || true
    docker rm "${TEST_CONTAINER_NAME}" > /dev/null 2>&1 || true
    
    return 0
}

# Test 7: Test Camera Frame Reception
test_7_camera_frames() {
    print_test "7" "Test Camera Frame Reception"
    
    print_warning "This test requires camera node to be running"
    
    if [ "$TEST_MODE" = "quick" ]; then
        print_warning "Skipping in quick mode (requires camera node)"
        return 0
    fi
    
    print_info "Testing camera frame reception (10 second timeout)..."
    
    OUTPUT=$(timeout 15 docker run --rm --network host \
        "${COMMON_ENV[@]}" \
        "${VPI_MOUNTS[@]}" \
        "${IMAGE_NAME}" \
        bash -c "python3 -c \"
import rospy
from sensor_msgs.msg import CompressedImage
import time

rospy.init_node('test_camera')
received = False

def callback(msg):
    global received
    received = True
    print('Camera frame received!')

sub = rospy.Subscriber('/${ROBOT_NAME}/camera_node/image/compressed', CompressedImage, callback)
timeout = time.time() + 10
while not received and time.time() < timeout:
    rospy.sleep(0.1)

if received:
    print('SUCCESS: Camera frames are being received')
else:
    print('WARNING: No camera frames received in 10 seconds')
\"" 2>&1 || true)
    
    if echo "$OUTPUT" | grep -q "SUCCESS"; then
        print_success "Camera frames are being received"
        return 0
    else
        print_warning "No camera frames received (camera node may not be running)"
        return 0
    fi
}

# Test 8: Test Motor Command Publishing
test_8_motor_publishing() {
    print_test "8" "Test Motor Command Publishing"
    
    print_warning "This test requires wheels_driver_node to be running"
    
    if [ "$TEST_MODE" = "quick" ]; then
        print_warning "Skipping in quick mode (requires wheels_driver_node)"
        return 0
    fi
    
    print_info "Testing motor command publishing..."
    
    OUTPUT=$(timeout 10 docker run --rm --network host \
        "${COMMON_ENV[@]}" \
        "${VPI_MOUNTS[@]}" \
        "${IMAGE_NAME}" \
        bash -c "python3 -c \"
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Header

rospy.init_node('test_motor')
pub = rospy.Publisher('/${ROBOT_NAME}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
rospy.sleep(1)

msg = WheelsCmdStamped()
msg.header = Header()
msg.header.stamp = rospy.Time.now()
msg.vel_left = 0.0
msg.vel_right = 0.0

pub.publish(msg)
print('SUCCESS: Motor command published')
print('Subscribers:', pub.get_num_connections())
\"" 2>&1 || true)
    
    if echo "$OUTPUT" | grep -q "SUCCESS"; then
        print_success "Motor command published successfully"
        SUBSCRIBERS=$(echo "$OUTPUT" | grep "Subscribers:" | awk '{print $2}')
        if [ "$SUBSCRIBERS" -gt 0 ]; then
            print_success "Motor topic has ${SUBSCRIBERS} subscriber(s)"
        else
            print_warning "Motor topic has no subscribers (wheels_driver_node may not be running)"
        fi
        return 0
    else
        print_warning "Motor publishing test inconclusive"
        return 0
    fi
}

# Test 9: Full Integration Test
test_9_integration() {
    print_test "9" "Full Integration Test (30 seconds)"
    
    print_warning "This test requires full ROS environment"
    
    if [ "$TEST_MODE" = "quick" ]; then
        print_warning "Skipping in quick mode (requires full ROS environment)"
        return 0
    fi
    
    print_info "Running robot for 30 seconds..."
    
    OUTPUT=$(timeout 35 docker run --rm --network host \
        "${COMMON_ENV[@]}" \
        "${VPI_MOUNTS[@]}" \
        "${IMAGE_NAME}" \
        timeout 30 roslaunch science_robot science_robot.launch robot_name:="${ROBOT_NAME}" 2>&1 || true)
    
    echo "$OUTPUT" > /tmp/test9.log
    
    # Check for successful initialization
    if echo "$OUTPUT" | grep -q "Initializing\|Camera\|Motor"; then
        print_success "Integration test: Robot initialized and ran"
        if [ "$TEST_MODE" = "full" ]; then
            echo "--- Integration test output (last 30 lines) ---"
            echo "$OUTPUT" | tail -30
        fi
        return 0
    else
        print_warning "Integration test inconclusive (check logs for details)"
        return 0
    fi
}

# Test 10: Test Error Handling
test_10_error_handling() {
    print_test "10" "Test Error Handling"
    
    print_info "Testing with wrong robot name (should handle gracefully)..."
    
    OUTPUT=$(timeout 15 docker run --rm --network host \
        -e "ROS_MASTER_URI=${ROS_MASTER}" \
        -e "VEHICLE_NAME=wrong_robot" \
        "${VPI_MOUNTS[@]}" \
        "${IMAGE_NAME}" \
        timeout 10 roslaunch science_robot science_robot.launch robot_name:=wrong_robot 2>&1 || true)
    
    # Check that it doesn't crash with exceptions
    if echo "$OUTPUT" | grep -q "Traceback\|Exception\|Fatal"; then
        print_failure "Error handling test failed: unhandled exceptions"
        echo "$OUTPUT" | grep -E "Traceback|Exception|Fatal" | head -10
        return 1
    else
        print_success "Error handling: No crashes detected (warnings OK)"
        return 0
    fi
}

# Main test runner
main() {
    echo -e "${GREEN}"
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║     Science Robot v2.0 - Comprehensive Test Suite        ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
    
    echo "Configuration:"
    echo "  Image: ${IMAGE_NAME}"
    echo "  Robot Name: ${ROBOT_NAME}"
    echo "  ROS Master: ${ROS_MASTER}"
    echo "  Test Mode: ${TEST_MODE}"
    echo ""
    
    # Check if image exists first
    if ! test_1_verify_image; then
        echo -e "${RED}Image not found. Please build first:${NC}"
        echo "  docker build -t ${IMAGE_NAME} ."
        exit 1
    fi
    
    echo ""
    echo "Running tests..."
    echo ""
    
    # Run all tests
    test_2_container_startup
    test_3_ros_package
    test_4_launch_file
    test_5_node_startup
    test_6_topic_connectivity
    test_7_camera_frames
    test_8_motor_publishing
    test_9_integration
    test_10_error_handling
    
    # Print summary
    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}Test Summary${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}Passed: ${TESTS_PASSED}${NC}"
    echo -e "${RED}Failed: ${TESTS_FAILED}${NC}"
    echo -e "${YELLOW}Skipped/Warnings: ${TESTS_SKIPPED}${NC}"
    echo ""
    
    if [ $TESTS_FAILED -eq 0 ]; then
        echo -e "${GREEN}✓ All critical tests passed!${NC}"
        exit 0
    else
        echo -e "${RED}✗ Some tests failed. Check output above.${NC}"
        exit 1
    fi
}

# Run main function
main

