"""
Web server for robot monitoring and control
Provides HTTP interface for camera feed and robot status
"""
import rospy
import cv2
import threading
import time
import logging
from flask import Flask, Response, jsonify, render_template_string

logger = logging.getLogger(__name__)

# Global state
robot_controller = None
latest_frame = None
frame_lock = threading.Lock()
robot_initialized = False
robot_status = {
    'state': 'initializing',
    'fps': 0.0,
    'frame_count': 0,
    'is_waving': False,
    'gesture': None,
    'wave_position': None,
    'motor_speed_left': 0.0,
    'motor_speed_right': 0.0,
    'initialized': False,
    'initialization_status': 'Starting robot...'
}

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Science Robot Monitor</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        * { box-sizing: border-box; }
        body { 
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif; 
            margin: 0; 
            padding: 10px; 
            background: #1a1a1a; 
            color: #fff; 
        }
        .container { 
            max-width: 1200px; 
            margin: 0 auto; 
        }
        .header { 
            text-align: center; 
            margin-bottom: 20px; 
        }
        .header h1 { 
            margin: 10px 0; 
            font-size: 24px; 
        }
        .status-panel { 
            background: #2a2a2a; 
            padding: 15px; 
            border-radius: 8px; 
            margin-bottom: 20px; 
            display: flex;
            flex-wrap: wrap;
            justify-content: space-around;
        }
        .status-item { 
            margin: 10px; 
            text-align: center;
            min-width: 100px;
        }
        .status-label { 
            color: #aaa; 
            font-size: 12px; 
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .status-value { 
            color: #0f0; 
            font-size: 20px; 
            font-weight: bold; 
            margin-top: 5px;
        }
        .status-value.warning { color: #ff0; }
        .status-value.error { color: #f00; }
        .video-container { 
            text-align: center; 
            background: #000; 
            padding: 10px; 
            border-radius: 8px; 
            margin-bottom: 20px;
            min-height: 400px;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        img { 
            max-width: 100%; 
            height: auto; 
            border: 2px solid #444; 
            border-radius: 4px;
        }
        .loading-screen {
            text-align: center;
            padding: 60px 20px;
            color: #fff;
            width: 100%;
        }
        .loading-spinner {
            width: 80px;
            height: 80px;
            border: 8px solid #333;
            border-top: 8px solid #0f0;
            border-radius: 50%;
            animation: spin 1s linear infinite;
            margin: 0 auto 30px;
        }
        @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
        }
        .loading-title {
            font-size: 32px;
            font-weight: bold;
            margin-bottom: 20px;
            color: #0f0;
        }
        .loading-message {
            font-size: 20px;
            margin-bottom: 10px;
            color: #fff;
        }
        .loading-details {
            font-size: 16px;
            color: #aaa;
            margin-top: 10px;
        }
        .loading-progress {
            width: 100%;
            max-width: 400px;
            height: 8px;
            background: #333;
            border-radius: 4px;
            margin: 20px auto;
            overflow: hidden;
        }
        .loading-progress-bar {
            height: 100%;
            background: linear-gradient(90deg, #0f0, #0ff);
            width: 0%;
            animation: progress 2s ease-in-out infinite;
        }
        @keyframes progress {
            0% { width: 0%; }
            50% { width: 70%; }
            100% { width: 100%; }
        }
        .controls { 
            text-align: center; 
            margin-top: 20px; 
        }
        button { 
            padding: 12px 24px; 
            margin: 5px; 
            font-size: 16px; 
            cursor: pointer; 
            border: none;
            border-radius: 6px;
            font-weight: bold;
        }
        .emergency { 
            background: #f00; 
            color: white; 
        }
        .emergency:hover {
            background: #c00;
        }
        .refresh { 
            background: #444; 
            color: white; 
        }
        .refresh:hover {
            background: #666;
        }
        @media (max-width: 768px) {
            .status-panel {
                flex-direction: column;
            }
            .status-item {
                width: 100%;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🤖 Science Robot v2.0 Monitor</h1>
        </div>
        
        <div class="status-panel">
            <div class="status-item">
                <div class="status-label">Status</div>
                <div class="status-value" id="initStatus">Initializing...</div>
            </div>
            <div class="status-item">
                <div class="status-label">State</div>
                <div class="status-value" id="state">-</div>
            </div>
            <div class="status-item">
                <div class="status-label">FPS</div>
                <div class="status-value" id="fps">-</div>
            </div>
            <div class="status-item">
                <div class="status-label">Frames</div>
                <div class="status-value" id="frames">-</div>
            </div>
            <div class="status-item">
                <div class="status-label">Waving</div>
                <div class="status-value" id="waving">-</div>
            </div>
            <div class="status-item">
                <div class="status-label">Gesture</div>
                <div class="status-value" id="gesture">-</div>
            </div>
            <div class="status-item">
                <div class="status-label">Motor L</div>
                <div class="status-value" id="motorL">-</div>
            </div>
            <div class="status-item">
                <div class="status-label">Motor R</div>
                <div class="status-value" id="motorR">-</div>
            </div>
            <div class="status-item">
                <div class="status-label">Collision Risk</div>
                <div class="status-value" id="collisionRisk">-</div>
            </div>
            <div class="status-item">
                <div class="status-label">Distance</div>
                <div class="status-value" id="distance">-</div>
            </div>
        </div>
        
        <div class="video-container" id="videoContainer">
            <div id="loadingMessage" class="loading-screen">
                <div class="loading-spinner"></div>
                <div class="loading-title">🤖 Science Robot Starting Up!</div>
                <div class="loading-message" id="loadingText">Please wait while the robot initializes...</div>
                <div class="loading-details" id="loadingDetails">This may take a few moments</div>
                <div class="loading-progress">
                    <div class="loading-progress-bar"></div>
                </div>
                <div class="loading-details" style="margin-top: 30px; font-size: 14px;">
                    💡 Tip: The robot is loading its camera, sensors, and AI brain!
                </div>
            </div>
            <img src="/video_feed" alt="Camera Feed" id="video" style="display: none;">
        </div>
        
        <div class="controls">
            <button onclick="emergencyStop()" class="emergency">🛑 Emergency Stop</button>
            <button onclick="quitRobot()" class="refresh">⏹️ Quit</button>
            <button onclick="location.reload()" class="refresh">🔄 Refresh</button>
        </div>
    </div>
    
    <script>
        // Show loading screen immediately on page load
        window.addEventListener('load', function() {
            const loadingMsg = document.getElementById('loadingMessage');
            const video = document.getElementById('video');
            loadingMsg.style.display = 'flex';
            video.style.display = 'none';
        });
        
        function updateStatus() {
            fetch('/status')
                .then(r => r.json())
                .then(data => {
                    // Show/hide loading message based on initialization
                    const initialized = data.initialized || false;
                    const loadingMsg = document.getElementById('loadingMessage');
                    const video = document.getElementById('video');
                    
                    if (initialized) {
                        // Robot is ready - hide loading, show video
                        loadingMsg.style.display = 'none';
                        video.style.display = 'block';
                    } else {
                        // Still loading - show loading screen with status
                        loadingMsg.style.display = 'flex';
                        video.style.display = 'none';
                        
                        const statusText = data.initialization_status || 'Initializing robot...';
                        document.getElementById('loadingText').textContent = statusText;
                        
                        // Add more friendly details based on status
                        let details = 'This may take a few moments';
                        if (statusText.includes('web server') || statusText.includes('Starting')) {
                            details = 'Starting up the robot...';
                        } else if (statusText.includes('ROS') || statusText.includes('brain')) {
                            details = 'Connecting to robot brain...';
                        } else if (statusText.includes('Camera') || statusText.includes('camera')) {
                            details = 'Starting camera vision system...';
                        } else if (statusText.includes('Treat') || statusText.includes('treat')) {
                            details = 'Preparing treat dispenser...';
                        } else if (statusText.includes('Ready') || statusText.includes('ready')) {
                            details = 'Almost there! Final checks...';
                        } else if (statusText.includes('FAILED') || statusText.includes('failed')) {
                            details = 'Something went wrong. Please check the logs.';
                        } else if (statusText.includes('systems') || statusText.includes('Loading')) {
                            details = 'Loading AI brain and sensors...';
                        }
                        document.getElementById('loadingDetails').textContent = details;
                    }
                    
                    // Update status
                    document.getElementById('initStatus').textContent = initialized ? 'Ready' : 'Initializing...';
                    document.getElementById('initStatus').className = 'status-value ' + (initialized ? '' : 'warning');
                    document.getElementById('state').textContent = data.state || '-';
                    document.getElementById('fps').textContent = initialized ? (data.fps || 0).toFixed(1) : '-';
                    document.getElementById('frames').textContent = data.frame_count || '0';
                    document.getElementById('waving').textContent = initialized && data.is_waving ? 'Yes' : '-';
                    document.getElementById('waving').className = 'status-value ' + (data.is_waving ? 'warning' : '');
                    document.getElementById('gesture').textContent = initialized ? (data.gesture || 'None') : '-';
                    document.getElementById('motorL').textContent = initialized ? (data.motor_speed_left || 0).toFixed(2) : '-';
                    document.getElementById('motorR').textContent = initialized ? (data.motor_speed_right || 0).toFixed(2) : '-';
                    
                    // Collision avoidance status
                    if (data.collision_risk) {
                        const risk = data.collision_risk.risk_level || 'none';
                        const riskEl = document.getElementById('collisionRisk');
                        riskEl.textContent = risk.toUpperCase();
                        riskEl.className = 'status-value ' + 
                            (risk === 'emergency' ? 'error' : 
                             risk === 'warning' ? 'warning' : '');
                        
                        const distance = data.collision_risk.distance;
                        document.getElementById('distance').textContent = 
                            distance ? distance.toFixed(2) + 'm' : '-';
                    } else {
                        document.getElementById('collisionRisk').textContent = '-';
                        document.getElementById('collisionRisk').className = 'status-value';
                        document.getElementById('distance').textContent = '-';
                    }
                })
                .catch(err => console.error('Status update error:', err));
        }
        
        function emergencyStop() {
            fetch('/emergency_stop', {method: 'POST'})
                .then(() => {
                    document.getElementById('initStatus').textContent = 'Emergency Stop';
                    document.getElementById('initStatus').className = 'status-value error';
                })
                .catch(err => console.error('Emergency stop error:', err));
        }
        
        function quitRobot() {
            if (confirm('Are you sure you want to quit the robot?')) {
                fetch('/quit', {method: 'POST'})
                    .then(() => {
                        alert('Quit command sent. Robot will shut down.');
                        setTimeout(() => location.reload(), 2000);
                    })
                    .catch(err => alert('Error: ' + err));
            }
        }
        
        // Update status every second
        setInterval(updateStatus, 1000);
        updateStatus();
        
        // Handle video errors
        document.getElementById('video').onerror = function() {
            this.src = '/video_feed?t=' + new Date().getTime();
        };
    </script>
</body>
</html>
"""

app = Flask(__name__)

def generate_frames():
    """Generate MJPEG frames from camera"""
    global latest_frame, frame_lock
    while True:
        with frame_lock:
            if latest_frame is not None:
                frame = latest_frame.copy()
            else:
                frame = None
        
        if frame is not None:
            # Encode frame as JPEG with quality setting for web
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ret:
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        else:
            # Send a placeholder frame if no camera feed
            placeholder = cv2.zeros((480, 640, 3), dtype=cv2.uint8)
            cv2.putText(placeholder, 'Waiting for camera...', (150, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            ret, buffer = cv2.imencode('.jpg', placeholder)
            if ret:
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        time.sleep(0.033)  # ~30 FPS for web stream

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/status')
def status():
    return jsonify(robot_status)

@app.route('/emergency_stop', methods=['POST'])
def emergency_stop():
    global robot_controller
    try:
        if robot_controller is None:
            logger.warning("Emergency stop requested but robot_controller is None")
            return jsonify({'status': 'error', 'message': 'Robot controller not initialized yet'}), 503
        
        # Call emergency stop on motor controller
        robot_controller.motor_controller.emergency_stop()
        robot_controller.state = 'idle'
        logger.warning("EMERGENCY STOP triggered via web interface")
        rospy.logwarn("EMERGENCY STOP triggered via web interface")
        return jsonify({'status': 'stopped', 'message': 'Emergency stop activated'})
    except Exception as e:
        logger.error(f"Error in emergency stop: {e}")
        import traceback
        logger.error(traceback.format_exc())
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/quit', methods=['POST'])
def quit():
    """Quit the robot application gracefully"""
    global robot_controller
    if robot_controller:
        try:
            logger.info("Quit command received via web interface")
            robot_controller.running = False
            # Signal ROS to shutdown
            rospy.signal_shutdown("Quit requested via web interface")
            return jsonify({'status': 'quitting', 'message': 'Robot shutting down...'})
        except Exception as e:
            logger.error(f"Error in quit: {e}")
            return jsonify({'status': 'error', 'message': str(e)}), 500
    return jsonify({'status': 'error', 'message': 'Robot controller not available'}), 503

def update_frame(frame):
    """Update latest frame for web stream"""
    global latest_frame, frame_lock
    with frame_lock:
        latest_frame = frame

def set_initialization_status(status_text):
    """Update initialization status message"""
    global robot_status, robot_initialized
    robot_status['initialization_status'] = status_text
    robot_status['initialized'] = False

def set_robot_initialized():
    """Mark robot as initialized"""
    global robot_status, robot_initialized
    robot_initialized = True
    robot_status['initialized'] = True
    robot_status['initialization_status'] = 'Robot ready!'

def update_status(state, fps, frame_count, is_waving, gesture, wave_position=None, 
                  motor_speed_left=0.0, motor_speed_right=0.0, collision_risk=None):
    """Update robot status for web display"""
    global robot_status, robot_initialized
    robot_status = {
        'state': state,
        'fps': fps,
        'frame_count': frame_count,
        'is_waving': is_waving,
        'gesture': gesture,
        'wave_position': wave_position,
        'motor_speed_left': motor_speed_left,
        'motor_speed_right': motor_speed_right,
        'collision_risk': collision_risk,
        'initialized': robot_initialized,
        'initialization_status': robot_status.get('initialization_status', 'Ready')
    }

def set_robot_controller(controller):
    """Set the robot controller reference for web server endpoints"""
    global robot_controller
    robot_controller = controller
    logger.info("Robot controller reference set in web server")

def start_web_server(controller, port=5000, host='0.0.0.0'):
    """Start web server in background thread"""
    global robot_controller
    robot_controller = controller
    
    def run_server():
        try:
            # Suppress Flask startup messages
            log = logging.getLogger('werkzeug')
            log.setLevel(logging.ERROR)
            app.run(host=host, port=port, threaded=True, debug=False, use_reloader=False)
        except Exception as e:
            logger.error(f"Web server error: {e}")
    
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()
    logger.info(f"Web server started on http://{host}:{port}")
    rospy.loginfo(f"Web dashboard available at http://{host}:{port}")
    return server_thread

