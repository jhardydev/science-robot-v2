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
robot_status = {
    'state': 'idle',
    'fps': 0.0,
    'frame_count': 0,
    'is_waving': False,
    'gesture': None,
    'wave_position': None,
    'motor_speed_left': 0.0,
    'motor_speed_right': 0.0
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
        }
        img { 
            max-width: 100%; 
            height: auto; 
            border: 2px solid #444; 
            border-radius: 4px;
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
        </div>
        
        <div class="video-container">
            <img src="/video_feed" alt="Camera Feed" id="video">
        </div>
        
        <div class="controls">
            <button onclick="emergencyStop()" class="emergency">🛑 Emergency Stop</button>
            <button onclick="location.reload()" class="refresh">🔄 Refresh</button>
        </div>
    </div>
    
    <script>
        function updateStatus() {
            fetch('/status')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('state').textContent = data.state || '-';
                    document.getElementById('fps').textContent = (data.fps || 0).toFixed(1);
                    document.getElementById('frames').textContent = data.frame_count || '0';
                    document.getElementById('waving').textContent = data.is_waving ? 'Yes' : 'No';
                    document.getElementById('waving').className = 'status-value ' + (data.is_waving ? 'warning' : '');
                    document.getElementById('gesture').textContent = data.gesture || 'None';
                    document.getElementById('motorL').textContent = (data.motor_speed_left || 0).toFixed(2);
                    document.getElementById('motorR').textContent = (data.motor_speed_right || 0).toFixed(2);
                })
                .catch(err => console.error('Status update error:', err));
        }
        
        function emergencyStop() {
            if (confirm('Are you sure you want to emergency stop?')) {
                fetch('/emergency_stop', {method: 'POST'})
                    .then(() => alert('Emergency stop sent!'))
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
    if robot_controller:
        try:
            robot_controller.motor_controller.emergency_stop()
            robot_controller.state = 'idle'
            logger.info("Emergency stop triggered via web interface")
            return jsonify({'status': 'stopped', 'message': 'Emergency stop activated'})
        except Exception as e:
            logger.error(f"Error in emergency stop: {e}")
            return jsonify({'status': 'error', 'message': str(e)}), 500
    return jsonify({'status': 'error', 'message': 'Robot controller not available'}), 503

def update_frame(frame):
    """Update latest frame for web stream"""
    global latest_frame, frame_lock
    with frame_lock:
        latest_frame = frame

def update_status(state, fps, frame_count, is_waving, gesture, wave_position=None, 
                  motor_speed_left=0.0, motor_speed_right=0.0):
    """Update robot status for web display"""
    global robot_status
    robot_status = {
        'state': state,
        'fps': fps,
        'frame_count': frame_count,
        'is_waving': is_waving,
        'gesture': gesture,
        'wave_position': wave_position,
        'motor_speed_left': motor_speed_left,
        'motor_speed_right': motor_speed_right
    }

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

