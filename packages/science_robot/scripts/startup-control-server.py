#!/usr/bin/env python3
"""
Standalone Robot Startup Control Web Server
Runs on host system (port 5001) to control robot Docker container
This is a lightweight server with minimal dependencies (Flask + psutil only)

TO REMOVE: Delete this file and the systemd service to undo
"""
import os
import sys
import subprocess
import logging
import json
from pathlib import Path

# Try to import Flask
try:
    from flask import Flask, jsonify, render_template_string, request
    FLASK_AVAILABLE = True
except ImportError:
    FLASK_AVAILABLE = False
    print("ERROR: Flask is not installed. Run: pip3 install flask")
    sys.exit(1)

# Try to import psutil (optional - for process detection)
try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [startup-control] - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Create Flask app
app = Flask(__name__)

# Configuration
ROBOT_CONTAINER_IMAGE = 'science-robot-v2:latest'
ROBOT_SCRIPT_NAME = 'run-with-web.sh'
DEFAULT_PORT = 5001
DEFAULT_HOST = '0.0.0.0'

# Get project root (assume script is in packages/science_robot/scripts/)
SCRIPT_DIR = Path(__file__).parent
PROJECT_ROOT = SCRIPT_DIR.parent.parent.parent.parent

class DockerContainerManager:
    """Manages robot Docker container lifecycle"""
    
    def __init__(self):
        self.project_root = PROJECT_ROOT
        self.container_image = ROBOT_CONTAINER_IMAGE
        self.run_script = self.project_root / ROBOT_SCRIPT_NAME
    
    def is_robot_running(self):
        """Check if robot Docker container is running"""
        try:
            # Check for running containers using the robot image
            result = subprocess.run(
                ['docker', 'ps', '--filter', f'ancestor={self.container_image}', '--format', '{{.ID}}'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5
            )
            
            if result.returncode == 0 and result.stdout.strip():
                container_id = result.stdout.strip().decode('utf-8')
                # Get container name/status
                info_result = subprocess.run(
                    ['docker', 'ps', '--filter', f'id={container_id}', '--format', '{{.Names}}|{{.Status}}'],
                    stdout=subprocess.PIPE,
                    timeout=5
                )
                status_info = info_result.stdout.decode('utf-8').strip() if info_result.returncode == 0 else ''
                return True, container_id, status_info
            return False, None, None
            
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            logger.error(f"Error checking Docker container status: {e}")
            return False, None, None
        except Exception as e:
            logger.error(f"Unexpected error checking container: {e}")
            return False, None, None
    
    def start_robot(self):
        """Start robot via Docker (using run-with-web.sh)"""
        is_running, container_id, _ = self.is_robot_running()
        if is_running:
            return False, f"Robot container is already running (ID: {container_id[:12]})"
        
        try:
            # Check if Docker is available
            docker_check = subprocess.run(['docker', '--version'], 
                                        stdout=subprocess.PIPE, 
                                        stderr=subprocess.PIPE,
                                        timeout=5)
            if docker_check.returncode != 0:
                return False, "Docker is not available or not running"
            
            # Check if run script exists
            if not self.run_script.exists():
                return False, f"Robot startup script not found: {self.run_script}"
            
            # Get environment variables
            robot_name = os.getenv('VEHICLE_NAME', 'robot1')
            ros_master = os.getenv('ROS_MASTER_URI', 'http://localhost:11311')
            log_dir = os.getenv('LOG_DIR', '/tmp/science-robot-logs')
            web_port = os.getenv('WEB_SERVER_PORT', '5000')
            
            # Create log directory
            os.makedirs(log_dir, exist_ok=True)
            
            # Prepare environment
            env = os.environ.copy()
            env.update({
                'VEHICLE_NAME': robot_name,
                'ROS_MASTER_URI': ros_master,
                'LOG_DIR': log_dir,
                'WEB_SERVER_PORT': web_port,
                'ENABLE_WEB_SERVER': 'true',
            })
            
            # Start robot in background (detached)
            # Use nohup or screen/tmux for true background execution
            # For now, use subprocess with start_new_session
            process = subprocess.Popen(
                ['bash', str(self.run_script)],
                env=env,
                cwd=str(self.project_root),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            
            logger.info(f"Started robot container via {self.run_script} (PID: {process.pid})")
            return True, f"Robot container starting (PID: {process.pid}). Please wait a few seconds..."
            
        except Exception as e:
            logger.error(f"Error starting robot: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False, f"Failed to start robot: {str(e)}"
    
    def stop_robot(self):
        """Stop robot Docker container gracefully"""
        is_running, container_id, status = self.is_robot_running()
        
        if not is_running:
            return False, "Robot container is not running"
        
        try:
            # Stop the container
            result = subprocess.run(
                ['docker', 'stop', container_id],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=30
            )
            
            if result.returncode == 0:
                logger.info(f"Stopped robot container: {container_id[:12]}")
                return True, f"Robot container stopped successfully"
            else:
                error_msg = result.stderr.decode('utf-8').strip()
                return False, f"Failed to stop container: {error_msg}"
                
        except subprocess.TimeoutExpired:
            return False, "Timeout stopping container (took too long)"
        except Exception as e:
            logger.error(f"Error stopping robot: {e}")
            return False, f"Failed to stop robot: {str(e)}"
    
    def shutdown_robot(self):
        """Shutdown robot (same as stop, but with ROS shutdown attempt)"""
        is_running, container_id, _ = self.is_robot_running()
        
        if not is_running:
            return False, "Robot container is not running"
        
        try:
            # Try ROS shutdown command inside container first
            try:
                subprocess.run(
                    ['docker', 'exec', container_id, 'rosservice', 'call', '/shutdown'],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=5
                )
            except:
                pass  # Ignore if ROS service not available
            
            # Stop container
            return self.stop_robot()
            
        except Exception as e:
            logger.error(f"Error shutting down robot: {e}")
            return False, f"Failed to shutdown robot: {str(e)}"

# Global container manager
container_manager = DockerContainerManager()

# Simple HTML interface
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Robot Startup Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        * { box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background: #1a1a1a;
            color: #fff;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
        }
        .container {
            background: #2a2a2a;
            border-radius: 12px;
            padding: 30px;
            max-width: 600px;
            width: 100%;
            box-shadow: 0 4px 20px rgba(0,0,0,0.3);
        }
        h1 {
            margin-top: 0;
            color: #fff;
            text-align: center;
        }
        .status {
            background: #333;
            padding: 15px;
            border-radius: 8px;
            margin-bottom: 20px;
            text-align: center;
        }
        .status-value {
            font-size: 18px;
            font-weight: bold;
            margin-top: 10px;
        }
        .status-value.running {
            color: #0a0;
        }
        .status-value.stopped {
            color: #a00;
        }
        .buttons {
            display: flex;
            flex-direction: column;
            gap: 10px;
        }
        button {
            padding: 15px 30px;
            font-size: 16px;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            font-weight: bold;
            transition: background 0.2s;
        }
        button:hover:not(:disabled) {
            opacity: 0.9;
        }
        button:disabled {
            opacity: 0.5;
            cursor: not-allowed;
        }
        .btn-start {
            background: #0a0;
            color: white;
        }
        .btn-stop {
            background: #fa0;
            color: white;
        }
        .btn-shutdown {
            background: #a00;
            color: white;
        }
        .info {
            margin-top: 20px;
            padding: 15px;
            background: #333;
            border-radius: 8px;
            font-size: 14px;
            line-height: 1.6;
        }
        .info a {
            color: #4af;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Robot Startup Control</h1>
        
        <div class="status">
            <div>Robot Status:</div>
            <div class="status-value" id="robotStatus">Checking...</div>
            <div id="statusDetails" style="font-size: 12px; margin-top: 5px; color: #aaa;"></div>
        </div>
        
        <div class="buttons">
            <button id="startBtn" class="btn-start" onclick="startRobot()">▶️ Start Robot</button>
            <button id="stopBtn" class="btn-stop" onclick="stopRobot()" disabled>⏸️ Stop Robot</button>
            <button id="shutdownBtn" class="btn-shutdown" onclick="shutdownRobot()" disabled>🔄 Shutdown Robot</button>
        </div>
        
        <div class="info">
            <strong>Information:</strong><br>
            • Port 5000: <a href="http://robot1.local:5000" target="_blank">Robot Web Interface</a> (when robot is running)<br>
            • Port 5001: This startup control page (always available)<br>
            • The robot runs in a Docker container and can be controlled from here
        </div>
    </div>
    
    <script>
        function updateStatus() {
            fetch('/status')
                .then(r => r.json())
                .then(data => {
                    const statusEl = document.getElementById('robotStatus');
                    const detailsEl = document.getElementById('statusDetails');
                    const startBtn = document.getElementById('startBtn');
                    const stopBtn = document.getElementById('stopBtn');
                    const shutdownBtn = document.getElementById('shutdownBtn');
                    
                    if (data.running) {
                        statusEl.textContent = 'Running';
                        statusEl.className = 'status-value running';
                        detailsEl.textContent = data.details || `Container: ${data.container_id || 'N/A'}`;
                        startBtn.disabled = true;
                        stopBtn.disabled = false;
                        shutdownBtn.disabled = false;
                    } else {
                        statusEl.textContent = 'Stopped';
                        statusEl.className = 'status-value stopped';
                        detailsEl.textContent = '';
                        startBtn.disabled = false;
                        stopBtn.disabled = true;
                        shutdownBtn.disabled = true;
                    }
                })
                .catch(err => {
                    console.error('Error checking status:', err);
                    document.getElementById('robotStatus').textContent = 'Error';
                    document.getElementById('robotStatus').className = 'status-value stopped';
                });
        }
        
        function startRobot() {
            if (!confirm('Start the robot? This will launch the Docker container.')) return;
            
            fetch('/start', {method: 'POST'})
                .then(r => r.json())
                .then(data => {
                    if (data.success) {
                        alert('Robot is starting! Please wait a few seconds...\n\n' + data.message);
                        setTimeout(() => {
                            updateStatus();
                            window.open('http://robot1.local:5000', '_blank');
                        }, 2000);
                    } else {
                        alert('Failed to start robot:\n' + data.message);
                    }
                    updateStatus();
                })
                .catch(err => {
                    alert('Error starting robot: ' + err);
                    updateStatus();
                });
        }
        
        function stopRobot() {
            if (!confirm('Stop the robot container?')) return;
            
            fetch('/stop', {method: 'POST'})
                .then(r => r.json())
                .then(data => {
                    if (data.success) {
                        alert('Robot stopped successfully!');
                    } else {
                        alert('Failed to stop robot:\n' + data.message);
                    }
                    updateStatus();
                })
                .catch(err => {
                    alert('Error stopping robot: ' + err);
                    updateStatus();
                });
        }
        
        function shutdownRobot() {
            if (!confirm('Shutdown the robot? This will stop the container and attempt ROS shutdown.')) return;
            
            fetch('/shutdown', {method: 'POST'})
                .then(r => r.json())
                .then(data => {
                    if (data.success) {
                        alert('Robot shutdown successfully!');
                    } else {
                        alert('Failed to shutdown robot:\n' + data.message);
                    }
                    updateStatus();
                })
                .catch(err => {
                    alert('Error shutting down robot: ' + err);
                    updateStatus();
                });
        }
        
        // Update status every 3 seconds
        setInterval(updateStatus, 3000);
        updateStatus();
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    """Main control page"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/status', methods=['GET'])
def status():
    """Get robot container status"""
    is_running, container_id, status_info = container_manager.is_robot_running()
    response = {
        'running': is_running,
        'container_id': container_id[:12] if container_id else None,
        'details': status_info
    }
    return jsonify(response)

@app.route('/start', methods=['POST'])
def start():
    """Start robot container"""
    success, message = container_manager.start_robot()
    return jsonify({
        'success': success,
        'message': message
    })

@app.route('/stop', methods=['POST'])
def stop():
    """Stop robot container"""
    success, message = container_manager.stop_robot()
    return jsonify({
        'success': success,
        'message': message
    })

@app.route('/shutdown', methods=['POST'])
def shutdown():
    """Shutdown robot"""
    success, message = container_manager.shutdown_robot()
    return jsonify({
        'success': success,
        'message': message
    })

def main():
    """Main entry point"""
    port = int(os.getenv('STARTUP_CONTROL_PORT', DEFAULT_PORT))
    host = os.getenv('STARTUP_CONTROL_HOST', DEFAULT_HOST)
    
    logger.info(f"Starting Robot Startup Control Server on http://{host}:{port}")
    logger.info(f"Project root: {PROJECT_ROOT}")
    logger.info(f"Container image: {ROBOT_CONTAINER_IMAGE}")
    
    try:
        # Suppress Flask startup messages
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        
        # Run Flask app
        app.run(host=host, port=port, debug=False, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        logger.info("Shutting down startup control server...")
    except Exception as e:
        logger.error(f"Error running startup control server: {e}")
        import traceback
        logger.error(traceback.format_exc())
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

