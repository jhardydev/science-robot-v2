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
import time
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

# Get project root by searching for run-with-web.sh file
def find_project_root():
    """Find the project root by looking for run-with-web.sh"""
    # Start from script location
    script_dir = Path(__file__).resolve().parent
    current = script_dir
    
    # Walk up the directory tree looking for run-with-web.sh
    for _ in range(6):  # Max 6 levels up
        run_script = current / 'run-with-web.sh'
        if run_script.exists():
            logger.info(f"Found project root: {current} (run-with-web.sh exists)")
            return current.resolve()
        current = current.parent
    
    # Fallback: try common locations
    potential_roots = [
        Path('/home/duckie/science-robot-v2'),
        Path.cwd(),
        Path.home() / 'science-robot-v2',
    ]
    
    for root in potential_roots:
        root_path = Path(root).resolve()
        if (root_path / 'run-with-web.sh').exists():
            logger.info(f"Found project root via fallback: {root_path}")
            return root_path
    
    # Last resort: use calculated path
    calculated_root = script_dir.parent.parent.parent.parent
    logger.warning(f"Could not find run-with-web.sh, using calculated path: {calculated_root}")
    return calculated_root.resolve()

PROJECT_ROOT = find_project_root()
logger.info(f"Project root determined: {PROJECT_ROOT}")

class DockerContainerManager:
    """Manages robot Docker container lifecycle"""
    
    def __init__(self):
        self.project_root = PROJECT_ROOT
        self.container_image = ROBOT_CONTAINER_IMAGE
        self.run_script = self.project_root / ROBOT_SCRIPT_NAME
        
        # Log initialization info
        logger.info(f"DockerContainerManager initialized:")
        logger.info(f"  Project root: {self.project_root}")
        logger.info(f"  Run script exists: {self.run_script.exists()}")
        logger.info(f"  Container image: {self.container_image}")
    
    def is_robot_running(self):
        """Check if robot Docker container is running"""
        try:
            # First, try to find by container name (science-robot-robot1)
            robot_name = os.getenv('VEHICLE_NAME', 'robot1')
            container_name = f'science-robot-{robot_name}'
            
            # Check for running containers by name
            result = subprocess.run(
                ['docker', 'ps', '--filter', f'name={container_name}', '--format', '{{.ID}}|{{.Status}}'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5
            )
            
            if result.returncode == 0 and result.stdout.strip():
                output = result.stdout.strip().decode('utf-8')
                parts = output.split('|', 1)
                container_id = parts[0]
                status_info = parts[1] if len(parts) > 1 else ''
                return True, container_id, status_info
            
            # Fallback: Check for running containers using the robot image
            result = subprocess.run(
                ['docker', 'ps', '--filter', f'ancestor={self.container_image}', '--format', '{{.ID}}|{{.Status}}'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5
            )
            
            if result.returncode == 0 and result.stdout.strip():
                output = result.stdout.strip().decode('utf-8')
                # Handle multiple containers (take first one)
                lines = output.split('\n')
                if lines and lines[0].strip():
                    parts = lines[0].split('|', 1)
                    container_id = parts[0]
                    status_info = parts[1] if len(parts) > 1 else ''
                    return True, container_id, status_info
            
            return False, None, None
            
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            logger.error(f"Error checking Docker container status: {e}")
            return False, None, None
        except Exception as e:
            logger.error(f"Unexpected error checking container: {e}")
            return False, None, None
    
    def start_robot(self):
        """Start robot Docker container directly (detached mode)"""
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
            
            # Check if Docker daemon is running
            docker_ps_check = subprocess.run(['docker', 'ps'], 
                                           stdout=subprocess.PIPE,
                                           stderr=subprocess.PIPE,
                                           timeout=5)
            if docker_ps_check.returncode != 0:
                return False, "Docker daemon is not running. Start Docker service first."
            
            # Get environment variables
            robot_name = os.getenv('VEHICLE_NAME', 'robot1')
            ros_master = os.getenv('ROS_MASTER_URI', 'http://localhost:11311')
            log_dir = os.getenv('LOG_DIR', '/tmp/science-robot-logs')
            web_port = os.getenv('WEB_SERVER_PORT', '5000')
            
            # Create log directory
            os.makedirs(log_dir, exist_ok=True)
            
            # Start Docker container directly in detached mode (not using run-with-web.sh)
            # This avoids the -it flag issue and runs in background properly
            docker_log_file = os.path.join(log_dir, 'docker-startup.log')
            
            with open(docker_log_file, 'a') as log_file:
                log_file.write(f"\n{'='*60}\n")
                log_file.write(f"Starting robot container at {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                log_file.write(f"Robot: {robot_name}, ROS Master: {ros_master}\n")
                log_file.write(f"{'='*60}\n")
            
            # Build docker run command (detached mode, no -it flags)
            docker_cmd = [
                'docker', 'run', '-d', '--rm',  # Detached mode, remove on exit
                '--network', 'host',
                '--name', f'science-robot-{robot_name}',  # Give it a name for easier management
                '-e', f'ROS_MASTER_URI={ros_master}',
                '-e', f'VEHICLE_NAME={robot_name}',
                '-e', 'LOG_DIR=/code/logs',
                '-e', 'ENABLE_WEB_SERVER=true',
                '-e', f'WEB_SERVER_PORT={web_port}',
                '-e', 'DISPLAY_OUTPUT=false',
                '-v', f'{log_dir}:/code/logs',
                '-v', '/usr/lib/python3/dist-packages/vpi:/host/usr/lib/python3/dist-packages/vpi:ro',
                '-v', '/usr/lib/aarch64-linux-gnu:/host/usr/lib/aarch64-linux-gnu:ro',
                '-p', f'{web_port}:{web_port}',
                self.container_image
            ]
            
            # Run Docker command
            result = subprocess.run(
                docker_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=30,
                cwd=str(self.project_root)
            )
            
            # Log output
            with open(docker_log_file, 'a') as log_file:
                log_file.write(f"Command: {' '.join(docker_cmd)}\n")
                if result.stdout:
                    log_file.write(f"STDOUT: {result.stdout.decode('utf-8')}\n")
                if result.stderr:
                    log_file.write(f"STDERR: {result.stderr.decode('utf-8')}\n")
                log_file.write(f"Return code: {result.returncode}\n")
            
            if result.returncode != 0:
                error_msg = result.stderr.decode('utf-8').strip()
                logger.error(f"Docker run failed: {error_msg}")
                with open(docker_log_file, 'a') as log_file:
                    log_file.write(f"ERROR: {error_msg}\n")
                return False, f"Failed to start Docker container: {error_msg}"
            
            container_id = result.stdout.decode('utf-8').strip().split('\n')[0]  # Take first line only
            
            # Wait a moment and verify container is running
            time.sleep(2)
            is_running_verify, verify_container_id, status = self.is_robot_running()
            
            if is_running_verify:
                logger.info(f"Started robot container: {container_id[:12]}")
                return True, f"Robot container started successfully! Container ID: {container_id[:12]}"
            else:
                # Container might have started but then stopped immediately
                logger.warning(f"Container {container_id[:12]} started but not found running")
                # Check container logs
                logs_result = subprocess.run(
                    ['docker', 'logs', container_id[:12]],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=5
                )
                if logs_result.stdout:
                    log_output = logs_result.stdout.decode('utf-8')[-500:]  # Last 500 chars
                    with open(docker_log_file, 'a') as log_file:
                        log_file.write(f"\nContainer logs:\n{log_output}\n")
                
                return False, f"Container started but stopped immediately. Check logs: {docker_log_file}"
            
        except subprocess.TimeoutExpired:
            logger.error("Timeout starting Docker container")
            return False, "Timeout starting container (took too long)"
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
            # Try stopping by name first (more reliable)
            robot_name = os.getenv('VEHICLE_NAME', 'robot1')
            container_name = f'science-robot-{robot_name}'
            
            # Try by name first
            result = subprocess.run(
                ['docker', 'stop', container_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=30
            )
            
            # If that failed, try by container ID
            if result.returncode != 0 and container_id:
                result = subprocess.run(
                    ['docker', 'stop', container_id],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=30
                )
            
            if result.returncode == 0:
                logger.info(f"Stopped robot container: {container_id[:12] if container_id else container_name}")
                return True, f"Robot container stopped successfully"
            else:
                error_msg = result.stderr.decode('utf-8').strip()
                logger.error(f"Failed to stop container: {error_msg}")
                return False, f"Failed to stop container: {error_msg}"
                
        except subprocess.TimeoutExpired:
            return False, "Timeout stopping container (took too long)"
        except Exception as e:
            logger.error(f"Error stopping robot: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False, f"Failed to stop robot: {str(e)}"
    
    def shutdown_robot(self):
        """Shutdown robot (same as stop, but with ROS shutdown attempt)"""
        is_running, container_id, _ = self.is_robot_running()
        
        if not is_running:
            return False, "Robot container is not running"
        
        try:
            # Try ROS shutdown command inside container first
            robot_name = os.getenv('VEHICLE_NAME', 'robot1')
            container_name = f'science-robot-{robot_name}'
            
            # Try by name first, then by ID
            try:
                subprocess.run(
                    ['docker', 'exec', container_name, 'rosservice', 'call', '/shutdown'],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=5
                )
            except:
                try:
                    if container_id:
                        subprocess.run(
                            ['docker', 'exec', container_id, 'rosservice', 'call', '/shutdown'],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            timeout=5
                        )
                except:
                    pass  # Ignore if ROS service not available or container not found
            
            # Stop container
            return self.stop_robot()
            
        except Exception as e:
            logger.error(f"Error shutting down robot: {e}")
            import traceback
            logger.error(traceback.format_exc())
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
            <button id="startBtn" class="btn-start">▶️ Start Robot</button>
            <button id="stopBtn" class="btn-stop" disabled>⏸️ Stop Robot</button>
            <button id="shutdownBtn" class="btn-shutdown" disabled>🔄 Shutdown Robot</button>
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
            console.log('startRobot() function called');
            
            if (!confirm('Start the robot? This will launch the Docker container.')) {
                console.log('User cancelled start');
                return;
            }
            
            console.log('User confirmed, proceeding with start...');
            
            const startBtn = document.getElementById('startBtn');
            if (!startBtn) {
                console.error('Start button element not found!');
                alert('Error: Start button not found. Please refresh the page.');
                return;
            }
            
            startBtn.disabled = true;
            startBtn.textContent = 'Starting...';
            
            console.log('Sending POST request to /start');
            
            fetch('/start', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                }
            })
                .then(r => {
                    console.log('Response status:', r.status);
                    if (!r.ok) {
                        throw new Error(`HTTP ${r.status}: ${r.statusText}`);
                    }
                    return r.json();
                })
                .then(data => {
                    console.log('Response data:', data);
                    startBtn.disabled = false;
                    startBtn.textContent = '▶️ Start Robot';
                    
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
                    console.error('Error starting robot:', err);
                    startBtn.disabled = false;
                    startBtn.textContent = '▶️ Start Robot';
                    alert('Error starting robot: ' + err.message);
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
        
        // Set up event listeners when page loads
        document.addEventListener('DOMContentLoaded', function() {
            console.log('DOM loaded, setting up event listeners');
            
            const startBtn = document.getElementById('startBtn');
            const stopBtn = document.getElementById('stopBtn');
            const shutdownBtn = document.getElementById('shutdownBtn');
            
            if (startBtn) {
                startBtn.addEventListener('click', function(e) {
                    e.preventDefault();
                    console.log('Start button clicked!');
                    startRobot();
                });
                console.log('Start button event listener attached');
            } else {
                console.error('Start button not found!');
            }
            
            if (stopBtn) {
                stopBtn.addEventListener('click', function(e) {
                    e.preventDefault();
                    console.log('Stop button clicked!');
                    stopRobot();
                });
            }
            
            if (shutdownBtn) {
                shutdownBtn.addEventListener('click', function(e) {
                    e.preventDefault();
                    console.log('Shutdown button clicked!');
                    shutdownRobot();
                });
            }
            
            // Initial status update
            updateStatus();
        });
        
        // Update status every 3 seconds
        setInterval(updateStatus, 3000);
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    """Main control page"""
    logger.info("Serving main page")
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

@app.route('/start', methods=['POST', 'OPTIONS'])
def start():
    """Start robot container"""
    # Handle CORS preflight
    if request.method == 'OPTIONS':
        response = jsonify({'status': 'ok'})
        response.headers.add('Access-Control-Allow-Origin', '*')
        response.headers.add('Access-Control-Allow-Methods', 'POST, OPTIONS')
        response.headers.add('Access-Control-Allow-Headers', 'Content-Type')
        return response
    
    logger.info("Received /start request")
    logger.info(f"Request headers: {dict(request.headers)}")
    
    try:
        success, message = container_manager.start_robot()
        logger.info(f"Start robot result: success={success}, message={message}")
        response = jsonify({
            'success': success,
            'message': message
        })
        response.headers.add('Access-Control-Allow-Origin', '*')
        return response
    except Exception as e:
        logger.error(f"Error in /start endpoint: {e}")
        import traceback
        logger.error(traceback.format_exc())
        response = jsonify({
            'success': False,
            'message': f"Internal error: {str(e)}"
        })
        response.headers.add('Access-Control-Allow-Origin', '*')
        return response, 500

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
        # Enable Flask request logging for debugging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.INFO)  # Changed from ERROR to INFO to see requests
        log.info(f"Flask request logging enabled")
        
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

