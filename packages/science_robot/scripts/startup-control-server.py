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
    script_dir = Path(__file__).resolve().parent
    current = script_dir
    
    for _ in range(6):
        run_script = current / 'run-with-web.sh'
        if run_script.exists():
            logger.info(f"Found project root: {current}")
            return current.resolve()
        current = current.parent
    
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
    
    calculated_root = script_dir.parent.parent.parent.parent
    logger.warning(f"Could not find run-with-web.sh, using calculated path: {calculated_root}")
    return calculated_root.resolve()

PROJECT_ROOT = find_project_root()
logger.info(f"Project root: {PROJECT_ROOT}")

class DockerContainerManager:
    """Manages robot Docker container lifecycle"""
    
    def __init__(self):
        self.project_root = PROJECT_ROOT
        self.container_image = ROBOT_CONTAINER_IMAGE
        self.run_script = self.project_root / ROBOT_SCRIPT_NAME
        logger.info(f"Container manager initialized - Project: {self.project_root}, Script exists: {self.run_script.exists()}")
    
    def is_robot_running(self):
        """Check if robot Docker container is running"""
        try:
            robot_name = os.getenv('VEHICLE_NAME', 'robot1')
            container_name = f'science-robot-{robot_name}'
            
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
            
            result = subprocess.run(
                ['docker', 'ps', '--filter', f'ancestor={self.container_image}', '--format', '{{.ID}}|{{.Status}}'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5
            )
            
            if result.returncode == 0 and result.stdout.strip():
                output = result.stdout.strip().decode('utf-8')
                lines = output.split('\n')
                if lines and lines[0].strip():
                    parts = lines[0].split('|', 1)
                    container_id = parts[0]
                    status_info = parts[1] if len(parts) > 1 else ''
                    return True, container_id, status_info
            
            return False, None, None
            
        except Exception as e:
            logger.error(f"Error checking container: {e}")
            return False, None, None
    
    def start_robot(self):
        """Start robot Docker container directly (detached mode)"""
        is_running, container_id, _ = self.is_robot_running()
        if is_running:
            return False, f"Robot is already running!"
        
        try:
            docker_check = subprocess.run(['docker', '--version'], 
                                        stdout=subprocess.PIPE, 
                                        stderr=subprocess.PIPE,
                                        timeout=5)
            if docker_check.returncode != 0:
                return False, "Docker is not available"
            
            docker_ps_check = subprocess.run(['docker', 'ps'], 
                                           stdout=subprocess.PIPE,
                                           stderr=subprocess.PIPE,
                                           timeout=5)
            if docker_ps_check.returncode != 0:
                return False, "Docker daemon is not running"
            
            robot_name = os.getenv('VEHICLE_NAME', 'robot1')
            ros_master = os.getenv('ROS_MASTER_URI', 'http://localhost:11311')
            log_dir = os.getenv('LOG_DIR', '/tmp/science-robot-logs')
            web_port = os.getenv('WEB_SERVER_PORT', '5000')
            
            os.makedirs(log_dir, exist_ok=True)
            docker_log_file = os.path.join(log_dir, 'docker-startup.log')
            
            with open(docker_log_file, 'a') as log_file:
                log_file.write(f"\n{'='*60}\n")
                log_file.write(f"Starting robot at {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            
            docker_cmd = [
                'docker', 'run', '-d', '--rm',
                '--network', 'host',
                '--name', f'science-robot-{robot_name}',
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
            
            result = subprocess.run(
                docker_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=30,
                cwd=str(self.project_root)
            )
            
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
                return False, f"Failed to start: {error_msg}"
            
            container_id = result.stdout.decode('utf-8').strip().split('\n')[0]
            
            time.sleep(2)
            is_running_verify, verify_container_id, status = self.is_robot_running()
            
            if is_running_verify:
                logger.info(f"Started robot container: {container_id[:12]}")
                return True, f"Robot is starting! Please wait a few seconds."
            else:
                logger.warning(f"Container {container_id[:12]} started but not found running")
                logs_result = subprocess.run(
                    ['docker', 'logs', container_id[:12]],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=5
                )
                if logs_result.stdout:
                    log_output = logs_result.stdout.decode('utf-8')[-500:]
                    with open(docker_log_file, 'a') as log_file:
                        log_file.write(f"\nContainer logs:\n{log_output}\n")
                
                return False, f"Robot started but stopped. Check logs: {docker_log_file}"
            
        except Exception as e:
            logger.error(f"Error starting robot: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False, f"Error: {str(e)}"
    
    def stop_robot(self):
        """Stop robot Docker container gracefully"""
        is_running, container_id, status = self.is_robot_running()
        
        if not is_running:
            return False, "Robot is not running"
        
        try:
            robot_name = os.getenv('VEHICLE_NAME', 'robot1')
            container_name = f'science-robot-{robot_name}'
            
            result = subprocess.run(
                ['docker', 'stop', container_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=30
            )
            
            if result.returncode != 0 and container_id:
                result = subprocess.run(
                    ['docker', 'stop', container_id],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=30
                )
            
            if result.returncode == 0:
                logger.info(f"Stopped robot container")
                return True, "Robot stopped successfully!"
            else:
                error_msg = result.stderr.decode('utf-8').strip()
                return False, f"Failed to stop: {error_msg}"
                
        except Exception as e:
            logger.error(f"Error stopping robot: {e}")
            return False, f"Error: {str(e)}"
    
    def shutdown_robot(self):
        """Shutdown robot"""
        is_running, container_id, _ = self.is_robot_running()
        
        if not is_running:
            return False, "Robot is not running"
        
        try:
            robot_name = os.getenv('VEHICLE_NAME', 'robot1')
            container_name = f'science-robot-{robot_name}'
            
            try:
                subprocess.run(
                    ['docker', 'exec', container_name, 'rosservice', 'call', '/shutdown'],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=5
                )
            except:
                pass
            
            return self.stop_robot()
            
        except Exception as e:
            logger.error(f"Error shutting down robot: {e}")
            return False, f"Error: {str(e)}"

container_manager = DockerContainerManager()

# Simple, bulletproof HTML template using single quotes to avoid issues
HTML_TEMPLATE = '''<!DOCTYPE html>
<html>
<head>
    <title>Robot Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { font-family: Arial, sans-serif; background: #1a1a1a; color: #fff; margin: 0; padding: 20px; display: flex; justify-content: center; align-items: center; min-height: 100vh; }
        .container { background: #2a2a2a; border-radius: 12px; padding: 30px; max-width: 500px; width: 100%; }
        h1 { text-align: center; margin-top: 0; font-size: 2em; }
        .status { background: #333; padding: 15px; border-radius: 8px; margin-bottom: 20px; text-align: center; }
        .status-value { font-size: 18px; font-weight: bold; margin-top: 10px; }
        .status-value.running { color: #0a0; }
        .status-value.stopped { color: #a00; }
        .buttons { display: flex; flex-direction: column; gap: 10px; }
        button { padding: 15px 30px; font-size: 18px; border: none; border-radius: 8px; cursor: pointer; font-weight: bold; }
        button:disabled { opacity: 0.5; cursor: not-allowed; }
        .btn-start { background: #0a0; color: white; }
        .btn-stop { background: #fa0; color: white; }
        .btn-shutdown { background: #a00; color: white; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Robot Control</h1>
        <div class="status">
            <div>Robot Status:</div>
            <div class="status-value" id="status">Checking...</div>
        </div>
        <div class="buttons">
            <button id="startBtn" class="btn-start">▶️ Start Robot</button>
            <button id="stopBtn" class="btn-stop" disabled>⏸️ Stop Robot</button>
            <button id="shutdownBtn" class="btn-shutdown" disabled>🔄 Shutdown Robot</button>
        </div>
    </div>
    <script>
    function updateStatus() {
        fetch('/status').then(function(r) { return r.json(); }).then(function(data) {
            var statusEl = document.getElementById('status');
            var startBtn = document.getElementById('startBtn');
            var stopBtn = document.getElementById('stopBtn');
            var shutdownBtn = document.getElementById('shutdownBtn');
            if (data.running) {
                statusEl.textContent = 'Running';
                statusEl.className = 'status-value running';
                startBtn.disabled = true;
                stopBtn.disabled = false;
                shutdownBtn.disabled = false;
            } else {
                statusEl.textContent = 'Stopped';
                statusEl.className = 'status-value stopped';
                startBtn.disabled = false;
                stopBtn.disabled = true;
                shutdownBtn.disabled = true;
            }
        }).catch(function(err) {
            document.getElementById('status').textContent = 'Error';
        });
    }
    function startRobot() {
        if (!confirm('Start the robot?')) return;
        var btn = document.getElementById('startBtn');
        btn.disabled = true;
        btn.textContent = 'Starting...';
        fetch('/start', {method: 'POST'}).then(function(r) { return r.json(); }).then(function(data) {
            btn.disabled = false;
            btn.textContent = '▶️ Start Robot';
            alert(data.success ? 'Robot is starting!' : 'Failed: ' + data.message);
            updateStatus();
        }).catch(function(err) {
            btn.disabled = false;
            btn.textContent = '▶️ Start Robot';
            alert('Error: ' + err.message);
            updateStatus();
        });
    }
    function stopRobot() {
        if (!confirm('Stop the robot?')) return;
        fetch('/stop', {method: 'POST'}).then(function(r) { return r.json(); }).then(function(data) {
            alert(data.success ? 'Robot stopped!' : 'Failed: ' + data.message);
            updateStatus();
        });
    }
    function shutdownRobot() {
        if (!confirm('Shutdown the robot?')) return;
        fetch('/shutdown', {method: 'POST'}).then(function(r) { return r.json(); }).then(function(data) {
            alert(data.success ? 'Robot shutdown!' : 'Failed: ' + data.message);
            updateStatus();
        });
    }
    function setup() {
        document.getElementById('startBtn').onclick = startRobot;
        document.getElementById('stopBtn').onclick = stopRobot;
        document.getElementById('shutdownBtn').onclick = shutdownRobot;
        updateStatus();
        setInterval(updateStatus, 3000);
    }
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', setup);
    } else {
        setup();
    }
    </script>
</body>
</html>'''

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
    if request.method == 'OPTIONS':
        response = jsonify({'status': 'ok'})
        response.headers.add('Access-Control-Allow-Origin', '*')
        return response
    
    logger.info("Received /start request")
    
    try:
        success, message = container_manager.start_robot()
        logger.info(f"Start result: success={success}")
        response = jsonify({
            'success': success,
            'message': message
        })
        response.headers.add('Access-Control-Allow-Origin', '*')
        return response
    except Exception as e:
        logger.error(f"Error in /start: {e}")
        import traceback
        logger.error(traceback.format_exc())
        response = jsonify({
            'success': False,
            'message': f"Error: {str(e)}"
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
    
    try:
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.WARNING)
        
        app.run(host=host, port=port, debug=False, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        logger.error(traceback.format_exc())
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
