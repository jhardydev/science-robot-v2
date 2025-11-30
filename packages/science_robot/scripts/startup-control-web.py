#!/usr/bin/env python3
"""
Standalone web server for robot startup control
Runs independently to control robot start/stop/shutdown
This server is always running (via systemd) and provides the control interface
"""
import os
import sys
import logging
from pathlib import Path

# Add parent directory to path for imports
script_dir = Path(__file__).parent
src_dir = script_dir.parent / 'src'
sys.path.insert(0, str(src_dir))

# Import web server components (minimal import for standalone mode)
try:
    from science_robot.web_server import app, robot_control_manager
    from science_robot import config
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Make sure you're running from the correct directory and dependencies are installed")
    sys.exit(1)

def main():
    """Run standalone startup control web server"""
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - [startup-control] - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger(__name__)
    
    # Get configuration
    port = int(os.getenv('WEB_SERVER_PORT', getattr(config, 'WEB_SERVER_PORT', 5000)))
    host = os.getenv('WEB_SERVER_HOST', getattr(config, 'WEB_SERVER_HOST', '0.0.0.0'))
    
    logger.info(f"Starting robot startup control web server on http://{host}:{port}")
    logger.info("This server controls robot start/stop/shutdown and is always running")
    logger.info("Access the control panel at: http://<robot-ip>:{port}/".format(port=port))
    
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

