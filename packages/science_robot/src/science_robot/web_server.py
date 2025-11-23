"""
Web server for robot monitoring and control
Provides HTTP interface for camera feed and robot status
"""
import rospy
import cv2
import numpy as np
import threading
import time
import logging
from flask import Flask, Response, jsonify, render_template_string, request

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
        .gesture-tuning, .wave-tuning {
            background: #2a2a2a;
            padding: 20px;
            border-radius: 8px;
            margin: 20px 0;
            display: none; /* Hidden by default, shown when robot is initialized */
        }
        .gesture-tuning.visible, .wave-tuning.visible {
            display: block;
        }
        .gesture-tuning-header, .wave-tuning-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            cursor: pointer;
            margin-bottom: 15px;
        }
        .gesture-tuning-title, .wave-tuning-title {
            font-size: 20px;
            font-weight: bold;
            color: #0f0;
        }
        .gesture-tuning-toggle, .wave-tuning-toggle {
            font-size: 24px;
            color: #aaa;
        }
        .gesture-tuning-content, .wave-tuning-content {
            display: none;
        }
        .gesture-tuning-content.expanded, .wave-tuning-content.expanded {
            display: block;
        }
        .tuning-param {
            margin: 15px 0;
            padding: 10px;
            background: #1a1a1a;
            border-radius: 6px;
        }
        .tuning-param-label {
            display: flex;
            justify-content: space-between;
            margin-bottom: 8px;
            color: #aaa;
            font-size: 14px;
        }
        .tuning-param-value {
            color: #0f0;
            font-weight: bold;
        }
        .tuning-slider {
            width: 100%;
            height: 8px;
            border-radius: 4px;
            background: #333;
            outline: none;
            -webkit-appearance: none;
        }
        .tuning-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: #0f0;
            cursor: pointer;
        }
        .tuning-slider::-moz-range-thumb {
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: #0f0;
            cursor: pointer;
            border: none;
        }
        .tuning-buttons {
            margin-top: 15px;
            text-align: center;
        }
        .tuning-button {
            padding: 8px 16px;
            margin: 5px;
            font-size: 14px;
            cursor: pointer;
            border: none;
            border-radius: 4px;
            background: #444;
            color: white;
        }
        .tuning-button:hover {
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
        
        <div class="gesture-tuning" id="gestureTuning">
            <div class="gesture-tuning-header" onclick="toggleGestureTuning()">
                <div class="gesture-tuning-title">🎯 Gesture Detection Tuning</div>
                <div class="gesture-tuning-toggle" id="gestureToggle">▼</div>
            </div>
            <div class="gesture-tuning-content" id="gestureContent">
                <div class="tuning-param">
                    <div class="tuning-param-label">
                        <span>Detection Confidence</span>
                        <span class="tuning-param-value" id="detectionConfValue">0.50</span>
                    </div>
                    <input type="range" min="0" max="100" value="50" class="tuning-slider" 
                           id="detectionConf" oninput="updateGestureParam('min_detection_confidence', this.value / 100)">
                </div>
                <div class="tuning-param">
                    <div class="tuning-param-label">
                        <span>Tracking Confidence</span>
                        <span class="tuning-param-value" id="trackingConfValue">0.50</span>
                    </div>
                    <input type="range" min="0" max="100" value="50" class="tuning-slider" 
                           id="trackingConf" oninput="updateGestureParam('min_tracking_confidence', this.value / 100)">
                </div>
                <div class="tuning-param">
                    <div class="tuning-param-label">
                        <span>Gesture Confidence Threshold</span>
                        <span class="tuning-param-value" id="gestureConfValue">0.70</span>
                    </div>
                    <input type="range" min="0" max="100" value="70" class="tuning-slider" 
                           id="gestureConf" oninput="updateGestureParam('gesture_confidence_threshold', this.value / 100)">
                </div>
                <div class="tuning-param">
                    <div class="tuning-param-label">
                        <span>Dance Hold Time (seconds)</span>
                        <span class="tuning-param-value" id="danceHoldValue">1.0</span>
                    </div>
                    <input type="range" min="10" max="50" value="10" class="tuning-slider" 
                           id="danceHold" oninput="updateGestureParam('dance_hold_time', this.value / 10)">
                </div>
                <div class="tuning-param">
                    <div class="tuning-param-label">
                        <span>Treat Hold Time (seconds)</span>
                        <span class="tuning-param-value" id="treatHoldValue">2.0</span>
                    </div>
                    <input type="range" min="10" max="100" value="20" class="tuning-slider" 
                           id="treatHold" oninput="updateGestureParam('treat_hold_time', this.value / 10)">
                </div>
                <div class="tuning-param">
                    <div class="tuning-param-label">
                        <span>Clap Finger Threshold</span>
                        <span class="tuning-param-value" id="clapFingerValue">0.120</span>
                    </div>
                    <input type="range" min="1" max="100" value="12" class="tuning-slider" 
                           id="clapFinger" oninput="updateGestureParam('clap_finger_threshold', this.value / 1000)">
                </div>
                <div class="tuning-param">
                    <div class="tuning-param-label">
                        <span>Clap Palm Threshold</span>
                        <span class="tuning-param-value" id="clapPalmValue">0.180</span>
                    </div>
                    <input type="range" min="1" max="100" value="18" class="tuning-slider" 
                           id="clapPalm" oninput="updateGestureParam('clap_palm_threshold', this.value / 1000)">
                </div>
                <div class="tuning-param">
                    <div class="tuning-param-label">
                        <span>Model Complexity (Speed vs Accuracy)</span>
                        <span class="tuning-param-value" id="modelComplexityValue">0</span>
                    </div>
                    <input type="range" min="0" max="2" value="0" step="1" class="tuning-slider" 
                           id="modelComplexity" oninput="updateGestureParam('model_complexity', parseInt(this.value))">
                    <div style="font-size: 11px; color: #888; margin-top: 5px;">
                        0 = Fastest (max FPS) | 1 = Balanced | 2 = Most Accurate (slower)
                    </div>
                </div>
                <div class="tuning-buttons">
                    <button class="tuning-button" onclick="resetGestureParams()">Reset to Defaults</button>
                    <button class="tuning-button" onclick="loadGestureParams()">Refresh Values</button>
                </div>
            </div>
        </div>
        
        <div class="wave-tuning" id="waveTuning">
            <div class="wave-tuning-header" onclick="toggleWaveTuning()">
                <div class="wave-tuning-title">👋 Wave Detection Tuning</div>
                <div class="wave-tuning-toggle" id="waveToggle">▼</div>
            </div>
            <div class="wave-tuning-content" id="waveContent">
                <div class="tuning-param">
                    <div class="tuning-param-label">
                        <span>Detection Frames</span>
                        <span class="tuning-param-value" id="detectionFramesValue">10</span>
                    </div>
                    <input type="range" min="5" max="30" value="10" class="tuning-slider" 
                           id="detectionFrames" oninput="updateWaveParam('history_size', parseInt(this.value))">
                    <div style="font-size: 11px; color: #888; margin-top: 5px;">
                        Number of frames to analyze (lower = faster response, higher = more stable)
                    </div>
                </div>
                <div class="tuning-param">
                    <div class="tuning-param-label">
                        <span>Motion Threshold (pixels)</span>
                        <span class="tuning-param-value" id="motionThresholdValue">20</span>
                    </div>
                    <input type="range" min="5" max="100" value="20" class="tuning-slider" 
                           id="motionThreshold" oninput="updateWaveParam('motion_threshold', parseInt(this.value))">
                    <div style="font-size: 11px; color: #888; margin-top: 5px;">
                        Minimum horizontal movement to detect (lower = more sensitive)
                    </div>
                </div>
                <div class="tuning-param">
                    <div class="tuning-param-label">
                        <span>Min Duration (seconds)</span>
                        <span class="tuning-param-value" id="minDurationValue">0.3</span>
                    </div>
                    <input type="range" min="1" max="20" value="3" class="tuning-slider" 
                           id="minDuration" oninput="updateWaveParam('min_duration', this.value / 10)">
                    <div style="font-size: 11px; color: #888; margin-top: 5px;">
                        How long wave must be sustained (lower = faster trigger)
                    </div>
                </div>
                <div class="tuning-param">
                    <div class="tuning-param-label">
                        <span>Sensitivity</span>
                        <span class="tuning-param-value" id="sensitivityValue">0.5</span>
                    </div>
                    <input type="range" min="0" max="100" value="50" class="tuning-slider" 
                           id="sensitivity" oninput="updateWaveParam('sensitivity', this.value / 100)">
                    <div style="font-size: 11px; color: #888; margin-top: 5px;">
                        Overall sensitivity multiplier (higher = more sensitive)
                    </div>
                </div>
                <div class="tuning-buttons">
                    <button class="tuning-button" onclick="resetWaveParams()">Reset to Defaults</button>
                    <button class="tuning-button" onclick="loadWaveParams()">Refresh Values</button>
                </div>
            </div>
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
                    const gestureTuning = document.getElementById('gestureTuning');
                    const waveTuning = document.getElementById('waveTuning');
                    
                    // Show/hide tuning panels
                    if (initialized) {
                        gestureTuning.classList.add('visible');
                        waveTuning.classList.add('visible');
                        // Load current parameters when robot becomes ready (first time only)
                        if (!gestureTuning.dataset.loaded) {
                            loadGestureParams();
                            gestureTuning.dataset.loaded = 'true';
                        }
                        if (!waveTuning.dataset.loaded) {
                            loadWaveParams();
                            waveTuning.dataset.loaded = 'true';
                        }
                    } else {
                        gestureTuning.classList.remove('visible');
                        waveTuning.classList.remove('visible');
                    }
                    
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
        
        // Gesture tuning functions
        let gestureTuningExpanded = false;
        let gestureParamUpdateTimeout = null;
        
        function toggleGestureTuning() {
            const content = document.getElementById('gestureContent');
            const toggle = document.getElementById('gestureToggle');
            gestureTuningExpanded = !gestureTuningExpanded;
            if (gestureTuningExpanded) {
                content.classList.add('expanded');
                toggle.textContent = '▲';
            } else {
                content.classList.remove('expanded');
                toggle.textContent = '▼';
            }
        }
        
        function updateGestureParam(paramName, value) {
            // Update the display value immediately
            const valueMap = {
                'min_detection_confidence': {element: 'detectionConfValue', format: (v) => v.toFixed(2)},
                'min_tracking_confidence': {element: 'trackingConfValue', format: (v) => v.toFixed(2)},
                'gesture_confidence_threshold': {element: 'gestureConfValue', format: (v) => v.toFixed(2)},
                'dance_hold_time': {element: 'danceHoldValue', format: (v) => v.toFixed(1)},
                'treat_hold_time': {element: 'treatHoldValue', format: (v) => v.toFixed(1)},
                'clap_finger_threshold': {element: 'clapFingerValue', format: (v) => v.toFixed(3)},
                'clap_palm_threshold': {element: 'clapPalmValue', format: (v) => v.toFixed(3)},
                'model_complexity': {element: 'modelComplexityValue', format: (v) => Math.round(v).toString()}
            };
            
            if (valueMap[paramName]) {
                document.getElementById(valueMap[paramName].element).textContent = 
                    valueMap[paramName].format(value);
            }
            
            // Debounce API calls - only send after user stops adjusting for 300ms
            clearTimeout(gestureParamUpdateTimeout);
            gestureParamUpdateTimeout = setTimeout(() => {
                const data = {};
                data[paramName] = value;
                
                fetch('/gesture_params', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(data)
                })
                .then(r => r.json())
                .then(result => {
                    if (result.status === 'updated') {
                        console.log('Gesture parameter updated:', paramName, '=', value);
                    } else {
                        console.error('Failed to update gesture parameter:', result.error);
                    }
                })
                .catch(err => console.error('Error updating gesture parameter:', err));
            }, 300);
        }
        
        function loadGestureParams() {
            fetch('/gesture_params')
                .then(r => r.json())
                .then(params => {
                    // Update all sliders and values
                    document.getElementById('detectionConf').value = Math.round(params.min_detection_confidence * 100);
                    document.getElementById('detectionConfValue').textContent = params.min_detection_confidence.toFixed(2);
                    
                    document.getElementById('trackingConf').value = Math.round(params.min_tracking_confidence * 100);
                    document.getElementById('trackingConfValue').textContent = params.min_tracking_confidence.toFixed(2);
                    
                    document.getElementById('gestureConf').value = Math.round(params.gesture_confidence_threshold * 100);
                    document.getElementById('gestureConfValue').textContent = params.gesture_confidence_threshold.toFixed(2);
                    
                    document.getElementById('danceHold').value = Math.round(params.dance_hold_time * 10);
                    document.getElementById('danceHoldValue').textContent = params.dance_hold_time.toFixed(1);
                    
                    document.getElementById('treatHold').value = Math.round(params.treat_hold_time * 10);
                    document.getElementById('treatHoldValue').textContent = params.treat_hold_time.toFixed(1);
                    
                    document.getElementById('clapFinger').value = Math.round(params.clap_finger_threshold * 1000);
                    document.getElementById('clapFingerValue').textContent = params.clap_finger_threshold.toFixed(3);
                    
                    document.getElementById('clapPalm').value = Math.round(params.clap_palm_threshold * 1000);
                    document.getElementById('clapPalmValue').textContent = params.clap_palm_threshold.toFixed(3);
                    
                    if (params.model_complexity !== undefined) {
                        document.getElementById('modelComplexity').value = params.model_complexity;
                        document.getElementById('modelComplexityValue').textContent = params.model_complexity.toString();
                    }
                })
                .catch(err => console.error('Error loading gesture parameters:', err));
        }
        
        function resetGestureParams() {
            // Default values from config.py
            const defaults = {
                min_detection_confidence: 0.5,
                min_tracking_confidence: 0.5,
                gesture_confidence_threshold: 0.7,
                dance_hold_time: 1.0,
                treat_hold_time: 2.0,
                clap_finger_threshold: 0.12,
                clap_palm_threshold: 0.18,
                model_complexity: 0
            };
            
            fetch('/gesture_params', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(defaults)
            })
            .then(r => r.json())
            .then(result => {
                if (result.status === 'updated') {
                    loadGestureParams(); // Refresh UI with new values
                    console.log('Gesture parameters reset to defaults');
                } else {
                    console.error('Failed to reset gesture parameters:', result.error);
                }
            })
            .catch(err => console.error('Error resetting gesture parameters:', err));
        }
        
        // Wave tuning functions
        let waveTuningExpanded = false;
        let waveParamUpdateTimeout = null;
        
        function toggleWaveTuning() {
            const content = document.getElementById('waveContent');
            const toggle = document.getElementById('waveToggle');
            waveTuningExpanded = !waveTuningExpanded;
            if (waveTuningExpanded) {
                content.classList.add('expanded');
                toggle.textContent = '▲';
            } else {
                content.classList.remove('expanded');
                toggle.textContent = '▼';
            }
        }
        
        function updateWaveParam(paramName, value) {
            // Update the display value immediately
            const valueMap = {
                'history_size': {element: 'detectionFramesValue', format: (v) => Math.round(v).toString()},
                'motion_threshold': {element: 'motionThresholdValue', format: (v) => Math.round(v).toString()},
                'min_duration': {element: 'minDurationValue', format: (v) => v.toFixed(1)},
                'sensitivity': {element: 'sensitivityValue', format: (v) => v.toFixed(2)}
            };
            
            if (valueMap[paramName]) {
                document.getElementById(valueMap[paramName].element).textContent = 
                    valueMap[paramName].format(value);
            }
            
            // Debounce API calls - only send after user stops adjusting for 300ms
            clearTimeout(waveParamUpdateTimeout);
            waveParamUpdateTimeout = setTimeout(() => {
                const data = {};
                data[paramName] = value;
                
                fetch('/wave_params', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(data)
                })
                .then(r => r.json())
                .then(result => {
                    if (result.status === 'updated') {
                        console.log('Wave parameter updated:', paramName, '=', value);
                    } else {
                        console.error('Failed to update wave parameter:', result.error);
                    }
                })
                .catch(err => console.error('Error updating wave parameter:', err));
            }, 300);
        }
        
        function loadWaveParams() {
            fetch('/wave_params')
                .then(r => r.json())
                .then(params => {
                    // Update all sliders and values
                    document.getElementById('detectionFrames').value = params.history_size;
                    document.getElementById('detectionFramesValue').textContent = params.history_size.toString();
                    
                    document.getElementById('motionThreshold').value = params.motion_threshold;
                    document.getElementById('motionThresholdValue').textContent = params.motion_threshold.toString();
                    
                    document.getElementById('minDuration').value = Math.round(params.min_duration * 10);
                    document.getElementById('minDurationValue').textContent = params.min_duration.toFixed(1);
                    
                    document.getElementById('sensitivity').value = Math.round(params.sensitivity * 100);
                    document.getElementById('sensitivityValue').textContent = params.sensitivity.toFixed(2);
                })
                .catch(err => console.error('Error loading wave parameters:', err));
        }
        
        function resetWaveParams() {
            // Default values from config.py (new improved defaults)
            const defaults = {
                history_size: 10,
                motion_threshold: 20,
                min_duration: 0.3,
                sensitivity: 0.5
            };
            
            fetch('/wave_params', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(defaults)
            })
            .then(r => r.json())
            .then(result => {
                if (result.status === 'updated') {
                    loadWaveParams(); // Refresh UI with new values
                    console.log('Wave parameters reset to defaults');
                } else {
                    console.error('Failed to reset wave parameters:', result.error);
                }
            })
            .catch(err => console.error('Error resetting wave parameters:', err));
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
            placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
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

@app.route('/gesture_params', methods=['GET'])
def get_gesture_params():
    """Get current gesture detection parameters"""
    global robot_controller
    if robot_controller and hasattr(robot_controller, 'gesture_detector'):
        try:
            params = robot_controller.gesture_detector.get_parameters()
            return jsonify(params)
        except Exception as e:
            logger.error(f"Error getting gesture parameters: {e}")
            return jsonify({'error': str(e)}), 500
    return jsonify({'error': 'Robot not initialized'}), 503

@app.route('/gesture_params', methods=['POST'])
def set_gesture_params():
    """Update gesture detection parameters"""
    global robot_controller
    if robot_controller and hasattr(robot_controller, 'gesture_detector'):
        try:
            data = request.get_json()
            if not data:
                return jsonify({'error': 'No data provided'}), 400
            
            robot_controller.gesture_detector.update_parameters(
                min_detection_confidence=data.get('min_detection_confidence'),
                min_tracking_confidence=data.get('min_tracking_confidence'),
                gesture_confidence_threshold=data.get('gesture_confidence_threshold'),
                dance_hold_time=data.get('dance_hold_time'),
                treat_hold_time=data.get('treat_hold_time'),
                clap_finger_threshold=data.get('clap_finger_threshold'),
                clap_palm_threshold=data.get('clap_palm_threshold'),
                model_complexity=data.get('model_complexity')
            )
            updated_params = robot_controller.gesture_detector.get_parameters()
            logger.info(f"Gesture parameters updated via web interface: {updated_params}")
            return jsonify({'status': 'updated', 'params': updated_params})
        except Exception as e:
            logger.error(f"Error updating gesture parameters: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return jsonify({'error': str(e)}), 500
    return jsonify({'error': 'Robot not initialized'}), 503

@app.route('/wave_params', methods=['GET'])
def get_wave_params():
    """Get current wave detection parameters"""
    global robot_controller
    if robot_controller and hasattr(robot_controller, 'wave_detector'):
        try:
            params = robot_controller.wave_detector.get_parameters()
            return jsonify(params)
        except Exception as e:
            logger.error(f"Error getting wave parameters: {e}")
            return jsonify({'error': str(e)}), 500
    return jsonify({'error': 'Robot not initialized'}), 503

@app.route('/wave_params', methods=['POST'])
def set_wave_params():
    """Update wave detection parameters"""
    global robot_controller
    if robot_controller and hasattr(robot_controller, 'wave_detector'):
        try:
            data = request.get_json()
            if not data:
                return jsonify({'error': 'No data provided'}), 400
            
            robot_controller.wave_detector.update_parameters(
                history_size=data.get('history_size'),
                motion_threshold=data.get('motion_threshold'),
                min_duration=data.get('min_duration'),
                sensitivity=data.get('sensitivity')
            )
            updated_params = robot_controller.wave_detector.get_parameters()
            logger.info(f"Wave parameters updated via web interface: {updated_params}")
            return jsonify({'status': 'updated', 'params': updated_params})
        except Exception as e:
            logger.error(f"Error updating wave parameters: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return jsonify({'error': str(e)}), 500
    return jsonify({'error': 'Robot not initialized'}), 503

def update_frame(frame):
    """Update latest frame for web stream"""
    global latest_frame, frame_lock
    with frame_lock:
        latest_frame = frame

def set_initialization_status(status_message, success=False, warning=False, error=False):
    """Update initialization status for web display"""
    global robot_status
    robot_status['initialization_status'] = status_message
    robot_status['initialized'] = success  # Only set to True if success is True
    # Optionally add warning/error states for styling
    if warning:
        robot_status['init_status_class'] = 'warning'
    elif error:
        robot_status['init_status_class'] = 'error'
    elif success:
        robot_status['init_status_class'] = 'success'
    else:
        robot_status['init_status_class'] = ''

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

