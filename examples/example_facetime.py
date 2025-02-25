#!/usr/bin/env python3

# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from flask import Flask, render_template_string, Response, jsonify
from flask_socketio import SocketIO
import cv2
import numpy as np
import json
import threading
import time
import base64
import pyaudio
import wave
import contextlib
import ctypes
import scipy
from lib.odrive_uart import ODriveUART
from lib.camera import StereoCamera
import scipy.signal as signal
from io import BytesIO
import struct
import queue

# Setup error handler for ALSA
ERROR_HANDLER_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int,
                                     ctypes.c_char_p, ctypes.c_int,
                                     ctypes.c_char_p)

def py_error_handler(filename, line, function, err, fmt):
    pass

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextlib.contextmanager
def noalsaerr():
    asound = ctypes.CDLL('libasound.so.2')
    asound.snd_lib_error_set_handler(c_error_handler)
    yield
    asound.snd_lib_error_set_handler(None)

# HTML template as a string variable
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Control Interface</title>
    <script src="https://cdn.socket.io/4.4.1/socket.io.min.js"></script>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 0;
            background-color: #f0f0f0;
            color: #333;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            padding: 20px;
        }
        h1 {
            text-align: center;
            color: #2c3e50;
        }
        .video-container {
            text-align: center;
            margin-bottom: 20px;
            display: flex;
            justify-content: center;
        }
        .video-feed {
            max-width: 100%;
            width: 640px; /* Set specific width for single camera */
            border: 3px solid #2c3e50;
            border-radius: 5px;
        }
        .controls {
            display: flex;
            flex-wrap: wrap;
            justify-content: center;
            gap: 20px;
            margin-bottom: 20px;
        }
        .control-group {
            background: #fff;
            padding: 15px;
            border-radius: 5px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }
        .movement-controls {
            display: grid;
            grid-template-columns: repeat(3, 60px);
            grid-template-rows: repeat(3, 60px);
            gap: 5px;
        }
        .movement-controls button {
            height: 60px;
            background: #3498db;
            border: none;
            border-radius: 5px;
            color: white;
            font-weight: bold;
            font-size: 18px;
            cursor: pointer;
            transition: background 0.2s;
        }
        .movement-controls button:hover {
            background: #2980b9;
        }
        .movement-controls button:active {
            background: #1c638d;
        }
        .movement-controls .empty {
            background: transparent;
        }
        .audio-controls {
            display: flex;
            flex-direction: column;
            gap: 10px;
        }
        .audio-controls button {
            padding: 10px 15px;
            background: #2ecc71;
            border: none;
            border-radius: 5px;
            color: white;
            font-weight: bold;
            cursor: pointer;
            transition: background 0.2s;
        }
        .audio-controls button:hover {
            background: #27ae60;
        }
        .audio-controls button:active {
            background: #1e8449;
        }
        .audio-controls button.disabled {
            background: #95a5a6;
            cursor: not-allowed;
        }
        .status {
            text-align: center;
            font-weight: bold;
            margin-top: 10px;
        }
        .error-message {
            color: #e74c3c;
            text-align: center;
            font-weight: bold;
            margin: 10px 0;
        }
        .info {
            background: #fff;
            padding: 15px;
            border-radius: 5px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
            margin-top: 20px;
        }
        .key-guide {
            display: grid;
            grid-template-columns: 100px 1fr;
            gap: 10px;
        }
        .key {
            font-weight: bold;
            background: #ecf0f1;
            padding: 5px 10px;
            border-radius: 3px;
            text-align: center;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Robot Control Interface</h1>
        
        <div class="video-container">
            <img src="{{ url_for('video_feed') }}" class="video-feed" alt="Robot Camera Feed">
            <div id="cameraErrorMessage" class="error-message" style="display: none;">Camera not available</div>
        </div>
        
        <div class="controls">
            <div class="control-group">
                <h2>Movement Controls</h2>
                <div class="movement-controls">
                    <button class="empty"></button>
                    <button id="btnW">W</button>
                    <button class="empty"></button>
                    <button id="btnA">A</button>
                    <button id="btnS">S</button>
                    <button id="btnD">D</button>
                    <button class="empty"></button>
                    <button class="empty"></button>
                    <button class="empty"></button>
                </div>
                <div class="status" id="movementStatus">No movement</div>
            </div>
            
            <div class="control-group">
                <h2>Audio Controls</h2>
                <div class="audio-controls">
                    <button id="btnListenMic">Start Listening to Robot Microphone</button>
                    <button id="btnStopMic">Stop Listening</button>
                    <button id="btnTalkToRobot">Push to Talk to Robot</button>
                </div>
                <div class="status" id="audioStatus">Audio inactive</div>
                <div id="audioErrorMessage" class="error-message" style="display: none;">Audio device not available</div>
            </div>
        </div>
        
        <div class="info">
            <h2>Keyboard Controls</h2>
            <p>You can also control the robot using your keyboard:</p>
            <div class="key-guide">
                <div class="key">W</div>
                <div>Move forward</div>
                <div class="key">S</div>
                <div>Move backward</div>
                <div class="key">A</div>
                <div>Turn left</div>
                <div class="key">D</div>
                <div>Turn right</div>
            </div>
        </div>
    </div>

    <script>
        // Connect to Socket.IO server
        const socket = io();
        
        // DOM elements
        const movementStatus = document.getElementById('movementStatus');
        const audioStatus = document.getElementById('audioStatus');
        const btnW = document.getElementById('btnW');
        const btnA = document.getElementById('btnA');
        const btnS = document.getElementById('btnS');
        const btnD = document.getElementById('btnD');
        const btnListenMic = document.getElementById('btnListenMic');
        const btnStopMic = document.getElementById('btnStopMic');
        const btnTalkToRobot = document.getElementById('btnTalkToRobot');
        const cameraErrorMessage = document.getElementById('cameraErrorMessage');
        const audioErrorMessage = document.getElementById('audioErrorMessage');
        
        // Variables for state
        let isListeningToMic = false;
        let isTalkingToRobot = false;
        let audioContext = null;
        let microphoneStream = null;
        let activeKeys = new Set();
        
        // Movement button event listeners
        const movementButtons = {
            'btnW': 'w',
            'btnA': 'a',
            'btnS': 's',
            'btnD': 'd'
        };
        
        // Add event listeners for movement buttons
        for (const [btnId, key] of Object.entries(movementButtons)) {
            const btn = document.getElementById(btnId);
            
            btn.addEventListener('mousedown', () => {
                socket.emit('keydown', { key });
                movementStatus.textContent = `Moving: ${key.toUpperCase()}`;
                activeKeys.add(key);
            });
            
            btn.addEventListener('mouseup', () => {
                socket.emit('keyup', { key });
                activeKeys.delete(key);
                if (activeKeys.size === 0) {
                    movementStatus.textContent = 'No movement';
                }
            });
            
            btn.addEventListener('mouseleave', () => {
                if (activeKeys.has(key)) {
                    socket.emit('keyup', { key });
                    activeKeys.delete(key);
                    if (activeKeys.size === 0) {
                        movementStatus.textContent = 'No movement';
                    }
                }
            });
        }
        
        // Keyboard event listeners
        document.addEventListener('keydown', (event) => {
            const key = event.key.toLowerCase();
            if (movementButtons.hasOwnProperty(`btn${key.toUpperCase()}`)) {
                if (!activeKeys.has(key)) {
                    socket.emit('keydown', { key });
                    movementStatus.textContent = `Moving: ${key.toUpperCase()}`;
                    activeKeys.add(key);
                }
                event.preventDefault();
            }
        });
        
        document.addEventListener('keyup', (event) => {
            const key = event.key.toLowerCase();
            if (movementButtons.hasOwnProperty(`btn${key.toUpperCase()}`)) {
                socket.emit('keyup', { key });
                activeKeys.delete(key);
                if (activeKeys.size === 0) {
                    movementStatus.textContent = 'No movement';
                }
                event.preventDefault();
            }
        });
        
        // Function to start listening to robot's microphone
        btnListenMic.addEventListener('click', () => {
            if (!isListeningToMic) {
                socket.emit('start_audio');
                isListeningToMic = true;
                btnListenMic.classList.add('disabled');
                btnStopMic.classList.remove('disabled');
                audioStatus.textContent = 'Listening to robot microphone';
            }
        });
        
        // Function to stop listening to robot's microphone
        btnStopMic.addEventListener('click', () => {
            if (isListeningToMic) {
                socket.emit('stop_audio');
                isListeningToMic = false;
                btnListenMic.classList.remove('disabled');
                btnStopMic.classList.add('disabled');
                audioStatus.textContent = 'Audio inactive';
            }
        });
        
        // Function to handle "Push to Talk" to robot
        btnTalkToRobot.addEventListener('mousedown', async () => {
            if (!isTalkingToRobot) {
                try {
                    audioContext = new (window.AudioContext || window.webkitAudioContext)();
                    microphoneStream = await navigator.mediaDevices.getUserMedia({ audio: true });
                    const microphone = audioContext.createMediaStreamSource(microphoneStream);
                    const processor = audioContext.createScriptProcessor(4096, 1, 1);
                    
                    processor.onaudioprocess = function(e) {
                        // Get audio data
                        const inputData = e.inputBuffer.getChannelData(0);
                        
                        // Convert to 16-bit PCM
                        const pcmData = new Int16Array(inputData.length);
                        for (let i = 0; i < inputData.length; i++) {
                            pcmData[i] = inputData[i] * 0x7FFF;
                        }
                        
                        // Convert to base64 and send
                        const base64data = arrayBufferToBase64(pcmData.buffer);
                        socket.emit('speaker_audio', { audio_data: base64data });
                    };
                    
                    microphone.connect(processor);
                    processor.connect(audioContext.destination);
                    
                    isTalkingToRobot = true;
                    audioStatus.textContent = 'Talking to robot';
                } catch (err) {
                    console.error('Error accessing microphone:', err);
                    audioStatus.textContent = 'Error: Could not access microphone';
                }
            }
        });
        
        btnTalkToRobot.addEventListener('mouseup', () => {
            if (isTalkingToRobot) {
                if (microphoneStream) {
                    microphoneStream.getTracks().forEach(track => track.stop());
                }
                
                if (audioContext) {
                    audioContext.close();
                }
                
                isTalkingToRobot = false;
                if (isListeningToMic) {
                    audioStatus.textContent = 'Listening to robot microphone';
                } else {
                    audioStatus.textContent = 'Audio inactive';
                }
            }
        });
        
        // Function to handle incoming audio data from robot
        socket.on('audio_data', (data) => {
            if (isListeningToMic && data.audio_data) {
                playAudio(data.audio_data);
            }
        });
        
        // Play audio data received from the robot
        function playAudio(base64Data) {
            if (!audioContext) {
                audioContext = new (window.AudioContext || window.webkitAudioContext)();
            }
            
            try {
                // Convert base64 to ArrayBuffer
                const audioData = base64ToArrayBuffer(base64Data);
                
                // Create an audio buffer source
                const source = audioContext.createBufferSource();
                
                // Process the audio data directly without decoding
                const audioBuffer = audioContext.createBuffer(1, audioData.byteLength / 2, 44100);
                const channelData = audioBuffer.getChannelData(0);
                
                // Convert Int16Array to Float32 for the audio buffer
                const int16Array = new Int16Array(audioData);
                for (let i = 0; i < int16Array.length; i++) {
                    // Convert from 16-bit PCM to float
                    channelData[i] = int16Array[i] / 32768.0;
                }
                
                // Connect the buffer to the source
                source.buffer = audioBuffer;
                source.connect(audioContext.destination);
                source.start(0);
                
                console.log("Playing audio packet");
            } catch (err) {
                console.error('Error playing audio data:', err);
            }
        }
        
        // Helper function to convert ArrayBuffer to base64
        function arrayBufferToBase64(buffer) {
            let binary = '';
            const bytes = new Uint8Array(buffer);
            const len = bytes.byteLength;
            for (let i = 0; i < len; i++) {
                binary += String.fromCharCode(bytes[i]);
            }
            return window.btoa(binary);
        }
        
        // Helper function to convert base64 to ArrayBuffer
        function base64ToArrayBuffer(base64) {
            const binary = window.atob(base64);
            const len = binary.length;
            const bytes = new Uint8Array(len);
            for (let i = 0; i < len; i++) {
                bytes[i] = binary.charCodeAt(i);
            }
            return bytes.buffer;
        }
        
        // Socket connection event handlers
        socket.on('connect', () => {
            console.log('Connected to server');
        });
        
        socket.on('disconnect', () => {
            console.log('Disconnected from server');
            isListeningToMic = false;
            isTalkingToRobot = false;
            audioStatus.textContent = 'Disconnected';
            movementStatus.textContent = 'Disconnected';
            
            // Clean up any active audio
            if (microphoneStream) {
                microphoneStream.getTracks().forEach(track => track.stop());
            }
            
            if (audioContext) {
                audioContext.close();
            }
        });

        // Handle hardware status messages from server
        socket.on('hardware_status', (data) => {
            if (data.camera_available === false) {
                cameraErrorMessage.style.display = 'block';
            } else {
                cameraErrorMessage.style.display = 'none';
            }
            
            if (data.audio_available === false) {
                audioErrorMessage.style.display = 'block';
                btnListenMic.disabled = true;
                btnListenMic.classList.add('disabled');
                btnTalkToRobot.disabled = true;
                btnTalkToRobot.classList.add('disabled');
            } else {
                audioErrorMessage.style.display = 'none';
                btnListenMic.disabled = false;
                btnTalkToRobot.disabled = false;
            }
        });
        
        // Initial UI setup
        btnStopMic.classList.add('disabled');
    </script>
</body>
</html>
"""

# Initialize Flask app
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables
camera = None
motor = None
audio_thread = None
is_streaming = False
audio_streaming = False
CHUNK = 4096
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
audio_frames = []
camera_available = False
audio_available = False
dummy_frame = None

# Add path to shared lib
sys.path.append(os.path.expanduser('~/quickstart/lib'))
from camera import StereoCamera
from odrive_uart import ODriveUART

# Add these global variables after other global variables
audio_buffer = queue.Queue(maxsize=20)  # Buffer up to 20 audio chunks
audio_playback_thread = None
audio_playback_active = False

def check_camera_devices():
    """Check for camera device 'video0' - returns a list with the camera if found"""
    available_cameras = []
    
    # Allow some time for camera devices to initialize
    time.sleep(1)
    
    # Only check for video0
    try:
        # Try with V4L2 backend first
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if cap.isOpened():
            # Get camera properties
            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = cap.get(cv2.CAP_PROP_FPS)
            
            # Only add if we can actually read a frame
            ret, _ = cap.read()
            if ret:
                # This is a working camera
                available_cameras.append({
                    'id': 0,
                    'resolution': f"{width}x{height}",
                    'fps': fps
                })
            
            cap.release()
        else:
            # Try with default backend
            cap = cv2.VideoCapture(0)
            if cap.isOpened():
                # Get camera properties
                width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                fps = cap.get(cv2.CAP_PROP_FPS)
                
                # Only add if we can actually read a frame
                ret, _ = cap.read()
                if ret:
                    # This is a working camera
                    available_cameras.append({
                        'id': 0,
                        'resolution': f"{width}x{height}",
                        'fps': fps
                    })
                
                cap.release()
    except Exception as e:
        print(f"Error checking camera index 0: {e}")
    
    # If no camera found, wait and retry once
    if not available_cameras:
        print("Camera not found on first attempt. Waiting 2 seconds and retrying...")
        time.sleep(2)
        
        # Retry detection
        try:
            cap = cv2.VideoCapture(0)
            if cap.isOpened():
                # Get camera properties
                width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                fps = cap.get(cv2.CAP_PROP_FPS)
                
                # Test if we can read a frame
                ret, _ = cap.read()
                if ret:
                    available_cameras.append({
                        'id': 0,
                        'resolution': f"{width}x{height}",
                        'fps': fps
                    })
                
                cap.release()
        except Exception as e:
            print(f"Error on retry for camera index 0: {e}")
    
    return available_cameras

def create_dummy_frame(message="Camera Not Available"):
    """Create a dummy frame with an error message for when camera is not available"""
    height, width = 480, 640
    img = np.zeros((height, width, 3), dtype=np.uint8)
    # Add a dark gray background
    img[:] = (50, 50, 50)
    
    # Add text
    font = cv2.FONT_HERSHEY_SIMPLEX
    text_size = cv2.getTextSize(message, font, 1, 2)[0]
    text_x = (width - text_size[0]) // 2
    text_y = (height + text_size[1]) // 2
    cv2.putText(img, message, (text_x, text_y), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
    
    # Return single frame, not stereo
    return img

def initialize_hardware():
    """Initialize camera, audio, and motor - uses single camera view"""
    global camera, motor, camera_available, audio_available, dummy_frame
    
    print("\nInitializing hardware...")
    
    # Create a dummy frame for fallback when camera is not available
    dummy_frame = np.zeros((360, 640, 3), dtype=np.uint8)
    # Add text to the dummy frame
    cv2.putText(dummy_frame, "Camera not available", (50, 180), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Initialize camera
    camera = None
    camera_available = False
    
    print("Checking for camera device video0...")
    available_cameras = check_camera_devices()
    
    if available_cameras:
        camera_available = restore_camera()  # Use our new function for camera initialization
    else:
        print("Camera video0 not found!")
    
    # Initialize audio
    audio_available = False
    try:
        # Initialize PyAudio
        p = pyaudio.PyAudio()
        
        # Check for audio devices
        info = p.get_host_api_info_by_index(0)
        numdevices = info.get('deviceCount')
        
        input_devices = []
        for i in range(numdevices):
            device_info = p.get_device_info_by_index(i)
            if device_info.get('maxInputChannels') > 0:
                input_devices.append(device_info)
                print(f"Input Device {i}: {device_info.get('name')}")
                
        output_devices = []
        for i in range(numdevices):
            device_info = p.get_device_info_by_index(i)
            if device_info.get('maxOutputChannels') > 0:
                output_devices.append(device_info)
                print(f"Output Device {i}: {device_info.get('name')}")
        
        if input_devices and output_devices:
            print(f"Found {len(input_devices)} input device(s) and {len(output_devices)} output device(s)")
            audio_available = True
        else:
            print("No audio devices found!")
            
    except Exception as e:
        print(f"Error initializing audio: {e}")
    
    # Initialize motor controller
    try:
        # Load motor directions
        motor_dir_path = os.path.expanduser('~/quickstart/lib/motor_dir.json')
        if os.path.exists(motor_dir_path):
            with open(motor_dir_path, 'r') as f:
                motor_dirs = json.load(f)
        else:
            print("Warning: motor_dir.json not found, using default values")
            motor_dirs = {"left": 1, "right": 1}
            
        # Initialize motor with ODriveUART
        motor = ODriveUART(
            port='/dev/ttyAMA1',
            left_axis=0, right_axis=1,
            dir_left=motor_dirs['left'], 
            dir_right=motor_dirs['right']
        )
        
        # Start motors
        motor.start_left()
        motor.start_right()
        motor.enable_velocity_mode_left()
        motor.enable_velocity_mode_right()
        motor.disable_watchdog_left()
        motor.disable_watchdog_right()
        motor.clear_errors_left()
        motor.clear_errors_right()
        print("Motor controller initialized")
    except Exception as e:
        print(f"Error initializing motor controller: {e}")
        motor = None
    
    print("Hardware initialization complete")
    print(f"Camera available: {camera_available}")
    print(f"Audio available: {audio_available}")
    
    # Return True if at least one hardware component initialized successfully
    return True

def cleanup():
    """Clean up resources"""
    global camera, motor
    if camera:
        if isinstance(camera, StereoCamera):
            camera.release()
        elif isinstance(camera, cv2.VideoCapture):
            camera.release()
    if motor:
        motor.set_speed_mps_left(0)
        motor.set_speed_mps_right(0)

def restore_camera():
    """Try to restore camera if it becomes unavailable - initializes video0 for a single camera feed"""
    global camera, camera_available, dummy_frame
    
    print("Attempting to restore camera...")
    
    # Release current camera if it exists
    if camera:
        try:
            if isinstance(camera, StereoCamera):
                camera.release()
            elif isinstance(camera, cv2.VideoCapture):
                camera.release()
        except Exception as e:
            print(f"Error releasing camera: {e}")
    
    # Wait for camera devices to become available again
    time.sleep(2)
    
    # Reset camera to None
    camera = None
    camera_available = False
    
    # Check if video0 is available
    try:
        print("Attempting to initialize camera with id 0")
        
        # Try with explicit V4L2 backend first
        print("Initializing StereoCamera with device_id=0 (will only use left camera)")
        camera = StereoCamera(device_id=0, scale=0.5)
        
        # Test if camera is working by getting a frame
        left, right = camera.get_stereo()
        if left is not None and right is not None:
            camera_available = True
            print(f"StereoCamera initialized successfully - will use left frame only: {left.shape}")
            return True
        else:
            print("Camera initialized but could not capture frames")
            if isinstance(camera, StereoCamera):
                camera.release()
            
            # Fallback to regular camera
            try:
                print("Trying fallback to regular camera with device_id=0")
                camera = cv2.VideoCapture(0)
                ret, frame = camera.read()
                if ret:
                    print(f"Regular camera initialized successfully - frame size: {frame.shape}")
                    camera_available = True
                    return True
                else:
                    print("Regular camera initialization failed to capture frames")
                    camera.release()
            except Exception as e:
                print(f"Error with fallback camera: {e}")
    except Exception as e:
        print(f"Error initializing StereoCamera with id 0: {e}")
    
    print("Could not initialize camera during restore attempt")
    return False

def gen_frames():
    """Generate camera frames from video0 - single camera view"""
    global camera, is_streaming, camera_available, dummy_frame
    is_streaming = True
    frames_without_camera = 0
    
    while is_streaming:
        try:
            if not camera_available or camera is None:
                # Try to restore camera automatically after some time without a camera
                frames_without_camera += 1
                if frames_without_camera > 30:  # After about 3 seconds (10 fps)
                    frames_without_camera = 0
                    restore_success = restore_camera()
                    if restore_success:
                        print("Camera video0 restored successfully")
                    else:
                        print("Camera video0 restore failed")
                
                # Return dummy frame if camera is not available
                ret, buffer = cv2.imencode('.jpg', dummy_frame)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                time.sleep(0.1)  # Add a small delay when using dummy frames
                continue
            
            # Reset counter when camera is working
            frames_without_camera = 0
                
            # Get frame from camera
            if isinstance(camera, StereoCamera):
                left, right = camera.get_stereo()
                if left is None:
                    # Use dummy frame if camera fails
                    ret, buffer = cv2.imencode('.jpg', dummy_frame)
                    frame = buffer.tobytes()
                else:
                    # Only use the left image (no side-by-side)
                    ret, buffer = cv2.imencode('.jpg', left)
                    frame = buffer.tobytes()
            else:
                # Regular camera
                ret, frame_data = camera.read()
                if not ret:
                    # Use dummy frame if camera fails
                    ret, buffer = cv2.imencode('.jpg', dummy_frame)
                    frame = buffer.tobytes()
                else:
                    # Use the frame directly (no duplication)
                    ret, buffer = cv2.imencode('.jpg', frame_data)
                    frame = buffer.tobytes()
            
            # Yield the frame in the format expected by Flask's Response
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        except Exception as e:
            print(f"Error generating frames from video0: {e}")
            # Use dummy frame on error
            try:
                ret, buffer = cv2.imencode('.jpg', dummy_frame)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except:
                pass
            time.sleep(0.5)  # Add a longer delay after an error
    
    is_streaming = False

def audio_stream_thread():
    """Thread for audio streaming"""
    global audio_streaming, audio_available, FORMAT, RATE, CHANNELS, CHUNK
    
    if not audio_available:
        print("Audio streaming not available")
        return
        
    try:
        with noalsaerr():
            p = pyaudio.PyAudio()
        
        # Try to find the USB audio device, with fallback options
        device_index = None
        device_name = "UACDemoV1.0"  # Name of the USB audio device
        
        # Print all audio devices for debugging
        print("\n=== AUDIO DEBUG INFO ===")
        print(f"Looking for input device '{device_name}'")
        for i in range(p.get_device_count()):
            try:
                dev_info = p.get_device_info_by_index(i)
                print(f"Device {i}: {dev_info['name']}")
                print(f"  Input channels: {dev_info['maxInputChannels']}")
                print(f"  Output channels: {dev_info['maxOutputChannels']}")
                print(f"  Default sample rate: {dev_info['defaultSampleRate']}")
            except Exception as e:
                print(f"Error getting device info for index {i}: {e}")
        
        # First try the specific device
        for i in range(p.get_device_count()):
            try:
                dev_info = p.get_device_info_by_index(i)
                if dev_info['maxInputChannels'] > 0 and device_name in dev_info['name']:
                    device_index = i
                    print(f"Found audio device '{dev_info['name']}' at index {i}")
                    break
            except Exception as e:
                print(f"Error getting device info for index {i}: {e}")
        
        # If specific device not found, try any device with input channels
        if device_index is None:
            for i in range(p.get_device_count()):
                try:
                    dev_info = p.get_device_info_by_index(i)
                    if dev_info['maxInputChannels'] > 0:
                        device_index = i
                        print(f"Using fallback audio device '{dev_info['name']}' at index {i}")
                        break
                except Exception as e:
                    print(f"Error getting device info for index {i}: {e}")
        
        if device_index is None:
            print("No suitable audio input device found")
            p.terminate()
            audio_available = False
            return
        
        try:
            # Make sure we use a supported sample rate for the selected device
            dev_info = p.get_device_info_by_index(device_index)
            sample_rate = int(dev_info['defaultSampleRate'])
            print(f"Using sample rate: {sample_rate}")
            # Update global RATE to be used when playing back audio
            RATE = sample_rate
            
            # Start recording with error handling
            stream = p.open(format=FORMAT,
                          channels=CHANNELS,
                          rate=RATE,
                          input=True,
                          input_device_index=device_index,
                          frames_per_buffer=CHUNK)
            
            print(f"Audio streaming started on device index {device_index}")
            print("=== END AUDIO DEBUG INFO ===\n")
            
            # Generate and send a test sine wave to verify audio
            print("Sending test audio packet (sine wave)")
            # Create 1 second of sine wave at 440 Hz
            duration = 0.2  # seconds
            volume = 0.5    # 0.0 to 1.0
            freq = 440.0    # Hz
            samples = int(RATE * duration)
            test_data = np.zeros(samples, dtype=np.int16)
            for i in range(samples):
                # Generate sine wave
                t = float(i) / RATE
                test_data[i] = int(32767.0 * volume * np.sin(2.0 * np.pi * freq * t))
            
            # Convert to bytes and send
            test_data_bytes = test_data.tobytes()
            test_audio_data = base64.b64encode(test_data_bytes).decode('utf-8')
            socketio.emit('audio_data', {'audio_data': test_audio_data})
            
            # Count packets for debugging
            packet_count = 0
            
            while audio_streaming:
                try:
                    data = stream.read(CHUNK, exception_on_overflow=False)
                    
                    # Print debug info every 100 packets
                    packet_count += 1
                    if packet_count % 100 == 0:
                        print(f"Audio packet count: {packet_count}")
                    
                    # Convert audio data to base64 for sending over socketio
                    audio_data = base64.b64encode(data).decode('utf-8')
                    socketio.emit('audio_data', {'audio_data': audio_data})
                except IOError as e:
                    print(f"Audio streaming warning: {e}")
                    time.sleep(0.1)  # Add small delay on error
                    continue
                    
        except Exception as e:
            print(f"Error starting audio stream: {e}")
            audio_available = False
            
    except Exception as e:
        print(f"Error in audio streaming: {e}")
        audio_available = False
    finally:
        if 'stream' in locals() and stream:
            try:
                stream.stop_stream()
                stream.close()
            except:
                pass
        try:
            p.terminate()
        except:
            pass
        print("Audio streaming stopped")

@app.route('/')
def index():
    """Render the main page"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print('Client connected')
    # Send hardware status to client
    socketio.emit('hardware_status', {
        'camera_available': camera_available,
        'audio_available': audio_available
    })

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print('Client disconnected')
    # Stop audio playback
    audio_playback_active = False
    # Stop the robot when client disconnects
    if motor:
        motor.set_speed_mps_left(0)
        motor.set_speed_mps_right(0)

@socketio.on('keydown')
def handle_keydown(data):
    """Handle key press events"""
    if not motor:
        return
    
    key = data.get('key', '').lower()
    speed = 0.35  # Default speed
    
    if key == 'w':
        motor.set_speed_mps_left(speed)
        motor.set_speed_mps_right(speed)
    elif key == 's':
        motor.set_speed_mps_left(-speed)
        motor.set_speed_mps_right(-speed)
    elif key == 'a':
        motor.set_speed_mps_left(-speed)
        motor.set_speed_mps_right(speed)
    elif key == 'd':
        motor.set_speed_mps_left(speed)
        motor.set_speed_mps_right(-speed)

@socketio.on('keyup')
def handle_keyup(data):
    """Handle key release events"""
    if not motor:
        return
    
    # Stop the motors
    motor.set_speed_mps_left(0)
    motor.set_speed_mps_right(0)

@socketio.on('start_audio')
def handle_start_audio():
    """Start audio streaming"""
    global audio_thread, audio_streaming
    
    if not audio_available:
        return {'status': 'unavailable', 'message': 'Audio hardware not available'}
    
    if audio_thread is None or not audio_thread.is_alive():
        audio_streaming = True
        audio_thread = threading.Thread(target=audio_stream_thread)
        audio_thread.daemon = True
        audio_thread.start()
        return {'status': 'started'}
    return {'status': 'already_running'}

@socketio.on('stop_audio')
def handle_stop_audio():
    """Stop audio streaming"""
    global audio_streaming
    audio_streaming = False
    return {'status': 'stopped'}

@socketio.on('speaker_audio')
def handle_speaker_audio(data):
    """Handle audio data to be played through robot's speaker"""
    global audio_available, audio_buffer, audio_playback_thread, audio_playback_active
    
    if not audio_available:
        print("Audio output is not available")
        return
    
    # Add audio data to buffer
    try:
        # Extract audio_data from the dictionary
        if isinstance(data, dict) and 'audio_data' in data:
            # Add to buffer, don't block if buffer is full (discard)
            audio_buffer.put_nowait(data['audio_data'])
            
            # Start the playback thread if it's not running
            if audio_playback_thread is None or not audio_playback_thread.is_alive():
                audio_playback_active = True
                audio_playback_thread = threading.Thread(target=continuous_audio_playback)
                audio_playback_thread.daemon = True
                audio_playback_thread.start()
                print("Audio playback thread started")
        else:
            print(f"Invalid data format received: {type(data)}")
    except queue.Full:
        # Skip this chunk if buffer is full
        print("Audio buffer full, skipping chunk")
        
    return {'status': 'audio_queued'}

# Create a global lock for audio device access
audio_device_lock = threading.Lock()

def continuous_audio_playback():
    """Continuous audio playback thread that processes buffered audio data"""
    global audio_buffer, audio_playback_active, audio_device_lock
    
    try:
        import sounddevice as sd
        import numpy as np
        
        # Find the USB audio device
        device_name = "UACDemoV1.0"  # Name of the USB audio device
        devices = sd.query_devices()
        device_id = None
        
        # Look for the target device
        for i, device in enumerate(devices):
            if device_name in device['name'] and device['max_output_channels'] > 0:
                device_id = i
                print(f"Audio playback using device: {device['name']} (ID: {i})")
                break
        
        # If target device not found, try to use default output
        if device_id is None:
            try:
                default_device = sd.query_devices(kind='output')
                device_id = default_device['index']
                print(f"Target device not found, using default: {default_device['name']} (ID: {device_id})")
            except Exception as e:
                print(f"Error getting default device: {e}")
                return
        
        # Get device info
        device_info = sd.query_devices(device_id)
        device_sample_rate = int(device_info['default_samplerate'])
        channels = min(2, device_info['max_output_channels'])
        
        # Set up the audio stream
        with sd.OutputStream(samplerate=device_sample_rate, 
                            device=device_id,
                            channels=channels,
                            blocksize=2048) as stream:
            
            print(f"Audio stream started: {device_sample_rate} Hz, {channels} channels")
            
            # Buffer to accumulate samples for smoother playback
            stream_buffer = np.zeros((8192, channels), dtype=np.float32) if channels > 1 else np.zeros(8192, dtype=np.float32)
            buffer_pos = 0
            
            # Acquire the lock once for the entire continuous playback
            if audio_device_lock.acquire(timeout=1.0):
                try:
                    while audio_playback_active:
                        try:
                            # Get audio data from buffer with small timeout
                            base64_data = audio_buffer.get(timeout=0.1)
                            
                            # Decode and process audio
                            audio_data = base64.b64decode(base64_data)
                            samples = np.frombuffer(audio_data, dtype=np.int16)
                            
                            # Convert to float32 normalized between -1 and 1
                            samples_float = samples.astype(np.float32) / 32767.0
                            
                            # Amplify the sound with clipping protection
                            volume_factor = 20.0  # Increased from 10 to 20
                            samples_float = np.clip(samples_float * volume_factor, -1.0, 1.0)
                            
                            # Resample if needed
                            input_sample_rate = 44100
                            if abs(input_sample_rate - device_sample_rate) > 100:
                                import scipy.signal
                                number_of_samples = int(round(len(samples_float) * float(device_sample_rate) / input_sample_rate))
                                samples_float = scipy.signal.resample(samples_float, number_of_samples)
                            
                            # Convert mono to stereo if needed
                            if channels == 2 and len(samples_float.shape) == 1:
                                samples_float = np.column_stack((samples_float, samples_float))
                            
                            # Write to stream in chunks to avoid buffer underruns
                            stream.write(samples_float)
                            
                        except queue.Empty:
                            # No data in queue, add a short silence to prevent buffer underruns
                            silence = np.zeros((1024, channels), dtype=np.float32) if channels > 1 else np.zeros(1024, dtype=np.float32)
                            stream.write(silence)
                            
                            # If buffer has been empty for a while, consider exiting
                            if audio_buffer.empty():
                                # Add some trailing silence for smooth ending
                                stream.write(silence)  # Add more silence
                                time.sleep(0.2)  # Short delay
                                if audio_buffer.empty():  # Double-check it's still empty
                                    break
                        
                        except Exception as e:
                            print(f"Error in audio playback: {e}")
                            time.sleep(0.1)  # Short delay on error
                finally:
                    # Release the lock when playback is complete
                    audio_device_lock.release()
            else:
                print("Could not acquire audio device lock for continuous playback")
                
    except Exception as e:
        print(f"Error in continuous audio playback thread: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up
        audio_playback_active = False
        print("Audio playback thread stopped")

if __name__ == '__main__':
    try:
        print("Starting Facetime Robot Interface")
        print("="*50)
        print("This example allows you to:")
        print("- Drive the robot using WASD keys")
        print("- View the camera feed")
        print("- Listen to the robot's microphone")
        print("- Talk through the robot's speaker")
        print("="*50)
        
        # Initialize hardware
        success = initialize_hardware()
        if success:
            print("Hardware initialized successfully")
            # Print status of camera and audio
            print(f"Camera available: {camera_available}")
            print(f"Audio available: {audio_available}")
            # Start the Flask app
            print(f"Open http://[ROBOT_IP]:5000 in your web browser")
            
            # Run without debug mode to prevent camera issues on reload
            socketio.run(app, host='0.0.0.0', port=5000, debug=False)
        else:
            print("Failed to initialize hardware")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        cleanup()
