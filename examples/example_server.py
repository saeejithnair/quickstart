#!/usr/bin/env python3
"""
FastAPI server with endpoints for robot control:
- Drive straight (forward/backward)
- Turn (left/right)
- Play audio from speaker
- Capture image from stereo camera

Dependencies:
- fastapi: Web framework
- uvicorn: ASGI server
- pydantic: Data validation
- sounddevice: Audio playback
- numpy: Audio processing
- opencv-python: Image processing
- python-multipart: For file uploads/handling

Install with:
pip install fastapi uvicorn pydantic sounddevice numpy opencv-python python-multipart
"""

# Add the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Import the package utility and ensure required packages are installed
from lib.package_utils import ensure_package

# Check and install required packages
ensure_package("fastapi")
ensure_package("uvicorn")
ensure_package("pydantic")
ensure_package("numpy")
ensure_package("sounddevice")
ensure_package("cv2", "opencv-python")
ensure_package("python-multipart")

# Standard imports
import json
import time
import base64
from typing import Optional, List
import threading

# Third-party imports
import numpy as np
from fastapi import FastAPI, HTTPException, BackgroundTasks, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
import cv2

# Try to import audio libraries
try:
    import sounddevice as sd
    AUDIO_AVAILABLE = True
except ImportError:
    print("Warning: sounddevice not available. Audio functions will be disabled.")
    AUDIO_AVAILABLE = False

# Try to import camera libraries
try:
    from lib.camera import StereoCamera
    CAMERA_AVAILABLE = True
except ImportError:
    print("Warning: StereoCamera not available. Camera functions will be simulated.")
    CAMERA_AVAILABLE = False

# Camera setup - initialize it once following example_yolo.py's approach
camera = None
if CAMERA_AVAILABLE:
    try:
        # Initialize with scale=0.5 as in example_yolo.py
        # camera = StereoCamera(scale=0.5)
        print("Stereo camera initialized successfully with scale=0.5")
    except Exception as e:
        print(f"Error initializing stereo camera: {e}")
        CAMERA_AVAILABLE = False

# Import robot control libraries
try:
    from lib.odrive_uart import ODriveUART
    DRIVE_AVAILABLE = True
except ImportError:
    print("Warning: ODriveUART not available. Driving functions will be simulated.")
    DRIVE_AVAILABLE = False

# Constants
WHEEL_BASE = 0.4  # in meters, distance between wheels
LINEAR_SPEED = 0.2  # in m/s
ANGULAR_SPEED = 1.2  # in rad/s

# Create the FastAPI app
app = FastAPI(
    title="Robot Control API",
    description="API for controlling robot movement and audio playback",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize motor controller if available
if DRIVE_AVAILABLE:
    try:
        # Load motor directions from JSON
        with open(os.path.expanduser('~/quickstart/lib/motor_dir.json'), 'r') as f:
            motor_dirs = json.load(f)
        
        # Initialize motor controller
        motor_controller = ODriveUART(
            port='/dev/ttyAMA1',
            left_axis=0, right_axis=1,
            dir_left=motor_dirs['left'], dir_right=motor_dirs['right']
        )
        
        # Start motors and set mode
        motor_controller.start_left()
        motor_controller.start_right()
        motor_controller.enable_velocity_mode_left()
        motor_controller.enable_velocity_mode_right()
        motor_controller.disable_watchdog_left()
        motor_controller.disable_watchdog_right()
        
        # Clear motor errors
        motor_controller.clear_errors_left()
        motor_controller.clear_errors_right()
    except Exception as e:
        print(f"Error initializing motor controller: {e}")
        DRIVE_AVAILABLE = False

# Data models
class DriveCommand(BaseModel):
    """Model for driving commands"""
    linear_velocity: float = 0.0  # m/s
    angular_velocity: float = 0.0  # rad/s
    duration: Optional[float] = None  # seconds, if None: continuous

class AudioCommand(BaseModel):
    """Model for audio playback commands"""
    frequency: float = 440.0  # Hz, for beep tones
    duration: float = 1.0  # seconds
    volume: float = 1.0  # 0.0 to 1.0
    
# Utility functions
def set_velocity(linear, angular):
    """Set motor velocities based on linear and angular components"""
    if not DRIVE_AVAILABLE:
        print(f"SIMULATION: Set speeds: Linear={linear} m/s, Angular={angular} rad/s")
        return
    
    # Calculate individual wheel speeds
    left = linear - (WHEEL_BASE / 2) * angular
    right = linear + (WHEEL_BASE / 2) * angular
    
    # Set motor speeds
    motor_controller.set_speed_mps_left(left)
    motor_controller.set_speed_mps_right(right)
    print(f"Set speeds: Left={left} m/s, Right={right} m/s")

def stop_motors():
    """Stop both motors"""
    set_velocity(0, 0)

def play_audio_in_background(audio_data, samplerate=24000):
    """Play audio data in a separate thread"""
    if not AUDIO_AVAILABLE:
        print("SIMULATION: Playing audio...")
        time.sleep(1)  # Simulate audio playback time
        print("SIMULATION: Audio playback complete")
        return
    
    # Play the audio
    sd.play(audio_data, samplerate=samplerate)
    sd.wait()  # Wait until audio playback is finished

def generate_tone(frequency, duration=1.0, volume=1.0, sample_rate=24000):
    """Generate a simple sine wave tone"""
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    tone = volume * np.sin(frequency * 2 * np.pi * t)
    return tone.astype(np.float32)

def get_camera_image():
    """Get image from the left camera using the same approach as example_yolo.py"""
    camera = StereoCamera(scale=0.5)
    if not CAMERA_AVAILABLE or camera is None:
        print("SIMULATION: Capturing camera image...")
        # Create a sample image for simulation (black square with text)
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(img, "Camera Simulation", (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        return img
    
    try:
        # Looking at example_yolo.py and example_follow.py, they initialize the camera with scale=0.5
        # Make sure we're using the camera the same way as in the examples
        left_image, _ = camera.get_stereo()
        print(left_image)
        
        # Debug information to help diagnose the issue
        print(f"Camera image captured: {'Success' if left_image is not None else 'Failed'}")
        if left_image is not None:
            print(f"Image shape: {left_image.shape}")
        
        if left_image is None:
            # If camera capture failed, return a black image with error message
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(img, "Camera Error", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return img
            
        return left_image
    except Exception as e:
        print(f"Error capturing image: {e}")
        # If an exception occurred, return a black image with error message
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(img, f"Error: {str(e)}", (100, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return img

# API Routes
@app.get("/")
def read_root():
    """Root endpoint with API information"""
    return {
        "message": "Robot Control API",
        "endpoints": {
            "drive": "POST /drive - Control robot movement",
            "stop": "GET /stop - Stop robot movement",
            "forward": "GET /forward - Move forward",
            "backward": "GET /backward - Move backward",
            "left": "GET /left - Turn left",
            "right": "GET /right - Turn right",
            "beep": "GET /beep - Play a beep sound",
            "image": "GET /image - Get current camera image",
            "image_base64": "GET /image/base64 - Get base64 encoded camera image"
        },
        "status": {
            "drive": DRIVE_AVAILABLE,
            "audio": AUDIO_AVAILABLE,
            "camera": CAMERA_AVAILABLE
        }
    }

@app.post("/drive")
def drive(command: DriveCommand, background_tasks: BackgroundTasks):
    """Control robot movement with precise velocity values"""
    # Cap velocities at safe limits
    linear = max(min(command.linear_velocity, LINEAR_SPEED), -LINEAR_SPEED)
    angular = max(min(command.angular_velocity, ANGULAR_SPEED), -ANGULAR_SPEED)
    
    # Set motor speeds
    set_velocity(linear, angular)
    
    # If duration specified, schedule stopping
    if command.duration is not None:
        def stop_after_delay():
            time.sleep(command.duration)
            stop_motors()
        
        background_tasks.add_task(stop_after_delay)
        return {"message": f"Driving with linear={linear} m/s, angular={angular} rad/s for {command.duration} seconds"}
    
    return {"message": f"Driving with linear={linear} m/s, angular={angular} rad/s continuously"}

@app.get("/stop")
def stop():
    """Stop all robot movement"""
    stop_motors()
    return {"message": "Robot stopped"}

@app.get("/forward")
def forward(speed: float = LINEAR_SPEED, duration: Optional[float] = None, background_tasks: BackgroundTasks = None):
    """Move robot forward at specified speed"""
    # Ensure speed is positive and capped
    speed = min(abs(speed), LINEAR_SPEED)
    
    # Set motor speeds
    set_velocity(speed, 0)
    
    # If duration specified, schedule stopping
    if duration is not None and background_tasks is not None:
        def stop_after_delay():
            time.sleep(duration)
            stop_motors()
        
        background_tasks.add_task(stop_after_delay)
        return {"message": f"Moving forward at {speed} m/s for {duration} seconds"}
    
    return {"message": f"Moving forward at {speed} m/s continuously"}

@app.get("/backward")
def backward(speed: float = LINEAR_SPEED, duration: Optional[float] = None, background_tasks: BackgroundTasks = None):
    """Move robot backward at specified speed"""
    # Ensure speed is positive (direction is negative)
    speed = min(abs(speed), LINEAR_SPEED)
    
    # Set motor speeds
    set_velocity(-speed, 0)
    
    # If duration specified, schedule stopping
    if duration is not None and background_tasks is not None:
        def stop_after_delay():
            time.sleep(duration)
            stop_motors()
        
        background_tasks.add_task(stop_after_delay)
        return {"message": f"Moving backward at {speed} m/s for {duration} seconds"}
    
    return {"message": f"Moving backward at {speed} m/s continuously"}

@app.get("/left")
def left(speed: float = ANGULAR_SPEED, duration: Optional[float] = None, background_tasks: BackgroundTasks = None):
    """Turn robot left at specified angular speed"""
    # Ensure speed is positive and capped
    speed = min(abs(speed), ANGULAR_SPEED)
    
    # Set motor speeds
    set_velocity(0, speed)
    
    # If duration specified, schedule stopping
    if duration is not None and background_tasks is not None:
        def stop_after_delay():
            time.sleep(duration)
            stop_motors()
        
        background_tasks.add_task(stop_after_delay)
        return {"message": f"Turning left at {speed} rad/s for {duration} seconds"}
    
    return {"message": f"Turning left at {speed} rad/s continuously"}

@app.get("/right")
def right(speed: float = ANGULAR_SPEED, duration: Optional[float] = None, background_tasks: BackgroundTasks = None):
    """Turn robot right at specified angular speed"""
    # Ensure speed is positive (direction is negative)
    speed = min(abs(speed), ANGULAR_SPEED)
    
    # Set motor speeds
    set_velocity(0, -speed)
    
    # If duration specified, schedule stopping
    if duration is not None and background_tasks is not None:
        def stop_after_delay():
            time.sleep(duration)
            stop_motors()
        
        background_tasks.add_task(stop_after_delay)
        return {"message": f"Turning right at {speed} rad/s for {duration} seconds"}
    
    return {"message": f"Turning right at {speed} rad/s continuously"}

@app.post("/beep")
async def beep(command: AudioCommand, background_tasks: BackgroundTasks):
    """Play a beep tone through the speaker"""
    if not AUDIO_AVAILABLE:
        return {"message": f"SIMULATION: Beeping at {command.frequency}Hz for {command.duration}s"}
    
    # Generate tone
    tone = generate_tone(command.frequency, command.duration, command.volume)
    
    # Play audio in background
    background_tasks.add_task(play_audio_in_background, tone)
    
    return {"message": f"Playing tone at {command.frequency}Hz for {command.duration}s"}

@app.get("/beep")
async def beep_get(
    frequency: float = 440.0,  # Hz (A4 note)
    duration: float = 1.0,     # seconds
    volume: float = 0.5,       # 0.0 to 1.0
    background_tasks: BackgroundTasks = None
):
    """Play a beep tone through the speaker (GET method)"""
    if not AUDIO_AVAILABLE:
        return {"message": f"SIMULATION: Beeping at {frequency}Hz for {duration}s"}
    
    # Generate tone
    tone = generate_tone(frequency, duration, volume)
    
    # Play audio in background
    background_tasks.add_task(play_audio_in_background, tone)
    
    return {"message": f"Playing tone at {frequency}Hz for {duration}s"}

@app.get("/image")
async def get_image(
    format: str = "jpeg",  # Format: jpeg, png
    quality: int = 90      # JPEG quality (1-100)
):
    """Get current camera image as binary data"""
    # Get image from camera
    img = get_camera_image()
    
    # Validate quality parameter
    quality = max(1, min(100, quality))
    
    # Encode image
    if format.lower() == "png":
        success, encoded_img = cv2.imencode(".png", img)
        media_type = "image/png"
    else:  # Default to JPEG
        success, encoded_img = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        media_type = "image/jpeg"
    
    if not success:
        raise HTTPException(status_code=500, detail="Failed to encode image")
    
    # Return as a simple Response with the raw bytes
    return Response(content=encoded_img.tobytes(), media_type=media_type)

@app.get("/image/base64")
async def get_image_base64(
    format: str = "jpeg",  # Format: jpeg, png
    quality: int = 90      # JPEG quality (1-100)
):
    """Get current camera image as base64 encoded string"""
    # Get image from camera
    img = get_camera_image()
    
    # Validate quality parameter
    quality = max(1, min(100, quality))
    
    # Encode image
    if format.lower() == "png":
        success, encoded_img = cv2.imencode(".png", img)
        img_format = "png"
    else:  # Default to JPEG
        success, encoded_img = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        img_format = "jpeg"
    
    if not success:
        raise HTTPException(status_code=500, detail="Failed to encode image")
    
    # Convert to base64
    base64_img = base64.b64encode(encoded_img.tobytes()).decode('utf-8')
    
    # Return as JSON with base64 data
    return {
        "format": img_format,
        "base64": base64_img,
        "data_uri": f"data:image/{img_format};base64,{base64_img}"
    }

# Run the server when the script is executed
if __name__ == "__main__":
    import uvicorn
    print("Starting Robot Control API server...")
    uvicorn.run("example_server:app", host="0.0.0.0", port=8000)
