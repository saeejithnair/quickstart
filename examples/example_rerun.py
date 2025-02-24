# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rerun as rr
from lib.camera import StereoCamera
import time
import cv2
import numpy as np

# Constants
SCALE = 0.5  # Scale factor for images (0.5 = half size)
BATCH_SIZE = 1  # How many frames to batch before sending

def main():
    # Initialize camera
    camera = StereoCamera()
    
    # Initialize Rerun with a unique application ID
    rr.init("stereo_stream", default_enabled=True)
    
    # Connect to the Rerun server running on your MacBook
    # Replace COMPUTER_IP with your Computer's IP address
    # to find it, run something like "ifconfig | grep 'inet ' | grep -v 127.0.0.1" on your Computer
    rr.connect("COMPUTER_IP:9876")
    
    print("\nStreaming stereo images to Rerun viewer...")
    print("Press Ctrl+C to stop")
    print(f"Scale factor: {SCALE}")
    
    try:
        frame = 0
        last_print = time.time()
        frames_since_print = 0
        
        while True:
            loop_start = time.time()
            
            # Get stereo images
            left, right = camera.get_stereo()
            if left is None or right is None:
                print("Failed to capture images")
                continue
            
            # Scale images if needed
            if SCALE != 1.0:
                h, w = left.shape[:2]
                new_h, new_w = int(h * SCALE), int(w * SCALE)
                left = cv2.resize(left, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
                right = cv2.resize(right, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
            
            # Set the current frame number as the time
            rr.set_time_sequence("frame", frame)
            
            # Log the images to Rerun
            # The paths define where they appear in the Rerun UI
            rr.log("stereo/left", rr.Image(left))
            rr.log("stereo/right", rr.Image(right))
            
            # Update counters
            frame += 1
            frames_since_print += 1
            
            # Print stats every second
            if time.time() - last_print >= 1.0:
                fps = frames_since_print / (time.time() - last_print)
                latency = (time.time() - loop_start) * 1000
                print(f"\rFPS: {fps:.1f} | Latency: {latency:.1f}ms | Frame: {frame}", end="")
                last_print = time.time()
                frames_since_print = 0
    
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        camera.release()
        print("\nShutdown complete")

if __name__ == "__main__":
    main()
