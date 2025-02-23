# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from examples.wrapper_camera import StereoCamera

# Load calibration data
calib_data = np.load('../tests/stereo_calibration.npz')
left_map1 = calib_data['left_maps_x']
left_map2 = calib_data['left_maps_y']
right_map1 = calib_data['right_maps_x']
right_map2 = calib_data['right_maps_y']

# Create stereo matcher
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=64,
    blockSize=7,
    P1=8 * 3 * 7**2,
    P2=32 * 3 * 7**2,
    disp12MaxDiff=1,
    uniquenessRatio=15,
    speckleWindowSize=100,
    speckleRange=32
)

# Initialize camera
camera = StereoCamera()

# Capture and process images
left = camera.get_left()
right = camera.get_right()

if left is not None and right is not None:
    # Convert to grayscale
    left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    
    # Rectify images
    left_rect = cv2.remap(left_gray, left_map1, left_map2, cv2.INTER_LINEAR)
    right_rect = cv2.remap(right_gray, right_map1, right_map2, cv2.INTER_LINEAR)
    
    # Compute disparity
    disparity = stereo.compute(left_rect, right_rect).astype(np.float32) / 16.0
    
    # Create color disparity map
    disparity_colored = cv2.applyColorMap(
        cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8),
        cv2.COLORMAP_JET
    )
    
    # Save results
    cv2.imwrite("depth_map.jpg", disparity_colored)
    print("Depth map saved as depth_map.jpg")
else:
    print("Unable to capture stereo images from the camera.")

# Clean up
camera.release()
