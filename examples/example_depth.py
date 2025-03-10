# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from time import time
from lib.camera import StereoCamera

# Experiment with different scale factors (1.0 = original size, 0.5 = half size, etc.)
SCALE = 0.5

calib_data = np.load('../tests/stereo_calibration.npz')
left_map1 = calib_data['left_maps_x'] 
left_map2 = calib_data['left_maps_y']
right_map1 = calib_data['right_maps_x']
right_map2 = calib_data['right_maps_y']

# Scale calibration maps if we're scaling the image
if SCALE != 1.0:
    h, w = left_map1.shape
    new_h, new_w = int(h * SCALE), int(w * SCALE)
    # Scale maps using proper interpolation
    left_map1 = cv2.resize(left_map1, (new_w, new_h), interpolation=cv2.INTER_LINEAR) * SCALE
    left_map2 = cv2.resize(left_map2, (new_w, new_h), interpolation=cv2.INTER_LINEAR) * SCALE
    right_map1 = cv2.resize(right_map1, (new_w, new_h), interpolation=cv2.INTER_LINEAR) * SCALE
    right_map2 = cv2.resize(right_map2, (new_w, new_h), interpolation=cv2.INTER_LINEAR) * SCALE

# Create stereo matcher with optimized parameters for OV9281
left_matcher = cv2.StereoSGBM_create(
    minDisparity=int(-32 * SCALE),  # Scale disparity parameters
    numDisparities=int(192 * SCALE),  # Scale disparity range
    blockSize=7,         # Keep block size constant
    P1=300,             # Higher for smoother results
    P2=1200,            # P2 = 4*P1 for better edges
    disp12MaxDiff=1,
    uniquenessRatio=20,  # Higher to reduce noise
    speckleWindowSize=100,
    speckleRange=24,     # Increased for better filtering
    preFilterCap=63,     # Maximum for better contrast
    mode=cv2.STEREO_SGBM_MODE_HH4  # High-quality mode
)

# Create right matcher for WLS
right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

# Create WLS filter
wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
wls_filter.setLambda(8000)  # Weight of smoothness
wls_filter.setSigmaColor(1.5)  # Edge sensitivity

camera = StereoCamera()

# Timing dictionary
timing = {
    'capture': 0,
    'preprocess': 0,
    'stereo': 0,
    'total': 0
}

try:
    print(f"Starting depth calculation (Scale factor: {SCALE})")
    print("Press Ctrl+C to stop")
    
    last_print = time()
    frames = 0
    
    while True:
        loop_start = time()
        
        # Capture both frames at once
        t_start = time()
        left, right = camera.get_stereo()
        timing['capture'] = time() - t_start
        
        if left is None or right is None:
            print("Failed to capture images")
            continue
        
        # Preprocessing
        t_start = time()
        
        # Scale down images if needed
        if SCALE != 1.0:
            h, w = left.shape[:2]
            new_h, new_w = int(h * SCALE), int(w * SCALE)
            left = cv2.resize(left, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
            right = cv2.resize(right, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        
        # Convert to grayscale
        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        
        # Enhance contrast
        left_gray = cv2.equalizeHist(left_gray)
        right_gray = cv2.equalizeHist(right_gray)
        
        # Rectify
        left_rect = cv2.remap(left_gray, left_map1, left_map2, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right_gray, right_map1, right_map2, cv2.INTER_LINEAR)
        timing['preprocess'] = time() - t_start
        
        # Compute disparity
        t_start = time()
        
        # Compute both disparities
        left_disp = left_matcher.compute(left_rect, right_rect)
        right_disp = right_matcher.compute(right_rect, left_rect)
        
        # Apply WLS filtering
        filtered_disp = wls_filter.filter(
            left_disp, left_rect, disparity_map_right=right_disp
        ).astype(np.float32) / 16.0
        
        # Scale disparity back to original scale
        if SCALE != 1.0:
            filtered_disp = filtered_disp / SCALE
        
        # Create colored depth map
        disparity_colored = cv2.applyColorMap(
            cv2.normalize(filtered_disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8),
            cv2.COLORMAP_JET
        )
        timing['stereo'] = time() - t_start
        
        # Update timing
        timing['total'] = time() - loop_start
        frames += 1
        
        # Print stats every second
        if time() - last_print >= 1.0:
            fps = frames / (time() - last_print)
            print("\nTiming (ms):")
            print(f"  Capture:    {timing['capture']*1000:6.1f}")
            print(f"  Preprocess: {timing['preprocess']*1000:6.1f}")
            print(f"  Stereo:     {timing['stereo']*1000:6.1f}")
            print(f"  Total:      {timing['total']*1000:6.1f}")
            print(f"FPS: {fps:.1f}")
            last_print = time()
            frames = 0
        
        # Save latest depth map
        cv2.imwrite("depth_map.jpg", disparity_colored)

except KeyboardInterrupt:
    print("\nStopping...")
except Exception as e:
    print(f"Error: {e}")
finally:
    camera.release()
    print("Shutdown complete")
