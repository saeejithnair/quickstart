# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Import the package utility and ensure required packages are installed
from lib.package_utils import ensure_package

# Check and install required packages
ensure_package("numpy")
# For depth calculation we need the contrib modules (ximgproc)
ensure_package("cv2", "opencv-contrib-python")
ensure_package("matplotlib")

import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
from lib.camera import StereoCamera

#############################################
# ADJUSTABLE PARAMETERS - Modify as needed
#############################################

# Image scaling (smaller = faster but less detailed)
SCALE = 0.5  # 1.0 = full resolution, 0.5 = half, etc.

# Rectification strength (1.0 = full rectification, 0.0 = no rectification)
RECTIFICATION_STRENGTH = 0.2  # Reduce this value to make rectification less aggressive

# Camera parameters
FOCAL_LENGTH_MM = 2.53  # mm (from datasheet)
PIXEL_SIZE_UM = 3.0     # μm (from datasheet)
BASELINE_MM = 64.5      # mm (actual measured camera-to-camera distance)

# Stereo matching parameters
MIN_DISPARITY = -32      # Minimum disparity (negative allows for objects crossing from left to right)
NUM_DISPARITIES = 160    # Range of disparity values (must be divisible by 16)
BLOCK_SIZE = 9          # Size of matching block (odd number: 5, 7, 9, 11...)
UNIQUENESS_RATIO = 5    # Lower values produce more matches but more noise
P1 = 8*3*BLOCK_SIZE**2  # Controls disparity smoothness (first parameter)
P2 = 32*3*BLOCK_SIZE**2 # Controls disparity smoothness (second parameter)
SPECKLE_WINDOW = 200    # Maximum area of speckles to remove
SPECKLE_RANGE = 2       # Maximum disparity variation in speckle region

# Post-processing options
ENABLE_WLS_FILTER = True  # Whether to use weighted least squares filter
WLS_LAMBDA = 8000         # Controls smoothing strength of WLS filter
WLS_SIGMA = 1.2           # Controls edge preservation of WLS filter

# Visualization options
USE_COLORMAP = cv2.COLORMAP_INFERNO  # COLORMAP_JET, COLORMAP_INFERNO, COLORMAP_VIRIDIS, etc.
SAVE_DEBUG_IMAGES = True             # Save additional debug visualizations

# Preprocessing options
USE_HISTOGRAM_EQ = True     # Apply histogram equalization
USE_CLAHE = True            # Apply Contrast Limited Adaptive Histogram Equalization
USE_GAUSSIAN_BLUR = True    # Apply Gaussian blur
BLUR_KERNEL_SIZE = 3        # Size of Gaussian blur kernel (odd number: 3, 5, 7...)

# Generate debug filenames with timestamp
timestamp = time.strftime("%Y%m%d_%H%M%S")
debug_dir = f"depth_debug_{timestamp}"
if SAVE_DEBUG_IMAGES:
    os.makedirs(debug_dir, exist_ok=True)

print(f"Starting stereo depth processing with:")
print(f"- Scale: {SCALE}")
print(f"- Rectification strength: {RECTIFICATION_STRENGTH}")
print(f"- Disparity range: {MIN_DISPARITY} to {MIN_DISPARITY + NUM_DISPARITIES}")
print(f"- Block size: {BLOCK_SIZE}")
print(f"- WLS filter: {'Enabled' if ENABLE_WLS_FILTER else 'Disabled'}")

# Load calibration data
print("Loading calibration data...")
calib_data = np.load('../lib/stereo_calibration.npz')
left_map1 = calib_data['left_maps_x']
left_map2 = calib_data['left_maps_y']
right_map1 = calib_data['right_maps_x']
right_map2 = calib_data['right_maps_y']

# Scale calibration maps if needed
if SCALE != 1.0:
    h, w = left_map1.shape
    new_h, new_w = int(h * SCALE), int(w * SCALE)
    # Scale maps using proper interpolation
    left_map1 = cv2.resize(left_map1, (new_w, new_h), interpolation=cv2.INTER_LINEAR) * SCALE
    left_map2 = cv2.resize(left_map2, (new_w, new_h), interpolation=cv2.INTER_LINEAR) * SCALE
    right_map1 = cv2.resize(right_map1, (new_w, new_h), interpolation=cv2.INTER_LINEAR) * SCALE
    right_map2 = cv2.resize(right_map2, (new_w, new_h), interpolation=cv2.INTER_LINEAR) * SCALE

# Initialize camera
camera = StereoCamera()

def preprocess_image(img):
    """Apply pre-processing to enhance image for stereo matching."""
    if SCALE != 1.0:
        h, w = img.shape[:2]
        new_h, new_w = int(h * SCALE), int(w * SCALE)
        img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    
    # Convert to grayscale
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    
    # Apply Gaussian blur to reduce noise
    if USE_GAUSSIAN_BLUR:
        gray = cv2.GaussianBlur(gray, (BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0)
    
    # Apply histogram equalization
    if USE_HISTOGRAM_EQ and not USE_CLAHE:
        gray = cv2.equalizeHist(gray)
    
    # Apply CLAHE (better than simple histogram equalization)
    if USE_CLAHE:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
    
    return gray

def save_debug_images(left, right, left_rect, right_rect, disparity_raw, disparity_processed, filename_suffix="debug"):
    """Save debug visualizations to help diagnose stereo matching issues."""
    if not SAVE_DEBUG_IMAGES:
        return
    
    # Normalize disparities for visualization
    if disparity_raw is not None:
        disp_raw_viz = cv2.normalize(disparity_raw, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        cv2.imwrite(f"{debug_dir}/disparity_raw_{filename_suffix}.jpg", disp_raw_viz)
        
    if disparity_processed is not None:
        disp_processed_viz = cv2.normalize(disparity_processed, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        # Basic grayscale visualization
        cv2.imwrite(f"{debug_dir}/disparity_norm_{filename_suffix}.jpg", disp_processed_viz)
        # Colored visualization
        disp_color = cv2.applyColorMap(disp_processed_viz, USE_COLORMAP)
        cv2.imwrite(f"{debug_dir}/disparity_color_{filename_suffix}.jpg", disp_color)
        
        # Create depth overlay (30% original, 70% depth)
        if left is not None:
            left_resized = cv2.resize(left, (disp_color.shape[1], disp_color.shape[0]))
            overlay = cv2.addWeighted(left_resized, 0.3, disp_color, 0.7, 0)
            cv2.imwrite(f"{debug_dir}/depth_overlay_{filename_suffix}.jpg", overlay)
    
    # Save original and rectified images
    if left is not None:
        cv2.imwrite(f"{debug_dir}/left_orig_{filename_suffix}.jpg", left)
    if right is not None:
        cv2.imwrite(f"{debug_dir}/right_orig_{filename_suffix}.jpg", right)
    if left_rect is not None:
        cv2.imwrite(f"{debug_dir}/left_rect_{filename_suffix}.jpg", left_rect)
    if right_rect is not None:
        cv2.imwrite(f"{debug_dir}/right_rect_{filename_suffix}.jpg", right_rect)
    
    # Create side-by-side comparison with horizontal lines
    if left_rect is not None and right_rect is not None:
        # Convert to color if grayscale
        if len(left_rect.shape) == 2:
            left_rect_color = cv2.cvtColor(left_rect, cv2.COLOR_GRAY2BGR)
            right_rect_color = cv2.cvtColor(right_rect, cv2.COLOR_GRAY2BGR)
        else:
            left_rect_color = left_rect.copy()
            right_rect_color = right_rect.copy()
        
        # Draw horizontal lines for epipolar visualization
        height = left_rect.shape[0]
        for y in range(0, height, 50):
            cv2.line(left_rect_color, (0, y), (left_rect.shape[1], y), (0, 255, 0), 1)
            cv2.line(right_rect_color, (0, y), (right_rect.shape[1], y), (0, 255, 0), 1)
        
        # Create side-by-side image
        side_by_side = np.hstack((left_rect_color, right_rect_color))
        cv2.imwrite(f"{debug_dir}/rectified_epipolar_{filename_suffix}.jpg", side_by_side)
    
    # Create anaglyph 3D image (red-cyan)
    if left_rect is not None and right_rect is not None:
        if len(left_rect.shape) == 2:
            anaglyph = np.zeros((left_rect.shape[0], left_rect.shape[1], 3), dtype=np.uint8)
            anaglyph[:,:,0] = 0  # Blue channel
            anaglyph[:,:,1] = 0  # Green channel
            anaglyph[:,:,2] = left_rect  # Red channel - left eye
            
            anaglyph[:,:,0] = right_rect  # Blue channel - right eye
            anaglyph[:,:,1] = right_rect  # Green channel - right eye
            
            cv2.imwrite(f"{debug_dir}/anaglyph_3d_{filename_suffix}.jpg", anaglyph)

# Create stereo matcher
left_matcher = cv2.StereoSGBM_create(
    minDisparity=MIN_DISPARITY,
    numDisparities=NUM_DISPARITIES,
    blockSize=BLOCK_SIZE,
    P1=P1,
    P2=P2,
    disp12MaxDiff=1,
    uniquenessRatio=UNIQUENESS_RATIO,
    speckleWindowSize=SPECKLE_WINDOW,
    speckleRange=SPECKLE_RANGE,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_HH4  # HH4 mode for better quality
)

# Create right matcher for WLS filter
if ENABLE_WLS_FILTER:
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
    
    # Create and configure WLS filter
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
    wls_filter.setLambda(WLS_LAMBDA)
    wls_filter.setSigmaColor(WLS_SIGMA)

def compute_disparity(left_rect, right_rect):
    """Compute disparity map with configured parameters."""
    # Compute left disparity
    left_disp = left_matcher.compute(left_rect, right_rect)
    
    if ENABLE_WLS_FILTER:
        # Compute right disparity for WLS filtering
        right_disp = right_matcher.compute(right_rect, left_rect)
        
        # Apply WLS filtering
        filtered_disp = wls_filter.filter(
            left_disp, left_rect, disparity_map_right=right_disp
        ).astype(np.float32) / 16.0
        
        return left_disp.astype(np.float32) / 16.0, filtered_disp
    else:
        return left_disp.astype(np.float32) / 16.0, left_disp.astype(np.float32) / 16.0

# Function to compute disparity and visualize results
def process_stereo_frame(left, right, frame_count=0):
    """Process stereo frames to produce disparity map."""
    # Apply preprocessing
    left_processed = preprocess_image(left)
    right_processed = preprocess_image(right)
    
    # Rectify images
    left_rect_full = cv2.remap(left_processed, left_map1, left_map2, cv2.INTER_LINEAR)
    right_rect_full = cv2.remap(right_processed, right_map1, right_map2, cv2.INTER_LINEAR)
    
    # Blend rectified images with original based on rectification strength
    if RECTIFICATION_STRENGTH < 1.0:
        left_rect = cv2.addWeighted(left_rect_full, RECTIFICATION_STRENGTH, 
                                   left_processed, 1.0 - RECTIFICATION_STRENGTH, 0)
        right_rect = cv2.addWeighted(right_rect_full, RECTIFICATION_STRENGTH, 
                                    right_processed, 1.0 - RECTIFICATION_STRENGTH, 0)
    else:
        left_rect = left_rect_full
        right_rect = right_rect_full
    
    # Calculate disparity
    raw_disp, filtered_disp = compute_disparity(left_rect, right_rect)
    
    # Apply additional post-processing if needed
    # Additional filtering can be added here
    
    # Save debug information periodically
    if SAVE_DEBUG_IMAGES and frame_count % 30 == 0:
        # Save both full rectification and blended rectification for comparison
        if RECTIFICATION_STRENGTH < 1.0:
            # Save the original full rectification too for comparison
            save_debug_images(left, right, left_rect_full, right_rect_full, 
                             raw_disp, filtered_disp, f"{frame_count}_full_rect")
        save_debug_images(left, right, left_rect, right_rect, raw_disp, filtered_disp, f"{frame_count}")
    
    # Create colored visualization
    normalized_disp = cv2.normalize(filtered_disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    colored_disp = cv2.applyColorMap(normalized_disp, USE_COLORMAP)
    
    # Create depth overlay
    overlay = cv2.addWeighted(
        cv2.resize(left, (colored_disp.shape[1], colored_disp.shape[0])),
        0.3, colored_disp, 0.7, 0
    )
    
    return colored_disp, normalized_disp, overlay, filtered_disp

# Main loop
print("Press Ctrl+C to stop")
try:
    frame_count = 0
    start_time = time.time()
    fps_counter = 0
    
    while True:
        # Capture stereo frames
        left, right = camera.get_stereo()
        
        if left is None or right is None:
            print("Failed to capture images")
            continue
        
        # Process frames
        colored_disp, raw_disp, overlay, disparity = process_stereo_frame(left, right, frame_count)
        
        # Display information every second
        fps_counter += 1
        if time.time() - start_time >= 1.0:
            fps = fps_counter / (time.time() - start_time)
            print(f"FPS: {fps:.1f}, Frame: {frame_count}, "
                 f"Disparity range: {disparity.min():.1f} to {disparity.max():.1f}")
            fps_counter = 0
            start_time = time.time()
        
        # Save current outputs
        cv2.imwrite("depth_map.jpg", colored_disp)
        cv2.imwrite("depth_overlay.jpg", overlay)
        cv2.imwrite("disparity_raw.jpg", raw_disp)
        
        frame_count += 1
        
except KeyboardInterrupt:
    print("\nStopping...")
except Exception as e:
    print(f"Error: {e}")
finally:
    camera.release()
    print("Shutdown complete")
