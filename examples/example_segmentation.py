import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from ultralytics import YOLO
from lib.camera import StereoCamera
import time

os.makedirs("yolo_segment_output", exist_ok=True)
camera = StereoCamera(scale=0.75)
model = YOLO("yolo11s-seg.pt")

total_frames = 0
try:
    while True:
        start_time = time.time()
        left_image, _ = camera.get_stereo()
        
        if left_image is None:
            time.sleep(0.5)
            continue
        
        results = model(source=left_image, conf=0.25, iou=0.45, verbose=False)
        annotated_image = results[0].plot(labels=True, boxes=False, masks=True)
        
        cv2.imwrite(f"yolo_segment_output/frame_{total_frames:04d}.jpg", annotated_image)
        
        proc_time = time.time() - start_time
        total_frames += 1
        num_objects = len(results[0].boxes)
        num_masks = len(results[0].masks) if hasattr(results[0], 'masks') and results[0].masks is not None else 0
        
        print(f"Frame {total_frames} | Objects: {num_objects} | Masks: {num_masks} | Time: {proc_time*1000:.1f}ms")
        
except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    camera.release()
    print("Camera released")