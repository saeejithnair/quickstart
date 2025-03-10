# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# https://docs.ultralytics.com/guides/raspberry-pi/
import cv2
from ultralytics import YOLO
from examples.wrapper_camera import StereoCamera

camera = StereoCamera()

model = YOLO("yolo11n.pt")
model.export(format="ncnn")

ncnn_model = YOLO("yolo11n_ncnn_model")

left_image = camera.get_left()

if left_image is not None:
    results = ncnn_model(left_image)

    annotated_image = results[0].plot()

    cv2.imwrite("labeled_left.jpg", annotated_image)
    print("Labeled image saved as labeled_left.jpg")
else:
    print("Unable to capture the left image from the stereo camera.")