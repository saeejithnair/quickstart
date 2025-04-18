import cv2
import numpy as np
import argparse
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("-c", dest='cam', type=int, default=1)
args = parser.parse_args()
addr = subprocess.check_output(['ipconfig', 'getifaddr', 'en0']).decode().strip()
# addr = '192.168.0.102'
rtmp_url = f"rtmp://{addr}/live"
num_cams = args.cam
cams = [f"cam{i+1}" for i in range(num_cams)]
frame_width = 640
frame_height = 360


print(f"RTMP server address: {addr}")
print(f"number of cameras: {args.cam}")

streams = {
    cam: f"{rtmp_url}/{cam}" for cam in cams
}

caps = {}
for name, url in streams.items():
    cam = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    # cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cam.set(cv2.CAP_PROP_FPS, 30)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    caps[name] = cam

while True:
    frames = []
    for name in streams.keys():
        cap = caps[name]
        ret, frame = cap.read()
        if not ret:
            print(f"No frame from {name}")
            frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
        frames.append(frame)

    frame = np.vstack((frames[0], frames[1]))
    cv2.imshow("Stream", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

for cap in caps.values():
    cap.release()

cv2.destroyAllWindows()

