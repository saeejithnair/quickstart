import cv2

class StereoCamera:
    def __init__(self, device_id=0):
        # Open the camera with V4L2 backend
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)

        # Set MJPEG format
        success_fourcc = self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # Set resolution to 2560x720
        success_width = self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        success_height = self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        # Set frame rate to 30 FPS
        success_fps = self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Check if settings were successful
        print(f"Set FOURCC to MJPG: {success_fourcc}")
        print(f"Set width to 2560: {success_width}")
        print(f"Set height to 720: {success_height}")
        print(f"Set FPS to 30: {success_fps}")

        # Verify actual settings
        fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = ''.join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"Actual FOURCC: {fourcc_str}")
        print(f"Actual Resolution: {width}x{height}")
        print(f"Actual FPS: {fps}")

    def get_stereo(self):
        ret, frame = self.cap.read()
        if not ret:
            return None, None
        # Assuming stereo image is split horizontally
        return frame[:, :1280], frame[:, 1280:]

    def release(self):
        self.cap.release()

# Example usage
if __name__ == "__main__":
    camera = StereoCamera(0)
    left, right = camera.get_stereo()
    if left is not None and right is not None:
        cv2.imwrite("left.jpg", left)
        cv2.imwrite("right.jpg", right)
        print("Saved stereo images to left.jpg and right.jpg")
    else:
        print("Failed to capture stereo images")
    camera.release()