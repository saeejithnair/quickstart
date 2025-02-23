import cv2

class StereoCamera:
    def __init__(self, device_id=0):
        self.cap = cv2.VideoCapture(device_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def get_frames(self):
        ret, f = self.cap.read()
        return f if ret else None

    def get_left(self):
        f = self.get_frames()
        return f[:, :1280] if f is not None else None

    def get_right(self):
        f = self.get_frames()
        return f[:, 1280:] if f is not None else None
        
    def save(self, prefix="stereo", split=False):
        f = self.get_frames()
        if f is not None:
            cv2.imwrite(f"{prefix}_full.jpg", f)
            if split:
                cv2.imwrite(f"{prefix}_left.jpg", f[:, :1280])
                cv2.imwrite(f"{prefix}_right.jpg", f[:, 1280:])

    def release(self):
        self.cap.release()
        
    def __del__(self):
        self.release()

