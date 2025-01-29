import cv2


class USBCamera:
    def __init__(self, index=0):
        # Initialize USB camera
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise Exception("Could not open video device")
        # Set properties if needed
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame
    
    def release(self):
        self.cap.release()

if __name__ == "__main__":
    usb = USBCamera()
