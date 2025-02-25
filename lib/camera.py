import cv2

class StereoCamera:
    def __init__(self, device_id=0, scale=1.0):
        # Open the camera with V4L2 backend
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        
        # Store the scale factor
        self.scale = scale

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
        
        if scale != 1.0:
            print(f"Images will be scaled by factor: {scale}")

    def set_scale(self, scale):
        """Set the scaling factor for the stereo images"""
        self.scale = scale
        print(f"Scale factor set to: {scale}")
        
    def get_scale(self):
        """Get the current scaling factor"""
        return self.scale

    def get_stereo(self, scale=None):
        """
        Get stereo images from the camera.
        
        Args:
            scale: Optional scaling factor to override the default scale.
                  If None, uses the scale set during initialization.
        
        Returns:
            Tuple of (left_image, right_image), or (None, None) if capture failed.
        """
        ret, frame = self.cap.read()
        if not ret:
            return None, None
            
        # Split the frame horizontally into left and right images
        left = frame[:, :1280]
        right = frame[:, 1280:]
        
        # Apply scaling if needed
        use_scale = scale if scale is not None else self.scale
        if use_scale != 1.0:
            h, w = left.shape[:2]
            new_h, new_w = int(h * use_scale), int(w * use_scale)
            left = cv2.resize(left, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
            right = cv2.resize(right, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
            
        return left, right

    def release(self):
        self.cap.release()

# Example usage
if __name__ == "__main__":
    camera = StereoCamera(0, scale=0.5)  # Initialize with 50% scaling
    left, right = camera.get_stereo()
    if left is not None and right is not None:
        cv2.imwrite("left.jpg", left)
        cv2.imwrite("right.jpg", right)
        print("Saved stereo images to left.jpg and right.jpg")
        
        # Try with a different scale
        left_full, right_full = camera.get_stereo(scale=1.0)
        cv2.imwrite("left_full.jpg", left_full)
        cv2.imwrite("right_full.jpg", right_full)
        print("Saved full-size stereo images")
    else:
        print("Failed to capture stereo images")
    camera.release()