import pyrealsense2 as rs
import numpy as np
import cv2
import time
import paho.mqtt.client as mqtt
import json
import base64
import logging

logger = logging.getLogger(__name__)

class RealSense:
    def __init__(self, mqtt_broker="localhost", mqtt_port=1883, mqtt_topic="realsense/frames", mock=False):
        """Initialize RealSense camera and MQTT publisher.
        
        Args:
            mqtt_broker: MQTT broker address
            mqtt_port: MQTT broker port
            mqtt_topic: MQTT topic for publishing frames
            mock: If True, use mock data instead of real camera
        """
        self.mqtt_topic = mqtt_topic
        self.mock = mock
        self.running = False
        self.pipeline_started = False
        
        # Initialize these only when needed
        self.pipeline = None
        self.config = None
        
        # MQTT setup with version 2 API
        self.client = mqtt.Client(
            protocol=mqtt.MQTTv5,
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2
        )
        self.client.connect(mqtt_broker, mqtt_port, 60)
        self.client.loop_start()
        logger.info(f"MQTT client connected to {mqtt_broker}:{mqtt_port}")

    def _init_camera(self):
        """Initialize the RealSense camera (lazy initialization)."""
        if self.pipeline is None:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Configure streams
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    def _get_mock_frames(self):
        """Generate mock frames for testing."""
        color_image = np.zeros((480, 640, 3), dtype=np.uint8)
        depth_image = np.zeros((480, 640), dtype=np.uint16)
        # Add some pattern to make it look like real data
        color_image[200:300, 200:300] = [0, 255, 0]
        depth_image[200:300, 200:300] = 1000
        return color_image, depth_image

    def start(self):
        """Start the RealSense pipeline if not already started."""
        if self.mock:
            self.running = True
            logger.info("Started in mock mode")
            return

        if not self.pipeline_started:
            self._init_camera()
            self.pipeline.start(self.config)
            self.pipeline_started = True
            logger.info("RealSense pipeline started")
        self.running = True

    def stop(self):
        """Stop the RealSense pipeline and MQTT client."""
        self.running = False
        
        if self.pipeline_started:
            try:
                self.pipeline.stop()
                self.pipeline_started = False
                logger.info("RealSense pipeline stopped")
            except Exception as e:
                logger.error(f"Error stopping pipeline: {str(e)}")
        
        try:
            self.client.loop_stop()
            self.client.disconnect()
            logger.info("MQTT client disconnected")
        except Exception as e:
            logger.error(f"Error disconnecting MQTT: {str(e)}")

    def get_frames(self):
        """Get a single pair of frames."""
        if self.mock:
            return self._get_mock_frames()
            
        if not self.pipeline_started:
            self.start()
            
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                logger.warning("Failed to get valid frames")
                return None, None
                
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            return color_image, depth_image
            
        except Exception as e:
            logger.error(f"Error getting frames: {str(e)}")
            return None, None

    def get_colorized_depth(self, depth_image):
        """Convert depth image to colorized version."""
        return cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    def publish_frames(self, color_image, depth_image):
        """Publish frames to MQTT topic."""
        if not self.running:
            return
            
        try:
            # Convert images to jpg for smaller payload
            _, color_encoded = cv2.imencode('.jpg', color_image)
            depth_colormap = self.get_colorized_depth(depth_image)
            _, depth_encoded = cv2.imencode('.jpg', depth_colormap)
            
            # Create payload
            payload = {
                'timestamp': time.time(),
                'color_frame': base64.b64encode(color_encoded).decode('utf-8'),
                'depth_frame': base64.b64encode(depth_encoded).decode('utf-8')
            }
            
            self.client.publish(self.mqtt_topic, json.dumps(payload))
            
        except Exception as e:
            logger.error(f"Error publishing frames: {str(e)}")

    def run_continuous(self):
        """Run continuous frame capture and publishing."""
        self.start()
        try:
            while self.running:
                color_image, depth_image = self.get_frames()
                if color_image is not None and depth_image is not None:
                    self.publish_frames(color_image, depth_image)
                else:
                    logger.warning("Skipping invalid frames")
                time.sleep(0.033)  # ~30 fps
        except Exception as e:
            logger.error(f"Error in continuous run: {str(e)}")
        finally:
            self.stop()
