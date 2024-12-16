import unittest
import threading
import time
import json
import queue
import paho.mqtt.client as mqtt
import base64
import cv2
import numpy as np
import sys
import os
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s.%(msecs)03d %(levelname)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from lib.realsense import RealSense

class TestRealsensePublishing(unittest.TestCase):
    """Test suite for RealSense MQTT publishing functionality."""

    def setUp(self):
        """Set up test environment with mock RealSense."""
        self.test_topic = "realsense/test/frames"
        self.msg_queue = queue.Queue()
        
        # Create RealSense in mock mode
        self.realsense = RealSense(
            mqtt_topic=self.test_topic,
            mock=True  # Use mock data instead of real camera
        )
        
        # Create primary subscriber
        self.subscriber = self.create_subscriber()
        time.sleep(0.5)  # Allow time for MQTT connections

    def tearDown(self):
        """Clean up test resources."""
        if hasattr(self, 'subscriber'):
            self.subscriber.loop_stop()
            self.subscriber.disconnect()
            
        if hasattr(self, 'realsense'):
            self.realsense.stop()
            
        # Clear message queue
        while not self.msg_queue.empty():
            self.msg_queue.get_nowait()

    def create_subscriber(self, name="test_sub", message_queue=None):
        """Create an MQTT subscriber."""
        client = mqtt.Client(
            client_id=name,
            protocol=mqtt.MQTTv5,
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2
        )
        
        queue_to_use = message_queue if message_queue is not None else self.msg_queue
        
        def on_message(client, userdata, message):
            try:
                payload = json.loads(message.payload.decode())
                queue_to_use.put({
                    'timestamp': time.time(),
                    'payload': payload
                })
            except json.JSONDecodeError:
                logger.error(f"Invalid JSON payload on topic {message.topic}")
        
        client.on_message = on_message
        client.connect("localhost", 1883, 60)
        client.subscribe(self.test_topic)
        client.loop_start()
        return client

    def test_single_subscriber_receives_frames(self):
        """Test basic publish/subscribe functionality."""
        # Start publishing
        publish_thread = threading.Thread(target=self.realsense.run_continuous)
        publish_thread.daemon = True
        publish_thread.start()

        try:
            # Collect messages for 2 seconds
            time.sleep(2)
            self.realsense.running = False
            publish_thread.join(timeout=1)

            # Get messages
            messages = []
            while not self.msg_queue.empty():
                messages.append(self.msg_queue.get())

            # Verify we got messages
            self.assertGreater(len(messages), 0, "No messages received")
            logger.info(f"Received {len(messages)} messages")

            # Verify message structure and sequence
            prev_time = None
            for msg in messages:
                self.assertIn('timestamp', msg['payload'])
                self.assertIn('color_frame', msg['payload'])
                self.assertIn('depth_frame', msg['payload'])
                
                if prev_time:
                    self.assertGreater(
                        msg['payload']['timestamp'],
                        prev_time,
                        "Messages out of order"
                    )
                prev_time = msg['payload']['timestamp']

            # Check frame rate
            duration = messages[-1]['payload']['timestamp'] - messages[0]['payload']['timestamp']
            fps = len(messages) / duration
            logger.info(f"Measured FPS: {fps:.1f}")
            self.assertGreater(fps, 25, "Frame rate too low")
            self.assertLess(fps, 35, "Frame rate too high")

        except Exception as e:
            self.realsense.running = False
            publish_thread.join(timeout=1)
            raise e

    def test_multiple_subscribers_receive_same_frames(self):
        """Test multiple subscribers receiving frames."""
        # Create two additional subscribers
        sub_queues = [queue.Queue() for _ in range(2)]
        subscribers = [
            self.create_subscriber(f"sub_{i}", q)
            for i, q in enumerate(sub_queues)
        ]

        try:
            # Start publishing
            publish_thread = threading.Thread(target=self.realsense.run_continuous)
            publish_thread.daemon = True
            publish_thread.start()

            # Collect messages for 2 seconds
            time.sleep(2)
            self.realsense.running = False
            publish_thread.join(timeout=1)

            # Get messages from all subscribers
            all_messages = []
            for q in [self.msg_queue] + sub_queues:
                messages = []
                while not q.empty():
                    messages.append(q.get())
                all_messages.append(messages)

            # Verify all subscribers got messages
            for i, messages in enumerate(all_messages):
                self.assertGreater(len(messages), 0, f"Subscriber {i} got no messages")
                logger.info(f"Subscriber {i} received {len(messages)} messages")

            # Compare message counts (should be within 10%)
            counts = [len(msgs) for msgs in all_messages]
            self.assertLess(max(counts) - min(counts), max(counts) * 0.1,
                          "Message counts too different between subscribers")

        finally:
            for sub in subscribers:
                sub.loop_stop()
                sub.disconnect()

    def test_late_subscriber_behavior(self):
        """Test subscriber joining mid-stream."""
        # Start publishing
        publish_thread = threading.Thread(target=self.realsense.run_continuous)
        publish_thread.daemon = True
        publish_thread.start()

        try:
            # Let it run for 1 second
            time.sleep(1)
            start_time = time.time()

            # Add late subscriber
            late_queue = queue.Queue()
            late_sub = self.create_subscriber("late_sub", late_queue)

            # Collect for 1 more second
            time.sleep(1)
            self.realsense.running = False
            publish_thread.join(timeout=1)

            # Get messages from late subscriber
            late_messages = []
            while not late_queue.empty():
                late_messages.append(late_queue.get())

            # Verify late subscriber only got new messages
            self.assertGreater(len(late_messages), 0, "Late subscriber got no messages")
            for msg in late_messages:
                self.assertGreater(
                    msg['payload']['timestamp'],
                    start_time,
                    "Late subscriber received old message"
                )

            logger.info(f"Late subscriber received {len(late_messages)} messages")

        finally:
            late_sub.loop_stop()
            late_sub.disconnect()

if __name__ == '__main__':
    unittest.main()
