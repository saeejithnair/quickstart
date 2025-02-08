import time
from sshkeyboard import listen_keyboard, stop_listening

from lib.messages.mqtt_utils import MQTTPublisher
from lib.messages.target_velocity_msg import TARGET_VELOCITY_MSG
from lib.messages.topic_to_message_type import TOPIC_TARGET_VELOCITY

# ------------------------------------------------------------------------------------
# Constants & Setup
# ------------------------------------------------------------------------------------
mqtt_publisher = MQTTPublisher(broker_address="localhost", topic_to_message_map={TOPIC_TARGET_VELOCITY: TARGET_VELOCITY_MSG})
mqtt_publisher.run()

def press(key):
    if key.lower() == 'w':  # Forward
        mqtt_publisher.publish_msg(TOPIC_TARGET_VELOCITY, TARGET_VELOCITY_MSG(time.time(), 0.25, 0.0))
    elif key.lower() == 's':  # Backward
        mqtt_publisher.publish_msg(TOPIC_TARGET_VELOCITY, TARGET_VELOCITY_MSG(time.time(), -0.25, 0.0))
    elif key.lower() == 'a':  # Left turn
        mqtt_publisher.publish_msg(TOPIC_TARGET_VELOCITY, TARGET_VELOCITY_MSG(time.time(), 0.0, 10.0))
    elif key.lower() == 'd':  # Right turn
        mqtt_publisher.publish_msg(TOPIC_TARGET_VELOCITY, TARGET_VELOCITY_MSG(time.time(), 0.0, -10.0))
    elif key.lower() == 'q':  # Quit
        stop_listening()

def release(key):
    print(f"Released key: {key}")
    # Stop motors when key is released
    mqtt_publisher.publish_msg(TOPIC_TARGET_VELOCITY, TARGET_VELOCITY_MSG(time.time(), 0.0, 0.0))

try:
    print("WASD to control, Q to quit")
    listen_keyboard(
        on_press=press,
        on_release=release,
        # delay_second_char=0.1,
        # delay_other_chars=0.01,
        # sequential=True,
    )

except Exception as e:
    print(f"Error: {e}")
finally:
    # Clean up
    mqtt_publisher.publish_msg(TOPIC_TARGET_VELOCITY, TARGET_VELOCITY_MSG(time.time(), 0.0, 0.0))
    mqtt_publisher.stop()
    print("Shutdown complete.")
