"""Auto-generated MQTT message class from TOML configuration config/bot_quickstart_msgs.toml at 2025-01-26 15:08:55.892965."""

import json

from lib.messages.mqtt_message_base import MqttMessageBase


class GYRO_DATA_MSG(MqttMessageBase):
    """MQTT message class for GYRO_DATA."""

    timestamp: float = None
    roll: float = None
    pitch: float = None
    yaw: float = None

    def __init__(self, timestamp: float | None = None, roll: float | None = None, pitch: float | None = None, yaw: float | None = None):
        """Initialize the message class with given fields."""
        self.timestamp = timestamp
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def convert_to_payload(self) -> str:
        """Convert the message fields to a JSON payload."""
        try:
            data = {
                'timestamp': self.timestamp,
                'roll': self.roll,
                'pitch': self.pitch,
                'yaw': self.yaw,
            }
            return json.dumps(data)
        except (TypeError, ValueError) as e:
            raise Exception(f'Error converting to payload: {e}') from e

    def convert_to_message(self, payload):
        """Convert a JSON payload to message fields."""
        try:
            data = json.loads(payload)
            if 'data' in data:
                data = data['data']
            self.timestamp = data['timestamp']
            self.roll = data['roll']
            self.pitch = data['pitch']
            self.yaw = data['yaw']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}') from e
