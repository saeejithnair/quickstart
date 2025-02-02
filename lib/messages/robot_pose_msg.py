"""Auto-generated MQTT message class from TOML configuration config/bot_quickstart_msgs.toml at 2025-02-01 21:01:24.899280."""

import json

from lib.messages.mqtt_message_base import MqttMessageBase


class ROBOT_POSE_MSG(MqttMessageBase):
    """MQTT message class for ROBOT_POSE."""

    timestamp: float = None
    x_m: float = None
    y_m: float = None
    theta_rad: float = None

    def __init__(self, timestamp: float | None = None, x_m: float | None = None, y_m: float | None = None, theta_rad: float | None = None):
        """Initialize the message class with given fields."""
        self.timestamp = timestamp
        self.x_m = x_m
        self.y_m = y_m
        self.theta_rad = theta_rad

    def convert_to_payload(self) -> str:
        """Convert the message fields to a JSON payload."""
        try:
            data = {
                'timestamp': self.timestamp,
                'x_m': self.x_m,
                'y_m': self.y_m,
                'theta_rad': self.theta_rad,
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
            self.x_m = data['x_m']
            self.y_m = data['y_m']
            self.theta_rad = data['theta_rad']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}') from e
