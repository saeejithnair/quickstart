"""Auto-generated MQTT message class from TOML configuration config/bot_quickstart_msgs.toml at 2025-02-01 21:01:24.899903."""

import json

from lib.messages.mqtt_message_base import MqttMessageBase


class WAVEMAP_OCCUPIED_POINTS_MSG(MqttMessageBase):
    """MQTT message class for WAVEMAP_OCCUPIED_POINTS."""

    timestamp: float = None
    occupied_points: list = None

    def __init__(self, timestamp: float | None = None, occupied_points: list | None = None):
        """Initialize the message class with given fields."""
        self.timestamp = timestamp
        self.occupied_points = occupied_points

    def convert_to_payload(self) -> str:
        """Convert the message fields to a JSON payload."""
        try:
            data = {
                'timestamp': self.timestamp,
                'occupied_points': self.occupied_points,
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
            self.occupied_points = data['occupied_points']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}') from e
