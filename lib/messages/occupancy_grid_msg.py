"""Auto-generated MQTT message class from TOML configuration config/bot_quickstart_msgs.toml at 2025-01-26 15:08:55.893518."""

import json

from lib.messages.mqtt_message_base import MqttMessageBase


class OCCUPANCY_GRID_MSG(MqttMessageBase):
    """MQTT message class for OCCUPANCY_GRID."""

    timestamp: float = None
    width: int = None
    flattened_grid_list: list = None

    def __init__(self, timestamp: float | None = None, width: int | None = None, flattened_grid_list: list | None = None):
        """Initialize the message class with given fields."""
        self.timestamp = timestamp
        self.width = width
        self.flattened_grid_list = flattened_grid_list

    def convert_to_payload(self) -> str:
        """Convert the message fields to a JSON payload."""
        try:
            data = {
                'timestamp': self.timestamp,
                'width': self.width,
                'flattened_grid_list': self.flattened_grid_list,
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
            self.width = data['width']
            self.flattened_grid_list = data['flattened_grid_list']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}') from e
