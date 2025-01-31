"""Auto-generated MQTT message class from TOML configuration config/bot_quickstart_msgs.toml at 2025-01-31 01:02:13.372907."""

import json

from lib.messages.mqtt_message_base import MqttMessageBase


class LOCALIZATION_INITIALIZED_MSG(MqttMessageBase):
    """MQTT message class for LOCALIZATION_INITIALIZED."""

    timestamp: float = None
    initialized: bool = None

    def __init__(self, timestamp: float | None = None, initialized: bool | None = None):
        """Initialize the message class with given fields."""
        self.timestamp = timestamp
        self.initialized = initialized

    def convert_to_payload(self) -> str:
        """Convert the message fields to a JSON payload."""
        try:
            data = {
                'timestamp': self.timestamp,
                'initialized': self.initialized,
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
            self.initialized = data['initialized']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}') from e
