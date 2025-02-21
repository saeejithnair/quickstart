#!/usr/bin/env python3

import tomli
import tomli_w
from datetime import datetime
import re
import os
from typing import Dict, Any, List, Tuple
import logging
import subprocess

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class SystemConfigMonitor:
    def __init__(self, config_file: str = "config_monitor.toml"):
        self.config_file = config_file
        self.config = self._load_config()

    def _load_config(self) -> Dict[str, Any]:
        """Load the TOML configuration file."""
        try:
            with open(self.config_file, "rb") as f:
                return tomli.load(f)
        except FileNotFoundError:
            logger.error(f"Configuration file {self.config_file} not found")
            raise
        except Exception as e:
            logger.error(f"Error loading configuration: {e}")
            raise

    def _save_config(self) -> None:
        """Save the current configuration back to the TOML file."""
        try:
            with open(self.config_file, "wb") as f:
                tomli_w.dump(self.config, f)
        except Exception as e:
            logger.error(f"Error saving configuration: {e}")
            raise

    def _run_command(self, command: str) -> Tuple[str, int]:
        """Run a shell command and return its output and return code."""
        try:
            result = subprocess.run(
                command.split(),
                capture_output=True,
                text=True
            )
            return result.stdout, result.returncode
        except Exception as e:
            logger.error(f"Error running command {command}: {e}")
            return str(e), -1

    def check_audio_devices(self) -> bool:
        """Check for audio input and output devices."""
        # Check playback devices
        output, rc = self._run_command("aplay -l")
        playback_devices = []
        if rc == 0:
            for line in output.split('\n'):
                if line.startswith('card '):
                    device = line.split(':')[1].strip()
                    playback_devices.append(device)
        
        # Check recording devices
        output, rc = self._run_command("arecord -l")
        recording_devices = []
        if rc == 0:
            for line in output.split('\n'):
                if line.startswith('card '):
                    device = line.split(':')[1].strip()
                    recording_devices.append(device)

        # Update the configuration
        self._update_status(
            'audio', 'playback',
            ', '.join(playback_devices) if playback_devices else "none"
        )
        self._update_status(
            'audio', 'recording',
            ', '.join(recording_devices) if recording_devices else "none"
        )

        return bool(playback_devices or recording_devices)

    def check_property(self, section: str, subsection: str) -> bool:
        """
        Check a specific property in the system configuration.
        Returns True if the property is found and matches expected value.
        """
        # Special handling for audio devices
        if section == 'audio':
            return self.check_audio_devices()

        config_section = self.config.get(section, {}).get(subsection)
        if not config_section:
            logger.error(f"Section {section}.{subsection} not found in config")
            return False

        file_path = config_section["file_path"]
        search_pattern = config_section["search_pattern"]

        try:
            if not os.path.exists(file_path):
                logger.error(f"File {file_path} does not exist")
                return self._update_status(section, subsection, "file_not_found")

            with open(file_path, 'r') as f:
                lines = f.readlines()

            for i, line in enumerate(lines, 1):
                match = re.search(search_pattern, line.strip())
                if match:
                    value = match.group(1)
                    self._update_status(
                        section, subsection,
                        value,
                        line_number=i
                    )
                    return True

            self._update_status(section, subsection, "not_found")
            return False

        except Exception as e:
            logger.error(f"Error checking property: {e}")
            self._update_status(section, subsection, f"error: {str(e)}")
            return False

    def _update_status(self, section: str, subsection: str, status: str, line_number: int = 0) -> bool:
        """Update the status of a property in the configuration."""
        try:
            # Create section if it doesn't exist
            if section not in self.config:
                self.config[section] = {}
            if subsection not in self.config[section]:
                self.config[section][subsection] = {}

            self.config[section][subsection]["current_status"] = status
            # Format the datetime in a human-readable format
            current_time = datetime.now()
            formatted_time = current_time.strftime("%B %d, %Y at %I:%M:%S %p")
            self.config[section][subsection]["last_checked"] = formatted_time
            if line_number:
                self.config[section][subsection]["line_number"] = line_number
            self._save_config()
            return True
        except Exception as e:
            logger.error(f"Error updating status: {e}")
            return False

    def check_all_properties(self) -> Dict[str, Any]:
        """Check all properties defined in the configuration."""
        results = {}
        for section in self.config:
            results[section] = {}
            for subsection in self.config[section]:
                results[section][subsection] = self.check_property(section, subsection)
        
        # Always check audio devices
        if 'audio' not in results:
            results['audio'] = {}
            results['audio']['devices'] = self.check_audio_devices()
        
        return results

def main():
    monitor = SystemConfigMonitor()
    results = monitor.check_all_properties()
    logger.info("Monitoring results:")
    for section, subsections in results.items():
        for subsection, result in subsections.items():
            logger.info(f"{section}.{subsection}: {result}")

if __name__ == "__main__":
    main() 