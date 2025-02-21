#!/usr/bin/env python3

import tomlkit
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

    def _load_config(self) -> 'tomlkit.TOMLDocument':
        """Load the TOML configuration file. Create it if it doesn't exist."""
        # Create initial config with tomlkit structures
        initial_config = tomlkit.document()
        boot = tomlkit.table()
        boot['i2c_firmware'] = tomlkit.table()
        boot['i2c_firmware']['file_path'] = '/boot/firmware/config.txt'
        boot['i2c_firmware']['search_pattern'] = '^dtparam=i2c_arm=(.+)'
        boot['i2c_firmware']['line_number'] = 0
        boot['i2c_firmware']['last_checked'] = ''
        boot['i2c_firmware']['current_status'] = ''
        boot['spi_firmware'] = tomlkit.table()
        boot['spi_firmware']['file_path'] = '/boot/firmware/config.txt'
        boot['spi_firmware']['search_pattern'] = '^dtparam=spi=(.+)'
        boot['spi_firmware']['line_number'] = 0
        boot['spi_firmware']['last_checked'] = ''
        boot['spi_firmware']['current_status'] = ''
        boot['uart_enable'] = tomlkit.table()
        boot['uart_enable']['file_path'] = '/boot/firmware/config.txt'
        boot['uart_enable']['search_pattern'] = '^enable_uart=(.+)'
        boot['uart_enable']['line_number'] = 0
        boot['uart_enable']['last_checked'] = ''
        boot['uart_enable']['current_status'] = ''
        boot['uart0_overlay'] = tomlkit.table()
        boot['uart0_overlay']['file_path'] = '/boot/firmware/config.txt'
        boot['uart0_overlay']['search_pattern'] = '^dtoverlay=uart0,(.+)'
        boot['uart0_overlay']['line_number'] = 0
        boot['uart0_overlay']['last_checked'] = ''
        boot['uart0_overlay']['current_status'] = ''
        boot['uart1_overlay'] = tomlkit.table()
        boot['uart1_overlay']['file_path'] = '/boot/firmware/config.txt'
        boot['uart1_overlay']['search_pattern'] = '^dtoverlay=uart1,(.+)'
        boot['uart1_overlay']['line_number'] = 0
        boot['uart1_overlay']['last_checked'] = ''
        boot['uart1_overlay']['current_status'] = ''
        initial_config['boot'] = boot

        audio = tomlkit.table()
        playback = tomlkit.table()
        playback['last_checked'] = ''
        playback['devices'] = tomlkit.array()
        audio['playback'] = playback
        recording = tomlkit.table()
        recording['last_checked'] = ''
        recording['devices'] = tomlkit.array()
        audio['recording'] = recording
        initial_config['audio'] = audio

        try:
            with open(self.config_file, "r") as f:
                return tomlkit.parse(f.read())
        except FileNotFoundError:
            logger.info(f"Configuration file {self.config_file} not found, creating initial configuration")
            with open(self.config_file, "w") as f:
                f.write(tomlkit.dumps(initial_config))
            return initial_config
        except Exception as e:
            logger.error(f"Error loading configuration: {e}")
            raise

    def _save_config(self) -> None:
        """Save the current configuration back to the TOML file."""
        try:
            with open(self.config_file, "w") as f:
                f.write(tomlkit.dumps(self.config))
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

    def _get_usb_info(self) -> Dict[str, Dict[str, str]]:
        """Get USB device information."""
        output, rc = self._run_command("lsusb")
        usb_info = {}
        if rc == 0:
            for line in output.split('\n'):
                if 'Audio' in line or 'CMTECK' in line or 'Sound' in line:
                    match = re.search(r'ID ([0-9a-f]{4}:[0-9a-f]{4}) (.+)', line)
                    if match:
                        vid_pid, name = match.groups()
                        usb_info[name] = {
                            'id': vid_pid,
                            'full_name': line.strip()
                        }
        return usb_info

    def check_audio_devices(self) -> bool:
        """Check for audio input and output devices."""
        usb_info = self._get_usb_info()
        
        # Check playback devices
        output, rc = self._run_command("aplay -l")
        playback_devices = []
        if rc == 0:
            for line in output.split('\n'):
                if line.startswith('card '):
                    # Get ALSA info
                    alsa_info = line.split(':')[1].strip()
                    hw_string = f'hw:{line.split()[1].strip(",")}'
                    
                    # Try to match with USB info
                    device_info = {'alsa_name': alsa_info, 'hw_string': hw_string, 'usb_info': {}}
                    for usb_name, usb_data in usb_info.items():
                        if any(word in alsa_info for word in usb_name.split()):
                            device_info['usb_info'] = usb_data.copy()
                    playback_devices.append(device_info)
        
        # Check recording devices
        output, rc = self._run_command("arecord -l")
        recording_devices = []
        if rc == 0:
            for line in output.split('\n'):
                if line.startswith('card '):
                    # Get ALSA info
                    alsa_info = line.split(':')[1].strip()
                    hw_string = f'hw:{line.split()[1].strip(",")}'
                    
                    # Try to match with USB info
                    device_info = {'alsa_name': alsa_info, 'hw_string': hw_string, 'usb_info': {}}
                    for usb_name, usb_data in usb_info.items():
                        if any(word in alsa_info for word in usb_name.split()):
                            device_info['usb_info'] = usb_data.copy()
                    recording_devices.append(device_info)

        # Update the configuration with detailed information
        current_time = datetime.now()
        formatted_time = current_time.strftime("%B %d, %Y at %I:%M:%S %p")

        # Create audio section if it doesn't exist
        if 'audio' not in self.config:
            self.config['audio'] = tomlkit.table()

        # Build playback section
        playback = tomlkit.table()
        playback['last_checked'] = formatted_time
        playback_devices_array = tomlkit.array()
        for dev in playback_devices:
            inline = tomlkit.inline_table()
            inline['alsa_name'] = dev['alsa_name']
            inline['hw_string'] = dev['hw_string']
            usb_info_inline = tomlkit.inline_table()
            if dev['usb_info']:
                usb_info_inline['id'] = dev['usb_info'].get('id', '')
                usb_info_inline['full_name'] = dev['usb_info'].get('full_name', '')
            inline['usb_info'] = usb_info_inline
            playback_devices_array.append(inline)
        playback['devices'] = playback_devices_array

        # Build recording section
        recording = tomlkit.table()
        recording['last_checked'] = formatted_time
        recording_devices_array = tomlkit.array()
        for dev in recording_devices:
            inline = tomlkit.inline_table()
            inline['alsa_name'] = dev['alsa_name']
            inline['hw_string'] = dev['hw_string']
            usb_info_inline = tomlkit.inline_table()
            if dev['usb_info']:
                usb_info_inline['id'] = dev['usb_info'].get('id', '')
                usb_info_inline['full_name'] = dev['usb_info'].get('full_name', '')
            inline['usb_info'] = usb_info_inline
            recording_devices_array.append(inline)
        recording['devices'] = recording_devices_array

        self.config['audio']['playback'] = playback
        self.config['audio']['recording'] = recording

        self._save_config()
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
                self.config[section] = tomlkit.table()
            if subsection not in self.config[section]:
                self.config[section][subsection] = tomlkit.table()

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
                if subsection not in ['playback', 'recording']:  # Skip audio devices as they're handled differently
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