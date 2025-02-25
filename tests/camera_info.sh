#!/bin/bash
# camera_info.sh - Script to display detailed information about connected cameras
# 
# This script collects and displays information about USB cameras connected to the system,
# including device details, supported formats, resolutions, and media topology.
#
# Usage: ./camera_info.sh

# Set text colors for better readability
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========== USB CAMERA DETECTION SCRIPT ==========${NC}"
echo -e "${BLUE}Date:${NC} $(date)"
echo -e "${BLUE}System:${NC} $(uname -a)"
echo

# Find USB cameras
echo -e "${GREEN}=== USB CAMERAS CONNECTED ===${NC}"
lsusb | grep -i camera
echo

# List video devices
echo -e "${GREEN}=== VIDEO DEVICES ===${NC}"
v4l2-ctl --list-devices
echo

# Function to get detailed info for a device
get_device_info() {
    local device=$1
    local device_num=$(echo $device | sed 's/\/dev\/video//')
    
    echo -e "${YELLOW}=== DEVICE: $device ===${NC}"
    
    # Check if device exists
    if [ ! -e $device ]; then
        echo "Device $device does not exist"
        return
    fi
    
    # Get device capabilities
    echo -e "${BLUE}--- DEVICE CAPABILITIES ---${NC}"
    v4l2-ctl --device=$device --info
    echo
    
    # Get all device settings
    echo -e "${BLUE}--- DEVICE SETTINGS ---${NC}"
    v4l2-ctl --device=$device --all | grep -v "^$"
    echo
    
    # Get supported formats
    echo -e "${BLUE}--- SUPPORTED FORMATS ---${NC}"
    v4l2-ctl --device=$device --list-formats-ext
    echo
    
    # Try to get controls
    echo -e "${BLUE}--- DEVICE CONTROLS ---${NC}"
    v4l2-ctl --device=$device --list-ctrls
    echo
    
    # Check if there's a corresponding media device
    local media_dev=$(find /dev/media* -type c 2>/dev/null | sort)
    if [ -n "$media_dev" ]; then
        for mdev in $media_dev; do
            # Try to find if this media device is related to our video device
            if media-ctl -d $mdev -p 2>/dev/null | grep -q $device; then
                echo -e "${BLUE}--- MEDIA TOPOLOGY FOR $mdev ---${NC}"
                media-ctl -d $mdev -p
                echo
                break
            fi
        done
    fi
}

# Get info for video0 (main camera)
get_device_info "/dev/video0"

# Get info for video1 (metadata node if exists)
get_device_info "/dev/video1"

echo -e "${GREEN}========== END OF CAMERA INFORMATION ==========${NC}"

# Print a summary of the stereo camera setup
echo -e "${GREEN}=== STEREO CAMERA SUMMARY ===${NC}"
echo -e "${BLUE}Main video device:${NC} /dev/video0"
echo -e "${BLUE}Current resolution:${NC} $(v4l2-ctl --device=/dev/video0 --get-fmt-video | grep "Width/Height" | awk '{print $3}')"
echo -e "${BLUE}Current format:${NC} $(v4l2-ctl --device=/dev/video0 --get-fmt-video | grep "Pixel Format" | awk '{print $4}')"
echo -e "${BLUE}Current FPS:${NC} $(v4l2-ctl --device=/dev/video0 --get-parm | grep "Frames per second" | awk '{print $4}')"
echo
echo "To use this camera in the StereoCamera class:"
echo "  - Device: /dev/video0"
echo "  - Resolution: 2560x720"
echo "  - Format: MJPG"
echo "  - FPS: 30"
echo
echo "The stereo image is split with:"
echo "  - Left image: first 1280 pixels horizontally"
echo "  - Right image: remaining 1280 pixels horizontally" 