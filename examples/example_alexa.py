"""
Example script that downloads a song from YouTube as MP3 and plays it through the speaker.

This example demonstrates:
1. Downloading a YouTube video's audio using yt-dlp
2. Converting/saving it as an MP3 file
3. Playing the audio through the speaker using multiple methods (with fallbacks)

Requirements:
- yt-dlp: For downloading YouTube videos (more reliable than pytube)
- sounddevice: For audio playback (method 1)
- soundfile: For reading audio files
- numpy: For array operations
- scipy: For audio resampling
- ffmpeg: For audio conversion and fallback playback
"""

import os
import sys
import time
import subprocess
import shutil
import threading
from pathlib import Path

# Add parent directory to path to import from lib
sys.path.append(str(Path(__file__).resolve().parent.parent))
from lib.package_utils import ensure_package

# Ensure required packages are installed
ensure_package("yt-dlp")
ensure_package("sounddevice")
ensure_package("soundfile")
ensure_package("numpy")
ensure_package("scipy")

# Import packages after ensuring they're installed
import yt_dlp
import sounddevice as sd
import soundfile as sf
import numpy as np
from scipy import signal

def download_youtube_audio(youtube_url, output_path="./"):
    """
    Download audio from a YouTube video and save it as an MP3 file using yt-dlp.
    
    Args:
        youtube_url (str): The URL of the YouTube video
        output_path (str): The directory to save the audio file
    
    Returns:
        str: The path to the downloaded MP3 file
    """
    print(f"Downloading audio from: {youtube_url}")
    
    try:
        # Create download options
        ydl_opts = {
            'format': 'bestaudio/best',
            'outtmpl': os.path.join(output_path, '%(title)s.%(ext)s'),
            'postprocessors': [{
                'key': 'FFmpegExtractAudio',
                'preferredcodec': 'mp3',
                'preferredquality': '192',
            }],
            'verbose': True,
        }
        
        # Create the downloader and download the audio
        with yt_dlp.YoutubeDL(ydl_opts) as ydl:
            # Get info first to get the title
            info = ydl.extract_info(youtube_url, download=False)
            title = info.get('title', 'audio')
            print(f"Video title: {title}")
            
            # Download the file
            print(f"Downloading audio: {title}")
            ydl.download([youtube_url])
            
            # Construct the expected output file path
            expected_file = os.path.join(output_path, f"{title}.mp3")
            
            # Clean filename - replace special characters if needed
            clean_expected_file = expected_file
            if not os.path.exists(expected_file):
                # Try a more aggressive sanitization for file finding
                base_path = os.path.dirname(expected_file)
                for file in os.listdir(base_path):
                    if file.endswith(".mp3") and os.path.getmtime(os.path.join(base_path, file)) > time.time() - 60:
                        clean_expected_file = os.path.join(base_path, file)
                        print(f"Found downloaded file with sanitized name: {file}")
                        break
            
            # Check if the file exists
            if os.path.exists(clean_expected_file):
                print(f"Download complete: {clean_expected_file}")
                return clean_expected_file
            else:
                # Try to find any MP3 file in the directory that was just created
                for file in os.listdir(output_path):
                    if file.endswith(".mp3") and os.path.getmtime(os.path.join(output_path, file)) > time.time() - 60:
                        mp3_file = os.path.join(output_path, file)
                        print(f"Found downloaded file: {mp3_file}")
                        return mp3_file
                        
                print("Could not locate the downloaded file.")
                return None
        
    except Exception as e:
        print(f"Error downloading YouTube audio: {e}")
        return None

def print_current_volume():
    """Print the current system volume for debugging."""
    try:
        # This will work on many Linux systems
        result = subprocess.run(['amixer', 'sget', 'Master'], 
                               stdout=subprocess.PIPE, 
                               stderr=subprocess.PIPE,
                               text=True)
        print(f"System volume status:\n{result.stdout}")
    except:
        print("Could not retrieve system volume information")

def play_audio_sounddevice(audio_file, device_name=None, volume=1.5):
    """
    Play an audio file using sounddevice (Method 1).
    
    Args:
        audio_file (str): Path to the audio file to play
        device_name (str, optional): Name of the audio device to use. If None, use default device.
        volume (float, optional): Volume multiplier. Default is 1.5 (50% boost).
    
    Returns:
        bool: True if playback successful, False otherwise
    """
    print("\n--- PLAYBACK METHOD 1: Using sounddevice ---")
    try:
        # Set up the audio device
        if device_name:
            try:
                device_info = sd.query_devices(device_name, 'output')
                device_id = device_info['index']
                device_sample_rate = device_info['default_samplerate']
                print(f"Using audio device: {device_name} (ID: {device_id})")
            except:
                print(f"Device '{device_name}' not found. Using default device.")
                device_id = None
                device_sample_rate = sd.query_devices(None, 'output')['default_samplerate']
        else:
            device_id = None
            default_device_info = sd.query_devices(None, 'output')
            device_sample_rate = default_device_info['default_samplerate']
            print(f"Using default audio device: {default_device_info['name']}")
            
        # Load the audio file
        print(f"Loading audio file: {audio_file}")
        data, sample_rate = sf.read(audio_file, dtype='float32')
        
        print(f"Audio data shape: {data.shape}, Sample rate: {sample_rate}Hz")
        print(f"Data min/max values: {data.min():.4f}/{data.max():.4f}")
        
        # Resample if necessary
        if sample_rate != device_sample_rate:
            print(f"Resampling from {sample_rate}Hz to {device_sample_rate}Hz")
            number_of_samples = int(round(len(data) * float(device_sample_rate) / sample_rate))
            data = signal.resample(data, number_of_samples)
            sample_rate = device_sample_rate
        
        # Apply volume adjustment
        if volume != 1.0:
            print(f"Adjusting volume by factor of {volume}")
            data = data * volume
            # Clip to prevent distortion if needed
            if data.max() > 1.0 or data.min() < -1.0:
                print("Warning: Clipping audio to prevent distortion")
                data = np.clip(data, -1.0, 1.0)
        
        # Play the audio
        print(f"Playing audio: {os.path.basename(audio_file)}")
        
        # Check that audio output is working
        print("Testing audio device with a short tone")
        test_tone = np.sin(2 * np.pi * 440 * np.arange(int(device_sample_rate * 0.5)) / device_sample_rate) * 0.3
        sd.play(test_tone, samplerate=device_sample_rate, device=device_id)
        sd.wait()
        print("Test tone complete - you should have heard a short beep")
        
        # Play the actual audio
        print("Starting main audio playback...")
        sd.play(data, samplerate=sample_rate, device=device_id)
        
        # Wait for playback to finish
        print("Audio playback started. Press Ctrl+C to stop...")
        sd.wait()
        print("Audio playback completed via sounddevice")
        return True
        
    except Exception as e:
        print(f"Error playing audio via sounddevice: {e}")
        return False

def play_audio_ffmpeg(audio_file, volume=1.5):
    """
    Play an audio file using ffplay from ffmpeg (Method 2).
    
    Args:
        audio_file (str): Path to the audio file to play
        volume (float, optional): Volume multiplier. Default is 1.5 (50% boost).
    
    Returns:
        bool: True if playback successful, False otherwise
    """
    print("\n--- PLAYBACK METHOD 2: Using ffplay (ffmpeg) ---")
    try:
        if not shutil.which("ffplay"):
            print("ffplay (from ffmpeg) not found in PATH. Cannot use this method.")
            return False
            
        # Start ffplay with appropriate flags
        volume_db = 20 * np.log10(volume)  # Convert to dB
        cmd = ["ffplay", "-nodisp", "-autoexit", "-volume", str(int(100 * volume)), "-af", f"volume={volume_db}dB", audio_file]
        print(f"Running command: {' '.join(cmd)}")
        
        process = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        if process.returncode == 0:
            print("Audio playback completed via ffplay")
            return True
        else:
            print(f"ffplay returned error code {process.returncode}")
            print(f"ffplay stderr: {process.stderr.decode('utf-8', errors='ignore')}")
            return False
            
    except Exception as e:
        print(f"Error playing audio via ffplay: {e}")
        return False

def play_audio_system(audio_file):
    """
    Play an audio file using system default player (Method 3).
    
    Args:
        audio_file (str): Path to the audio file to play
        
    Returns:
        bool: True if playback successful, False otherwise
    """
    print("\n--- PLAYBACK METHOD 3: Using system default player ---")
    try:
        # Determine the platform
        import platform
        system = platform.system().lower()
        
        if system == "linux":
            cmd = ["xdg-open", audio_file]
            print(f"Running command: {' '.join(cmd)}")
            process = subprocess.Popen(cmd)
            print("Audio file opened with system player")
            time.sleep(5)  # Give the system time to start playback
            return True
            
        elif system == "darwin":  # macOS
            cmd = ["open", audio_file]
            print(f"Running command: {' '.join(cmd)}")
            process = subprocess.Popen(cmd)
            print("Audio file opened with system player")
            time.sleep(5)  # Give the system time to start playback
            return True
            
        elif system == "windows":
            cmd = ["start", "", audio_file]
            print(f"Running command: {' '.join(cmd)}")
            process = subprocess.Popen(cmd, shell=True)
            print("Audio file opened with system player")
            time.sleep(5)  # Give the system time to start playback
            return True
            
        else:
            print(f"Unsupported platform: {system}")
            return False
            
    except Exception as e:
        print(f"Error playing audio via system player: {e}")
        return False

def play_audio(audio_file, device_name=None, volume=1.5):
    """
    Play an audio file using multiple methods with fallback.
    
    Args:
        audio_file (str): Path to the audio file to play
        device_name (str, optional): Name of the audio device to use. If None, use default device.
        volume (float, optional): Volume multiplier. Default is 1.5 (boosted volume).
    """
    # Debug: Print the system volume
    print_current_volume()
    
    # Method 1: Use sounddevice (most direct control)
    if play_audio_sounddevice(audio_file, device_name, volume):
        return
        
    # Method 2: Use ffplay if available
    if play_audio_ffmpeg(audio_file, volume):
        return
        
    # Method 3: Use system default player
    if play_audio_system(audio_file):
        return
        
    print("\nAll playback methods failed. Audio could not be played.")
    print("Please check your audio settings and ensure speakers are connected and volume is up.")

def check_audio_devices():
    """Print available audio devices to help with troubleshooting."""
    print("\nAvailable audio devices:")
    try:
        devices = sd.query_devices()
        for i, device in enumerate(devices):
            device_type = "Input/Output" if device.get('max_input_channels', 0) > 0 and device.get('max_output_channels', 0) > 0 else \
                         "Input only" if device.get('max_input_channels', 0) > 0 else "Output only"
            print(f"{i}: {device['name']} ({device_type})")
        print()
    except Exception as e:
        print(f"Error querying audio devices: {e}")

def main():
    # Create a directory for downloads if it doesn't exist
    download_dir = os.path.join(os.path.dirname(__file__), "downloads")
    os.makedirs(download_dir, exist_ok=True)
    
    # List available audio devices for reference
    check_audio_devices()
    
    # Check for system dependencies
    if not shutil.which("ffmpeg"):
        print("Warning: ffmpeg is not found in PATH. Audio conversion might fail.")
        print("Please install ffmpeg to ensure proper functionality.")
    
    # YouTube URLs to try (in order of preference)
    youtube_urls = [
        "https://www.youtube.com/watch?v=vFZh32JlfvA",  # BAYNK - Water
        "https://www.youtube.com/watch?v=LDU_Txk06tM",  # Crab Rave
        "https://www.youtube.com/watch?v=3MteSlpxCpo",  # Quad City DJs - Space Jam
        "https://www.youtube.com/watch?v=dQw4w9WgXcQ",  # Rick Astley - Never Gonna Give You Up
    ]
    
    # Optional: Specify a device name (comment out to use default device)
    # device_name = "UACDemoV1.0"  # Replace with your device name if needed
    device_name = None  # Use default device
    
    # Set volume level (1.0 = original volume, >1.0 increases volume)
    volume = 1.5  # Boost volume by 50%
    
    # Try each URL until one works
    audio_file = None
    for url in youtube_urls:
        print(f"\nTrying URL: {url}")
        audio_file = download_youtube_audio(url, download_dir)
        if audio_file:
            break
    
    if audio_file:
        # Play the downloaded audio
        play_audio(audio_file, device_name, volume)
    else:
        print("\nFailed to download audio from all URLs.")
        
        # Try to find any existing MP3 files in the downloads directory
        mp3_files = [os.path.join(download_dir, f) for f in os.listdir(download_dir) if f.endswith('.mp3')]
        if mp3_files:
            print(f"\nFound {len(mp3_files)} existing MP3 file(s). Will play the most recent one.")
            most_recent = max(mp3_files, key=os.path.getmtime)
            play_audio(most_recent, device_name, volume)
        else:
            print("No existing MP3 files found in the downloads directory.")
            print("Cannot play any audio.")

if __name__ == "__main__":
    main()
