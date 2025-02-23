# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import subprocess
import time

def install_dependencies():
    """Install all required system dependencies"""
    print("Installing required dependencies...")
    subprocess.run(['sudo', 'apt-get', 'update'], check=True)
    
    # Install all required packages
    packages = [
        'libsdl2-dev',      # SDL2 for audio capture
        'cmake',            # For building
        'make',            # For building
        'g++',             # C++ compiler
        'gcc',             # C compiler
        'git',             # For cloning
        'alsa-utils',      # For audio device access
        'libasound2-dev'   # ALSA development files
    ]
    
    subprocess.run(['sudo', 'apt-get', 'install', '-y'] + packages, check=True)

def ensure_whisper_ready():
    """Make sure whisper.cpp is installed and ready to use"""
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)  # Change to script directory
    
    # Clone and build whisper.cpp if needed
    if not os.path.exists('whisper.cpp'):
        print("Setting up whisper.cpp...")
        # First make sure all dependencies are installed
        install_dependencies()
        
        # Clone whisper.cpp
        subprocess.run(['git', 'clone', 'https://github.com/ggerganov/whisper.cpp.git'], check=True)
        
        # Build with NEON optimizations and SDL2
        build_dir = os.path.join('whisper.cpp', 'build')
        os.makedirs(build_dir, exist_ok=True)
        os.chdir(build_dir)
        
        # Configure with both NEON and SDL2
        subprocess.run(['cmake', '..', 
                       '-DWHISPER_NEON=ON',
                       '-DWHISPER_SDL2=ON'], check=True)
        
        # Build the stream example specifically
        subprocess.run(['make', '-j4', 'whisper-stream'], check=True)
        os.chdir('../..')
        
        # Download the tiny model
        os.chdir('whisper.cpp')
        subprocess.run(['bash', 'models/download-ggml-model.sh', 'tiny.en'], check=True)
        os.chdir('..')
    
    # Return paths needed for streaming
    return {
        'model': os.path.join('whisper.cpp', 'models', 'ggml-tiny.en.bin'),
        'stream': os.path.join('whisper.cpp', 'build', 'bin', 'whisper-stream')
    }

def main():
    # Make sure whisper is ready
    paths = ensure_whisper_ready()
    
    # Start real-time streaming transcription
    print("\nStarting real-time transcription... (Press Ctrl+C to stop)")
    try:
        # Using settings optimized for Raspberry Pi from the discussion
        subprocess.run([
            paths['stream'],
            '-m', paths['model'],
            '--step', '4000',      # Process 4 seconds at a time
            '--length', '8000',    # Use 8 second context
            '-c', '0',            # Audio capture device
            '-t', '4',            # Number of threads
            '-ac', '512'          # Reduced context size for speed
        ])
    except KeyboardInterrupt:
        print("\nStopped transcription")

if __name__ == "__main__":
    main()
