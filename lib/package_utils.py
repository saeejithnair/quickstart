"""
Utility functions for package management.
This module provides functions to check if required packages are installed
and install them if necessary.
"""

import sys
import subprocess
import importlib.util

def ensure_package(package_name, pip_name=None):
    """
    Check if a package is installed and install it if not.
    
    Args:
        package_name (str): The name of the package to import
        pip_name (str, optional): The name of the package in pip if different from import name
                                 Defaults to package_name if not specified.
    
    Returns:
        bool: True if the package is already installed or was successfully installed,
              False if installation failed.
    """
    if pip_name is None:
        pip_name = package_name
    
    try:
        # First try to import the package
        spec = importlib.util.find_spec(package_name)
        if spec is None:
            print(f"{package_name} not found. Installing...")
            try:
                # Use check_call to ensure the command completes successfully
                subprocess.check_call([sys.executable, "-m", "pip", "install", pip_name])
                print(f"{package_name} installed successfully.")
                
                # Verify installation was successful
                if importlib.util.find_spec(package_name) is None:
                    print(f"Warning: {package_name} was installed but still cannot be imported.")
                    return False
                return True
            except subprocess.CalledProcessError as e:
                print(f"Error installing {package_name}: pip command failed with exit code {e.returncode}")
                return False
        else:
            print(f"{package_name} is already installed.")
            return True
    except Exception as e:
        print(f"Unexpected error installing {package_name}: {e}")
        return False 