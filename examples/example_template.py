"""
Template for example files with automatic package installation.
Copy this template and modify it for your specific example.
"""

# Add the parent directory to the Python path to access the lib folder
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Import the package utility and ensure required packages are installed
from lib.package_utils import ensure_package

# Check and install required packages
# Add your required packages here, for example:
# ensure_package("cv2", "opencv-python")
# ensure_package("numpy")
# ensure_package("other_package")

# Now you can safely import the required packages
# import package1
# import package2

def main():
    """Main function for the example."""
    print("Example template running...")
    # Your code here

if __name__ == "__main__":
    main() 