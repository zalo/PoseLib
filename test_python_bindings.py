#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, '/home/runner/work/PoseLib/PoseLib/build/pybind')

import poselib
import numpy as np

def test_equirectangular_camera():
    print("Testing Equirectangular Camera Model in Python")
    
    # Create an equirectangular camera
    camera = poselib.Camera()
    camera.model_id = 7  # EQUIRECTANGULAR model id
    camera.width = 1920
    camera.height = 960
    camera.params = [1920.0, 960.0]
    
    print(f"Model ID: {camera.model_id}")
    print(f"Model Name: {camera.model_name()}")
    print(f"Width: {camera.width}, Height: {camera.height}")
    print(f"Parameters: {camera.params}")
    
    # Check that the model name is correct
    if camera.model_name() == "EQUIRECTANGULAR":
        print("✓ Model name is correct!")
    else:
        print(f"✗ Expected 'EQUIRECTANGULAR', got '{camera.model_name()}'")
        return False
        
    # Test that focal methods work (should return 1.0 for equirectangular since no focal concept)
    try:
        focal = camera.focal()
        print(f"Focal length: {focal}")
        print("✓ Python bindings work correctly!")
        return True
        
    except Exception as e:
        print(f"✗ Error testing Python bindings: {e}")
        return False

if __name__ == "__main__":
    success = test_equirectangular_camera()
    sys.exit(0 if success else 1)