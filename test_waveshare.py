#!/usr/bin/env python3
"""Test script for WaveShare ST-3215 HS servo support."""

import sys
import os

# Add the orca_core directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from orca_core.hardware.waveshare_client import WaveShareClient

def test_waveshare_client():
    """Test the WaveShare client initialization."""
    print("Testing WaveShare client...")
    
    try:
        # Create a client instance
        client = WaveShareClient(
            motor_ids=[1, 2, 3],
            port="/dev/ttyUSB0",
            baudrate=1000000,
            lazy_connect=False  # Don't connect automatically
        )
        
        print("✓ WaveShareClient created successfully")
        print(f"  Motor IDs: {client.motor_ids}")
        print(f"  Port: {client.port_name}")
        print(f"  Baudrate: {client.baudrate}")
        print(f"  Position scale: {client.pos_scale}")
        print(f"  Velocity scale: {client.vel_scale}")
        print(f"  Current scale: {client.cur_scale}")
        
        # Test property access
        print(f"  Requires offset calibration: {client.requires_offset_calibration}")
        
        return True
        
    except ImportError as e:
        print(f"✗ Import error: {e}")
        print("  Make sure the feetech library is available.")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def test_config_loading():
    """Test loading a WaveShare configuration."""
    print("\nTesting configuration loading...")
    
    try:
        from orca_core.hand_config import OrcaHandConfig
        
        # Use the example configuration
        config_path = "orca_core/models/waveshare_example/config.yaml"
        
        if os.path.exists(config_path):
            config = OrcaHandConfig.from_config_path(config_path)
            print(f"✓ Configuration loaded successfully")
            print(f"  Motor type: {config.motor_type}")
            print(f"  Motor IDs: {config.motor_ids[:5]}...")  # Show first 5
            print(f"  Control mode: {config.control_mode}")
            print(f"  Baudrate: {config.baudrate}")
            return True
        else:
            print(f"✗ Configuration file not found: {config_path}")
            return False
            
    except Exception as e:
        print(f"✗ Error loading configuration: {e}")
        return False

def test_hardware_hand_integration():
    """Test integration with OrcaHand."""
    print("\nTesting OrcaHand integration...")
    
    try:
        from orca_core.hardware_hand import OrcaHand
        
        print("✓ OrcaHand imports successfully with WaveShare support")
        
        # Check if _create_motor_client handles waveshare type
        import inspect
        source = inspect.getsource(OrcaHand._create_motor_client)
        if "waveshare" in source.lower():
            print("✓ WaveShare support detected in OrcaHand._create_motor_client")
            return True
        else:
            print("✗ WaveShare support not found in OrcaHand._create_motor_client")
            return False
            
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

if __name__ == "__main__":
    print("=" * 60)
    print("WaveShare ST-3215 HS Servo Support Test")
    print("=" * 60)
    
    tests_passed = 0
    tests_total = 3
    
    # Run tests
    if test_waveshare_client():
        tests_passed += 1
    
    if test_config_loading():
        tests_passed += 1
        
    if test_hardware_hand_integration():
        tests_passed += 1
    
    print("\n" + "=" * 60)
    print(f"Test Results: {tests_passed}/{tests_total} tests passed")
    
    if tests_passed == tests_total:
        print("✓ All tests passed! WaveShare support is ready.")
        print("\nNext steps:")
        print("1. Connect WaveShare servos to your computer")
        print("2. Update the config.yaml with your servo IDs")
        print("3. Run: uv run python scripts/tension.py orca_core/models/waveshare_example")
        print("4. Run: uv run python scripts/calibrate.py orca_core/models/waveshare_example")
    else:
        print("✗ Some tests failed. Please check the implementation.")
    
    sys.exit(0 if tests_passed == tests_total else 1)