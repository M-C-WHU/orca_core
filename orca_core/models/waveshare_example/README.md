# WaveShare ST-3215 HS Servo Support for OrcaHand

This directory contains configuration and documentation for using WaveShare ST-3215 HS servo motors with OrcaHand.

## Overview

WaveShare ST-3215 HS is a high-speed digital servo that uses a serial communication protocol similar to Feetech SCServo protocol. This implementation provides compatibility with the OrcaHand control system.

## Prerequisites

1. **Hardware Requirements:**
   - WaveShare ST-3215 HS servo motors
   - USB to serial adapter (typically CH340 or CP2102 based)
   - Appropriate power supply (6.0-8.4V recommended)
   - Servo cables and connectors

2. **Software Requirements:**
   - The `feetech` Python library (used for communication protocol)
   - OrcaHand core package

## Configuration

### 1. Basic Configuration

Edit the `config.yaml` file to match your setup:

```yaml
motor_type: waveshare
port: /dev/ttyUSB0  # Adjust for your system
baudrate: 1000000   # Typical for WaveShare servos
motor_ids: [1, 2, 3, ...]  # Your servo IDs
```

### 2. Servo ID Configuration

WaveShare servos typically come with ID 1 by default. You may need to change IDs if using multiple servos. Refer to WaveShare documentation for ID change procedure.

### 3. Mechanical Configuration

Adjust the following parameters based on your mechanical design:
- `joint_to_motor_map`: Mapping between joints and servo IDs
- `joint_roms`: Range of motion for each joint (in degrees)
- `neutral_position`: Resting position for each joint

## Usage

### 1. Basic Test

Create a simple test script:

```python
from orca_core import OrcaHand
import time

# Use the WaveShare configuration
hand = OrcaHand("orca_core/models/waveshare_example")
status = hand.connect()
print(status)

if status[0]:
    hand.enable_torque()
    
    # Test movement
    hand.set_joint_pos({"index_mcp": 30}, num_steps=25, step_size=0.001)
    time.sleep(2)
    
    hand.disable_torque()
    hand.disconnect()
```

### 2. Calibration

Run calibration for your WaveShare servos:

```bash
uv run python scripts/tension.py orca_core/models/waveshare_example
uv run python scripts/calibrate.py orca_core/models/waveshare_example
```

## Important Notes

### Protocol Compatibility

The WaveShare client uses the Feetech SCServo library for communication, as WaveShare servos appear to use a similar protocol. Key assumptions:

1. **Control Table:** Similar register addresses to Feetech servos
2. **Communication:** Serial protocol with same packet structure
3. **Modes:** Position control (servo mode) and velocity control (wheel mode)

### Parameter Adjustments

You may need to adjust these parameters based on actual servo behavior:

1. **Scaling Factors:**
   - `pos_scale`: Position scaling (raw units to radians)
   - `vel_scale`: Velocity scaling
   - `cur_scale`: Current scaling

2. **Motion Parameters:**
   - `_default_speed`: Default movement speed
   - `_default_acc`: Default acceleration
   - `_default_torque`: Default torque limit

### Troubleshooting

1. **Connection Issues:**
   - Check serial port permissions: `sudo chmod 666 /dev/ttyUSB0`
   - Verify baudrate matches servo configuration
   - Ensure proper power supply

2. **Movement Issues:**
   - Check joint inversion in `joint_to_motor_map`
   - Verify position limits in `joint_roms`
   - Adjust current limits if servos stall

3. **Communication Errors:**
   - Verify servo IDs match configuration
   - Check cable connections
   - Try different baudrates (115200, 57600, 9600)

## WaveShare ST-3215 HS Specifications

- **Model:** ST-3215 HS
- **Operating Voltage:** 6.0-8.4V
- **Stall Torque:** 32kg·cm @ 8.4V
- **Speed:** 0.10sec/60° @ 8.4V
- **Control System:** Digital
- **Communication:** Serial (similar to Feetech SCServo)
- **Position Resolution:** 0.29°
- **Operating Angle:** 360° (continuous in wheel mode)
- **Weight:** 66g
- **Dimensions:** 40.2×20.2×40.0mm

## References

1. [WaveShare ST-3215 HS Product Page](https://www.waveshare.com/st-3215-hs.htm)
2. [Feetech SCServo Protocol Documentation](https://github.com/feetech/SCServo)
3. [OrcaHand Documentation](https://orcahand.com/docs)