#!/usr/bin/env python3
"""Connect to OrcaHand, enable torque, send fully open positions (minimal init)."""

import sys
import time
import math
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from orca_core import OrcaHand, OrcaJointPositions

CONFIG_PATH = PROJECT_ROOT / "orca_core" / "models" / "v2" / "orcahand_right" / "config.yaml"

def deg_to_rad(deg):
    return deg * math.pi / 180.0

# Fully open: all joints at maximum extension (upper ROM limit)
FULLY_OPEN_DEG = {
    "wrist": 35,
    "thumb_cmc": 33,
    "thumb_abd": 55,
    "thumb_mcp": 90,
    "thumb_dip": 107,
    "index_abd": 25,
    "index_mcp": 100,
    "index_pip": 107,
    "middle_abd": 27,
    "middle_mcp": 100,
    "middle_pip": 107,
    "ring_abd": 27,
    "ring_mcp": 100,
    "ring_pip": 107,
    "pinky_abd": 30,
    "pinky_mcp": 100,
    "pinky_pip": 107,
}
FULLY_OPEN_RAD = {k: deg_to_rad(v) for k, v in FULLY_OPEN_DEG.items()}

def main():
    print("[1] Loading config...")
    hand = OrcaHand(config_path=str(CONFIG_PATH))

    print("[2] Connecting...")
    ok, msg = hand.connect()
    if not ok:
        print(f"  Connect FAILED: {msg}")
        sys.exit(1)
    print(f"  Connected: {msg}")

    print("[3] Enabling torque (all motors)...")
    try:
        hand.enable_torque()
        print("  Torque enabled.")
    except Exception as e:
        print(f"  Torque warning: {e}")

    time.sleep(0.5)

    print("[4] Setting control mode to position...")
    try:
        hand.set_control_mode("position")
        print("  Control mode set.")
    except Exception as e:
        print(f"  Mode warning: {e}")

    time.sleep(0.5)

    print("[5] Sending FULLY OPEN positions...")
    jp = OrcaJointPositions.from_dict(FULLY_OPEN_RAD)
    hand.set_joint_positions(jp, num_steps=1)
    print(f"  Sent: {FULLY_OPEN_RAD}")
    print("  Hand is opening now (async).")

    # Wait for movement
    time.sleep(3)

    print("[6] Reading back positions...")
    try:
        final = hand.get_joint_position()
        for name in FULLY_OPEN_DEG:
            val = final.data.get(name, "?")
            target = FULLY_OPEN_RAD[name]
            print(f"  {name}: {val:.4f} rad (target: {target:.4f})")
    except Exception as e:
        print(f"  Read error: {e}")

    print("\nDone.")

if __name__ == "__main__":
    main()
