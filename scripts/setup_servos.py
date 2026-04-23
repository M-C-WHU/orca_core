"""ORCA Hand partial setup script — operate on specific servos only.

Same workflow as setup.py (tension -> calibrate -> test -> verify, 3 rounds)
but scoped to a user-specified subset of joints/servos.

Usage:
    python scripts/setup_servos.py [config_path] joint1 joint2 ...
    python scripts/setup_servos.py config.yaml thumb_cmc thumb_mcp index_mcp
    python scripts/setup_servos.py  # all joints (same as setup.py)
"""

import argparse
import time

from common import connect_hand, create_hand, shutdown_hand
from orca_core import OrcaJointPositions


DIVIDER = "=" * 60

ALL_JOINT_IDS = [
    "wrist",
    "thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip",
    "index_abd", "index_mcp", "index_pip",
    "middle_abd", "middle_mcp", "middle_pip",
    "ring_abd", "ring_mcp", "ring_pip",
    "pinky_abd", "pinky_mcp", "pinky_pip",
]


def pose_from_fractions(hand, fractions: dict[str, float], target_joints: set[str] | None = None) -> OrcaJointPositions:
    """Build a joint-position dict from fractional ROM values.

    If *target_joints* is given, only those joints are set; the rest keep
    their current neutral position.
    """
    pose = dict(hand.config.neutral_position)
    for joint, fraction in fractions.items():
        if target_joints is not None and joint not in target_joints:
            continue
        if joint not in hand.config.joint_roms_dict:
            continue
        joint_min, joint_max = hand.config.joint_roms_dict[joint]
        pose[joint] = joint_min + fraction * (joint_max - joint_min)
    return OrcaJointPositions.from_dict(pose)


def wait_for_enter(msg="Press ENTER to continue..."):
    try:
        response = input(f"\n>>> {msg} ('s' to skip) ")
        return response.strip().lower() in ('s', 'skip')
    except KeyboardInterrupt:
        print()
        return True


def print_step(step_num, title):
    print(f"\n{DIVIDER}")
    print(f"  STEP {step_num}: {title}")
    print(DIVIDER)


# ---------------------------------------------------------------------------
# Filtered calibration
# ---------------------------------------------------------------------------

def _filter_calibration_sequence(calib_seq, target_joints: set[str]) -> list:
    """Return only the steps whose joints intersect *target_joints*.

    Each step in calib_seq is a dict like ``{"step": N, "joints": {name: dir}}``.
    We keep a step if any of its joints are in *target_joints*, and strip out
    joints that aren't.
    """
    filtered = []
    for step in calib_seq:
        joints = step.get("joints", {})
        relevant = {j: d for j, d in joints.items() if j in target_joints}
        if relevant:
            filtered.append({"step": step.get("step", 0), "joints": relevant})
    return filtered


def run_calibrate(hand, step_num, label, target_joints: set[str], force_wrist: bool = False):
    """Run calibration scoped to *target_joints*.

    We patch ``hand.config.calibration_sequence`` on the fly so that only
    steps involving the target joints are executed, and we manage the
    ``wrist_calibrated`` flag to control wrist inclusion.
    """
    print_step(step_num, f"CALIBRATE — {label}")
    print(f"  Target joints: {', '.join(sorted(target_joints))}")
    print("  Press Ctrl+C to skip.")

    include_wrist = "wrist" in target_joints

    # If wrist is in target joints, we need force_wrist-like behaviour
    # regardless of its current calibration state.
    # Config is a frozen dataclass; use object.__setattr__ to patch it.
    saved_seq = hand.config.calibration_sequence
    saved_wrist_cal = hand.wrist_calibrated
    try:
        object.__setattr__(
            hand.config,
            'calibration_sequence',
            _filter_calibration_sequence(saved_seq, target_joints),
        )
        # If wrist is a target, pretend it's not calibrated so the calibration
        # routine doesn't skip it.
        if include_wrist:
            hand._wrist_calibrated = False
        hand.calibrate(force_wrist=(include_wrist and force_wrist))
        print("  Calibration complete.")
    except KeyboardInterrupt:
        print("\n  Calibration skipped.")
    finally:
        object.__setattr__(hand.config, 'calibration_sequence', saved_seq)
        hand._wrist_calibrated = saved_wrist_cal


# ---------------------------------------------------------------------------
# Filtered tension
# ---------------------------------------------------------------------------

def run_tension(hand, step_num, label, target_joints: set[str]):
    """Run tension scoped to *target_joints* only."""
    print_step(step_num, f"TENSION — {label}")
    print(f"  Target joints: {', '.join(sorted(target_joints))}")
    print("  Motors will move to set initial tension, then hold.")
    print("  Use the tensioning tool or pliers to turn the top spool clockwise.")
    print("  Do NOT overtension — just enough to remove slack.")
    if wait_for_enter("Press ENTER to begin tensioning, or 's' to skip..."):
        print("  Tension skipped.")
        return

    # Resolve target motor IDs
    target_motor_ids = [
        mid for j, mid in hand.config.joint_to_motor_map.items()
        if j in target_joints and mid in hand.config.motor_ids
    ]
    if not target_motor_ids:
        print("  No matching motors found for the specified joints. Skipping.")
        return

    print(f"  Target motors: {target_motor_ids}")
    print("  Press Ctrl+C when tensioning is done.")

    # Replicate the move-motors logic from _tension, but only for our motors.
    saved_mode = hand.config.control_mode
    try:
        from orca_core.hardware_hand import CURRENT_BASED_POSITION
        hand.set_control_mode(CURRENT_BASED_POSITION)
        hand.set_max_current(hand.config.calibration_current)

        duration = 8
        increment = 0.1
        increments_right = {mid: increment for mid in target_motor_ids}
        increments_left = {mid: -increment for mid in target_motor_ids}

        start = time.time()
        while time.time() - start < duration:
            if hand._task_stop_event.is_set():
                break
            hand._set_motor_pos(increments_left, rel_to_current=True)
            time.sleep(0.1)

        start = time.time()
        while time.time() - start < duration:
            if hand._task_stop_event.is_set():
                break
            hand._set_motor_pos(increments_right, rel_to_current=True)
            time.sleep(0.1)

        hand.set_max_current(hand.config.max_current)
        hand.enable_torque()
        print("Holding motors. Please tension carefully. Press Ctrl+C to exit.")
        while not hand._task_stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n  Tension complete.")
    finally:
        try:
            hand.set_control_mode(saved_mode)
            hand.disable_torque()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Neutral / motion test (joint-scoped)
# ---------------------------------------------------------------------------

def run_neutral(hand, step_num, target_joints: set[str] | None = None):
    print_step(step_num, "NEUTRAL POSITION")
    print("  Moving target joints to neutral position...")
    print("  Press Ctrl+C to skip.")
    hand.enable_torque()
    hand.set_control_mode('current_based_position')
    try:
        if target_joints is not None:
            # Only move the target joints to neutral; keep the rest where they are.
            current = hand.get_joint_position()
            neutral = hand.config.neutral_position
            target_pos = dict(current.as_dict() if hasattr(current, 'as_dict') else current)
            for j in target_joints:
                if j in neutral:
                    target_pos[j] = neutral[j]
            hand.set_joint_positions(OrcaJointPositions.from_dict(target_pos), num_steps=25, step_size=0.001)
        else:
            hand.set_neutral_position()
        print("  Joints are in neutral position.")
    except KeyboardInterrupt:
        print("\n  Neutral position skipped.")


def run_motion_test(hand, step_num, target_joints: set[str], duration=60):
    print_step(step_num, f"MOTION TEST — {duration}s")
    print(f"  Target joints: {', '.join(sorted(target_joints))}")
    print("  Opening and closing to verify calibration.")
    print("  Press Ctrl+C to skip.\n")

    hand.enable_torque()
    hand.set_control_mode('current_based_position')

    # Full open/close poses — but pose_from_fractions will only set target joints
    open_fractions = {
        "thumb_cmc": 0.70, "thumb_abd": 0.80, "thumb_mcp": 0.85, "thumb_dip": 0.80,
        "index_abd": 0.10, "middle_abd": 0.50, "ring_abd": 0.70, "pinky_abd": 0.85,
        "index_mcp": 0.15, "middle_mcp": 0.15, "ring_mcp": 0.15, "pinky_mcp": 0.15,
        "index_pip": 0.10, "middle_pip": 0.10, "ring_pip": 0.10, "pinky_pip": 0.10,
        "wrist": 0.30,
    }
    closed_fractions = {
        "thumb_cmc": 0.35, "thumb_abd": 0.55, "thumb_mcp": 0.20, "thumb_dip": 0.85,
        "index_mcp": 0.85, "middle_mcp": 0.85, "ring_mcp": 0.85, "pinky_mcp": 0.85,
        "index_pip": 0.90, "middle_pip": 0.90, "ring_pip": 0.90, "pinky_pip": 0.90,
        "wrist": 0.55,
    }

    open_pos = pose_from_fractions(hand, open_fractions, target_joints=target_joints)
    closed_pos = pose_from_fractions(hand, closed_fractions, target_joints=target_joints)

    try:
        start = time.time()
        cycle = 0
        while True:
            remaining = duration - (time.time() - start)
            if remaining <= 0:
                break

            if cycle % 2 == 0:
                print(f"  [{int(remaining):3d}s left]  OPEN")
                hand.set_joint_positions(open_pos, num_steps=25, step_size=0.001)
            else:
                print(f"  [{int(remaining):3d}s left]  CLOSE")
                hand.set_joint_positions(closed_pos, num_steps=25, step_size=0.001)
            cycle += 1

            hold_end = min(time.time() + 2.0, start + duration)
            while time.time() < hold_end:
                time.sleep(0.1)

        print("  Motion test complete.")
    except KeyboardInterrupt:
        print("\n  Motion test skipped.")

    run_neutral(hand, 0, target_joints=target_joints)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def validate_joints(joint_names: list[str], hand) -> set[str]:
    """Validate that all requested joint names exist in the hand config."""
    valid = set(hand.config.joint_to_motor_map.keys())
    requested = set(joint_names)
    unknown = requested - valid
    if unknown:
        print(f"ERROR: Unknown joint(s): {', '.join(sorted(unknown))}")
        print(f"  Valid joints: {', '.join(sorted(valid))}")
        raise SystemExit(1)
    return requested


def main():
    parser = argparse.ArgumentParser(
        description="Partial ORCA Hand setup — operate on specific servos only."
    )
    parser.add_argument(
        "config_path", type=str, nargs="?", default=None,
        help="Path to the hand config.yaml file"
    )
    parser.add_argument(
        "joints", type=str, nargs="*",
        help="Joint names to set up (e.g. thumb_cmc index_mcp). "
             "If omitted, all joints are set up (same as setup.py)."
    )
    args = parser.parse_args()

    print(DIVIDER)
    print("  ORCA HAND SETUP (SERVO-SCOPED)")
    print("  Full calibration and verification workflow for selected servos")
    print("  Type 's' at any prompt or Ctrl+C to skip a step")
    print(DIVIDER)

    hand = create_hand(args.config_path, use_mock=False)
    connect_hand(hand)
    print("  Connected and ready.")

    # Resolve target joints
    if args.joints:
        target_joints = validate_joints(args.joints, hand)
    else:
        target_joints = set(hand.config.joint_to_motor_map.keys())

    print(f"  Target joints: {', '.join(sorted(target_joints))}")

    try:
        # --- Round 1 ---
        run_tension(hand, 1, "Initial tensioning", target_joints)
        wait_for_enter("Place the hand in a neutral position, then press ENTER...")
        run_calibrate(hand, 2, "First calibration (with wrist if selected)",
                       target_joints, force_wrist=True)

        # --- Round 2 ---
        run_tension(hand, 3, "Second tensioning", target_joints)
        run_neutral(hand, 4, target_joints=target_joints)
        run_calibrate(hand, 5, "Second calibration (fingers only)", target_joints)

        # --- Motion test ---
        run_motion_test(hand, 6, target_joints, duration=60)

        # --- Round 3 ---
        run_tension(hand, 7, "Final tensioning", target_joints)
        run_neutral(hand, 8, target_joints=target_joints)
        run_calibrate(hand, 9, "Final calibration (fingers only)", target_joints)
        run_neutral(hand, 10, target_joints=target_joints)

        print(f"\n{DIVIDER}")
        print("  Done. Have fun playing with ORCA!")
        print(DIVIDER)

    except KeyboardInterrupt:
        print("\n\n  Setup interrupted by user.")
    finally:
        shutdown_hand(hand)


if __name__ == "__main__":
    main()
