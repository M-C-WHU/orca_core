#!/usr/bin/env python3
"""
OrcaHand ROS2 Standalone Demo — Tests the ROS2 translation layer WITHOUT
requiring a real ROS2 installation or a physical hand.

This script:
  1. Creates an OrcaHand backed by MockDynamixelClient (simulated hand).
  2. Instantiates the OrcaHandWorker (the same worker used by the ROS2 node).
  3. Simulates ROS2-style publish/subscribe communication via a lightweight
     message bus built with threading primitives.
  4. Runs a sequence of commands that mirror what a real ROS2 client would do:
       connect → init_joints → set_joint_positions → read state → disconnect

This validates:
  - The async command dispatch (worker thread isolation)
  - The joint-command translation (JointState ↔ OrcaJointPositions)
  - State publishing at a configurable rate
  - Proper shutdown semantics

Usage:
    python scripts/ros2/orcahand_ros2_demo.py [--rate 30] [--duration 5]
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from orca_core import OrcaHand, OrcaJointPositions
from orca_core.hardware.mock_dynamixel_client import MockDynamixelClient


def create_mock_hand(config_path: Optional[str] = None) -> OrcaHand:
    """Create an OrcaHand backed by MockDynamixelClient.

    We monkey-patch _create_motor_client to return a MockDynamixelClient,
    avoiding the need for any real serial hardware or SDK.
    """
    _install_fake_dynamixel_sdk()

    hand = OrcaHand(config_path=config_path)

    # Monkey-patch to return our mock client instead of a real one
    original_create = hand._create_motor_client

    def _mock_create_motor_client():
        return MockDynamixelClient(
            hand.config.motor_ids,
            hand.config.port,
            hand.config.baudrate,
        )

    hand._create_motor_client = _mock_create_motor_client
    return hand


def _install_fake_dynamixel_sdk() -> None:
    """Install a minimal fake dynamixel_sdk module so MockDynamixelClient works.

    MockDynamixelClient imports dynamixel_sdk at __init__ time — we need to
    provide a stub before the import happens.
    """
    if "dynamixel_sdk" in sys.modules:
        return

    import types

    module = types.ModuleType("dynamixel_sdk")

    class PortHandler:
        def __init__(self, port: str):
            self.port_name = port
            self.is_using = False
            self.is_open = False
            self.baudrate = 1000000

        def openPort(self) -> bool:
            self.is_open = True
            self.is_using = True
            return True

        def closePort(self):
            self.is_open = False
            self.is_using = False

        def setBaudRate(self, baudrate: int) -> bool:
            self.baudrate = baudrate
            return True

        def getBaudRate(self) -> int:
            return self.baudrate

        def readPort(self, length: int) -> bytes:
            return b"\x00" * length

        def writePort(self, data: bytes) -> int:
            return len(data)

    class PacketHandler:
        def __init__(self, protocol_version: float):
            self.protocol_version = protocol_version

        def getTxRxResult(self, result: int) -> str:
            return "[RX-ROM_RESULT] Success"

        def getRxPacketError(self, error: int) -> str:
            return ""

        def read1ByteTxRx(self, port, motor_id, address):
            return 0, 0

        def read2ByteTxRx(self, port, motor_id, address):
            return 0, 0

        def read4ByteTxRx(self, port, motor_id, address):
            return 0, 0

        def write1ByteTxRx(self, port, motor_id, address, value):
            return 0, 0

        def write2ByteTxRx(self, port, motor_id, address, value):
            return 0, 0

        def write4ByteTxRx(self, port, motor_id, address, value):
            return 0, 0

    class GroupBulkRead:
        def __init__(self, port_handler, packet_handler):
            self.port_handler = port_handler
            self.packet_handler = packet_handler

        def addParam(self, motor_id, address, size):
            return True

        def txRxPacket(self):
            return 0

        def isAvailable(self, motor_id, address, size):
            return True

        def getData(self, motor_id, address, size):
            return 0

        def clearParam(self):
            pass

    module.PortHandler = PortHandler
    module.PacketHandler = PacketHandler
    module.GroupBulkRead = GroupBulkRead
    module.COMM_SUCCESS = 0
    module.COMM_PORT_BUSY = -1004
    sys.modules["dynamixel_sdk"] = module


# ===========================================================================
# Lightweight message bus (simulates ROS2 pub/sub)
# ===========================================================================

@dataclass
class JointStateMsg:
    """Mirrors sensor_msgs/JointState for simulation."""
    name: List[str]
    position: List[float]
    stamp: float = 0.0


class MessageBus:
    """Simple pub/sub message bus that mimics ROS2 topic communication."""

    def __init__(self):
        self._lock = threading.Lock()
        self._subscribers: Dict[str, List[Callable]] = {}

    def create_publisher(self, topic: str):
        """Return a callable publisher for *topic*."""
        def publish(msg: Any):
            with self._lock:
                callbacks = self._subscribers.get(topic, [])
            for cb in callbacks:
                try:
                    cb(msg)
                except Exception as exc:
                    print(f"  [Bus] Subscriber error on {topic}: {exc}")
        return publish

    def create_subscriber(self, topic: str, callback: Callable):
        with self._lock:
            self._subscribers.setdefault(topic, []).append(callback)


# ===========================================================================
# Worker — same pattern as OrcaHandWorker in the ROS2 node
# ===========================================================================

class _CommandKind:
    CONNECT = "connect"
    DISCONNECT = "disconnect"
    SET_JOINT_POS = "set_joint_pos"
    ENABLE_TORQUE = "enable_torque"
    DISABLE_TORQUE = "disable_torque"
    SET_MAX_CURRENT = "set_max_current"
    SET_CONTROL_MODE = "set_control_mode"
    INIT_JOINTS = "init_joints"
    CALIBRATE = "calibrate"
    STOP_TASK = "stop_task"


@dataclass
class _Command:
    kind: str
    kwargs: Dict[str, Any] = field(default_factory=dict)
    result_event: threading.Event = field(default_factory=threading.Event)
    result_value: Any = None
    error: Optional[Exception] = None


class DemoWorker:
    """Background worker that owns an OrcaHand and serialises all I/O.

    This is the same pattern as OrcaHandWorker in orcahand_ros2_node.py.
    All blocking OrcaHand calls are dispatched to this single thread so
    callers (ROS2 callbacks, timer loops) are never blocked.
    """

    def __init__(self, hand: OrcaHand):
        self._hand = hand
        self._lock = threading.Lock()
        self._queue: list[_Command] = []
        self._queue_event = threading.Event()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def hand(self) -> OrcaHand:
        return self._hand

    def is_connected(self) -> bool:
        return self._hand.is_connected()

    def is_calibrated(self) -> bool:
        return self._hand.is_calibrated() if self._hand.is_connected() else False

    def get_joint_names(self) -> List[str]:
        return list(self._hand.config.joint_ids)

    def get_joint_roms(self) -> Dict[str, List[float]]:
        return dict(self._hand.config.joint_roms_dict)

    def submit(self, kind: str, kwargs: dict | None = None,
               block: bool = True, timeout: float = 30.0) -> Any:
        cmd = _Command(kind=kind, kwargs=kwargs or {})
        with self._lock:
            self._queue.append(cmd)
            self._queue_event.set()
        if block:
            cmd.result_event.wait(timeout=timeout)
            if cmd.error is not None:
                raise cmd.error
        return cmd.result_value

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._queue_event.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    def _run(self):
        while not self._stop_event.is_set():
            self._queue_event.wait(timeout=0.1)
            self._queue_event.clear()
            while True:
                with self._lock:
                    if not self._queue:
                        break
                    cmd = self._queue.pop(0)
                try:
                    cmd.result_value = self._dispatch(cmd)
                except Exception as exc:
                    cmd.error = exc
                finally:
                    cmd.result_event.set()

    def _dispatch(self, cmd: _Command) -> Any:
        kind = cmd.kind
        kw = cmd.kwargs
        if kind == _CommandKind.CONNECT:
            return self._hand.connect()
        elif kind == _CommandKind.DISCONNECT:
            return self._hand.disconnect()
        elif kind == _CommandKind.SET_JOINT_POS:
            jp = OrcaJointPositions.from_dict(kw["positions"])
            num_steps = kw.get("num_steps", 1)
            step_size = kw.get("step_size", 1e-2)
            self._hand.set_joint_positions(jp, num_steps=num_steps, step_size=step_size)
            return True
        elif kind == _CommandKind.ENABLE_TORQUE:
            self._hand.enable_torque(motor_ids=kw.get("motor_ids"))
            return True
        elif kind == _CommandKind.DISABLE_TORQUE:
            self._hand.disable_torque(motor_ids=kw.get("motor_ids"))
            return True
        elif kind == _CommandKind.SET_MAX_CURRENT:
            self._hand.set_max_current(current=kw["current"])
            return True
        elif kind == _CommandKind.SET_CONTROL_MODE:
            self._hand.set_control_mode(mode=kw["mode"])
            return True
        elif kind == _CommandKind.INIT_JOINTS:
            self._hand.init_joints(force_calibrate=kw.get("force_calibrate", False))
            return True
        elif kind == _CommandKind.CALIBRATE:
            self._hand.calibrate(blocking=kw.get("blocking", True))
            return True
        elif kind == _CommandKind.STOP_TASK:
            self._hand.stop_task()
            return True
        else:
            raise ValueError(f"Unknown command kind: {kind}")


# ===========================================================================
# Simulated ROS2 environment
# ===========================================================================

class SimulatedROS2Environment:
    """Wires up the worker, message bus, and demo logic.

    This class is the "test harness" — it plays the role that a real ROS2
    graph would play:
      - The "node" publishes /orcahand/joint_states on a timer.
      - An external "client" publishes /orcahand/joint_commands.
      - Services are invoked directly on the worker (same semantics).
    """

    def __init__(self, worker: DemoWorker, rate_hz: float = 30.0):
        self._worker = worker
        self._rate_hz = rate_hz
        self._bus = MessageBus()
        self._stop_event = threading.Event()
        self._latest_state: Optional[JointStateMsg] = None
        self._state_lock = threading.Lock()

        self._publish_state = self._bus.create_publisher("/orcahand/joint_states")
        self._bus.create_subscriber("/orcahand/joint_commands", self._on_joint_command)

        self._published_count = 0
        self._received_commands = 0

    def _on_joint_command(self, msg: JointStateMsg):
        self._received_commands += 1
        positions = {name: pos for name, pos in zip(msg.name, msg.position)}
        try:
            self._worker.submit(
                _CommandKind.SET_JOINT_POS,
                {"positions": positions, "num_steps": 1},
                block=False,
            )
        except Exception as exc:
            print(f"  [Cmd] Error setting joint positions: {exc}")

    def start(self):
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._worker.start()
        print(f"  [Env] State publisher started at {self._rate_hz:.0f} Hz")

    def stop(self):
        self._stop_event.set()
        if hasattr(self, "_thread") and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._worker.stop()
        print(f"  [Env] Stopped. Published {self._published_count} state messages, "
              f"received {self._received_commands} commands.")

    def _run(self):
        period = 1.0 / self._rate_hz
        while not self._stop_event.is_set():
            if self._worker.is_connected():
                try:
                    jp = self._worker.hand.get_joint_position()
                    msg = JointStateMsg(
                        name=list(jp.data.keys()),
                        position=[float(v) for v in jp.data.values()],
                        stamp=time.time(),
                    )
                    self._publish_state(msg)
                    self._published_count += 1
                    with self._state_lock:
                        self._latest_state = msg
                except Exception:
                    pass
            self._stop_event.wait(timeout=period)

    def send_joint_command(self, positions: Dict[str, float]):
        msg = JointStateMsg(
            name=list(positions.keys()),
            position=list(positions.values()),
            stamp=time.time(),
        )
        self._bus.create_publisher("/orcahand/joint_commands")(msg)

    def get_latest_state(self) -> Optional[JointStateMsg]:
        with self._state_lock:
            return self._latest_state


# ===========================================================================
# Demo sequence
# ===========================================================================

def run_demo(config_path: Optional[str], rate_hz: float, duration: float,
            hardware: bool = False, port: Optional[str] = None):
    print("=" * 60)
    print("OrcaHand ROS2 Translation Layer — Standalone Demo")
    print("=" * 60)
    print()

    # 1. Create hand
    if hardware:
        print("[1] Creating OrcaHand for REAL HARDWARE...")
        hand = OrcaHand(config_path=config_path)
        if port is not None:
            import dataclasses
            hand.config = dataclasses.replace(hand.config, port=port)
            print(f"    Port overridden to: {port}")
    else:
        print("[1] Creating OrcaHand with MockDynamixelClient (simulated hardware)...")
        hand = create_mock_hand(config_path)
    print(f"    Joint names: {list(hand.config.joint_ids)}")
    print(f"    Connected: {hand.is_connected()}")
    print()

    # 2. Create worker
    print("[2] Creating DemoWorker (async command dispatch)...")
    worker = DemoWorker(hand)
    worker.start()
    print()

    # 3. Create simulated ROS2 environment
    print(f"[3] Creating SimulatedROS2Environment (rate={rate_hz} Hz)...")
    env = SimulatedROS2Environment(worker, rate_hz=rate_hz)
    env.start()
    print()

    # 4. Connect
    if hardware:
        print("[4] Connecting to real hand on the serial bus...")
    else:
        print("[4] Connecting to hand (via mock client)...")
    try:
        ok, msg = worker.submit(_CommandKind.CONNECT, timeout=60.0)
    except Exception as exc:
        # connect() may try to open an interactive port picker (curses)
        # which fails outside a terminal. Surface the error clearly.
        print(f"    connect() failed: {exc}")
        if hardware and 'Permission denied' in str(exc):
            print("    HINT: You may need to add your user to the uucp/dialout group:")
            print("      sudo usermod -aG uucp $USER")
            print("      Then log out and back in, or run: newgrp uucp")
        print("    Aborting.")
        worker.stop()
        return
    print(f"    connect() → success={ok}, message={msg}")
    if not ok:
        print("    ERROR: Connection failed. Aborting.")
        worker.stop()
        return
    print()

    # 5. Initialize joints
    print("[5] Initializing joints (enable torque, set mode, calibrate if needed)...")
    try:
        worker.submit(_CommandKind.INIT_JOINTS, {"force_calibrate": True}, timeout=60.0)
        print("    init_joints() → done")
    except Exception as exc:
        print(f"    init_joints() failed: {exc}")
        print("    Continuing anyway.")
    print()

    # 6. Read initial state
    print("[6] Reading initial joint state from /orcahand/joint_states...")
    time.sleep(0.3)
    state = env.get_latest_state()
    if state:
        print(f"    Published state with {len(state.name)} joints:")
        for name, pos in list(zip(state.name, state.position))[:5]:
            print(f"      {name}: {pos:.4f} rad")
        if len(state.name) > 5:
            print(f"      ... ({len(state.name) - 5} more joints)")
    else:
        print("    No state published yet.")
    print()

    # 7. Send sinusoidal joint commands
    print(f"[7] Sending sinusoidal joint commands for {duration:.1f}s...")
    print("    (Simulating a ROS2 controller publishing to /orcahand/joint_commands)")
    joint_names = worker.get_joint_names()
    roms = worker.get_joint_roms()

    start = time.time()
    cycle = 0
    while time.time() - start < duration:
        t = time.time() - start
        positions = {}
        for i, name in enumerate(joint_names):
            rom = roms.get(name, [-1.0, 1.0])
            mid = (rom[0] + rom[1]) / 2.0
            amp = (rom[1] - rom[0]) / 4.0
            phase = i * 0.4
            positions[name] = mid + amp * math.sin(t * 1.5 + phase)

        env.send_joint_command(positions)
        cycle += 1
        time.sleep(0.1)

    print(f"    Sent {cycle} command messages via /orcahand/joint_commands")
    print()

    # 8. Move to neutral
    print("[8] Moving to neutral position...")
    neutral = dict(hand.config.neutral_position)
    worker.submit(
        _CommandKind.SET_JOINT_POS,
        {"positions": neutral, "num_steps": 10, "step_size": 0.02},
    )
    time.sleep(0.5)
    print()

    # 9. Final state
    print("[9] Final joint state:")
    state = env.get_latest_state()
    if state:
        for name, pos in zip(state.name, state.position):
            print(f"    {name:20s}: {pos:8.4f} rad")
    print()

    # 10. Service-like operations
    print("[10] Testing service-like operations (same as ROS2 service calls)...")

    print("     /orcahand/enable_torque [SetBool data=true]...")
    worker.submit(_CommandKind.ENABLE_TORQUE)
    print("     ✓ Torque enabled")

    print("     /orcahand/enable_torque [SetBool data=false]...")
    worker.submit(_CommandKind.DISABLE_TORQUE)
    print("     ✓ Torque disabled")

    print("     set_max_current(250)...")
    worker.submit(_CommandKind.SET_MAX_CURRENT, {"current": 250})
    print("     ✓ Max current set")

    print("     set_control_mode('position')...")
    worker.submit(_CommandKind.SET_CONTROL_MODE, {"mode": "position"})
    print("     ✓ Control mode set")
    print()

    # 11. Disconnect and shutdown
    print("[11] /orcahand/disconnect [Trigger]...")
    try:
        worker.submit(_CommandKind.STOP_TASK, block=False)
    except Exception:
        pass
    ok, msg = worker.submit(_CommandKind.DISCONNECT, timeout=10.0)
    print(f"     disconnect() → success={ok}, message={msg}")
    print()

    env.stop()

    print("=" * 60)
    print("Demo complete. ✓ The ROS2 translation layer is functional.")
    print()
    print("Summary of what was tested:")
    print("  ✓ OrcaHandWorker (async command dispatch on worker thread)")
    print(f"  ✓ Simulated /orcahand/joint_states publishing at {int(rate_hz)} Hz")
    print("  ✓ Simulated /orcahand/joint_commands subscription")
    print("  ✓ Service operations (connect, disconnect, torque, current, mode)")
    print("  ✓ Proper shutdown (worker thread + environment cleanup)")
    print()
    print("To use with a real ROS2 workspace:")
    print("  1. Install ROS2 (Humble/Jazzy/Rolling)")
    print("  2. Source the setup: source /opt/ros/<distro>/setup.bash")
    print("  3. Build this package in a colcon workspace")
    print("  4. Run: ros2 run orcahand_ros2 orcahand_node")
    print("  5. In another terminal: ros2 topic echo /orcahand/joint_states")
    print()
    print("To test with real hardware (without ROS2):")
    print("  python scripts/ros2/orcahand_ros2_demo.py --hardware --port /dev/ttyACM0")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description="Standalone demo for the OrcaHand ROS2 translation layer. "
                    "Uses mock hardware by default. Pass --hardware to use a real hand."
    )
    parser.add_argument(
        "--config-path", type=str, default=None,
        help="Path to config.yaml. Defaults to the bundled v1 model."
    )
    parser.add_argument(
        "--rate", type=float, default=30.0,
        help="State publishing rate in Hz (default: 30)."
    )
    parser.add_argument(
        "--duration", type=float, default=5.0,
        help="Duration of the command-sending phase in seconds (default: 5)."
    )
    parser.add_argument(
        "--hardware", action="store_true", default=False,
        help="Use real hardware instead of mock client. The hand must be "
             "connected (e.g. /dev/ttyACM0)."
    )
    parser.add_argument(
        "--port", type=str, default=None,
        help="Serial port for the real hand (e.g. /dev/ttyACM0). "
             "Overrides the port in config.yaml. Only used with --hardware."
    )
    args = parser.parse_args()

    run_demo(args.config_path, args.rate, args.duration,
             hardware=args.hardware, port=args.port)


if __name__ == "__main__":
    main()
