#!/usr/bin/env python3
"""
OrcaHand ROS2 Node — Translation layer between ROS2 and the OrcaHand API.

Exposes the OrcaHand as a standard ROS2-controlled robot hand:
  - Publishes JointState on /orcahand/joint_states
  - Subscribes to JointState on /orcahand/joint_commands
  - Provides services for connect / disconnect / calibrate / torque / init
  - Properly isolates blocking OrcaHand calls on a worker thread so the
    ROS2 executor is never starved.

Usage (with a real ROS2 workspace):
    ros2 run orcahand_ros2 orcahand_node --ros-args \
        -p config_path:=/path/to/config.yaml \
        -p publish_rate_hz:=30.0
"""

from __future__ import annotations

import enum
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

# --- ROS2 imports (deferred — will fail only at runtime if not installed) ---
try:
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger, SetBool
    from example_interfaces.srv import Trigger as ExampleTrigger  # fallback

    _HAS_RCLPY = True
except ImportError:
    _HAS_RCLPY = False

# --- OrcaHand import ---
import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from orca_core import OrcaHand, OrcaJointPositions


def _install_fake_dynamixel_sdk():
    """Install a minimal fake dynamixel_sdk for MockDynamixelClient."""
    if "dynamixel_sdk" in sys.modules:
        return
    import types as _types
    _mod = _types.ModuleType("dynamixel_sdk")

    class _PortHandler:
        def __init__(self, port):
            self.port_name = port
            self.is_using = False
            self.is_open = False
            self.baudrate = 1000000
        def openPort(self):
            self.is_open = True
            self.is_using = True
            return True
        def closePort(self):
            self.is_open = False
            self.is_using = False
        def setBaudRate(self, baudrate):
            self.baudrate = baudrate
            return True
        def getBaudRate(self):
            return self.baudrate
        def readPort(self, length):
            return b"\x00" * length
        def writePort(self, data):
            return len(data)

    class _PacketHandler:
        def __init__(self, protocol_version):
            self.protocol_version = protocol_version
        def getTxRxResult(self, result):
            return "[RX-ROM_RESULT] Success"
        def getRxPacketError(self, error):
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

    class _GroupBulkRead:
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

    _mod.PortHandler = _PortHandler
    _mod.PacketHandler = _PacketHandler
    _mod.GroupBulkRead = _GroupBulkRead
    _mod.COMM_SUCCESS = 0
    _mod.COMM_PORT_BUSY = -1004
    sys.modules["dynamixel_sdk"] = _mod


def _create_mock_hand(config_path: Optional[str] = None) -> OrcaHand:
    """Create an OrcaHand backed by MockDynamixelClient for testing."""
    _install_fake_dynamixel_sdk()
    from orca_core.hardware.mock_dynamixel_client import MockDynamixelClient

    if config_path is None:
        config_path = str(
            _PROJECT_ROOT / "orca_core" / "models" / "v1" / "orcahand_right" / "config.yaml"
        )

    hand = OrcaHand(config_path=config_path)

    # Monkey-patch to return MockDynamixelClient instead of real hardware
    def _mock_create_motor_client():
        return MockDynamixelClient(
            hand.config.motor_ids, hand.config.port, hand.config.baudrate
        )

    hand._create_motor_client = _mock_create_motor_client
    return hand


# ===========================================================================
# Worker thread — serialises all OrcaHand I/O so we never race the bus
# ===========================================================================

class _CommandKind(enum.Enum):
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
    kind: _CommandKind
    kwargs: Dict[str, Any] = field(default_factory=dict)
    result_event: threading.Event = field(default_factory=threading.Event)
    result_value: Any = None
    error: Optional[Exception] = None


class OrcaHandWorker:
    """Background worker that owns the OrcaHand and serialises all I/O.

    ROS2 callbacks push commands into a thread-safe queue; the worker
    executes them sequentially and signals completion via per-command
    ``Event`` objects.  This guarantees:
      1. No two OrcaHand methods race on the serial bus.
      2. ROS2 executor threads never block on hardware I/O.
      3. Async-friendly: callers can await the result or fire-and-forget.
    """

    def __init__(self, config_path: Optional[str] = None, mock: bool = False):
        self._lock = threading.Lock()
        self._queue: list[_Command] = []
        self._queue_event = threading.Event()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._mock = mock

        # Create the hand but do NOT connect yet
        if mock:
            self._hand = _create_mock_hand(config_path)
        else:
            self._hand = OrcaHand(config_path=config_path)

    # --- public queries (thread-safe, read-only) ---

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

    # --- command dispatch ---

    def _enqueue(self, kind: _CommandKind, kwargs: dict | None = None,
                 block: bool = True, timeout: float = 30.0) -> _Command:
        cmd = _Command(kind=kind, kwargs=kwargs or {})
        with self._lock:
            self._queue.append(cmd)
            self._queue_event.set()
        if block:
            cmd.result_event.wait(timeout=timeout)
        return cmd

    def submit(self, kind: _CommandKind, kwargs: dict | None = None,
               block: bool = True, timeout: float = 30.0) -> Any:
        """Submit a command and optionally wait for the result."""
        cmd = self._enqueue(kind, kwargs, block, timeout)
        if block and cmd.error is not None:
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

    # --- internal loop ---

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
        handler = {
            _CommandKind.SET_JOINT_POS: self._do_set_joint_pos,
            _CommandKind.ENABLE_TORQUE: self._do_enable_torque,
            _CommandKind.DISABLE_TORQUE: self._do_disable_torque,
            _CommandKind.SET_MAX_CURRENT: self._do_set_max_current,
            _CommandKind.SET_CONTROL_MODE: self._do_set_control_mode,
            _CommandKind.INIT_JOINTS: self._do_init_joints,
            _CommandKind.CALIBRATE: self._do_calibrate,
            _CommandKind.STOP_TASK: self._do_stop_task,
        }.get(cmd.kind)
        if handler is None:
            raise ValueError(f"Unknown command kind: {cmd.kind}")
        return handler(cmd.kwargs)

    # --- command implementations ---

    def _do_connect(self, kwargs: dict) -> tuple[bool, str]:
        return self._hand.connect()

    def _do_disconnect(self, kwargs: dict) -> tuple[bool, str]:
        return self._hand.disconnect()

    def _do_set_joint_pos(self, kwargs: dict) -> bool:
        positions = kwargs["positions"]  # dict[str, float]
        num_steps = kwargs.get("num_steps", 1)
        step_size = kwargs.get("step_size", 1e-2)
        jp = OrcaJointPositions.from_dict(positions)
        self._hand.set_joint_positions(jp, num_steps=num_steps, step_size=step_size)
        return True

    def _do_enable_torque(self, kwargs: dict) -> bool:
        motor_ids = kwargs.get("motor_ids")
        self._hand.enable_torque(motor_ids=motor_ids)
        return True

    def _do_disable_torque(self, kwargs: dict) -> bool:
        motor_ids = kwargs.get("motor_ids")
        self._hand.disable_torque(motor_ids=motor_ids)
        return True

    def _do_set_max_current(self, kwargs: dict) -> bool:
        self._hand.set_max_current(current=kwargs["current"])
        return True

    def _do_set_control_mode(self, kwargs: dict) -> bool:
        self._hand.set_control_mode(mode=kwargs["mode"])
        return True

    def _do_init_joints(self, kwargs: dict) -> bool:
        force_calibrate = kwargs.get("force_calibrate", False)
        self._hand.init_joints(force_calibrate=force_calibrate)
        return True

    def _do_calibrate(self, kwargs: dict) -> bool:
        blocking = kwargs.get("blocking", True)
        self._hand.calibrate(blocking=blocking)
        return True

    def _do_stop_task(self, kwargs: dict) -> bool:
        self._hand.stop_task()
        return True


# ===========================================================================
# ROS2 Node
# ===========================================================================

if _HAS_RCLPY:

    class OrcaHandNode(Node):
        """ROS2 node that translates standard ROS2 messages to OrcaHand API calls.

        Topics:
          Published:
            /orcahand/joint_states  [sensor_msgs/JointState]  — current joint positions
          Subscribed:
            /orcahand/joint_commands [sensor_msgs/JointState]  — desired joint positions

        Services:
          /orcahand/connect         [std_srvs/Trigger]
          /orcahand/disconnect      [std_srvs/Trigger]
          /orcahand/init_joints     [std_srvs/Trigger]
          /orcahand/calibrate       [std_srvs/Trigger]
          /orcahand/enable_torque   [std_srvs/SetBool]   (data=True → enable, False → disable)
          /orcahand/stop_task       [std_srvs/Trigger]

        Parameters:
          config_path (str)   — path to config.yaml (default: bundled model)
          publish_rate_hz (double) — joint-state publish rate (default: 30.0)
          mock (bool)         — use MockOrcaHand (default: False)
        """

        def __init__(self):
            super().__init__("orcahand_node")

            # --- Declare parameters ---
            self.declare_parameter("config_path", "")
            self.declare_parameter("publish_rate_hz", 30.0)
            self.declare_parameter("mock", False)

            config_path = self.get_parameter("config_path").get_parameter_value().string_value
            publish_rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
            mock = self.get_parameter("mock").get_parameter_value().bool_value

            config_path = config_path if config_path else None

            # --- Create worker ---
            self._worker = OrcaHandWorker(config_path=config_path, mock=mock)
            self._worker.start()

            self._joint_names = self._worker.get_joint_names()

            # --- QoS profiles ---
            # Joint states: best-effort for high-frequency sensor data
            state_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=1,
            )
            # Commands: reliable — we don't want to lose control messages
            cmd_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=1,
            )

            cb_group = ReentrantCallbackGroup()

            # --- Publishers ---
            self._state_pub = self.create_publisher(
                JointState, "/orcahand/joint_states", state_qos
            )

            # --- Subscribers ---
            self._cmd_sub = self.create_subscription(
                JointState,
                "/orcahand/joint_commands",
                self._on_joint_command,
                cmd_qos,
                callback_group=cb_group,
            )

            # --- Services ---
            self._srv_connect = self.create_service(
                Trigger, "/orcahand/connect", self._on_connect,
                callback_group=cb_group,
            )
            self._srv_disconnect = self.create_service(
                Trigger, "/orcahand/disconnect", self._on_disconnect,
                callback_group=cb_group,
            )
            self._srv_init = self.create_service(
                Trigger, "/orcahand/init_joints", self._on_init_joints,
                callback_group=cb_group,
            )
            self._srv_calibrate = self.create_service(
                Trigger, "/orcahand/calibrate", self._on_calibrate,
                callback_group=cb_group,
            )
            self._srv_torque = self.create_service(
                SetBool, "/orcahand/enable_torque", self._on_torque,
                callback_group=cb_group,
            )
            self._srv_stop = self.create_service(
                Trigger, "/orcahand/stop_task", self._on_stop_task,
                callback_group=cb_group,
            )

            # --- Timer for periodic state publishing ---
            period = 1.0 / publish_rate if publish_rate > 0 else 0.1
            self._timer = self.create_timer(period, self._publish_state)

            self.get_logger().info(
                f"OrcaHand node started (mock={mock}, joints={len(self._joint_names)}, "
                f"publish_rate={publish_rate:.1f} Hz)"
            )

        # --- Timer callback ---

        def _publish_state(self):
            if not self._worker.is_connected():
                return
            try:
                jp = self._worker.hand.get_joint_position()
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = list(jp.data.keys())
                msg.position = [float(v) for v in jp.data.values()]
                self._state_pub.publish(msg)
            except Exception as exc:
                self.get_logger().warn(f"Failed to read joint state: {exc}")

        # --- Subscriber callback ---

        def _on_joint_command(self, msg: JointState):
            positions = {}
            for name, pos in zip(msg.name, msg.position):
                positions[name] = float(pos)
            if not positions:
                return
            try:
                self._worker.submit(
                    _CommandKind.SET_JOINT_POS,
                    {"positions": positions, "num_steps": 1},
                    block=False,
                )
            except Exception as exc:
                self.get_logger().error(f"Failed to set joint positions: {exc}")

        # --- Service callbacks ---

        def _on_connect(self, request, response):
            try:
                ok, msg = self._worker._do_connect({})
                response.success = ok
                response.message = msg
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response

        def _on_disconnect(self, request, response):
            try:
                ok, msg = self._worker._do_disconnect({})
                response.success = ok
                response.message = msg
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response

        def _on_init_joints(self, request, response):
            try:
                self._worker.submit(_CommandKind.INIT_JOINTS, timeout=60.0)
                response.success = True
                response.message = "Joints initialized."
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response

        def _on_calibrate(self, request, response):
            try:
                self._worker.submit(_CommandKind.CALIBRATE, timeout=120.0)
                response.success = True
                response.message = "Calibration complete."
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response

        def _on_torque(self, request, response):
            try:
                if request.data:
                    self._worker.submit(_CommandKind.ENABLE_TORQUE)
                else:
                    self._worker.submit(_CommandKind.DISABLE_TORQUE)
                response.success = True
                response.message = "Torque enabled." if request.data else "Torque disabled."
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response

        def _on_stop_task(self, request, response):
            try:
                self._worker.submit(_CommandKind.STOP_TASK)
                response.success = True
                response.message = "Task stopped."
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response

        def destroy_node(self):
            self._worker.stop()
            # Best-effort disconnect
            try:
                if self._worker.is_connected():
                    self._worker._do_disconnect({})
            except Exception:
                pass
            super().destroy_node()


# ===========================================================================
# Entry point
# ===========================================================================

def main(args=None):
    if not _HAS_RCLPY:
        print(
            "ERROR: rclpy is not installed. Install ROS2 and source the setup "
            "before running this node. For testing without ROS2, use the "
            "standalone demo: python scripts/ros2/orcahand_ros2_demo.py"
        )
        sys.exit(1)

    rclpy.init(args=args)
    node = OrcaHandNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
