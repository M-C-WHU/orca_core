# OrcaHand ROS2 Integration

This directory contains a ROS2 translation layer for the ORCA dexterous robotic hand. It bridges the OrcaHand Python API with standard ROS2 communication patterns (topics + services), making the hand controllable from any ROS2-compatible system.

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    ROS2 Graph                            │
│                                                          │
│  /orcahand/joint_commands ──► OrcaHandNode ──► Worker   │
│  (sensor_msgs/JointState)          │            Thread   │
│                                     │                    │
│  /orcahand/joint_states  ◄─── Timer ◄─── OrcaHand API  │
│  (sensor_msgs/JointState)                                │
│                                                          │
│  Services:                                               │
│    /orcahand/connect        [Trigger]                    │
│    /orcahand/disconnect     [Trigger]                    │
│    /orcahand/init_joints    [Trigger]                    │
│    /orcahand/calibrate      [Trigger]                    │
│    /orcahand/enable_torque  [SetBool]                    │
│    /orcahand/stop_task      [Trigger]                    │
└──────────────────────────────────────────────────────────┘
```

### Key Design Decision: Worker Thread Isolation

The OrcaHand API is **synchronous and blocking** — motor I/O operations (read positions, set positions, calibrate) can take tens of milliseconds to seconds. ROS2 callback functions must not block the executor, or the entire node freezes.

**Solution:** All OrcaHand I/O is dispatched to a dedicated `OrcaHandWorker` thread via a command queue. ROS2 callbacks (subscription callbacks, service handlers) push commands into the queue and return immediately. The worker executes commands sequentially, which also:

1. **Prevents bus contention** — no two methods race on the serial bus.
2. **Preserves ordering** — commands execute in FIFO order.
3. **Enables async ROS2 patterns** — service calls can block-wait on the result, while subscription callbacks fire-and-forget.

## Files

| File | Description |
|------|-------------|
| `orcahand_ros2_node.py` | Main ROS2 node + OrcaHandWorker class |
| `orcahand_ros2_demo.py` | Standalone demo (no ROS2 or hardware required) |
| `package.xml` | ROS2 package manifest |
| `setup.py` | Python package setup for colcon build |
| `resource/` | ROS2 package resource marker |

## Quick Start

### Without ROS2 (Standalone Demo)

Test the translation layer with a simulated hand — no ROS2 or physical hardware needed:

```bash
cd /path/to/orca_core
uv run python scripts/ros2/orcahand_ros2_demo.py --duration 5
```

This will:
1. Create a MockOrcaHand (simulated hardware)
2. Start the OrcaHandWorker (the same worker the ROS2 node uses)
3. Simulate ROS2 pub/sub messaging
4. Run a sinusoidal joint motion sequence
5. Test all service-like operations

### With ROS2 (Real Deployment)

1. **Install ROS2** (Humble, Jazzy, or Rolling):
   ```bash
   # Ubuntu example
   sudo apt install ros-jazzy-desktop
   source /opt/ros/jazzy/setup.bash
   ```

2. **Create a colcon workspace** and copy/symlink this package:
   ```bash
   mkdir -p ~/ros2_ws/src
   ln -s /path/to/orca_core/scripts/ros2 ~/ros2_ws/src/orcahand_ros2
   cd ~/ros2_ws
   colcon build --packages-select orcahand_ros2
   source install/setup.bash
   ```

3. **Run the node**:
   ```bash
   ros2 run orcahand_ros2 orcahand_node --ros-args \
       -p config_path:=/path/to/config.yaml \
       -p publish_rate_hz:=30.0
   ```

4. **In another terminal**, interact:
   ```bash
   # Read joint states
   ros2 topic echo /orcahand/joint_states

   # Send a position command
   ros2 topic pub --once /orcahand/joint_commands sensor_msgs/msg/JointState \
       "{name: ['index_mcp', 'thumb_mcp'], position: [0.5, 0.3]}"

   # Connect to the hand
   ros2 service call /orcahand/connect std_srvs/srv/Trigger

   # Enable torque
   ros2 service call /orcahand/enable_torque std_srvs/srv/SetBool "{data: true}"

   # Initialize joints (enable torque + calibrate if needed)
   ros2 service call /orcahand/init_joints std_srvs/srv/Trigger
   ```

### With Mock Hardware in ROS2

For testing within a ROS2 graph without a real hand:

```bash
ros2 run orcahand_ros2 orcahand_node --ros-args -p mock:=true
```

## Topics

### Published

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/orcahand/joint_states` | `sensor_msgs/JointState` | Best-effort, volatile | Current joint positions (rad) at `publish_rate_hz` |

### Subscribed

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/orcahand/joint_commands` | `sensor_msgs/JointState` | Reliable, volatile | Desired joint positions (rad). Only joints listed in `name` are set. |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/orcahand/connect` | `std_srvs/Trigger` | Open serial connection to the hand |
| `/orcahand/disconnect` | `std_srvs/Trigger` | Disable torque and close connection |
| `/orcahand/init_joints` | `std_srvs/Trigger` | Full initialization: torque + mode + calibrate + neutral |
| `/orcahand/calibrate` | `std_srvs/Trigger` | Run calibration routine |
| `/orcahand/enable_torque` | `std_srvs/SetBool` | `data=true` → enable, `data=false` → disable |
| `/orcahand/stop_task` | `std_srvs/Trigger` | Stop any running background task |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `config_path` | `string` | `""` | Path to `config.yaml`. Empty = bundled model. |
| `publish_rate_hz` | `double` | `30.0` | Joint state publish rate |
| `mock` | `bool` | `false` | Use MockOrcaHand (no hardware) |

## Async Considerations

The OrcaHand API has several blocking operations:

- **`connect()`** — scans ports, may take seconds
- **`calibrate()`** — drives each joint to limits, takes minutes
- **`set_joint_positions()`** with interpolation — sleeps between waypoints
- **`init_joints()`** — combines all of the above

All of these are dispatched to the worker thread. The ROS2 executor continues processing:
- Joint state publishing continues during calibration
- New commands are queued and executed after the current one finishes
- Service calls block their caller but don't freeze the node

For long-running operations (calibration, init), consider using the `blocking=False` option in the underlying API and monitor state via the `/orcahand/joint_states` topic instead of blocking on the service call.

## Joint Names

The standard OrcaHand v2 has 17 joints:

```
wrist, thumb_cmc, thumb_abd, thumb_mcp, thumb_dip,
index_abd, index_mcp, index_pip,
middle_abd, middle_mcp, middle_pip,
ring_abd, ring_mcp, ring_pip,
pinky_abd, pinky_mcp, pinky_pip
```

All positions are in **radians**.
