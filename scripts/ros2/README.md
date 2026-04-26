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

#### With Real Hardware (No ROS2)

To test the demo with a physical ORCA hand connected via serial:

```bash
uv run python scripts/ros2/orcahand_ros2_demo.py --hardware --port /dev/ttyACM0 --duration 5
```

This will:
1. Create a real OrcaHand instance (not a mock)
2. Connect to the hand on the specified serial port
3. Run the same command sequence (init → sinusoidal motion → neutral → disconnect)

**Note:** You may need to add your user to the `uucp` (or `dialout`) group for serial port access:
```bash
sudo usermod -aG uucp $USER
# Log out and back in for group changes to take effect
```

### With ROS2 via Docker (Arch Linux / Any Linux Without Native ROS2)

This is the recommended approach on Arch Linux, where ROS2 packages are not available in the official repos.

1. **Install Docker and configure Chinese mirror** (if in China):
   ```bash
   # Install Docker
   sudo pacman -S docker
   sudo systemctl enable --now docker
   sudo usermod -aG docker $USER  # re-login after this

   # Optional: configure Docker mirror for faster pulls in China
   sudo mkdir -p /etc/docker
   cat | sudo tee /etc/docker/daemon.json << 'EOF'
   {
     "registry-mirrors": [
       "https://docker.1ms.run",
       "https://docker.xuanyuan.me"
     ]
   }
   EOF
   sudo systemctl restart docker
   ```

2. **Create a colcon workspace and copy the ROS2 package** (symlinks don't resolve inside the container):
   ```bash
   mkdir -p ~/ros2_ws/src
   cp -r /path/to/orca_core/scripts/ros2 ~/ros2_ws/src/orcahand_ros2
   ```

3. **Build the package** inside a ROS2 Jazzy container:
   ```bash
   docker pull osrf/ros:jazzy-desktop-full
   docker run --rm \
     -v ~/ros2_ws:/home/user/ros2_ws \
     -v /path/to/orca_core:/home/user/orca_core \
     osrf/ros:jazzy-desktop-full \
     bash -c "
       cd /home/user/ros2_ws && \
       source /opt/ros/jazzy/setup.bash && \
       colcon build --packages-select orcahand_ros2
     "
   ```

4. **Run the ROS2 node** with serial passthrough and TTY allocation:
   ```bash
   # Install python3-serial inside container, then start the node
   docker run --rm -it \
     --name orcahand_node \
     --network host \
     -v ~/ros2_ws:/home/user/ros2_ws \
     -v /path/to/orca_core:/home/user/orca_core \
     -v /dev/ttyACM0:/dev/ttyACM0 \
     osrf/ros:jazzy-desktop-full \
     bash -c "
       apt-get update -qq && apt-get install -y -qq python3-serial && \
       source /opt/ros/jazzy/setup.bash && \
       export PYTHONPATH=/home/user/orca_core:\$PYTHONPATH && \
       cd /home/user/ros2_ws/src/orcahand_ros2 && \
       exec python3 orcahand_ros2_node.py --ros-args \
         -p config_path:=/home/user/orca_core/orca_core/models/v2/orcahand_right/config.yaml \
         -p publish_rate_hz:=30.0
     "
   ```

   > **Note:** The node is run directly via `python3` (not `ros2 run`) because the package lacks a proper Python `__init__.py`, so colcon doesn't install the module as an importable package. `PYTHONPATH` must include the `orca_core` root so that `from orca_core import OrcaHand` works.

5. **In another terminal**, interact with the running node via ROS2 CLI:
   ```bash
   # Allocate a TTY to avoid "setupterm" errors from the serial library
   docker exec -it orcahand_node bash -c '
     source /opt/ros/jazzy/setup.bash

     # Connect to the hand
     ros2 service call /orcahand/connect std_srvs/srv/Trigger "{}"

     # Enable torque
     ros2 service call /orcahand/enable_torque std_srvs/srv/SetBool "{data: true}"

     # Send fully open positions (all joints to max extension, in radians)
     ros2 topic pub --once /orcahand/joint_commands sensor_msgs/msg/JointState \
       "{name: [\"wrist\", \"thumb_cmc\", \"thumb_abd\", \"thumb_mcp\", \"thumb_dip\", \"index_abd\", \"index_mcp\", \"index_pip\", \"middle_abd\", \"middle_mcp\", \"middle_pip\", \"ring_abd\", \"ring_mcp\", \"ring_pip\", \"pinky_abd\", \"pinky_mcp\", \"pinky_pip\"], position: [0.611, 0.576, 0.960, 1.571, 1.869, 0.436, 1.745, 1.869, 0.471, 1.745, 1.869, 0.471, 1.745, 1.869, 0.524, 1.745, 1.869]}"

     # Read joint states
     ros2 topic echo /orcahand/joint_states
   '
   ```

   > **Note:** The `-it` flag for `docker exec` is required because OrcaHand's serial library uses terminal capabilities internally, which requires a TTY.

### With ROS2 (Ubuntu / Native Installation)

If you have a native ROS2 installation (Ubuntu/Debian with `apt`), use the standard workflow:

1. **Ensure ROS2 is installed**:
   ```bash
   sudo apt install ros-jazzy-desktop
   source /opt/ros/jazzy/setup.bash
   ```

2. **Create a colcon workspace** and copy this package:
   ```bash
   mkdir -p ~/ros2_ws/src
   cp -r /path/to/orca_core/scripts/ros2 ~/ros2_ws/src/orcahand_ros2
   cd ~/ros2_ws
   colcon build --packages-select orcahand_ros2
   source install/setup.bash
   ```

3. **Run the node directly** (use `python3`, not `ros2 run`):
   ```bash
   export PYTHONPATH=/path/to/orca_core:$PYTHONPATH
   python3 ~/ros2_ws/src/orcahand_ros2/orcahand_ros2_node.py --ros-args \
       -p config_path:=/path/to/orca_core/orca_core/models/v2/orcahand_right/config.yaml \
       -p publish_rate_hz:=30.0
   ```

4. **In another terminal**, interact via standard ROS2 CLI:
   ```bash
   source /opt/ros/jazzy/setup.bash

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
python3 orcahand_ros2_node.py --ros-args -p mock:=true
```

> **Why `python3` instead of `ros2 run`?**
> The ROS2 package does not contain a Python `__init__.py`, so `find_packages()` in `setup.py` finds nothing. Colcon only installs the egg-info metadata without the actual module files. Running via `python3` with the correct `PYTHONPATH` is the reliable workaround. To fix this properly, add `__init__.py` to the package directory or change `find_packages()` to use `py_modules` instead.

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
