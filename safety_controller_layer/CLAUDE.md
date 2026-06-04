# safety_controller_layer/ — real-time motion + emergency braking (ROS2, on the rover)

The physical-execution layer below the agent: a `ament_python` ROS2 package with two nodes that
turn a coarse `heading + distance` goal into safe motor commands. Runs **on the rover**.

## Key files

- `control_math.py` — **pure, the core logic.** `SafetyController` takes injected callables
  (`get_yaw`, `get_position`, `publish_twist`, `sleep`, `now`, `should_abort`) and implements
  `rotate_to_heading` (P-control, `heading_kp`, `wrap_angle`, `proportional_turn`) then
  `drive_distance` (open-loop to odom target), each with a computed timeout budget. Plus
  `ControllerParams`, `ControllerTimeoutError`, `ControllerCancelledError`. **Put control logic here.**
- `safety_controller_node.py` — **hardware-only.** `ExecuteCommand` action server. Subscribes
  `/imu`+`/odom`, publishes `/cmd_vel_raw`. MultiThreadedExecutor + ReentrantCallbackGroup so
  sensors keep flowing during a blocking goal. Missing/stale `/imu`/`/odom` → aborts the goal.
- `aeb_math.py` — **pure.** `BrakeStateMachine` (hysteresis: trip <0.40 m, release >0.60 m for
  0.5 s — the dead band stops chatter), `gate_twist` (zero forward when braking; pass rotation +
  reverse), `min_forward_range` (nearest lidar reading in the forward arc). Plus `AebParams`.
- `aeb_node.py` — **hardware-only.** Subscribes `/cmd_vel_raw`+`/scan`, publishes `/cmd_vel` at
  ~20 Hz. `/scan` QoS is BEST_EFFORT (sensor_data) to match the YDLidar. Fail-safe: stale/missing
  `/scan` → brake; stale/missing `/cmd_vel_raw` → zero Twist.

## ⚠️ The one thing to never forget

`aeb_node` is **the only republisher of `/cmd_vel_raw → /cmd_vel`**, and `limo_base` listens only
to `/cmd_vel`. So **AEB is mandatory for motion** — `use_aeb:=false` = no motion (rover sits, goal
aborts "did not converge"). Always launch with AEB on; verify `ros2 topic info /cmd_vel` shows
`Publisher count: 1`. See ADR `context/decisions/0003-aeb-is-the-cmd-vel-relay.md`.

## Build / run

This package + `safety_controller_layer_interfaces/` (ament_cmake, defines `ExecuteCommand.action`)
build via colcon on the rover. Launch both nodes: `ros2 launch safety_controller_layer
rovermind.launch.py` (the launch file lives in `launch/`). The launch tears down if either node exits.

## Tests

`tests/test_control_math.py`, `test_controller.py`, `test_aeb_math.py`, `test_aeb_qos.py`,
`test_execute_command_integration.py`, `test_missing_sensors_integration.py`, `test_launch_file.py`,
`test_ros_package_loads.py`.
