# ROS 2 Teleop Ackermann Joy

A ROS 2 node that maps joystick input to `AckermannDriveStamped` commands. Supports Xbox-style controllers with dual-trigger throttle or single-axis mode.

## Dependencies

- ROS 2 (tested on Jazzy)
- `sensor_msgs`, `ackermann_msgs`, `std_msgs`
- A joystick driver publishing to `/joy` (e.g. `joy_linux`)

## Build

```bash
cd ~/your_ws
colcon build --packages-select teleop_ackermann_joy
source install/setup.bash
```

## Run

```bash
ros2 run teleop_ackermann_joy teleop_ackermann_joy_node
```

Or with parameters via a YAML file:

```bash
ros2 run teleop_ackermann_joy teleop_ackermann_joy_node --ros-args --params-file config.yaml
```

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/joy` | `sensor_msgs/Joy` | Subscribe | Joystick input |
| `/activate` | `std_msgs/Bool` | Subscribe | Enable/disable output (starts inactive) |
| `/teleop/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | Publish | Drive command |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `require_enable_button` | bool | `false` | Require a button held to send commands |
| `enable_button` | int | `4` | Joystick button index for enable (e.g. LB) |
| `axis_dead_zone` | double | `0.1` | Ignore axis values below this threshold |
| `use_trigger_mode` | bool | `true` | Use dual triggers for throttle (forward/reverse) |
| `axis_speed` | int | `1` | Axis index for speed (single-axis mode) |
| `axis_forward_trigger` | int | `5` | Axis index for forward trigger (RT) |
| `axis_reverse_trigger` | int | `4` | Axis index for reverse trigger (LT) |
| `max_speed_ms` | double | `1.0` | Max speed in m/s |
| `axis_steering` | int | `0` | Axis index for steering (left stick horizontal) |
| `scale_steering` | double | `1.0` | Max steering angle in radians |
| `invert_steering` | bool | `false` | Flip steering direction |

## Activation

The node starts **inactive** and publishes zero commands until it receives `True` on `/activate`. This allows a control multiplexer to switch between teleop and autonomous modes. Publishing `False` deactivates the node and sends a zero command for safety.

## License

Apache-2.0
