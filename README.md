# drive_arduino

`drive_arduino` provides ros2_control hardware interfaces for Arduino and Pico
based motor drivers. The original TB6612 plugin remains available while the new
**MecaBridge** plugin connects a Raspberry Pi 4B (ROS 2) to a Raspberry Pi Pico
responsible for PWM generation and encoder sampling on a mecanum-wheel robot.

## MecaBridge Overview

- **Transport:** USB CDC serial link running a simple framed protocol
  (start-byte, frame-id, length, payload, CRC16).
- **Actuators:** four mecanum wheel motors, a 180° positional servo, a 360°
  continuous rotation servo and two auxiliary ESC outputs.
- **Feedback:** wheel positions/velocities, encoder counts, servo state and ESC
  values are reported back to ROS 2.
- **Safety:** heartbeat frames are exchanged continuously. If the Pico detects a
  missing heartbeat it disables all outputs.

The ROS 2 side is implemented in `mecabridge::MecaBridgeHardwareInterface`. The
matching Pico firmware lives in `firmware/pico/` with modular drivers for
motors, servos and ESCs controlled via compile-time feature flags.

## Configuration

A single YAML file (`controllers/mecabridge_config.yaml`) acts as the source of
truth for joint names, wheel geometry, encoder resolution and command limits for
servos/ESCs. The file also registers the controllers:

- `mecanum_drive_controller` listening on `/cmd_vel`.
- `joint_state_broadcaster` publishing feedback.
- Forward command controllers for the servos and ESCs.

## Launching

```
ros2 launch drive_arduino mecabridge_bringup.launch.py
```

The launch file starts `ros2_control_node` with the MecaBridge hardware plugin
and spawns all controllers. Override the configuration with the `config`
argument if required:

```
ros2 launch drive_arduino mecabridge_bringup.launch.py \
  config:=/path/to/custom_mecabridge.yaml
```

## Serial Protocol

- **Command frames (ID 0x01)** contain wheel velocities (rad/s), servo position
  (rad), continuous servo velocity (rad/s), ESC throttle values and a heartbeat
  counter plus safety flags.
- **State frames (ID 0x02)** return wheel positions/velocities, encoder counts,
  servo state, ESC state and status flags.
- **Heartbeat frames (ID 0x03)** can be sent by either side to confirm liveness.
- **Stop frames (ID 0x7E)** instruct the Pico to brake all actuators immediately.

CRC16-CCITT (seed 0xFFFF, polynomial 0x1021) is applied to the frame header and
payload.

The protocol is intentionally compact to keep the microcontroller firmware
simple while leaving headroom for future extensions.
