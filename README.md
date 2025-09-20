

# MecaBridge Hardware Interface

ROS 2 (ros2_control) Hardware Interface for Raspberry Pi Pico providing a safe, low-latency serial bridge for mecanum mobile bases with DC wheel motors, servos, and ESCs.

## Architecture

- **SystemInterface**: Supports multiple wheel configurations (2 or 4 wheels)
- **Drive Types**: Configurable for "mecanum" or "differential" drive modes
- **Safety**: Watchdog-enforced safe stop (≤150ms), CRC integrity validation
- **Performance**: End-to-end latency ≤20ms (p95), deterministic frame protocol
- **Configuration**: Single authoritative YAML config for geometry, joints, and calibration

## Structure

```
src/mecabridge_hardware/     # Main ROS 2 package
├── include/mecabridge_hardware/  # SystemInterface headers + protocol contracts
├── src/mecabridge_hardware/      # ros2_control implementation + protocol lib
├── src/mecabridge_utils/         # shared helpers (CRC, YAML validation, watchdog)
├── test/mecabridge/              # GTest + integration harnesses
├── config/                       # YAML templates & sample configs
└── launch/                       # bringup + test launches

firmware/mecabridge_pico/    # Pico firmware
├── src/                     # Pico firmware modules (serial, pwm, watchdog)
├── include/                 # Headers
└── tests/                   # Firmware tests
```

## Quickstart

1. **Build Package**:
   ```bash
   colcon build --packages-select mecabridge_hardware
   ```

2. **Launch Mecanum Drive**:
   ```bash
   ros2 launch mecabridge_hardware mecanum_drive.launch.py
   ```

3. **Launch Differential Drive**:
   ```bash
   ros2 launch mecabridge_hardware differential_drive.launch.py
   ```

4. **Test with cmd_vel**:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"
   ```

## Configuration

Edit `config/mecabridge_config.yaml` to:
- Define joint names and wheel geometry
- Set encoder parameters and calibration
- Configure servo and ESC scaling
- Adjust safety parameters (watchdog timeout, etc.)

## Protocol

The hardware interface communicates with Pico firmware using a deterministic frame protocol:
- **Frame Format**: START_BYTE (0xAA) + frame_id + length + payload + CRC-16
- **Safety**: Watchdog timeout ≤150ms, safe state on communication loss
- **Integrity**: CRC-16/CCITT-FALSE validation for all frames

# multi_drive_pico_ros2_control

ROS 2 (ros2_control) Hardware-Interface Vorlage für Raspberry Pi Pico mit modularer Unterstützung für Mecanum- und Differentialantriebe.

Ziel
- Ein SystemInterface, das mehrere Räder (2 oder 4) unterstützt.
- Konfigurierbar für "mecanum" oder "diff" Betriebsarten.
- Beispiel-URDF/XACRO, ros2_control-Parameter und Controller-Configs für mecanum_drive_controller und diff_drive_controller.
- Hilfs-Node zur Konvertierung von /cmd_vel -> wheel velocities (optional, wenn kein mecanum controller installiert ist).
- Dokumentation des Serial-/USB-Protokolls für Raspberry Pi Pico.

Quickstart
1. Paket anlegen: kopiere diese Struktur in ein neues ROS2-Workspace/src Verzeichnis.
2. Anpassung:
   - Passen `config/ros2_control_params.yaml` joint names & hardware params an.
   - Implementiere die Serial-/USB-Funktionen in src/multi_drive_hardware.cpp (stubs vorhanden).
   - Passe die Pico-Firmware an das Protokoll in docs/pico_protocol.md an.
3. Build:
   colcon build --packages-select multi_drive_pico_ros2_control
4. Start:
   ros2 launch multi_drive_pico_ros2_control drive_launch.py drive_type:=mecanum
   oder
   ros2 launch multi_drive_pico_ros2_control drive_launch.py drive_type:=diff
5. Controller laden (launch lädt bereits den passenden Spawner): kontrolliere mit
   ros2 service call /controller_manager/list_controllers ...
6. Teste:
   - Für mecanum: ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped ...
   - Für diff: ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped ...
