# MecaBridge Hardware Interface Spezifikation

## Übersicht
MecaBridge ist eine ros2_control SystemInterface-Implementierung für den omnidirektionalen ROS2-Roboter mit Nerf-Launcher. Sie verbindet den Raspberry Pi 4B (ros2_control) mit dem Raspberry Pi Pico (Low-Level-Hardware-Steuerung) über serielle Kommunikation.

## Architektur-Mapping

### High-Level (Raspberry Pi 4B)
- **ros2_control Controller Manager**: Führt die Update-Schleife (read → update → write) aus
- **mecanum_drive_controller**: Konvertiert geometry_msgs/TwistStamped zu Radgeschwindigkeiten
- **MecaBridge SystemInterface**: Hardware-Abstraktionsschicht für Pico-Kommunikation
- **joint_state_broadcaster**: Publiziert Joint-States für alle Komponenten

### Low-Level (Raspberry Pi Pico)  
- **Mecanum Drive**: 4x DC-Motoren GM3865-520 mit Hall-Encodern über TB6612FNG
- **Turret Control**: Pan/Tilt-Servos für Nerf-Launcher-Ausrichtung
- **Launcher Motors**: 2x RS2205 Brushless-Motoren mit ESCs
- **Sensor Fusion**: VL53L0X ToF, ICM-20948 IMU, Encoder-Feedback

## Hardware Interface Design

### State Interfaces (read() Methode)
```yaml
# Mecanum Wheels (4x)
front_left_wheel_joint/velocity     # rad/s aus Encoder
front_right_wheel_joint/velocity
rear_left_wheel_joint/velocity
rear_right_wheel_joint/velocity

front_left_wheel_joint/position     # rad kumulativ
front_right_wheel_joint/position
rear_left_wheel_joint/position
rear_right_wheel_joint/position

# Turret System
turret_pan_joint/position           # Servo-Position in rad
turret_tilt_joint/position

# Battery/System Health
battery_voltage                     # INA3221-Werte
battery_current
system_temperature
```

### Command Interfaces (write() Methode)
```yaml
# Mecanum Wheels (4x)
front_left_wheel_joint/velocity     # Kommandierte rad/s
front_right_wheel_joint/velocity
rear_left_wheel_joint/velocity
rear_right_wheel_joint/velocity

# Turret Control
turret_pan_joint/position           # Servo-Zielposition
turret_tilt_joint/position

# Launcher System
launcher_flywheel_speed             # Brushless Motor RPM
launcher_fire_trigger               # Boolean für Schussauslösung
```

## Protokoll-Spezifikation

### Frame Format
```
START_BYTE (0xAA) | FRAME_ID (1B) | LENGTH (1B) | PAYLOAD (N) | CRC16 (2B)
```

### Kommando-Frames (Pi 4B → Pico)
```
CMD_WHEEL_VELOCITIES    = 0x10    # 4x float32 (rad/s)
CMD_TURRET_POSITION     = 0x11    # 2x float32 (pan/tilt rad)  
CMD_LAUNCHER_CONTROL    = 0x12    # float32 speed + uint8 fire
CMD_SYSTEM_RESET        = 0x1F    # Emergency stop
```

### Status-Frames (Pico → Pi 4B) 
```
STATUS_WHEEL_FEEDBACK   = 0x20    # 4x float32 vel + 4x float32 pos
STATUS_TURRET_FEEDBACK  = 0x21    # 2x float32 current pos
STATUS_BATTERY_HEALTH   = 0x22    # 3x float32 (V, A, temp)
STATUS_ERROR_CODE       = 0x2F    # uint16 error flags
```

### Safety & Timing
- **Watchdog**: Pico stoppt Motoren bei >150ms ohne Valid-Frame
- **CRC-16**: CCITT-FALSE für Integrität
- **Update-Rate**: 50Hz (mecanum_drive_controller Standard)
- **Latency-Ziel**: <20ms end-to-end (95th percentile)

## ros2_control Integration

### URDF Configuration
```xml
<ros2_control name="MecaBridgeSystem" type="system">
  <hardware>
    <plugin>mecabridge_hardware/MecaBridgeSystemHardware</plugin>
    <param name="device_port">/dev/ttyACM0</param>
    <param name="baud_rate">115200</param>
    <param name="timeout_ms">150</param>
    <param name="wheel_radius">0.04</param>
    <param name="wheelbase_width">0.169</param>
    <param name="wheelbase_length">0.16</param>
  </hardware>
  
  <!-- Mecanum Wheels -->
  <joint name="front_left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
  <!-- ... weitere 3 Räder ... -->
  
  <!-- Turret -->
  <joint name="turret_pan_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
  </joint>
  <joint name="turret_tilt_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
  </joint>
</ros2_control>
```

### Controller Configuration
```yaml
controller_manager:
  ros__parameters:
    update_rate: 50

mecanum_drive_controller:
  ros__parameters:
    reference_timeout: 0.5
    front_left_wheel_command_joint_name: "front_left_wheel_joint"
    front_right_wheel_command_joint_name: "front_right_wheel_joint"
    rear_right_wheel_command_joint_name: "rear_right_wheel_joint"
    rear_left_wheel_command_joint_name: "rear_left_wheel_joint"
    
    kinematics:
      wheels_radius: 0.04                    # 80mm Mecanum-Räder / 2
      sum_of_robot_center_projection_on_X_Y_axis: 0.329  # (0.169 + 0.16) 
    
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    enable_odom_tf: true

joint_state_broadcaster:
  type: joint_state_broadcaster/JointStateBroadcaster

turret_position_controller:
  type: position_controllers/JointGroupPositionController
  joints:
    - turret_pan_joint
    - turret_tilt_joint
```

## Implementierung SystemInterface

### Klassen-Struktur
```cpp
namespace mecabridge_hardware {

class MecaBridgeSystemHardware : public hardware_interface::SystemInterface {
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;  
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  std::unique_ptr<SerialComm> serial_;
  std::vector<double> wheel_commands_;     // 4x velocity commands  
  std::vector<double> wheel_velocities_;   // 4x measured velocities
  std::vector<double> wheel_positions_;    // 4x encoder positions
  std::vector<double> turret_commands_;    // 2x position commands
  std::vector<double> turret_positions_;   // 2x servo positions
};

}
```

### read() Implementation
```cpp
hardware_interface::return_type MecaBridgeSystemHardware::read(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  
  // Request aktueller Status vom Pico
  if (!serial_->requestStatusUpdate()) {
    RCLCPP_WARN(logger_, "Failed to request status from Pico");
    return hardware_interface::return_type::ERROR;
  }
  
  // Parse eingehende Status-Frames
  StatusFrame frame;
  if (serial_->readStatusFrame(frame)) {
    switch (frame.type) {
      case STATUS_WHEEL_FEEDBACK:
        std::memcpy(wheel_velocities_.data(), frame.wheel_velocities, 4 * sizeof(float));
        std::memcpy(wheel_positions_.data(), frame.wheel_positions, 4 * sizeof(float));
        break;
      case STATUS_TURRET_FEEDBACK:
        std::memcpy(turret_positions_.data(), frame.turret_positions, 2 * sizeof(float));
        break;
      // ... weitere Status-Frame-Typen
    }
  }
  
  return hardware_interface::return_type::OK;
}
```

### write() Implementation  
```cpp
hardware_interface::return_type MecaBridgeSystemHardware::write(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
    
  // Sende Wheel-Velocity-Kommandos
  CommandFrame wheel_cmd;
  wheel_cmd.type = CMD_WHEEL_VELOCITIES;
  std::memcpy(wheel_cmd.wheel_velocities, wheel_commands_.data(), 4 * sizeof(float));
  
  if (!serial_->sendCommand(wheel_cmd)) {
    RCLCPP_ERROR(logger_, "Failed to send wheel commands");
    return hardware_interface::return_type::ERROR;
  }
  
  // Sende Turret-Position-Kommandos
  CommandFrame turret_cmd;
  turret_cmd.type = CMD_TURRET_POSITION;
  std::memcpy(turret_cmd.turret_positions, turret_commands_.data(), 2 * sizeof(float));
  
  if (!serial_->sendCommand(turret_cmd)) {
    RCLCPP_ERROR(logger_, "Failed to send turret commands");  
    return hardware_interface::return_type::ERROR;
  }
  
  return hardware_interface::return_type::OK;
}
```

## Pico-Firmware-Architektur

### Hauptkomponenten
```c
// Core Loop (50Hz mit Timer-Interrupt)
void core_control_loop() {
    // 1. Lese Encoder & Sensoren
    update_wheel_encoders();
    update_sensor_readings();
    
    // 2. Führe Motor-Regelung aus  
    pid_control_wheels();
    update_servo_positions();
    
    // 3. Sende Status zurück
    send_status_frames();
    
    // 4. Watchdog-Check
    check_communication_timeout();
}

// Serial Handler (Interrupt-basiert)
void handle_incoming_command(CommandFrame* frame) {
    switch (frame->type) {
        case CMD_WHEEL_VELOCITIES:
            set_target_wheel_velocities(frame->wheel_velocities);
            reset_watchdog();
            break;
        case CMD_TURRET_POSITION:
            set_turret_target_positions(frame->turret_positions);
            break;
        case CMD_SYSTEM_RESET:
            emergency_stop_all();
            break;
    }
}
```

### Safety Implementation
```c
#define WATCHDOG_TIMEOUT_MS 150

static uint32_t last_valid_command_time = 0;

void check_communication_timeout() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    if (current_time - last_valid_command_time > WATCHDOG_TIMEOUT_MS) {
        // Emergency Stop
        stop_all_motors();
        disable_launcher();
        set_status_led(LED_ERROR);
        RCLCPP_ERROR("Communication timeout - entering safe state");
    }
}
```

## Launch-Konfiguration

### mecanum_drive.launch.py
```python
def generate_launch_description():
    # Load robot description
    robot_description = Command(['xacro ', get_package_share_directory('robot_description'), '/urdf/robot.urdf.xacro'])
    
    # Load controller configuration  
    controller_params = os.path.join(
        get_package_share_directory('mecabridge_hardware'),
        'config', 'controllers.yaml'
    )
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # ROS2 Control Node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_params],
            output='both'
        ),
        
        # Controller Spawners
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        ),
        Node(
            package='controller_manager', 
            executable='spawner',
            arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager'],
        ),
    ])
```

## Test & Validierung

### Integration Tests
```bash
# 1. Build & Install
colcon build --packages-select mecabridge_hardware
source install/setup.bash

# 2. Launch System
ros2 launch mecabridge_hardware mecanum_drive.launch.py

# 3. Test Controller Status
ros2 control list_controllers
ros2 topic echo /joint_states

# 4. Manual Teleop Test
ros2 topic pub /mecanum_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "
  twist:
    linear: {x: 0.5, y: 0.0, z: 0.0}
    angular: {x: 0.0, y: 0.0, z: 0.1}
"

# 5. Turret Control Test  
ros2 topic pub /turret_position_controller/commands std_msgs/msg/Float64MultiArray "
  data: [1.57, 0.5]  # pan=90°, tilt=30°
"
```

### Performance Monitoring
```bash
# Latency Measurement
ros2 topic hz /joint_states
ros2 topic hz /mecanum_drive_controller/odom

# Error Detection  
ros2 topic echo /mecabridge_diagnostics
journalctl -u ros2-robot.service -f
```

## Error Handling & Diagnostics

### Error Codes (Pico → Pi 4B)
```
0x0001: Encoder_Failure_FL        # Front-Left Encoder offline
0x0002: Encoder_Failure_FR        
0x0004: Motor_Driver_Overheat     # TB6612FNG thermal shutdown
0x0008: Battery_Voltage_Low       # <10.5V kritisch
0x0010: Communication_CRC_Error   # Frame corruption  
0x0020: Launcher_Motor_Jam        # ESC error signal
0x0040: Servo_Position_Error      # Servo feedback mismatch
0x8000: System_Emergency_Stop     # Manual E-Stop aktiviert
```

### Diagnostic Publisher
```cpp
class MecaBridgeDiagnostics {
  void publishDiagnostics() {
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    
    // Battery Health
    diagnostic_msgs::msg::DiagnosticStatus battery_status;
    battery_status.name = "mecabridge_battery";
    battery_status.level = (battery_voltage_ > 11.0) ? 
      diagnostic_msgs::msg::DiagnosticStatus::OK : 
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    
    // Communication Health
    diagnostic_msgs::msg::DiagnosticStatus comm_status;
    comm_status.name = "mecabridge_communication"; 
    comm_status.level = (communication_errors_ < 5) ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      
    diag_array.status = {battery_status, comm_status};
    diagnostics_pub_->publish(diag_array);
  }
};
```

## Konfigurationsparameter

### Hardware-spezifische Parameter
```yaml
mecabridge_hardware:
  ros__parameters:
    # Serial Communication
    device_port: "/dev/ttyACM0"
    baud_rate: 115200
    timeout_ms: 150
    
    # Mechanical Properties
    wheel_radius: 0.04              # 80mm Mecanum wheels
    wheel_separation_x: 0.169       # Distance between left/right wheels  
    wheel_separation_y: 0.16        # Distance between front/rear wheels
    
    # Motor Specifications
    encoder_counts_per_rev: 1440    # GM3865-520 Hall encoder
    gear_ratio: 30.0                # Getriebe-Untersetzung
    max_wheel_velocity: 10.0        # rad/s mechanisches Limit
    
    # Servo Limits
    pan_min_angle: -3.14159         # ±180° Pan-Bereich  
    pan_max_angle: 3.14159
    tilt_min_angle: -0.17453        # -10° bis +30° Tilt
    tilt_max_angle: 0.52360
    
    # Safety Parameters
    emergency_stop_deceleration: 5.0  # rad/s² bei E-Stop
    battery_voltage_warn: 11.0        # Warning-Schwelle
    battery_voltage_critical: 10.5    # Critical-Schwelle
```

Diese Spezifikation definiert eine vollständige ros2_control-Bridge für das omnidirektionale ROS2-Roboter-Projekt, die alle funktionalen Requirements aus den Projektdokumenten erfüllt und mit dem mecanum_drive_controller kompatibel ist.