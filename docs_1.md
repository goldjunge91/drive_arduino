````markdown name=README.md
```markdown
# mecanum_arduino_ros2_control

Projektvorlage: ROS 2 + ros2_control Hardware-Interface für ein Mecanum-Fahrwerk mit Arduino (oder ähnlichem MCU).

Ziel
- Eine saubere Projektstruktur, die eine Hardware-Interface-Implementierung für ros2_control enthält (SystemInterface).
- Beispiel-Konfigurationen für ros2_control und controller(s) (Wheel velocity controller).
- Beispiel-Launch, URDF/XACRO-Fragment und Hilfs-Node zur Umrechnung von /cmd_vel -> Radgeschwindigkeiten (falls kein fertiger mecanum controller vorhanden ist).
- Leicht anpassbar für eigene Hardware/Protokoll.

Ordnerstruktur (empfohlen)
- package.xml
- CMakeLists.txt
- include/mecanum_hardware/
  - mecanum_hardware.hpp
- src/
  - mecanum_hardware.cpp
  - cmdvel_to_wheels.cpp (optional, kann auch python sein)
- urdf/
  - mecanum_robot.urdf.xacro
- config/
  - ros2_control_params.yaml
  - controllers.yaml
- launch/
  - mecanum_control.launch.py
- docs/
  - arduino_protocol.md
- README.md

Kurzbeschreibung der wichtigsten Bestandteile
- include/... & src/...: Implementierung der Hardware-Interface-Klasse, die Serial/I²C/SPI zur MCU nutzt. Die Klasse erbt von hardware_interface::SystemInterface, implementiert on_init/on_configure/on_activate/on_deactivate, export_state_interfaces/export_command_interfaces, read() und write().
- urdf/: enthält die Gelenk-Definitionen (4 Räder), inertiale Daten optional, und den ros2_control-Block, der das Hardware-Plugin referenziert.
- config/ros2_control_params.yaml: Parameter für das Hardware-Plugin (serial_port, baudrate, joint names, wheel radius, track width, etc.) sowie Controller-Parameter.
- config/controllers.yaml: Controller-Konfiguration (z. B. JointGroupVelocityController oder ein vorhandener mecanum-Controller).
- launch/: Startet ros2_control_node (controller_manager) mit der Hardware-Plugin-Konfiguration, lädt/spawnt Controller und ggf. die Hilfsnode.

Wie verwenden / anpassen
1. Firmware / Protokoll
   - Entscheide ein robustes Serial-Protokoll (Header, Länge, Checksumme). docs/arduino_protocol.md enthält eine Vorlage.
   - MCU muss Encoder-Werte (oder Offsets) und Status liefern und Befehle für Radgeschwindigkeiten/PWM empfangen.

2. Hardware-Implementierung
   - Implementiere MecanumHardware::read() zum Lesen von Encoder-Werten und Berechnen von wheel positions/velocities.
   - Implementiere write() zum Senden der gewünschten Radgeschwindigkeiten an die MCU.
   - Achte auf Thread-Safety und Timeouts (Serial ist nicht RT-safe).

3. URDF & ros2_control
   - Definiere die 4 wheel joints im URDF mit gleichen Namen wie in config.
   - Füge den ros2_control RobotHardware-Block ein, der den Plugin-Namen deiner Hardware-Klasse referenziert.

4. Controller
   - Wenn ein mecanum-spezifischer Controller verfügbar ist, konfiguriere ihn entsprechend (joints mapping, wheel geometry).
   - Alternative: verwende einen JointGroupVelocityController für die 4 Wheel-Joints und ergänze eine kleine Node, die /cmd_vel in einzelne wheel velocities umrechnet und die wheel velocity commands an den controller_manager published (oder direkt stateful commands setzt).

5. Testen
   - Beginne mit einer Fake-Hardware-Implementierung (zum Debuggen ohne MCU).
   - Teste mit ros2 topic echo, rqt_graph, ros2 control list_controllers, ros2 service call /controller_manager/load_controller, ros2 service call /controller_manager/switch_controller.

Ich habe hier eine Beispiel-Projektstruktur mit Templates und Basisskeletten erstellt, die du als Ausgangspunkt verwenden kannst. Die Dateien unten sind Beispiele, die du in dein Paket kopieren, anpassen und erweitern kannst.
```
````

```name=package.xml
<?xml version="1.0"?>
<package format="3">
  <name>mecanum_arduino_ros2_control</name>
  <version>0.1.0</version>
  <description>ROS2 ros2_control hardware interface for Mecanum drive using Arduino</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>hardware_interface</depend>
  <depend>controller_manager</depend>
  <depend>pluginlib</depend>
  <depend>ros2_control_core</depend>
  <depend>ros2_controllers</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <build_depend>ament_cmake</build_depend>
  <exec_depend>ros2_control_node</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

```name=CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(mecanum_arduino_ros2_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control_core REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(mecanum_hardware SHARED
  src/mecanum_hardware.cpp
)

ament_target_dependencies(mecanum_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_manager
  pluginlib
  ros2_control_core
  geometry_msgs
  sensor_msgs
)

# export plugin so controller_manager / ros2_control_node can load it
pluginlib_export_plugin_description_file(hardware_interface mecanum_hardware_plugins.xml)

install(
  TARGETS mecanum_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch urdf config docs DESTINATION share/${PROJECT_NAME}/)

ament_package()
```

```name=mecanum_hardware_plugins.xml
<library path="lib/mecanum_hardware">
  <class name="mecanum_hardware::MecanumHardware" type="mecanum_hardware::MecanumHardware" base_class_type="hardware_interface::SystemInterface">
    <description>Mecanum Hardware Interface</description>
  </class>
</library>
```

```name=include/mecanum_hardware/mecanum_hardware.hpp
#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"

namespace mecanum_hardware
{

class MecanumHardware : public hardware_interface::SystemInterface
{
public:
  MecanumHardware() = default;
  ~MecanumHardware() override = default;

  // Lifecycle-like init
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // parameters read from ros2_control params
  std::string serial_port_;
  int baudrate_;
  std::vector<std::string> joint_names_;
  double wheel_radius_;
  double wheel_base_x_; // half width/half length as needed

  // state arrays
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_; // velocity commands for wheels

  // serial / comms
  std::mutex serial_mutex_;
  // placeholder for actual serial object, replace with your serial library
  // std::unique_ptr<SerialPort> serial_;

  // helpers
  bool open_serial();
  void close_serial();
  bool send_wheel_commands(const std::vector<double> & wheel_vels);
  bool read_encoders_and_update_state();
};

} // namespace mecanum_hardware
```

```name=src/mecanum_hardware.cpp
#include "mecanum_hardware/mecanum_hardware.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace mecanum_hardware
{

hardware_interface::CallbackReturn MecanumHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  // read params from info->hardware_parameters or from info->joints
  serial_port_ = info_.hardware_parameters["serial_port"].value_or("/dev/ttyACM0");
  baudrate_ = std::stoi(info_.hardware_parameters["baudrate"].value_or("115200"));

  // joints
  joint_names_.clear();
  for (const auto & j : info_.joints) {
    joint_names_.push_back(j.name);
  }
  const size_t n = joint_names_.size();
  hw_positions_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  hw_efforts_.assign(n, 0.0);
  hw_commands_.assign(n, 0.0);

  // other params
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"].value_or("0.05"));
  wheel_base_x_ = std::stod(info_.hardware_parameters["wheel_base_x"].value_or("0.25"));

  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "on_init: serial_port=%s baud=%d", serial_port_.c_str(), baudrate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumHardware::on_configure(const rclcpp_lifecycle::State &)
{
  // open serial
  if (!open_serial()) {
    RCLCPP_ERROR(rclcpp::get_logger("MecanumHardware"), "Failed to open serial port");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumHardware::on_activate(const rclcpp_lifecycle::State &)
{
  // optional: send enable to motors
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  // optional: stop motors
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecanumHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecanumHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type MecanumHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // read encoder values from MCU and update hw_positions_ / hw_velocities_
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (!read_encoders_and_update_state()) {
    RCLCPP_WARN(rclcpp::get_logger("MecanumHardware"), "Failed to read encoders");
    // still return OK, controller may handle missing updates
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // send hw_commands_ values as wheel velocities to MCU
  std::vector<double> wheel_cmds = hw_commands_;
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (!send_wheel_commands(wheel_cmds)) {
    RCLCPP_WARN(rclcpp::get_logger("MecanumHardware"), "Failed to send wheel commands");
  }
  return hardware_interface::return_type::OK;
}

// --- stub implementations for serial I/O, replace with real code ---
bool MecanumHardware::open_serial()
{
  // open your serial here and perform handshake
  // serial_.reset(new SerialPort(serial_port_, baudrate_));
  // return serial_->isOpen();
  return true;
}

void MecanumHardware::close_serial()
{
  // close serial
}

bool MecanumHardware::send_wheel_commands(const std::vector<double> & wheel_vels)
{
  // send format: e.g. 'W' <v1> <v2> <v3> <v4> <checksum>\n
  (void)wheel_vels;
  return true;
}

bool MecanumHardware::read_encoders_and_update_state()
{
  // read from serial and update hw_positions_ and hw_velocities_
  // For testing, we can leave zeros
  return true;
}

} // namespace mecanum_hardware

PLUGINLIB_EXPORT_CLASS(mecanum_hardware::MecanumHardware, hardware_interface::SystemInterface)
```

```name=config/ros2_control_params.yaml
controller_manager:
  ros__parameters:
    update_rate: 50
    # Hardware plugin will be created by ros2_control_node using this parameter set
    robot_description: ''
    ros2_control:
      hardware_components:
        - name: mecanum_hardware
          type: system
          plugin: "mecanum_hardware::MecanumHardware"
          joints:
            - name: wheel_front_left_joint
              command_interfaces: ["velocity"]
              state_interfaces: ["position","velocity"]
            - name: wheel_front_right_joint
              command_interfaces: ["velocity"]
              state_interfaces: ["position","velocity"]
            - name: wheel_rear_left_joint
              command_interfaces: ["velocity"]
              state_interfaces: ["position","velocity"]
            - name: wheel_rear_right_joint
              command_interfaces: ["velocity"]
              state_interfaces: ["position","velocity"]
          parameters:
            serial_port: "/dev/ttyACM0"
            baudrate: 115200
            wheel_radius: 0.05
            wheel_base_x: 0.22
```

```name=config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 50

# Option A: Use JointGroupVelocityController for wheel velocity control
wheel_velocity_controller:
  type: "joint_group_velocity_controller/JointGroupVelocityController"
  joints:
    - wheel_front_left_joint
    - wheel_front_right_joint
    - wheel_rear_left_joint
    - wheel_rear_right_joint

# Option B: If you have a mecanum-specific controller available, configure it here:
# mecanum_drive_controller:
#   type: "mecanum_controller/MecanumDriveController"
#   wheel_names:
#     front_left: wheel_front_left_joint
#     front_right: wheel_front_right_joint
#     rear_left: wheel_rear_left_joint
#     rear_right: wheel_rear_right_joint
#   wheel_radius: 0.05
#   wheel_base_x: 0.22
#   wheel_base_y: 0.18
```

```name=urdf/mecanum_robot.urdf.xacro
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="mecanum_robot">
  <!-- Example minimal robot: base_link + 4 wheel joints -->
  <link name="base_link"/>
  <!-- wheel links -->
  <link name="wheel_front_left_link"/>
  <link name="wheel_front_right_link"/>
  <link name="wheel_rear_left_link"/>
  <link name="wheel_rear_right_link"/>

  <!-- joints (revolute) -->
  <joint name="wheel_front_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_front_left_link"/>
    <origin xyz="0.2 0.12 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="wheel_front_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_front_right_link"/>
    <origin xyz="0.2 -0.12 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="wheel_rear_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_rear_left_link"/>
    <origin xyz="-0.2 0.12 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="wheel_rear_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_rear_right_link"/>
    <origin xyz="-0.2 -0.12 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- ros2_control block -->
  <ros2_control name="MecanumHardware" type="system">
    <hardware>
      <plugin>mecanum_hardware::MecanumHardware</plugin>
      <param name="serial_port">/dev/ttyACM0</param>
      <param name="baudrate">115200</param>
      <param name="wheel_radius">0.05</param>
      <param name="wheel_base_x">0.22</param>
    </hardware>
  </ros2_control>
</robot>
```

```name=launch/mecanum_control.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_arduino_ros2_control')
    ros2_control_params = os.path.join(pkg_share, 'config', 'ros2_control_params.yaml')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mecanum_robot.urdf.xacro')

    # ros2_control_node (controller_manager) will load the hardware plugin from params
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[ros2_control_params, {'robot_description': open(urdf_file).read()}]
    )

    load_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        controller_manager_node,
        load_controllers
    ])
```

```name=src/cmdvel_to_wheels.cpp
// Optional helper node: converts /cmd_vel (Twist) -> wheel velocities and publishes to the controller command topics.
// Example publishes a Float64MultiArray on a command topic consumed by JointGroupVelocityController or directly uses parameter update services.
// For simplicity this skeleton uses publisher to /wheel_velocity_controller/commands which may differ per controller used.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <cmath>

class CmdVelToWheels : public rclcpp::Node {
public:
  CmdVelToWheels(): Node("cmdvel_to_wheels") {
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&CmdVelToWheels::cmdvel_cb, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_velocity_controller/commands", 10);

    wheel_radius_ = this->declare_parameter("wheel_radius", 0.05);
    wheel_base_x_ = this->declare_parameter("wheel_base_x", 0.22);
    wheel_base_y_ = this->declare_parameter("wheel_base_y", 0.18);
  }

private:
  void cmdvel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double omega = msg->angular.z;

    // Mecanum wheel inverse kinematics (simple)
    // [w1]   1/r * [ 1 -1 -(L+W)] [vx]
    // [w2]        [ 1  1  (L+W)] [vy]
    // ... (example, make sure signs and geometry are correct for your robot)
    double L = wheel_base_x_;
    double W = wheel_base_y_;
    double r = wheel_radius_;

    std_msgs::msg::Float64MultiArray out;
    out.data.resize(4);

    // Compute wheel speeds (rad/s) -- ensure this matches your wheel orientation
    out.data[0] = (1.0/r)*(vx - vy - (L+W)*omega); // front left
    out.data[1] = (1.0/r)*(vx + vy + (L+W)*omega); // front right
    out.data[2] = (1.0/r)*(vx + vy - (L+W)*omega); // rear left
    out.data[3] = (1.0/r)*(vx - vy + (L+W)*omega); // rear right

    pub_->publish(out);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  double wheel_radius_;
  double wheel_base_x_;
  double wheel_base_y_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToWheels>());
  rclcpp::shutdown();
  return 0;
}
```

```name=docs/arduino_protocol.md
# Arduino Serial Protocol (Beispiel)

Dieses Dokument skizziert ein einfaches, robustes Serial-Protokoll für die Kommunikation zwischen ROS2-Hardware-Interface und Arduino.

Nachrichtenformat (binär oder ASCII, hier ASCII-beispiel)
- Kommando vom Host zu Arduino (Setze Radgeschwindigkeiten):
  Start: 'S'
  Payload: v1,v2,v3,v4  (je ein Float als ASCII, z.B. "0.12,-0.12,0.12,-0.12")
  End: '\\n'
  Beispiel: S0.12,-0.12,0.12,-0.12\n

- Antwort Arduino zu Host (Encoder / Status):
  Start: 'R'
  Payload: p1,p2,p3,p4,timestamp
  End: '\\n'
  Beispiel: R1234,2345,3456,4567,1616161616\n

Tipps:
- Füge eine einfache Checksumme (z. B. XOR aller Bytes) hinzu oder verwende ein framing-Protokoll (SLIP).
- Definiere Kommando-IDs, damit alte/neu Befehle unterschieden werden können.
- Rate-Limiting: Sende Befehle nicht schneller als MCU verarbeiten kann; hardware_interface::write() wird oftmals mit update_rate (z. B. 50 Hz) aufgerufen.
- Bei Paketfehlern: Arduino sende Status-Fehlercodes und benutze watchdogs zum sicheren Stoppen der Motoren.

Arduino-Seite:
- Empfange String bis '\\n', parse, überprüfe checksum, setze Motor-PWM / Drehzahlregelung.
- Lese Encoder in Interrupts oder zeitgesteuert und sende periodisch Encoderwerte (z. B. 20-50 Hz).
```

Zusätzliche Hinweise / Schritte für dich
- Ersetze die Stub-Serial-Funktionen im C++-Slang durch eine echte Serielle-Bibliothek (z. B. boost::asio, libserial oder platformabhängige Lösung).
- Passe die URDF Geometrie (wheel positions, wheel radius) für dein robot chassis an.
- Prüfe vorhandene ros2_controllers-Pakete: Falls ein Mecanum-spezifischer Controller verfügbar ist, kannst du diesen statt der Umrechnung-Node verwenden.
- Debugging: Starte erst mit Fake-Hardware (implementiere read() so, dass es deterministisch simulierte Encoder liefert). Dann schließe die echte MCU an.
- Lifecycle: Nutze den Lifecycle des ros2_control_node / controller_manager korrekt (configure, activate), sonst werden Controller nicht gestartet.

Wenn du willst, kann ich als nächstes:
- das Hardware-Interface an deine bestehende Arduino-Firmware anpassen, wenn du mir das Protokoll oder Beispiel-Ausgaben zeigst,
- oder eine minimal lauffähige Version erzeugen, die als Fake-Hardware simuliert, damit du das End-to-End-Setup testen kannst.

Welche Variante möchtest du als Nächstes?


Ich habe ein modulares Starter-Projekt entworfen, das sich an deiner Vorgabe orientiert (Raspberry Pi Pico statt Arduino, modulare Unterstützung für Mecanum und DiffDrive, Nutzung von ros2_control / ros2_controllers). Es enthält eine Hardware-Interface-Implementierung, die variabel mit einer beliebigen Anzahl von Rädern arbeiten kann, Konfigurationen für beide Controller-Typen und eine Hilfs-Node zur Umrechnung von /cmd_vel in Radgeschwindigkeiten (falls du keinen vorhandenen mecanum controller nutzt). Außerdem ein Protokoll-Dokument für den Raspberry Pi Pico (USB-CDC oder UART).

Ich habe die wichtigsten Dateien als Vorlage erstellt — du kannst sie in ein neues ROS 2 ament_cmake Paket kopieren (z. B. multi_drive_pico_ros2_control) und an dein Hardware-Protokoll anpassen. Nach der Dateiliste erzähle ich kurz, was als Nächstes sinnvoll ist.

````markdown name=README.md
```markdown
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

Hinweis
- Die C++-Dateien enthalten Stubs für die serielle Kommunikation (ersetze mit boost::asio, serialib oder platformabhängiger Lösung).
- Verwende TinyUSB / pico-sdk für Firmware-Seite; das Host-Interface kann USB-CDC (virtueller Serialport) oder UART nutzen.
```
````

```name=package.xml
<?xml version="1.0"?>
<package format="3">
  <name>multi_drive_pico_ros2_control</name>
  <version>0.1.0</version>
  <description>Modular ros2_control hardware interface for Mecanum and DiffDrive using Raspberry Pi Pico</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>hardware_interface</depend>
  <depend>controller_manager</depend>
  <depend>pluginlib</depend>
  <depend>ros2_control_core</depend>
  <depend>ros2_controllers</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_msgs</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

```name=CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(multi_drive_pico_ros2_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control_core REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(multi_drive_hardware SHARED
  src/multi_drive_hardware.cpp
)

ament_target_dependencies(multi_drive_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_manager
  pluginlib
  ros2_control_core
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_msgs
  std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface multi_drive_hardware_plugins.xml)

add_executable(cmdvel_to_wheels src/cmdvel_to_wheels.cpp)
ament_target_dependencies(cmdvel_to_wheels rclcpp geometry_msgs std_msgs)

install(
  TARGETS multi_drive_hardware cmdvel_to_wheels
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch urdf config docs DESTINATION share/${PROJECT_NAME}/)

ament_package()
```

```name=multi_drive_hardware_plugins.xml
<library path="lib/multi_drive_hardware">
  <class name="multi_drive_pico::MultiDriveHardware" type="multi_drive_pico::MultiDriveHardware" base_class_type="hardware_interface::SystemInterface">
    <description>MultiDrive hardware interface for Raspberry Pi Pico (mecanum/diff)</description>
  </class>
</library>
```

```name=include/multi_drive_pico/multi_drive_hardware.hpp
#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace multi_drive_pico
{

class MultiDriveHardware : public hardware_interface::SystemInterface
{
public:
  MultiDriveHardware() = default;
  ~MultiDriveHardware() override = default;

  // Lifecycle-like
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // params
  std::string serial_port_;
  int baudrate_;
  std::string drive_type_; // "mecanum" or "diff"
  size_t wheel_count_;
  std::vector<std::string> joint_names_;
  double wheel_radius_;
  double wheel_base_x_;
  double wheel_base_y_;

  // state/command vectors
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_;

  // comms
  std::mutex comms_mutex_;
  // TODO: replace with actual serial/usb object (boost::asio, serial, or custom wrapper)
  // std::unique_ptr<YourSerialClass> comms_;

  // helpers
  bool open_connection();
  void close_connection();
  bool send_wheel_commands(const std::vector<double> & wheel_vels);
  bool read_encoder_packet_and_update_state();

  // utils
  double ticks_to_rad(int64_t ticks);
};

} // namespace multi_drive_pico
```

```name=src/multi_drive_hardware.cpp
#include "multi_drive_pico/multi_drive_hardware.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace multi_drive_pico
{

hardware_interface::CallbackReturn MultiDriveHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  // store info in info_ member from SystemInterface
  info_ = info;

  // parameters in hardware_parameters
  serial_port_ = info_.hardware_parameters["serial_port"].value_or("/dev/ttyACM0");
  baudrate_ = std::stoi(info_.hardware_parameters["baudrate"].value_or("115200"));
  drive_type_ = info_.hardware_parameters["drive_type"].value_or("mecanum"); // "mecanum" or "diff"
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"].value_or("0.05"));
  wheel_base_x_ = std::stod(info_.hardware_parameters["wheel_base_x"].value_or("0.22"));
  wheel_base_y_ = std::stod(info_.hardware_parameters["wheel_base_y"].value_or("0.18"));

  joint_names_.clear();
  for (const auto & j : info_.joints) {
    joint_names_.push_back(j.name);
  }
  wheel_count_ = joint_names_.size();

  hw_positions_.assign(wheel_count_, 0.0);
  hw_velocities_.assign(wheel_count_, 0.0);
  hw_efforts_.assign(wheel_count_, 0.0);
  hw_commands_.assign(wheel_count_, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("MultiDriveHardware"), "on_init: port=%s baud=%d drive=%s wheels=%zu",
              serial_port_.c_str(), baudrate_, drive_type_.c_str(), wheel_count_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MultiDriveHardware::on_configure(const rclcpp_lifecycle::State &)
{
  if (!open_connection()) {
    RCLCPP_ERROR(rclcpp::get_logger("MultiDriveHardware"), "Failed to open connection to Pico");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MultiDriveHardware::on_activate(const rclcpp_lifecycle::State &)
{
  // optionally send enable command to motors
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MultiDriveHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  // optional: stop motors, then close
  close_connection();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MultiDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < wheel_count_; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MultiDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < wheel_count_; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type MultiDriveHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(comms_mutex_);
  if (!read_encoder_packet_and_update_state()) {
    RCLCPP_DEBUG(rclcpp::get_logger("MultiDriveHardware"), "Failed reading encoder packet (non-fatal)");
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MultiDriveHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::vector<double> wheel_cmds = hw_commands_;
  std::lock_guard<std::mutex> lock(comms_mutex_);
  if (!send_wheel_commands(wheel_cmds)) {
    RCLCPP_WARN(rclcpp::get_logger("MultiDriveHardware"), "Failed to send wheel commands");
  }
  return hardware_interface::return_type::OK;
}

// ---- Stubs for communication (replace with actual serial/USB implementation) ----
bool MultiDriveHardware::open_connection()
{
  // Example: open /dev/ttyACM0 using boost::asio or serial library
  // For now return true to allow testing with fake data
  return true;
}

void MultiDriveHardware::close_connection()
{
  // close comms
}

bool MultiDriveHardware::send_wheel_commands(const std::vector<double> & wheel_vels)
{
  // Format a packet to Pico: for example ASCII: "S v0 v1 v2 v3 CRC\n"
  // Respect wheel_count_ when formatting
  (void)wheel_vels;
  return true;
}

bool MultiDriveHardware::read_encoder_packet_and_update_state()
{
  // Read encoder values from Pico and update hw_positions_ and hw_velocities_
  // For now keep zeros (fake hardware) or implement reading/parsing here.
  return true;
}

double MultiDriveHardware::ticks_to_rad(int64_t ticks)
{
  // convert encoder ticks to radians: ticks * (2*pi / ticks_per_rev)
  // Need ticks_per_rev parameter from hardware if used
  (void)ticks;
  return 0.0;
}

} // namespace multi_drive_pico

PLUGINLIB_EXPORT_CLASS(multi_drive_pico::MultiDriveHardware, hardware_interface::SystemInterface)
```

```name=config/ros2_control_params.yaml
controller_manager:
  ros__parameters:
    update_rate: 50
    ros2_control:
      hardware_components:
        - name: multi_drive_hardware
          type: system
          plugin: "multi_drive_pico::MultiDriveHardware"
          # Joint list must match your URDF/joint names
          joints:
            - name: wheel_front_left_joint
              command_interfaces: ["velocity"]
              state_interfaces: ["position","velocity"]
            - name: wheel_front_right_joint
              command_interfaces: ["velocity"]
              state_interfaces: ["position","velocity"]
            - name: wheel_rear_left_joint
              command_interfaces: ["velocity"]
              state_interfaces: ["position","velocity"]
            - name: wheel_rear_right_joint
              command_interfaces: ["velocity"]
              state_interfaces: ["position","velocity"]
          parameters:
            serial_port: "/dev/ttyACM0"
            baudrate: 115200
            drive_type: "mecanum"  # or "diff"
            wheel_radius: 0.05
            wheel_base_x: 0.22
            wheel_base_y: 0.18
```

```name=config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 50

# Mecanum controller configuration (use existing mecanum_drive_controller package if installed)
mecanum_drive_controller:
  ros__parameters:
    type: "mecanum_controller/MecanumDriveController"  # adjust to actual package/type in your system
    wheel_names:
      front_left: wheel_front_left_joint
      front_right: wheel_front_right_joint
      rear_left: wheel_rear_left_joint
      rear_right: wheel_rear_right_joint
    wheel_radius: 0.05
    wheel_base_x: 0.22
    wheel_base_y: 0.18
    # other params according to mecanum controller documentation

# DiffDrive controller configuration
diff_drive_controller:
  ros__parameters:
    type: "diff_drive_controller/DiffDriveController" # adjust to actual type
    left_wheel: wheel_front_left_joint   # if using front left/right as left/right
    right_wheel: wheel_front_right_joint
    wheel_separation: 0.44
    wheel_radius: 0.05
```

```name=urdf/multi_drive_robot.urdf.xacro
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="multi_drive_robot">
  <xacro:property name="drive_type" value="${drive_type}"/>

  <link name="base_link"/>

  <link name="wheel_front_left_link"/>
  <link name="wheel_front_right_link"/>
  <link name="wheel_rear_left_link"/>
  <link name="wheel_rear_right_link"/>

  <joint name="wheel_front_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_front_left_link"/>
    <origin xyz="0.2 0.12 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="wheel_front_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_front_right_link"/>
    <origin xyz="0.2 -0.12 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="wheel_rear_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_rear_left_link"/>
    <origin xyz="-0.2 0.12 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="wheel_rear_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_rear_right_link"/>
    <origin xyz="-0.2 -0.12 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- ros2_control block: plugin loads the hardware and params -->
  <ros2_control name="MultiDrive" type="system">
    <hardware>
      <plugin>multi_drive_pico::MultiDriveHardware</plugin>
      <param name="serial_port">/dev/ttyACM0</param>
      <param name="baudrate">115200</param>
      <param name="drive_type">${drive_type}</param>
      <param name="wheel_radius">0.05</param>
      <param name="wheel_base_x">0.22</param>
      <param name="wheel_base_y">0.18</param>
    </hardware>
  </ros2_control>
</robot>
```

```name=launch/drive_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('multi_drive_pico_ros2_control')
    ros2_control_params = os.path.join(pkg_share, 'config', 'ros2_control_params.yaml')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'multi_drive_robot.urdf.xacro')

    drive_type_arg = DeclareLaunchArgument('drive_type', default_value='mecanum', description='mecanum or diff')
    drive_type = LaunchConfiguration('drive_type')

    # ros2_control_node (controller_manager)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[ros2_control_params, {'robot_description': open(urdf_file).read()}, {'drive_type': drive_type}]
    )

    # spawn the appropriate controller using spawner; we choose based on drive_type at runtime.
    # For simplicity launch both spawners but the one not present will fail harmlessly.
    spawner_mecanum = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawner_diff = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Optional helper node that converts /cmd_vel -> wheel velocity commands (if needed)
    cmdvel_node = Node(
        package='multi_drive_pico_ros2_control',
        executable='cmdvel_to_wheels',
        name='cmdvel_to_wheels',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'controllers.yaml')]
    )

    return LaunchDescription([
        drive_type_arg,
        controller_manager_node,
        spawner_mecanum,
        spawner_diff,
        cmdvel_node
    ])
```

```name=src/cmdvel_to_wheels.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class CmdVelToWheels : public rclcpp::Node {
public:
  CmdVelToWheels(): Node("cmdvel_to_wheels") {
    sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10, std::bind(&CmdVelToWheels::cmdvel_cb, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_velocity_controller/commands", 10);

    wheel_radius_ = this->declare_parameter("wheel_radius", 0.05);
    wheel_base_x_ = this->declare_parameter("wheel_base_x", 0.22);
    wheel_base_y_ = this->declare_parameter("wheel_base_y", 0.18);
    drive_type_ = this->declare_parameter("drive_type", std::string("mecanum"));
  }

private:
  void cmdvel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    double vx = msg->twist.linear.x;
    double vy = msg->twist.linear.y;
    double omega = msg->twist.angular.z;

    double L = wheel_base_x_;
    double W = wheel_base_y_;
    double r = wheel_radius_;

    std_msgs::msg::Float64MultiArray out;
    if (drive_type_ == "mecanum") {
      out.data.resize(4);
      out.data[0] = (1.0/r)*(vx - vy - (L+W)*omega);
      out.data[1] = (1.0/r)*(vx + vy + (L+W)*omega);
      out.data[2] = (1.0/r)*(vx + vy - (L+W)*omega);
      out.data[3] = (1.0/r)*(vx - vy + (L+W)*omega);
    } else { // diff: map to 2-wheel velocities and duplicate to 4-wheel if needed
      double v_left = (vx - omega*(L/2.0));
      double v_right = (vx + omega*(L/2.0));
      out.data.resize(4);
      // Front left, front right, rear left, rear right
      out.data[0] = v_left / r;
      out.data[1] = v_right / r;
      out.data[2] = v_left / r;
      out.data[3] = v_right / r;
    }
    pub_->publish(out);
  }

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;

  double wheel_radius_;
  double wheel_base_x_;
  double wheel_base_y_;
  std::string drive_type_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToWheels>());
  rclcpp::shutdown();
  return 0;
}
```

```name=docs/pico_protocol.md

# Raspberry Pi Pico Serial/USB Protocol (Beispiel)

Ziel: robustes, einfaches Protokoll für Kommando (Host -> Pico) und Telemetrie (Pico -> Host).

Kommunikationskanal
- USB CDC (empfohlen): Pico erscheint als /dev/ttyACMx (bei Linux).
- Alternativ UART über TTL-Pegel (pins) an USB-UART-Adapter.

Nachrichten (ASCII-framed, einfache Checksumme)
- Host -> Pico (Set Wheel Speeds)
  Format: S v0 v1 v2 v3\r\n
  Beispiel: S 0.12 -0.12 0.12 -0.12\r\n
  - vN in rad/s (float)
  - Anzahl Werte: abhängig von wheel_count (1-4). Wenn weniger Werte, rest = 0.
  - Option: füge CRC8 am Ende vor '\r\n' für höhere Robustheit.

- Pico -> Host (Encoder report / status)
  Format: R e0 e1 e2 e3 ts\r\n
  Beispiel: R 12345 23456 34567 45678 1616161616\r\n
  - eN sind Encoder tick counts (integers)
  - ts optional timestamp (ms)
  - Option: sende periodisch (z.B. 20-50 Hz) oder auf Anfrage.

Tipps
- Verwende eine einfache Start-Byte (S/R) und '\n' als Ende, damit Pakete eindeutig sind.
- Implementiere auf Pico: ring buffer + parsing state machine (start, read payload, verify checksum, commit).
- Rate-Limiting: Host sollte nicht schneller senden, als Pico verarbeiten kann. 20-50 Hz empfohlen.
- Watchdog: wenn Host-Kommandos für >timeout ausbleiben, Pico schaltet Motoren in sicheren Zustand.
- Handshake: beim Verbindungsaufbau kann Pico eine "HELLO" Nachricht senden, Host bestätigt, dann beginnt der Datenaustausch.

Pico-Firmware Hinweise
- nutze TinyUSB (CDC) oder pico-sdk + UART.
- encoder reading: Interrupts oder PIO für hohe Frequenz.
- Motorsteuerung: prefer closed-loop (on-Pico PID) and send wheel speed commands from host.
```

Was ich gemacht habe
- Ein modular aufgebautes Paket-Skelett erstellt, das:
  - Ein hardware SystemInterface (MultiDriveHardware) bereitstellt, das parametrisierbar ist für drive_type ("mecanum" oder "diff") und eine variable Anzahl joints akzeptiert.
  - Eine plugin-Beschreibung für ros2_control (multi_drive_hardware_plugins.xml).
  - Konfigurationen: ros2_control params und controllers.yaml mit Beispielen für mecanum_drive_controller und diff_drive_controller.
  - Ein XACRO-URDF mit Parametrisierbarem drive_type.
  - Einen Launch, der sowohl controller_manager als auch controller-spawner startet und ein optionales cmdvel_to_wheels Node.
  - Ein Protokoll-Dokument für Raspberry Pi Pico (USB-CDC/UART) mit Empfehlungen.

Was du als Nächstes tun solltest (konkrete Arbeitsschritte)
1. Pico-Firmware
   - Implementiere auf dem Raspberry Pi Pico das Protokoll aus docs/pico_protocol.md (TinyUSB CDC oder UART).
   - Implementiere Encoder-Reading + optional motor closed-loop on-Pico (empfohlen).

2. Host-Serial/USB Implementation
   - Ersetze Stubs in src/multi_drive_hardware.cpp durch echten Serial-Code (z. B. boost::asio oder libserial).
   - Implementiere send_wheel_commands() entsprechend dem Pico-Protokoll.
   - Implementiere read_encoder_packet_and_update_state() – parse Encoder-Werte, konvertiere in Position/Velocity.

3. Controller-Auswahl
   - Wenn du das vorhandene mecanum_drive_controller Paket verwendest, passe controllers.yaml an seine parameterbibliothek an.
   - Für DiffDrive nutze den diff_drive_controller (oder JointGroupVelocityController + cmdvel_to_wheels).

4. Tests
   - Starte mit Fake-Hardware: implementiere read() so, dass es deterministische simulierte Encoder liefert, damit du End-to-End testen kannst.
   - Dann Schließe den Pico an und teste Live-Kommunikation.

Wenn du möchtest, kann ich jetzt:
- die Serial-Implementierung in C++ mit boost::asio als konkretes Beispiel schreiben (Host-seitig), oder
- ein minimales Pico-Firmware-Beispiel (TinyUSB CDC) mit Parser-Sketch liefern,
- oder eine angepasste controllers.yaml/parameter library erstellen, wenn du mir sagst, welches konkrete mecanum controller Paket (genauer package/name) du einsetzen willst.

Welche Datei möchtest du als erstes umgesetzt sehen — Host-Serial (boost::asio) oder Pico TinyUSB-Firmware-Beispiel?