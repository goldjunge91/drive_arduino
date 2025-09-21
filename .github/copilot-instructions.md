# GitHub Copilot Instructions for MecaBridge

## Project Overview

This is a **ROS 2 hardware interface** (`ros2_control` SystemInterface) for robotics platforms with multiple drive configurations (differential, mecanum, four-wheel). The system bridges high-level ROS 2 control with low-level Raspberry Pi Pico firmware via a deterministic serial protocol.

**Core Philosophy**: Safety-first design with watchdog-enforced failsafes, deterministic communication, and modular drive type support.

## Architecture Patterns

### Component Hierarchy
```
MecaBridgeHardwareInterface (SystemInterface)
├── MecaBridgeSerialProtocol (Serial Protocol Layer)
├── MecaBridgeDriveConfig (Parameter Management)  
└── Wheel[] (Joint State Management)
```

**Namespace Convention**: All classes use `mecabridge_hardware` namespace. Follow existing pattern when adding new components.

### Drive Type Abstraction
The system supports multiple drive configurations through runtime switching:
- **Differential**: 2 wheels (`left_wheel`, `right_wheel`)
- **Mecanum**: 4 wheels (`front_left_wheel`, `front_right_wheel`, `rear_left_wheel`, `rear_right_wheel`)
- **Four Wheel**: 4 independent wheels

**Key Pattern**: Joint names and wheel count are determined by `drive_type` parameter in YAML config. When adding drive types, update both `MecaBridgeDriveConfig::setupJoints()` and corresponding controller YAML templates.

### Safety Architecture
- **Watchdog**: ≤150ms timeout, enforced in firmware
- **CRC-16/CCITT-FALSE**: Message integrity validation
- **Safe State**: Zero velocities on communication loss
- **Error Hierarchy**: WATCHDOG_TIMEOUT > CRC_FAIL > MALFORMED_FRAME

## Development Workflows

### Docker-Centric Development
```bash
# Primary build command
docker-compose run --rm ros2-dev bash -c "colcon build --packages-select drive_arduino --cmake-args -DBUILD_TESTING=ON"

# Test execution
./test_mecabridge_comms_docker.sh  # Runs full test suite
```

**Testing Strategy**: 
- Unit tests use `MockSerial` injection (#ifdef TESTING_MODE)
- Integration tests load actual plugins via `pluginlib::ClassLoader`
- Hardware-in-the-loop via `/test/test_mecabridge_hardware.launch.py`

### Build Patterns
- Use `colcon build --packages-select drive_arduino` for focused builds
- Always include `--cmake-args -DBUILD_TESTING=ON` for test coverage
- Test results: `colcon test-result --all --verbose`

## Configuration Conventions

## Key Files to Reference

- **Hardware Interface**: `src/mecabridge_hardware/include/mecabridge_hardware/mecabridge_hardware_interface.h`
- **Serial Protocol**: `src/mecabridge_hardware/include/mecabridge_hardware/mecabridge_comms.h`
- **Configuration**: `src/mecabridge_hardware/config/mecabridge_hardware_params.yaml`
- **Protocol Spec**: `specs/001-mecabridge-plan/contracts/frame_protocol.md`
- **Safety Spec**: `specs/001-mecabridge-plan/contracts/watchdog.md`
- **Test Examples**: `test/mecabridge/test_mecabridge_hardware_interface.cpp`

## Common Pitfalls

1. **Joint Name Mismatches**: Ensure hardware params, controller config, and URDF joint names align exactly
2. **Serial Port Permissions**: Use `sudo usermod -a -G dialout $USER` for device access
3. **CRC Validation**: Always validate before processing - firmware safety depends on it
4. **Drive Type Consistency**: Wheel count must match between hardware config and controller expectations
5. **Testing Mode**: Use `#ifdef TESTING_MODE` for mock injection, not runtime flags
