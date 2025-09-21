# MecaBridge Pico Firmware Skeleton

This directory contains the Raspberry Pi Pico firmware for **MecaBridge**. The
code is designed to build with the [Pico SDK](https://github.com/raspberrypi/pico-sdk)
and mirrors the frame format implemented on the ROS 2 side.

Key design goals for the first revision:

- Modular drivers for mecanum wheel motors, positional servos, continuous servos
  and ESCs.
- Deterministic frame parser that accepts command frames and emits state frames
  with CRC16 validation.
- Compile time feature flags to enable or disable subsystems (e.g. disable ESCs
  on early prototypes).
- Immediate safe-stop behaviour whenever the USB link is lost or a CRC error is
  detected.

The firmware is intentionally conservative: it does not contain logging,
calibration or advanced diagnostics yet. Those hooks can be added in later
phases without altering the public protocol.

## Building

Create a Pico SDK application that includes the files from `src/` and the
headers from `include/`. The firmware targets C++17 (as supported by the SDK).
The provided `CMakeLists.txt` snippet shows the minimum required structure.

```
cmake_minimum_required(VERSION 3.13)
project(mecabridge_pico C CXX)

pico_sdk_init()

add_executable(mecabridge
  src/main.cpp
  src/frame.cpp
  src/motor_module.cpp
  src/servo_module.cpp
  src/esc_module.cpp
)

pico_enable_stdio_usb(mecabridge 1)
pico_enable_stdio_uart(mecabridge 0)

pico_add_extra_outputs(mecabridge)

target_include_directories(mecabridge PRIVATE include)

target_compile_definitions(mecabridge PRIVATE
  MECABRIDGE_ENABLE_MOTORS
  MECABRIDGE_ENABLE_SERVOS
  MECABRIDGE_ENABLE_ESCS
)

```

Compile-time flags toggle modules:

- `MECABRIDGE_ENABLE_MOTORS` — enable the mecanum motor PWM driver.
- `MECABRIDGE_ENABLE_SERVOS` — enable both servo controllers.
- `MECABRIDGE_ENABLE_ESCS` — enable ESC support.

## Safety Model

The Pico continually emits heartbeat frames that include the last command
counter it has applied. If a heartbeat or state frame is not acknowledged by the
Pi within `HEARTBEAT_TIMEOUT_MS`, the firmware will cut motor power and reset
servo/ESC commands to neutral. CRC failures and framing errors cause the same
behaviour. This keeps the robot stationary whenever the bridge breaks.

## File Overview

- `include/mecabridge_pico/frame.hpp` — Frame structures, CRC helper and parser.
- `include/mecabridge_pico/modules/motor_module.hpp` — Motor PWM handling.
- `include/mecabridge_pico/modules/servo_module.hpp` — Servo abstraction for
  positional and continuous servos.
- `include/mecabridge_pico/modules/esc_module.hpp` — ESC output wrapper.
- `src/frame.cpp` — Frame packing/unpacking and CRC implementation.
- `src/motor_module.cpp`, `src/servo_module.cpp`, `src/esc_module.cpp` — Module
  implementations.
- `src/main.cpp` — Firmware entry point tying everything together, handling
  heartbeats and safe-stop.

The implementation focuses on clarity and separation of concerns so that future
phases can introduce PID loops, sensor fusion or calibration without reworking
the bridge.
