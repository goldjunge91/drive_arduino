cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(Testing C CXX)

# Standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

add_executable(Testing
  src/main.cpp
  src/motors.cpp
  src/esc_servo.cpp
  src/term.cpp
  src/enc.cpp
)

# enable usb output
pico_enable_stdio_usb(Testing 1)

# Headers live in src/ (e.g., config.h)
target_include_directories(Testing PRIVATE src)

# Link against required pico-sdk libs
target_link_libraries(Testing
  pico_stdlib
  hardware_pwm
  hardware_gpio
  hardware_sync
)
# erlauben: reboot via 1200-baud und via Vendor-Interface (picotool)
target_compile_definitions(Testing PRIVATE
  PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1
  PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=1
)


# USB stdio on, UART off
pico_enable_stdio_usb(Testing 1)
pico_enable_stdio_uart(Testing 0)

# Create UF2/ELF/bin
pico_add_extra_outputs(Testing)

# --- convenience targets ---
add_custom_target(flash
  COMMAND picotool load -f ${CMAKE_CURRENT_BINARY_DIR}/Testing.uf2
  DEPENDS Testing
  COMMENT "Flashing Testing.uf2 via picotool"
)
