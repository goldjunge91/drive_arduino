cmake_minimum_required(VERSION 3.16)
project(mecabridge_hardware)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Ensure the package include/ is available to tests and consumers using
# #include "mecabridge_hardware/..." style headers.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Protocol library
add_library(mecabridge_protocol SHARED
  src/mecabridge_utils/protocol/frame.cpp
  src/mecabridge_utils/protocol/crc16.cpp
  src/mecabridge_utils/safety/watchdog.cpp
  src/mecabridge_utils/config/config.cpp
)

set_target_properties(mecabridge_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(mecabridge_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME} SHARED
  src/mecabridge_hardware/mecabridge_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  mecabridge_protocol
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface mecabridge_hardware.xml)

# Install
install(TARGETS mecabridge_protocol ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # Some tests use GoogleMock matchers and mocks
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  
  # Linting
  ament_lint_auto_find_test_dependencies()
  
  # Protocol tests
  ament_add_gtest(test_protocol_empty
    test/mecabridge/protocol/test_protocol_empty.cpp
  )
  target_link_libraries(test_protocol_empty mecabridge_protocol)
  target_include_directories(test_protocol_empty PRIVATE src ${CMAKE_CURRENT_SOURCE_DIR}/include)
  
  ament_add_gtest(test_crc16
    test/mecabridge/protocol/test_crc16.cpp
  )
  target_link_libraries(test_crc16 mecabridge_protocol)
  target_include_directories(test_crc16 PRIVATE src ${CMAKE_CURRENT_SOURCE_DIR}/include)
  
  ament_add_gtest(test_frame_roundtrip
    test/mecabridge/protocol/test_frame_roundtrip.cpp
  )
  target_link_libraries(test_frame_roundtrip mecabridge_protocol)
  target_include_directories(test_frame_roundtrip PRIVATE src ${CMAKE_CURRENT_SOURCE_DIR}/include)

  ament_add_gtest(test_protocol_handshake
    test/mecabridge/protocol/test_protocol_handshake.cpp
  )
  target_link_libraries(test_protocol_handshake mecabridge_protocol)
  target_include_directories(test_protocol_handshake PRIVATE src ${CMAKE_CURRENT_SOURCE_DIR}/include)

  ament_add_gtest(test_frame_errors
    test/mecabridge/protocol/test_frame_errors.cpp
  )
  target_link_libraries(test_frame_errors mecabridge_protocol)
  target_include_directories(test_frame_errors PRIVATE src ${CMAKE_CURRENT_SOURCE_DIR}/include)

  ament_add_gtest(test_watchdog
    test/mecabridge/safety/test_watchdog.cpp
  )
  target_link_libraries(test_watchdog mecabridge_protocol)
  target_include_directories(test_watchdog PRIVATE src ${CMAKE_CURRENT_SOURCE_DIR}/include)

  # Config tests
  ament_add_gtest(test_config
    test/mecabridge/config/test_config.cpp
  )
  target_link_libraries(test_config mecabridge_protocol)
  target_include_directories(test_config PRIVATE src ${CMAKE_CURRENT_SOURCE_DIR}/include)

  # Hardware plugin load test (uses gmock matchers in some environments)
  ament_add_gmock(test_mecabridge_hardware_interface
    test/mecabridge/hardware/test_mecabridge_hardware_interface.cpp
  )
  target_link_libraries(test_mecabridge_hardware_interface ${PROJECT_NAME})
  target_include_directories(test_mecabridge_hardware_interface PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src)
  ament_target_dependencies(test_mecabridge_hardware_interface pluginlib hardware_interface)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
