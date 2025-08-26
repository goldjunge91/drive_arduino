cmake_minimum_required(VERSION 3.10)
project(drive_arduino)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial REQUIRED)






# TB6612 Hardware Interface - Main library

add_library(tb6612_hardware SHARED 
  src/tb6612_hardware_interface.cpp 
  src/tb6612_comms.cpp 
  src/wheel.cpp
)

target_include_directories(
  tb6612_hardware
  PRIVATE
  include
)

ament_target_dependencies(
  tb6612_hardware
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
  serial
)

pluginlib_export_plugin_description_file(hardware_interface tb6612_hardware.xml)

install(
  TARGETS tb6612_hardware
  DESTINATION lib
)




install(
  DIRECTORY controllers launch/
  DESTINATION share/${PROJECT_NAME}
)


ament_export_libraries(
  tb6612_hardware
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
  serial
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Configure linting to skip test directory
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    test/
  )
  
  # Lint tests
  ament_lint_auto_find_test_dependencies()
  
  # Unit tests for TB6612Comms - Simple tests
  ament_add_gtest(test_tb6612_comms_simple
    test/test_tb6612_comms_simple.cpp
  )
  
  target_include_directories(test_tb6612_comms_simple PRIVATE
    include
  )
  
  ament_target_dependencies(test_tb6612_comms_simple
    rclcpp
    serial
  )
  
  target_link_libraries(test_tb6612_comms_simple
    tb6612_hardware
  )
  
  # Unit tests for TB6612Comms - Comprehensive tests
  ament_add_gtest(test_tb6612_comms_comprehensive
    test/test_tb6612_comms_comprehensive.cpp
  )
  
  target_include_directories(test_tb6612_comms_comprehensive PRIVATE
    include
  )
  
  ament_target_dependencies(test_tb6612_comms_comprehensive
    rclcpp
    serial
  )
  
  target_link_libraries(test_tb6612_comms_comprehensive
    tb6612_hardware
  )
  
  # Unit tests for TB6612HardwareInterface
  ament_add_gtest(test_tb6612_hardware_interface
    test/test_tb6612_hardware_interface.cpp
  )
  
  target_include_directories(test_tb6612_hardware_interface PRIVATE
    include
  )
  
  ament_target_dependencies(test_tb6612_hardware_interface
    hardware_interface
    rclcpp
    serial
  )
  
  target_link_libraries(test_tb6612_hardware_interface
    tb6612_hardware
  )
  
  # Integration tests for TB6612 Hardware Interface
  ament_add_gtest(test_tb6612_integration
    test/test_tb6612_integration.cpp
  )
  
  target_include_directories(test_tb6612_integration PRIVATE
    include
  )
  
  ament_target_dependencies(test_tb6612_integration
    hardware_interface
    controller_manager
    rclcpp
    pluginlib
    serial
  )
  
  target_link_libraries(test_tb6612_integration
    tb6612_hardware
  )

endif()

ament_package()


