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

# Optional legacy mecabridge build (disabled by default)
option(BUILD_mecabridge "Build legacy mecabridge hardware and tests" OFF)
set(mecabridge_XML ${CMAKE_CURRENT_SOURCE_DIR}/mecabridge_hardware.xml)
set(mecabridge_SRC_OK FALSE)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/mecabridge_hardware_interface.cpp)
  set(mecabridge_SRC_OK TRUE)
endif()
if(BUILD_mecabridge AND NOT mecabridge_SRC_OK)
  message(WARNING "BUILD_mecabridge=ON but mecabridge sources not found; skipping legacy build.")
endif()
if(BUILD_mecabridge AND mecabridge_SRC_OK)
  find_package(serial REQUIRED)
endif()






if(BUILD_mecabridge AND mecabridge_SRC_OK)
  # mecabridge Hardware Interface - Main library
  add_library(mecabridge_hardware SHARED 
    src/mecabridge_hardware_interface.cpp 
    src/mecabridge_comms.cpp 
    src/wheel.cpp
  )

  target_include_directories(
    mecabridge_hardware
    PRIVATE
    include
  )

  ament_target_dependencies(
    mecabridge_hardware
    hardware_interface
    controller_manager
    rclcpp
    pluginlib
    serial
  )

  if(EXISTS ${mecabridge_XML})
    pluginlib_export_plugin_description_file(hardware_interface mecabridge_hardware.xml)
  else()
    message(WARNING "mecabridge plugin XML not found at ${mecabridge_XML}; skipping plugin export.")
  endif()

  install(
    TARGETS mecabridge_hardware
    DESTINATION lib
  )
endif()




if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/controllers)
  install(
    DIRECTORY controllers
    DESTINATION share/${PROJECT_NAME}
  )
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}
  )
endif()


if(BUILD_mecabridge AND mecabridge_SRC_OK)
  ament_export_libraries(
    mecabridge_hardware
  )
  ament_export_dependencies(
    hardware_interface
    controller_manager
    rclcpp
    pluginlib
    serial
  )
else()
  ament_export_dependencies(
    hardware_interface
    controller_manager
    rclcpp
    pluginlib
  )
endif()

# Testing
if(BUILD_TESTING AND BUILD_mecabridge AND mecabridge_SRC_OK)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Configure linting to skip test directory
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    test/
  )
  
  # Lint tests
  ament_lint_auto_find_test_dependencies()
  
  # Unit tests for MecaBridgeSerialProtocol - Simple tests
  ament_add_gtest(test_mecabridge_comms_simple
    test/test_mecabridge_comms_simple.cpp
  )
  
  target_include_directories(test_mecabridge_comms_simple PRIVATE
    include
  )
  
  ament_target_dependencies(test_mecabridge_comms_simple
    rclcpp
    serial
  )
  
  target_link_libraries(test_mecabridge_comms_simple
    mecabridge_hardware
  )
  
  # Unit tests for MecaBridgeSerialProtocol - Comprehensive tests
  ament_add_gtest(test_mecabridge_comms_comprehensive
    test/test_mecabridge_comms_comprehensive.cpp
  )
  
  target_include_directories(test_mecabridge_comms_comprehensive PRIVATE
    include
  )
  
  ament_target_dependencies(test_mecabridge_comms_comprehensive
    rclcpp
    serial
  )
  
  target_link_libraries(test_mecabridge_comms_comprehensive
    mecabridge_hardware
  )
  
  # Unit tests for MecaBridgeHardwareInterface
  ament_add_gtest(test_mecabridge_hardware_interface
    test/test_mecabridge_hardware_interface.cpp
  )
  
  target_include_directories(test_mecabridge_hardware_interface PRIVATE
    include
  )
  
  ament_target_dependencies(test_mecabridge_hardware_interface
    hardware_interface
    rclcpp
    serial
  )
  
  target_link_libraries(test_mecabridge_hardware_interface
    mecabridge_hardware
  )
  
  # Integration tests for mecabridge Hardware Interface
  ament_add_gtest(test_mecabridge_integration
    test/test_mecabridge_integration.cpp
  )
  
  target_include_directories(test_mecabridge_integration PRIVATE
    include
  )
  
  ament_target_dependencies(test_mecabridge_integration
    hardware_interface
    controller_manager
    rclcpp
    pluginlib
    serial
  )
  
  target_link_libraries(test_mecabridge_integration
    mecabridge_hardware
  )

endif()

ament_package()


