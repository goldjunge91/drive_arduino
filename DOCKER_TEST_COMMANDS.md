# Docker Test Commands for TB6612Comms Unit Tests

Since you're running in a Docker container, here are the commands to build and test the TB6612Comms unit tests:

## Method 1: Direct Docker Commands

```bash
# Start your Docker container
docker-compose run --rm robot bash

# Inside the container, run these commands:
colcon build --packages-select drive_arduino --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select drive_arduino
colcon test-result --all --verbose
```

## Method 2: Using the Docker Test Script

From your host machine (outside Docker):
```bash
./test_tb6612_comms_docker.sh
```

## Method 3: Manual Commands in Running Container

If you already have a running container:
```bash
# Enter the container
docker exec -it <container_name> bash

# Run the tests
cd /ros2_ws
colcon build --packages-select drive_arduino --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select drive_arduino
colcon test-result --all --verbose
```

## Expected Test Results

The tests are designed to run without hardware and should show:
- Connection tests that properly handle missing hardware
- Motor command tests that validate input parameters
- Encoder reading tests that handle disconnected state
- Error handling tests that verify proper exception handling

## Troubleshooting

If you get ROS environment errors:
```bash
# Try sourcing ROS setup manually
source /opt/ros/humble/setup.bash
# or
source /ros_entrypoint.sh
```

If colcon is not found:
```bash
# Install colcon in the container
apt-get update
apt-get install -y python3-colcon-common-extensions
```

## Test Files Location

The test files are located at:
- `/ros2_ws/src/drive_arduino/test/test_tb6612_comms_simple.cpp`
- `/ros2_ws/src/drive_arduino/test/test_tb6612_comms_comprehensive.cpp`
- `/ros2_ws/src/drive_arduino/test/README.md` (detailed documentation)

## Build Output Location

After building, test executables will be in:
- `/ros2_ws/build/drive_arduino/`

Test results will be in:
- `/ros2_ws/build/drive_arduino/Testing/`