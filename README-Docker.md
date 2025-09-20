# Docker Development Environment

This Docker setup provides a complete ROS 2 Humble development environment preloaded with the `drive_arduino` workspace. The image copies the whole repository into `/ros2_ws/src/drive_arduino`, so CI builds and manual `docker build` invocations always see a self-contained tree. During day-to-day development the compose file bind-mounts the host checkout over that path so edits are reflected instantly.

## Quick Start

### Option 1: Using the helper script
```bash
# Build and start the development container, dropping you into a shell
./scripts/dev.sh
```

### Option 2: Manual Docker Compose
```bash
# Build the image and start the container
docker-compose up --build -d

# Enter the running container
docker-compose exec ros2-dev bash

# Stop the container when finished
docker-compose down
```

## Inside the Container

```bash
# Always source ROS 2
source /opt/ros/humble/setup.bash

# Build the entire workspace
colcon build --symlink-install

# Or target individual packages
colcon build --packages-select drive_arduino mecabridge_hardware --symlink-install

# Source the overlay
source install/setup.bash

# Run tests
colcon test --packages-select mecabridge_hardware
```

Build, install, and log directories live in Docker named volumes to avoid polluting the host.

## What's Included

- ROS 2 Humble base image
- ros2_control stack (`hardware_interface`, `controller_manager`, controllers, pluginlib)
- Serial driver dependency for the hardware interface
- Colcon + common development utilities (git, cmake, build-essential, editors)
- Optional desktop tools (twist_mux, joint_state_publisher_gui, xacro) and Gazebo when enabled

## Optional Packages & Build Arguments

To keep the default image lightweight, heavy desktop/Gazebo packages are disabled unless requested. Enable them via build arguments:

```bash
# Enable desktop helpers (twist_mux, joint_state_publisher_gui, xacro)
docker build --build-arg INSTALL_DESKTOP_TOOLS=1 -t drive_arduino:desktop .

# Enable Gazebo integration packages
docker build --build-arg INSTALL_GAZEBO=1 -t drive_arduino:gazebo .

# Enable both when using docker-compose
docker-compose build --build-arg INSTALL_DESKTOP_TOOLS=1 --build-arg INSTALL_GAZEBO=1
```

These installs now share a BuildKit cache, so the first build still pulls the packages but subsequent builds reuse the cached `.deb` downloads. If BuildKit is disabled globally, enable it temporarily:

```bash
export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1
```

When an optional package is unavailable on your ROS mirror, the build logs a warning and continues.

## File & Volume Layout

- `Dockerfile` ? installs dependencies and copies the repository into `/ros2_ws/src/drive_arduino`
- `Dockerfile.basic` ? slimmer variant with only the essentials
- `docker-compose.yml` ? defines the `ros2-dev` service and bind-mounts the host repo
- `.dockerignore` ? keeps build artifacts, VCS data, and generated files out of the build context
- Named volumes `ros2_build_cache`, `ros2_install_cache`, `ros2_log_cache` store build outputs

To reset the cached build state:
```bash
docker-compose down -v
```

## Troubleshooting

### File Ownership on the Host
If files created in the container end up owned by root, fix them on the host:
```bash
sudo chown -R $USER:$USER build install log
```

### GUI Applications (Linux)
Uncomment the X11 lines in `docker-compose.yml` and run `xhost +local:` before starting the container.

### Clean Rebuild
```bash
docker-compose down -v
docker system prune -f

docker-compose up --build
```
