# Docker Development Environment

This setup provides a complete ROS 2 Humble development environment using Docker Compose.

## Quick Start

### Option 1: Using the helper script
```bash
# Start development environment (builds container and enters shell)
./scripts/dev.sh
```

### Option 2: Manual Docker Compose
```bash
# Build and start the container
docker-compose up --build -d

# Enter the development container
docker-compose exec ros2-dev bash

# Stop the container when done
docker-compose down
```

## Inside the Container

Once inside the container, you can build and work with your ROS 2 packages:

```bash
# Build the workspace (with helper script)
./scripts/build.sh

# Or build manually
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Clean build
./scripts/build.sh --clean

# Source the workspace
source install/setup.bash

# List available packages
ros2 pkg list

# Test specific package
colcon build --packages-select tb6612_hardware --symlink-install
```

## What's Included

The Docker environment includes:
- **ROS 2 Humble** base installation
- **ros2_control** framework (hardware_interface, controller_manager, etc.)
- **Gazebo** simulation packages
- **Development tools** (colcon, git, vim, nano)
- **All project dependencies** pre-installed

## File Structure

- `Dockerfile` - Container definition with all ROS 2 dependencies
- `docker-compose.yml` - Service configuration with volume mounts
- `scripts/build.sh` - Helper script for building ROS 2 workspace
- `scripts/dev.sh` - Quick start script for development
- `.dockerignore` - Excludes build artifacts from Docker context

## Persistent Data

Build artifacts are stored in Docker volumes to speed up rebuilds:
- `ros2_build_cache` - CMake build files
- `ros2_install_cache` - Installed packages
- `ros2_log_cache` - Build logs

To start fresh, remove the volumes:
```bash
docker-compose down -v
```

## Troubleshooting

### Permission Issues
If you encounter permission issues with files created in the container:
```bash
# Fix ownership (run on host)
sudo chown -R $USER:$USER build/ install/ log/
```

### GUI Applications (Linux only)
To run GUI applications like RViz or Gazebo, uncomment the X11 forwarding lines in `docker-compose.yml`.

### Clean Rebuild
```bash
# Remove all containers and volumes
docker-compose down -v
docker system prune -f

# Rebuild from scratch
docker-compose up --build
```