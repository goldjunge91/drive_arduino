FROM ros:humble

# Update package lists
RUN apt-get update

# Install basic development tools first
RUN apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    vim \
    nano

# Install core ROS 2 control packages
RUN apt-get install -y \
    ros-humble-hardware-interface \
    ros-humble-controller-manager \
    ros-humble-pluginlib \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers

# Install additional ROS 2 packages (try to install what's available)
RUN apt-get install -y \
    ros-humble-twist-mux \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    || echo "Some optional packages not available"

# Try to install Gazebo packages if available
RUN apt-get install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    || echo "Gazebo packages not available"

# Clean up
RUN rm -rf /var/lib/apt/lists/*

# Set up ROS 2 workspace structure
WORKDIR /ros2_ws
RUN mkdir -p src

# Copy project files to workspace
COPY launch/ src/robot/launch/
COPY worlds/ src/robot/worlds/
COPY config/ src/robot/config/
COPY description/ src/robot/description/
COPY serial/ src/serial/
COPY package.xml src/robot/
COPY CMakeLists.txt src/robot/
COPY scripts/ src/robot/scripts/

# Copy diffdrive_arduino to src
COPY diffdrive_arduino-main/ src/diffdrive_arduino/

# Set workspace as working directory
WORKDIR /ros2_ws

# Source ROS 2 setup in bashrc for convenience
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc

# Set default command
CMD ["bash"]