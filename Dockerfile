FROM ros:humble

ARG INSTALL_DESKTOP_TOOLS=0
ARG INSTALL_GAZEBO=0

ENV DEBIAN_FRONTEND=noninteractive

# Core tooling and ROS dependencies (single layer for better caching)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    vim \
    nano \
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-pluginlib \
    ros-humble-rclcpp \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-serial-driver \
 && if [ "$INSTALL_DESKTOP_TOOLS" = "1" ]; then \
    apt-get install -y --no-install-recommends \
      ros-humble-twist-mux \
      ros-humble-joint-state-publisher-gui \
      ros-humble-xacro \
    || echo "Optional desktop tools not available"; \
 fi \
 && if [ "$INSTALL_GAZEBO" = "1" ]; then \
    apt-get install -y --no-install-recommends \
      ros-humble-gazebo-ros \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-ros2-control \
    || echo "Gazebo packages not available"; \
 fi \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# Colcon workspace layout
ENV ROS_WORKSPACE=/ros2_ws
WORKDIR ${ROS_WORKSPACE}
RUN mkdir -p src

# Copy the whole package (relies on .dockerignore for exclusions)
COPY . ${ROS_WORKSPACE}/src/drive_arduino

# Convenience: source ROS 2 and workspace overlays on shell start
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "if [ -f ${ROS_WORKSPACE}/install/setup.bash ]; then source ${ROS_WORKSPACE}/install/setup.bash; fi" >> /root/.bashrc

CMD ["bash"]
