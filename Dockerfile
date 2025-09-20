FROM althack/ros2:humble-dev

ARG INSTALL_DESKTOP_TOOLS=1
ARG INSTALL_GAZEBO=1
ARG INSTALL_ROBOT=1

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

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
   ros-humble-rclcpp-lifecycle \
   ros-humble-ros2-control \
   ros-humble-ros2-controllers \
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

ENV ROS_WORKSPACE=/ros2_ws
WORKDIR ${ROS_WORKSPACE}

# Workspace tree and sources (uses .dockerignore to keep cache small)
RUN mkdir -p ${ROS_WORKSPACE}/src \
   && echo "Skipping external dependencies for testing"


# `mecabridge_hardware` end up at /ros2_ws/src/mecabridge_hardware
COPY ./src ${ROS_WORKSPACE}/src
# Copy only the robot test package (kept under repo/robot) into the workspace src
COPY ./robot ${ROS_WORKSPACE}/src/robot

# Convenience: source ROS 2 and workspace overlays on shell start
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
   && echo "if [ -f ${ROS_WORKSPACE}/install/setup.bash ]; then source ${ROS_WORKSPACE}/install/setup.bash; fi" >> /root/.bashrc

CMD ["bash"]
