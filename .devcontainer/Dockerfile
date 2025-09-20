FROM ros:humble-ros-base

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

# Install some dependencies packages
RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    software-properties-common \
    python3-pip \
    xauth \
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
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ENV ROS_WORKSPACE=/ros2_ws
WORKDIR ${ROS_WORKSPACE}


# Create and switch to user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME

RUN mkdir -p ${ROS_WORKSPACE}/src \
    && (test -d ${ROS_WORKSPACE}/src/serial || git clone --depth 1 --branch newans_ros2 https://github.com/joshnewans/serial.git ${ROS_WORKSPACE}/src/serial) \
    && if [ "$INSTALL_ROBOT" = "1" ]; then \
    (test -d ${ROS_WORKSPACE}/src/robot || git clone --depth 1 --branch humble https://github.com/goldjunge91/robot.git ${ROS_WORKSPACE}/src/robot); \
    fi
# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws


# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.bash' >> /home/$USERNAME/.bashrc

# Setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
