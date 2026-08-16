FROM ros:jazzy-ros-base

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=franka

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    g++ \
    wget \
    unzip \
    pybind11-dev \
    python3-pip \
    libyaml-cpp-dev \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pangolin \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ament-cmake-python \
    ros-${ROS_DISTRO}-librealsense2 \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    libpcl-dev && \
    rm -rf /var/lib/apt/lists/*

# create non-root user without sudo
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME

COPY --chown=$USERNAME:$USERNAME . /ros2_ws/src/ORB_SLAM3_ROS2/
COPY --chown=$USERNAME:$USERNAME ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# -e: stop at first error
WORKDIR /ros2_ws/src/ORB_SLAM3_ROS2/ORB_SLAM3

RUN bash -e -c "source /opt/ros/jazzy/setup.bash && ./build.sh"

WORKDIR /ros2_ws

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build && \
    source install/setup.bash"

ENTRYPOINT ["/entrypoint.sh"]
CMD [ "/bin/bash" ]
