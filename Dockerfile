FROM ros:jazzy-ros-base

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=user
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ccache \
    xterm \
    gdb \
    ninja-build \
    git \
    g++ \
    wget \
    unzip \
    pybind11-dev \
    python3-pip \
    libyaml-cpp-dev \
    ros-${ROS_DISTRO}-nav2-map-server \
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
RUN userdel -r ubuntu 2>/dev/null || true \
    && groupdel ubuntu 2>/dev/null || true \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

USER $USERNAME

COPY --chown=$USERNAME:$USERNAME ./ORB_SLAM3/ /ros2_ws/src/ORB_SLAM3_ROS2/ORB_SLAM3/
WORKDIR /ros2_ws/src/ORB_SLAM3_ROS2/ORB_SLAM3
RUN bash -e -c "source /opt/ros/jazzy/setup.bash && ./build.sh"

WORKDIR /ros2_ws
COPY --chown=$USERNAME:$USERNAME ./CMakeLists.txt ./package.xml /ros2_ws/src/ORB_SLAM3_ROS2/
COPY --chown=$USERNAME:$USERNAME ./src/ /ros2_ws/src/ORB_SLAM3_ROS2/src/
COPY --chown=$USERNAME:$USERNAME ./launch/ /ros2_ws/src/ORB_SLAM3_ROS2/launch/
COPY --chown=$USERNAME:$USERNAME ./config/ /ros2_ws/src/ORB_SLAM3_ROS2/config/
COPY --chown=$USERNAME:$USERNAME ./clouds/ /ros2_ws/src/ORB_SLAM3_ROS2/clouds/

RUN --mount=type=cache,target=/home/${USERNAME}/.ccache,uid=${USER_UID},gid=${USER_GID} \
    . /opt/ros/jazzy/setup.sh \
    && colcon build --cmake-args -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

COPY --chmod=755 ./entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
