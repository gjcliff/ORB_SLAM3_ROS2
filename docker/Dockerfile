FROM ros:jazzy-ros-base

WORKDIR /app

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
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-pcl-conversions \
    libpcl-dev && \
    rm -rf /var/lib/apt/lists/*

# create non-root user without sudo
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME

COPY ./* /ros2_ws/src/ORB_SLAM3_ROS2
COPY --chown=$USERNAME:$USERNAME ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# COPY ./include /app/include
# COPY ./src /app/src
# COPY ./databases /app/databases
# COPY ./models /app/models
# COPY ./CMakeLists.txt /app/CMakeLists.txt
# COPY ./entrypoint.sh /app/entrypoint.sh

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash \
    colcon build \
    source install/setup.bash"

ENTRYPOINT ["/entrypoint.sh"]
CMD [ "/bin/bash" ]
