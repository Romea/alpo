ARG FROM_IMAGE=ros:humble
FROM ${FROM_IMAGE}

RUN apt-get update \
    && apt-get install -yq --no-install-recommends \
        curl \
        ca-certificates \
        openssh-client \
        # python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# add ros one repositories
RUN curl -sSL https://ros.packages.techfak.net/gpg.key -o /etc/apt/keyrings/ros-one-keyring.gpg \
    && . /etc/os-release \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $VERSION_CODENAME main" \
        | tee /etc/apt/sources.list.d/ros1.list

# install ros one
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-rmw-zenoh-cpp \
        ros-one-ros-core \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -pm 700 ~/.ssh \
    && ssh-keyscan github.com >>~/.ssh/known_hosts \
    && ssh-keyscan forge.inrae.fr >>~/.ssh/known_hosts

# import ros1 packages (only keep msgs)
RUN --mount=type=bind,source=docker/ros1.repos,target=/ros1_ws/ros1.repos \
    --mount=type=ssh \
    cd /ros1_ws \
    && vcs import -w6 <ros1.repos \
    && find src -name package.xml -exec dirname '{}' \; | grep -v 'msgs$' | xargs rm -rf

# import ros2 packages (only keep msgs)
RUN --mount=type=bind,source=docker/ros2.repos,target=/ros2_ws/ros2.repos \
    --mount=type=ssh \
    cd /ros2_ws \
    && vcs import -w6 <ros2.repos \
    && find src -name package.xml -exec dirname '{}' \; | grep -v 'msgs$' | xargs rm -rf

# install dependencies
RUN --mount=type=bind,source=package.xml,target=/tmp/bridge/package.xml \
    apt-get update \
    && rosdep update \
    && rosdep install -iyr --from-paths /ros2_ws /tmp/bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros1_ws
RUN unset ROS_DISTRO \
    && . /opt/ros/one/setup.sh \
    && catkin_make install \
    && rm -rf build devel src

WORKDIR /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/local_setup.sh \
    && colcon --log-base /dev/null build \
    && rm -rf build src

# override entrypoint
COPY docker/entrypoint.sh /ros_entrypoint.sh

# compile this package (the bridge)
WORKDIR /bridge_ws
RUN --mount=type=bind,source=.,target=/bridge_ws/src/bridge \
    ROS2_DISTRO=${ROS_DISTRO} \
    && . /ros1_ws/install/setup.sh \
    && . /opt/ros/${ROS2_DISTRO}/local_setup.sh \
    && . /ros2_ws/install/local_setup.sh \
    && ROS_VERSION=1 colcon --log-base /dev/null build --cmake-args -DCMAKE_BUILD_TYPE=Debug \
    && rm -rf build

# debug
# RUN apt-get update && apt-get install -y --no-install-recommends \
#         gdbserver


LABEL org.opencontainers.image.source=https://github.com/Romea/alpo
LABEL org.opencontainers.image.description="bridge between ROS2 and ROS1 for low-level commands"
LABEL org.opencontainers.image.licenses=Apache-2.0
