FROM amd64/ros:galactic-ros-base-focal

ENV ROS1_DISTRO noetic
ENV ROS2_DISTRO galactic

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        openssh-client \
        python3-catkin-tools \
        curl && \
    rm -rf /var/lib/apt/lists/*

RUN echo 'deb http://packages.ros.org/ros/ubuntu focal main' \
        >/etc/apt/sources.list.d/ros1-latest.list && \
    curl -s 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc' | apt-key add -

RUN mkdir -pm 700 ~/.ssh && \
    ssh-keyscan github.com >>~/.ssh/known_hosts && \
    ssh-keyscan gitlab-ssh.irstea.fr >>~/.ssh/known_hosts && \
    mkdir -p /ros1_ws /ros2_ws

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-noetic-ros-comm \
        ros-noetic-actionlib-msgs \
        ros-noetic-ackermann-msgs \
        ros-noetic-sensor-msgs \
        ros-noetic-nav-msgs \
        ros-noetic-tf \
        ros-galactic-actionlib-msgs \
        ros-galactic-ackermann-msgs \
        ros-galactic-sensor-msgs \
        ros-galactic-nav-msgs \
        ros-galactic-tf2-msgs \
        && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ros1_ws
COPY ./ros1.repos ros1.repos
COPY ./ros_entrypoint_ros1.sh /ros_entrypoint.sh
RUN --mount=type=ssh \
    mkdir -p src && \
    vcs import -w6 <ros1.repos && \
    find src/romea_path -mindepth 1 -maxdepth 1 ! -name '*msgs' | xargs rm -rf
RUN catkin init && \
    catkin config --extend /opt/ros/noetic && \
    catkin build

WORKDIR /ros2_ws
COPY ./ros2.repos ros2.repos
COPY ./ros_entrypoint_ros1_ros2.sh /ros_entrypoint.sh
RUN --mount=type=ssh \
    vcs import -w6 <ros2.repos
RUN ["/bin/bash", \
    "-c", \
    "source /ros_entrypoint.sh && ROS_VERSION=1 colcon build --symlink-install --packages-up-to alpo_bridge"]
