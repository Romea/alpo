FROM amd64/ros:galactic-ros-base-focal


# setup keys
RUN apt-get update
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-galactic-ackermann-msgs 

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV ROS1_DISTRO noetic
ENV ROS2_DISTRO galactic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-comm \
    ros-noetic-ackermann-msgs \
    ros-galactic-ackermann-msgs \
    ros-noetic-sensor-msgs \
    ros-galactic-sensor-msgs \
    ros-noetic-nav-msgs \
    ros-galactic-nav-msgs \
    python3-catkin-tools \
  && rm -rf /var/lib/apt/lists/*

#RUN mkdir -p /ros1_ws/src
#COPY ./ros1.repos /ros1_ws/src
#RUN ["/bin/bash", "-c", "cd /ros1_ws/src && vcs import < ros1.repos "]
#WORKDIR /ros1_ws
#COPY ./ros_entrypoint_ros1.sh /ros_entrypoint.sh
#RUN ["/bin/bash", "-c", "source /ros_entrypoint.sh && catkin init && catkin config --extend /opt/ros/noetic && catkin build"]

#RUN mkdir -p /ros2_ws/src
#COPY ./ros2.repos /ros2_ws/src
#RUN ["/bin/bash", "-c", "cd /ros2_ws/src && vcs import < ros2.repos "]
#WORKDIR /ros2_ws
#COPY ./ros_entrypoint_ros2.sh /ros_entrypoint.sh
#RUN ["/bin/bash", "-c", "source /ros_entrypoint.sh && colcon build --symlink-install"]

RUN mkdir -p /bridge_ws/src
WORKDIR /bridge_ws
#RUN git clone https://gitlab+deploy-token-799537:CsfF-mNAMrUzvX1CaUrd@gitlab.com/labcom-tiara/ros2/ros1_bridge.git /bridge_ws/src/ros1_bridge
COPY ./ros_entrypoint_ros1_ros2.sh /ros_entrypoint.sh
RUN ["/bin/bash", "-c", "source /ros_entrypoint.sh && colcon build --symlink-install --paths src/ros1_bridge/ "]

COPY ./ros_entrypoint_ros1_ros2_ws.sh /ros_entrypoint.sh
