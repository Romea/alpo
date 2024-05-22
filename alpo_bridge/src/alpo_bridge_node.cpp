// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>

// std
#include <memory>
#include <string>

//
#include "alpo_bridge.hpp"

namespace
{

const std::string & master = "http://192.168.100.1:11311";

std::string find_local_end_ip()
{
  std::string local_end_ip;

  ifaddrs * addrs;
  if (getifaddrs(&addrs) != -1) {
    for (auto addr = addrs; addr != NULL; addr = addr->ifa_next) {
      // No address? Skip.
      if (addr->ifa_addr == nullptr) {
        continue;
      }

      // Interface isn't active? Skip.
      if (!(addr->ifa_flags & IFF_UP)) {
        continue;
      }

      if (addr->ifa_addr->sa_family == AF_INET) {
        std::string ip = inet_ntoa(reinterpret_cast<sockaddr_in *>(addr->ifa_addr)->sin_addr);
        if (ip.find("192.168.100") == 0) {
          local_end_ip = ip;
        }
      }
    }
  }

  freeifaddrs(addrs);
  return local_end_ip;
}

}  // namespace

int main(int argc, char * argv[])
{
  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node_ptr = rclcpp::Node::make_shared("bridge");

  ros2_node_ptr->declare_parameter("override_ros1_master", true);
  auto override_param = ros2_node_ptr->get_parameter("override_ros1_master");
  bool override_ros1_master = override_param.get_value<bool>();
  // RCLCPP_INFO_STREAM(ros2_node_ptr->get_logger(), "override_ros1_master: " << override_ros1_master);

  ros::M_string remappings;

  if (override_ros1_master) {
    std::string ip = find_local_end_ip();
    if (ip.empty()) {
      RCLCPP_FATAL(
        ros2_node_ptr->get_logger(), "No ethernet connection with alpo robot!, Alpo bridge failed");
      return 1;
    }

    remappings["__ip"] = ip;
    remappings["__master"] = master;
  }

  ros::init(remappings, "alpo_bridge");

  auto ros1_node_ptr = std::make_shared<ros::NodeHandle>();

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(4);
  async_spinner.start();

  // Start bridge
  AlpoBridge bridge(ros1_node_ptr, ros2_node_ptr);
  bridge.start();

  // ROS 2 spinning loop
  rclcpp::executors::MultiThreadedExecutor executor;
  while (ros1_node_ptr->ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node_ptr, std::chrono::milliseconds(1));
  }

  return 0;
}
