#include "alpo_bridge.hpp"

#include <ifaddrs.h>
#include <net/if.h>
#include <arpa/inet.h>

namespace {

const std::string & master =  "http://192.168.100.1:11311";

std::string find_local_end_ip()
{
  std::string local_end_ip;

  ifaddrs *addrs;
  if (getifaddrs(&addrs) != -1)
  {
    for (auto addr = addrs; addr != NULL; addr = addr->ifa_next) {

      // No address? Skip.
      if (addr->ifa_addr == nullptr) continue;

      // Interface isn't active? Skip.
      if (!(addr->ifa_flags & IFF_UP)) continue;


      if(addr->ifa_addr->sa_family == AF_INET) {

        std::string ip = inet_ntoa(reinterpret_cast<sockaddr_in *>(addr->ifa_addr)->sin_addr);
        if(ip.find("192.168.100")==0)
        {
          local_end_ip=ip;
        }
      }
    }
  }

  freeifaddrs(addrs);
  return local_end_ip;
}

}

int main(int argc, char * argv[])
{
  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node_ptr = rclcpp::Node::make_shared("ros_bridge");

  // Find local end ip
  //  std::string ip  = find_local_end_ip();
  //  if(ip.empty())
  //  {
  //    RCLCPP_FATAL(ros2_node_ptr->get_logger(),
  //                 "No ethernet connection with alpo robot, bridge failed");
  //    return 1;
  //  }

  // ROS 1 node
  ros::M_string remappings;
  //  remappings["__ip"]= ip;
  //  remappings["__master"]= master;
  ros::init(remappings, "alpo_bridge");
  auto ros1_node_ptr = std::make_shared<ros::NodeHandle>();

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(4);
  async_spinner.start();

  // Start bridge
  AlpoBridge bridge(ros1_node_ptr,ros2_node_ptr);
  bridge.start();

  // ROS 2 spinning loop
  rclcpp::executors::MultiThreadedExecutor executor;
  while (ros1_node_ptr->ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node_ptr, std::chrono::milliseconds(1));
  }

  return 0;
}
