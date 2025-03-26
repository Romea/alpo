// Copyright 2025 INRAE, French National Research Institute for Agriculture, Food and Environment
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

#ifndef ALPO_HARDWARE__DEADZONE_HANDLER_HPP_
#define ALPO_HARDWARE__DEADZONE_HANDLER_HPP_

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <rclcpp/node.hpp>

namespace romea::ros2
{

class DeadzoneHandler
{
public:
  virtual ~DeadzoneHandler() = default;

  // compute a better steering angle that takes into account the deadzone of the robot
  virtual double adapt_steering_angle(const ackermann_msgs::msg::AckermannDrive &) = 0;
};

}  // namespace romea::ros2

#endif
