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

#ifndef ALPO_TELEOP__ALPO_TELEOP_HPP_
#define ALPO_TELEOP__ALPO_TELEOP_HPP_


// ros
#include "rclcpp/node.hpp"

// romea
#include "alpo_teleop/visibility_control.h"
#include "romea_teleop_drivers/one_axle_steering_teleop.hpp"

namespace romea
{

class AlpoTeleop : public OneAxleSteeringTeleop
{
public:
  ALPO_TELEOP_PLUGIN_PUBLIC
  explicit AlpoTeleop(const rclcpp::NodeOptions & options);

  ALPO_TELEOP_PLUGIN_PUBLIC
  virtual ~AlpoTeleop() = default;

};

}  // namespace romea

#endif  // ALPO_TELEOP__ALPO_TELEOP_HPP_
