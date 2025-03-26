
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

#include <cmath>
#include <random>

// romea
#include <romea_mobile_base_utils/ros2_control/info/hardware_info_common.hpp>

#include "alpo_hardware/deadzone_gaussian_noise.hpp"

namespace romea::ros2
{

DeadzoneGaussianNoise::DeadzoneGaussianNoise(const hardware_interface::HardwareInfo & hardware_info)
: rng_{std::random_device{}()},
  distrib_{0., get_parameter<double>(hardware_info, "deadzone_std_dev")}
{
}

double DeadzoneGaussianNoise::adapt_steering_angle(const ackermann_msgs::msg::AckermannDrive & cmd)
{
  constexpr auto epsilon = std::numeric_limits<double>::epsilon();
  double angle = cmd.steering_angle;
  if (std::abs(cmd.steering_angle) > epsilon && std::abs(cmd.speed) > epsilon) {
    angle += distrib_(rng_);
  }
  return angle;
}

}  // namespace romea::ros2
