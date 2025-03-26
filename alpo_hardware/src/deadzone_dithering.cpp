
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

#include "alpo_hardware/deadzone_dithering.hpp"

namespace romea::ros2
{

DeadzoneDithering::DeadzoneDithering(const hardware_interface::HardwareInfo & hardware_info)
: quantization_size_{get_parameter<double>(hardware_info, "deadzone_quantization_size")},
  deadzone_size_{get_parameter<double>(hardware_info, "deadzone_size")},
  rng_{std::random_device{}()},
  unit_distrib_{0., 1.}
{
}

DeadzoneDithering::DeadzoneDithering(double quantization_size, double deadzone_size)
: quantization_size_(quantization_size),
  deadzone_size_(deadzone_size),
  rng_{std::random_device{}()},
  unit_distrib_{0., 1.}
{
}

double DeadzoneDithering::adapt_steering_angle(const ackermann_msgs::msg::AckermannDrive & cmd)
{
  double angle = cmd.steering_angle;

  if(std::abs(angle) < 0.001) {
    return angle;
  }

  // for angle around zero, use deadzone_size instead of quantization_size
  double step_size = std::abs(angle) < deadzone_size_ ? deadzone_size_ : quantization_size_;

  double remainder = std::fmod(angle, step_size);
  double random_angle_incr = step_size * unit_distrib_(rng_);
  double incr = random_angle_incr < std::abs(remainder) ? step_size : 0.;
  return angle - remainder + std::copysign(incr, angle);
}

}  // namespace romea::ros2
