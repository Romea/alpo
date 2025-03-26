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

#ifndef ALPO_HARDWARE__DEADZONE_DITHERING_HPP_
#define ALPO_HARDWARE__DEADZONE_DITHERING_HPP_

#include <random>
#include <fstream>

// romea
#include <romea_mobile_base_hardware/hardware_system_interface.hpp>

// local
#include "alpo_hardware/deadzone_handler.hpp"

namespace romea::ros2
{

class DeadzoneDithering : public DeadzoneHandler
{
public:
  using Distrib = std::uniform_real_distribution<double>;

public:
  DeadzoneDithering(const hardware_interface::HardwareInfo & hardware_info);
  DeadzoneDithering(double quantization_size, double deadzone_size);

  double adapt_steering_angle(const ackermann_msgs::msg::AckermannDrive &) override;

private:
  double quantization_size_;
  double deadzone_size_;
  std::mt19937 rng_;
  Distrib unit_distrib_;
};

}  // namespace romea::ros2

#endif
