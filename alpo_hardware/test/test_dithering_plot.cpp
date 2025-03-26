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

#include <cmath>
#include <fstream>

#include "alpo_hardware/deadzone_dithering.hpp"

namespace
{

void generate_dithering()
{
  std::ofstream file{"dithering.csv"};
  file << "t,input,dithering\n";

  romea::ros2::DeadzoneDithering dithering{0.04, 0.1};

  for (float t = 0; t < 10; t += 0.01) {
    ackermann_msgs::msg::AckermannDrive msg;
    msg.speed = 1.0;
    msg.steering_angle = 0.204F * std::sin(0.1F * t * 2.F * M_PIf);
    double dithering_angle = dithering.adapt_steering_angle(msg);

    file << t << ',' << msg.steering_angle << ',' << dithering_angle << '\n';
  }

  file.close();
}

}  // namespace

int main()
{
  generate_dithering();
}
