/*
 Copyright 2023 ROS2 LLC

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */


#include "ros2_fixed_path/fixed_path.hpp"

namespace ros2_fixed_path
{
FixedPath::FixedPath()
: Node("ros2_fixed_path")
{
    RCLCPP_INFO(this->get_logger(), "Construct a new Fixed Path object");
}

FixedPath::~FixedPath()
{
    RCLCPP_INFO(this->get_logger(), "Destroy the Fixed Path object");
}






} // namespace ros2_fixed_path
