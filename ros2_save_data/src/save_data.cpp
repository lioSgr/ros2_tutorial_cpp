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

#include "ros2_save_data/save_data.hpp"

using std::placeholders::_1;

namespace ros2_save_data
{
SaveData::SaveData()
: Node("save_data")
{
     RCLCPP_INFO(this->get_logger(), "Construct a new Save Data object");
     auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "odom",
          qos,
          std::bind(&SaveData::odomSubCallback, this, _1));
}

SaveData::~SaveData()
{
     RCLCPP_INFO(this->get_logger(), "Destroy the Save Data object");
}

void
SaveData::odomSubCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
     tf2::Quaternion quaternion;
     tf2::fromMsg(msg->pose.pose.orientation, quaternion);
     double roll, pitch, yaw;
     tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
     RCLCPP_INFO(this->get_logger(), "Current pose: (x %lf, y %lf, yaw %lf)", msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);

}



} // namespace ros2_sava_data
