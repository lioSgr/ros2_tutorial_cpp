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

#ifndef ROS2_SAVE_DATA__SAVE_DATA_HPP_
#define ROS2_SAVE_DATA__SAVE_DATA_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ros2_save_data
{
class SaveData : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Save Data object
     * 
     */
    SaveData();
    /**
     * @brief Destroy the Save Data object
     * 
     */
    ~SaveData();

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    void odomSubCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};
}  // namespace ros2_save_data
#endif  // ROS2_SAVE_DATA__SAVE_DATA_HPP_
