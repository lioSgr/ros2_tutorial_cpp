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

#include "ros2_keepout_mask/keepout_mask.hpp"

namespace ros2_keepout_mask
{
KeepoutMask::KeepoutMask()
: Node("keepout_mask")
{
     RCLCPP_INFO(this->get_logger(), "Construct a new Keepout Mask object");
     timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&KeepoutMask::timerCallback, this));
     costmap_filter_info_pub_ = this->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
          "/costmap_filter_info_keepout",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
     keepout_mask_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "/keepout_filter_mask",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

KeepoutMask::~KeepoutMask()
{
     RCLCPP_INFO(this->get_logger(), "Destroy the Keepout Mask object");
}

void
KeepoutMask::timerCallback()
{
     RCLCPP_WARN(this->get_logger(), "Timer event");
     auto msg = nav2_msgs::msg::CostmapFilterInfo();
     msg.type = 0;
     msg.filter_mask_topic = "/keepout_filter_mask";
     msg.base = 0.0f;
     msg.multiplier = 1.0f;
     msg.header.frame_id = "map";
     msg.header.stamp = rclcpp::Clock().now();
     RCLCPP_INFO(this->get_logger(), "Publishing message");
     costmap_filter_info_pub_->publish(msg);

     nav_msgs::msg::OccupancyGrid mask;
     std::string mask_yaml = "/home/lio/3_simulation_ws/src/navigation2_tutorials/nav2_costmap_filters_demo/maps/rectangle_20x3_mask_2.yaml";
     nav2_map_server::loadMapFromYaml(mask_yaml, mask);

     auto msg_mask = nav_msgs::msg::OccupancyGrid();
     msg_mask = mask;
     RCLCPP_INFO(this->get_logger(), "Publishing message");
     keepout_mask_pub_->publish(msg_mask);
     timer_->cancel();

}

} // namespace ros2_keepout_mask
