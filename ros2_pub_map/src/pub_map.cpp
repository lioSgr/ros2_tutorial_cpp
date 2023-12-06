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


#include "ros2_pub_map/pub_map.hpp"

namespace ros2_pub_map
{

PubMap::PubMap()
: Node("ros2_pub_map")
{
     RCLCPP_INFO(this->get_logger(), "Construct a new Pub Map object");
     map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "/map",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
     initParameters();

     spreadMap();
}

PubMap::~PubMap()
{
     RCLCPP_INFO(this->get_logger(), "Destroy the Pub Map object");
}

void
PubMap::initParameters()
{
     this->declare_parameter<std::string>("map", "/home/lio/3_simulation_ws/maps/concave/concave.yaml");
     map_url_ = this->get_parameter("map").get_value<std::string>();
     RCLCPP_INFO(this->get_logger(), "map_url_: %s", map_url_.c_str());
}

void
PubMap::spreadMap()
{
     std::string url;
     nav_msgs::msg::OccupancyGrid map;
     url = map_url_;
     nav2_map_server::loadMapFromYaml(url, map);

     auto msg = nav_msgs::msg::OccupancyGrid();
     msg = map;
     RCLCPP_INFO(this->get_logger(), "Publishing map");
     map_pub_->publish(msg);
}


} // namespace ros2_pub_map
