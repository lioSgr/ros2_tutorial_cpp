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

#ifndef ROS2_KEEPOUT_MASK__KEEPOUT_MASK_HPP_
#define ROS2_KEEPOUT_MASK__KEEPOUT_MASK_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_map_server/map_io.hpp"

namespace ros2_keepout_mask
{
class KeepoutMask : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Keepout Mask object
     * 
     */
    KeepoutMask();
    /**
     * @brief Destroy the Keepout Mask object
     * 
     */
    ~KeepoutMask();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr costmap_filter_info_pub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr keepout_mask_pub_;

    void timerCallback();



};
}  // namespace ros2_keepout_mask
#endif  // ROS2_KEEPOUT_MASK__KEEPOUT_MASK_HPP_
