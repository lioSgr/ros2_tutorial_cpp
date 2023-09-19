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


#ifndef ROS2_RVIZ2__DISPLAY_BEZIER_HPP_
#define ROS2_RVIZ2__DISPLAY_BEZIER_HPP_

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace ros2_rviz2
{
class DisplayBezier : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Display Bezier object
     * 
     */
    DisplayBezier();
    /**
     * @brief Destroy the Display Bezier object
     * 
     */
    ~DisplayBezier();

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timerCallback();
    std::vector<geometry_msgs::msg::Point> calculateBezierPoints(geometry_msgs::msg::Point p0, geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2, geometry_msgs::msg::Point p3, int num_points);
    void displaySingleBezier();
    void displayMultipleBeziers();

};
}  // namespace ros2_rviz2
#endif  // ROS2_RVIZ2__DISPLAY_BEZIER_HPP_
