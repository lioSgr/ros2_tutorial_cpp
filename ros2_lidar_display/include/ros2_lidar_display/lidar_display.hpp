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


#ifndef ROS2_LIDAR_DISPLAY__LIDAR_DISPLAY_HPP_
#define ROS2_LIDAR_DISPLAY__LIDAR_DISPLAY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace lidar_display
{
class LidarDisplay : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Lidar Display object
     * 
     */
    LidarDisplay();
    /**
     * @brief Destroy the Lidar Display object
     * 
     */
    ~LidarDisplay();

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    void lidarScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    sensor_msgs::msg::LaserScan lidar_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr tf_timer_;

    void tfTimerCallback();

    // tf relationship
	double tf_lidar2map_x_;
	double tf_lidar2map_y_;
	double tf_lidar2map_yaw_;

    void lidarDisplay();

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

};
}  // namespace lidar_display
#endif  // ROS2_LIDAR_DISPLAY__LIDAR_DISPLAY_HPP_
