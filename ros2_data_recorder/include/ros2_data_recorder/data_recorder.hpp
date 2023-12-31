/*
 Copyright 2024 ROS2 LLC

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


#ifndef ROS2_DATA_RECORDER__DATA_RECORDER_HPP_
#define ROS2_DATA_RECORDER__DATA_RECORDER_HPP_

#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace ros2_data_recorder
{
class DataRecorder : public rclcpp::Node
{
public:
    DataRecorder();
    ~DataRecorder();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_data_sub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;

    void timerCallback();

    std::string getCurrentTimeAsString();

    void odomDataSubCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void imuDataSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    void odomSubCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void imuSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    void amclPoseSubCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

private:
    std::fstream file_;

    nav_msgs::msg::Odometry odom_data_;

    sensor_msgs::msg::Imu imu_data_;

    nav_msgs::msg::Odometry odom_;

    sensor_msgs::msg::Imu imu_;

    geometry_msgs::msg::PoseWithCovarianceStamped amcl_pose_;




};
}  // namespace ros2_data_recorder
#endif  // ROS2_DATA_RECORDER__DATA_RECORDER_HPP_
