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


#ifndef ROS2_TOPIC_RECORDER__TOPIC_RECORDER_HPP_
#define ROS2_TOPIC_RECORDER__TOPIC_RECORDER_HPP_

#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace ros2_topic_recorder
{
class TopicRecorder : public rclcpp::Node
{
public:
    TopicRecorder();
    ~TopicRecorder();

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    void odomSubCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void imuSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    std::string getCurrentTimeAsString();

    std::fstream odom_file_;

    std::fstream imu_file_;

    

};

}  // namespace ros2_topic_recorder
#endif  // ROS2_TOPIC_RECORDER__TOPIC_RECORDER_HPP_
