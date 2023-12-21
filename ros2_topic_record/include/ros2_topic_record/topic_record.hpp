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


#ifndef ROS2_TOPIC_RECORD__TOPIC_RECORD_HPP_
#define ROS2_TOPIC_RECORD__TOPIC_RECORD_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ros2_topic_record
{
class TopicRecord : public rclcpp::Node
{
public:
    TopicRecord();
    ~TopicRecord();

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_data_sub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_filter_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_filter_pub_;


    void odoDataSubCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void imuDataSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg);






};
}  // namespace ros2_topic_record
#endif  // ROS2_TOPIC_RECORD__TOPIC_RECORD_HPP_
