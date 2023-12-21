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

#include "ros2_topic_record/topic_record.hpp"
#include "tf2/utils.h"
#include <chrono>
#include <ctime>

namespace ros2_topic_record
{

TopicRecord::TopicRecord()
: Node("ros2_topic_record")
{
    // 获取当前系统时间点
    auto currentTime = std::chrono::system_clock::now();

    // 将时间点转换为time_t以便使用std::localtime
    std::time_t currentTime_t = std::chrono::system_clock::to_time_t(currentTime);

    // 使用std::localtime将time_t转换为tm结构体
    std::tm* currentTime_tm = std::localtime(&currentTime_t);

    RCLCPP_INFO(this->get_logger(), "Time: %d-%d-%d %d:%d:%d",
        currentTime_tm->tm_year + 1900, currentTime_tm->tm_mon, currentTime_tm->tm_mday,
        currentTime_tm->tm_hour, currentTime_tm->tm_min, currentTime_tm->tm_sec);

    odom_data_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_data",
        10,
        std::bind(&TopicRecord::odoDataSubCallback, this, std::placeholders::_1));
    imu_data_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_data",
        10,
        std::bind(&TopicRecord::imuDataSubCallback, this, std::placeholders::_1));
    odom_filter_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "odom_filter",
        10);
    imu_filter_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu_filter",
        10);
}

TopicRecord::~TopicRecord()
{
    // 
}

void
TopicRecord::odoDataSubCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    static uint64_t cnt = 0;
    cnt = cnt + 1;
    double yaw = tf2::getYaw(msg->pose.pose.orientation);
    RCLCPP_INFO(this->get_logger(),
        "odom_data %ld: [x %.4lf, y %.4lf, z %.4lf, <yaw %.4lf> ox %.4lf, oy %.4lf, oz %.4lf, ow %.4lf]",
        cnt,
        msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, yaw,
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    
    // static bool is_pub = true;
    // auto msg_filter = nav_msgs::msg::Odometry();
    // msg_filter = *msg;
    // if (is_pub)
    // {
    //     odom_filter_pub_->publish(msg_filter);
    //     is_pub = !is_pub;
    // }
    // else
    // {
    //     is_pub = !is_pub;
    // }

}

void
TopicRecord::imuDataSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    static uint64_t cnt = 0;
    cnt = cnt + 1;
    double yaw = tf2::getYaw(msg->orientation);
    RCLCPP_INFO(this->get_logger(),
        "imu_data %ld: [<yaw %.4lf>, ox %.4lf, oy %.4lf, oz %.4lf, ow %.4lf, a_v_x %.4lf, a_v_y %.4lf, a_v_z %.4lf, l_a_x %.4lf, l_a_y %.4lf, l_a_z %.4lf]",
        cnt,
        yaw,
        msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w,
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
// #if 1    
//     static bool is_pub = true;
//     auto msg_filter = sensor_msgs::msg::Imu();
//     msg_filter = *msg;
//     if (is_pub)
//     {
//         imu_filter_pub_->publish(msg_filter);
//         is_pub = !is_pub;
//     }
//     else
//     {
//         is_pub = !is_pub;
//     }
// #endif
// #if 0
//     auto msg_filter = sensor_msgs::msg::Imu();
//     msg_filter = *msg;
//     imu_filter_pub_->publish(msg_filter);
// #endif
}






} // namespace ros2_topic_record
