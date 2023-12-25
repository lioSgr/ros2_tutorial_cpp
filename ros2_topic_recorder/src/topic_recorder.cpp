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


#include "ros2_topic_recorder/topic_recorder.hpp"
#include <string>
#include <chrono>
#include <ctime>
#include <cstdio>

namespace ros2_topic_recorder
{

TopicRecorder::TopicRecorder()
: Node("ros2_topic_recorder")
{
    // 订阅 "odom" 主题，并绑定回调函数
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        10,
        std::bind(&TopicRecorder::odomSubCallback, this, std::placeholders::_1));

    // 订阅 "imu" 主题，并绑定回调函数
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu",
        10,
        std::bind(&TopicRecorder::imuSubCallback, this, std::placeholders::_1));
    
    std::string time;
    time = getCurrentTimeAsString();
    RCLCPP_INFO(this->get_logger(), "Time Of Recorder: %s", time.c_str());

    // 文件路径
    std::string path = "/home/lio/9_Matlab/0_script";
    // 构建文件名
    std::string odom_file_name = path + "/odom_" + time + ".txt";
    // 打开文件流并创建文件
    odom_file_.open(odom_file_name, std::ios::out);
    if (!odom_file_.is_open()) 
    {
        RCLCPP_ERROR(this->get_logger(), "File %s open failed", odom_file_name.c_str());
    }

    std::string imu_file_name = path + "/imu_" + time + ".txt";
    imu_file_.open(imu_file_name, std::ios::out);
    if (!imu_file_.is_open()) 
    {
        RCLCPP_ERROR(this->get_logger(), "File %s open failed", imu_file_name.c_str());
    }

}

TopicRecorder::~TopicRecorder()
{
    // 关闭文件流
    if (odom_file_.is_open()) 
    {
        odom_file_.close();
    }
    if (imu_file_.is_open()) 
    {
        imu_file_.close();
    }
}

std::string
TopicRecorder::getCurrentTimeAsString()
{
    // 获取当前系统时间点
    auto currentTime = std::chrono::system_clock::now();

    // 将时间点转换为time_t以便使用std::localtime
    std::time_t currentTime_t = std::chrono::system_clock::to_time_t(currentTime);

    // 使用std::localtime将time_t转换为tm结构体
    std::tm* currentTime_tm = std::localtime(&currentTime_t);

    // 获取毫秒数
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch() % std::chrono::seconds(1));

    // 格式化时间为字符串
    char timeString[100];
    std::sprintf(timeString, "%d_%02d_%02d_%02d_%02d_%02d_%03lld",
        currentTime_tm->tm_year + 1900, currentTime_tm->tm_mon + 1, currentTime_tm->tm_mday,
        currentTime_tm->tm_hour, currentTime_tm->tm_min, currentTime_tm->tm_sec, static_cast<long long>(milliseconds.count()));

    // 返回格式化后的时间字符串
    return std::string(timeString);
}

void
TopicRecorder::odomSubCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::string time;
    time = getCurrentTimeAsString();
    // 写入数据到日志文件
    if (odom_file_.is_open()) 
    {
        odom_file_ << time.c_str() << " "
            << "Odom Data(x, y, z, ox, oy, oz, ow): " 
            << msg->pose.pose.position.x << " "
            << msg->pose.pose.position.y << " "
            << msg->pose.pose.position.z << " "
            << msg->pose.pose.orientation.x << " "
            << msg->pose.pose.orientation.y << " "
            << msg->pose.pose.orientation.z << " "
            << msg->pose.pose.orientation.w << " "
            << "\n";
    }
}

void
TopicRecorder::imuSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::string time;
    time = getCurrentTimeAsString();
    // 写入数据到日志文件
    if (imu_file_.is_open()) 
    {
        imu_file_ << time.c_str() << " "
            << "IMU Data(ox, oy, oz, ow, angular_vx, angular_vy, angular_vz, linear_ax, linear_ay, linear_az): "
            << msg->orientation.x << " "
            << msg->orientation.y << " "
            << msg->orientation.z << " "
            << msg->orientation.w << " "
            << msg->angular_velocity.x << " "
            << msg->angular_velocity.y << " "
            << msg->angular_velocity.z << " "
            << msg->linear_acceleration.x << " "
            << msg->linear_acceleration.y << " "
            << msg->linear_acceleration.z << " "
            << "\n";
    }
}

} // namespace ros2_topic_recorder