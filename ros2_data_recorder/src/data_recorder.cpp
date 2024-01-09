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


#include "ros2_data_recorder/data_recorder.hpp"
#include <string>
#include <chrono>
#include <ctime>
#include <cstdio>
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


namespace ros2_data_recorder
{

DataRecorder::DataRecorder()
: Node("data_recorder")
{
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&DataRecorder::timerCallback, this));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        10,
        std::bind(&DataRecorder::odomSubCallback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu",
        10,
        std::bind(&DataRecorder::imuSubCallback, this, std::placeholders::_1));
    odom_data_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_data",
        10,
        std::bind(&DataRecorder::odomDataSubCallback, this, std::placeholders::_1));
    imu_data_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_data",
        10,
        std::bind(&DataRecorder::imuDataSubCallback, this, std::placeholders::_1));
    amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "amcl_pose",
        10,
        std::bind(&DataRecorder::amclPoseSubCallback, this, std::placeholders::_1));

    // 
    std::string time;
    time = getCurrentTimeAsString();
    RCLCPP_INFO(this->get_logger(), "Time Of Recorder: %s", time.c_str());

    // 文件路径
    std::string path = "/home/lio/9_Matlab/data";
    // 构建文件名
    std::string odom_file_name = path + "/data_recorder_" + time + ".txt";
    // 打开文件流并创建文件
    file_.open(odom_file_name, std::ios::out);
    if (!file_.is_open()) 
    {
        RCLCPP_ERROR(this->get_logger(), "File %s open failed", odom_file_name.c_str());
    }
}

DataRecorder::~DataRecorder()
{
    if (file_.is_open())
    {
        file_.close();
    }
}

void
DataRecorder::timerCallback()
{
    std::string time;
    time = getCurrentTimeAsString();

    double odom_data_yaw = tf2::getYaw(odom_data_.pose.pose.orientation);
    double imu_data_yaw = tf2::getYaw(imu_data_.orientation);
    double odom_yaw = tf2::getYaw(odom_.pose.pose.orientation);
    double imu_yaw = tf2::getYaw(imu_.orientation);
    double amcl_pose_yaw = tf2::getYaw(amcl_pose_.pose.pose.orientation);
    // write data to file
    if (file_.is_open())
    {
        file_ << time.c_str() << " "
            << "odom_data --- " 
            << "x: " << odom_data_.pose.pose.position.x << ", "
            << "y: " << odom_data_.pose.pose.position.y << ", "
            << "z: " << odom_data_.pose.pose.position.z << ", "
            << "yaw: " << odom_data_yaw << ", "
            << "ox: " << odom_data_.pose.pose.orientation.x << ", "
            << "oy: " << odom_data_.pose.pose.orientation.y << ", "
            << "oz: " << odom_data_.pose.pose.orientation.z << ", "
            << "ow: " << odom_data_.pose.pose.orientation.w << " "
            << "imu_data --- "
            << "yaw: " << imu_data_yaw << ", "
            << "ox: " << imu_data_.orientation.x << ", "
            << "oy: " << imu_data_.orientation.y << ", "
            << "oz: " << imu_data_.orientation.z << ", "
            << "ow: " << imu_data_.orientation.w << ", "
            << "angular_vx: " << imu_data_.angular_velocity.x << ", "
            << "angular_vy: " << imu_data_.angular_velocity.y << ", "
            << "angular_vz: " << imu_data_.angular_velocity.z << ", "
            << "linear_ax: " << imu_data_.linear_acceleration.x << ", "
            << "linear_ay: " << imu_data_.linear_acceleration.y << ", "
            << "linear_az: " << imu_data_.linear_acceleration.z << ", "
            << "odom --- " 
            << "x: " << odom_.pose.pose.position.x << ", "
            << "y: " << odom_.pose.pose.position.y << ", "
            << "z: " << odom_.pose.pose.position.z << ", "
            << "yaw: " << odom_yaw << ", "
            << "ox: " << odom_.pose.pose.orientation.x << ", "
            << "oy: " << odom_.pose.pose.orientation.y << ", "
            << "oz: " << odom_.pose.pose.orientation.z << ", "
            << "ow: " << odom_.pose.pose.orientation.w << " "
            << "imu --- "
            << "yaw: " << imu_yaw << ", "
            << "ox: " << imu_.orientation.x << ", "
            << "oy: " << imu_.orientation.y << ", "
            << "oz: " << imu_.orientation.z << ", "
            << "ow: " << imu_.orientation.w << ", "
            << "angular_vx: " << imu_.angular_velocity.x << ", "
            << "angular_vy: " << imu_.angular_velocity.y << ", "
            << "angular_vz: " << imu_.angular_velocity.z << ", "
            << "linear_ax: " << imu_.linear_acceleration.x << ", "
            << "linear_ay: " << imu_.linear_acceleration.y << ", "
            << "linear_az: " << imu_.linear_acceleration.z << ", "
            << "amcl_pose ---"
            << "x: " << amcl_pose_.pose.pose.position.x << ", "
            << "y: " << amcl_pose_.pose.pose.position.y << ", "
            << "yaw: " << amcl_pose_yaw << ", "
            << "\n";
    }

}

std::string
DataRecorder::getCurrentTimeAsString()
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
DataRecorder::odomDataSubCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_data_ = *msg;
}

void
DataRecorder::imuDataSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_data_ = *msg;
}

void
DataRecorder::odomSubCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_ = *msg;
}

void
DataRecorder::imuSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_ = *msg;
}

void
DataRecorder::amclPoseSubCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    amcl_pose_ = *msg;
}






} // namespace ros2_data_recorder
