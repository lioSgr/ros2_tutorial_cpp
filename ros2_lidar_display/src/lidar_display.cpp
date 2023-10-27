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


#include "ros2_lidar_display/lidar_display.hpp"

namespace lidar_display
{
LidarDisplay::LidarDisplay()
: Node("lidar_display")
{
     RCLCPP_DEBUG(this->get_logger(), "Construct a new Lidar Display object");
     using std::placeholders::_1;
     lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan",
          rclcpp::SensorDataQoS(),
          std::bind(&LidarDisplay::lidarScanCallback, this, _1));
     
     tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

     tf_timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&LidarDisplay::tfTimerCallback, this));

     marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "visualization_points",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

}

LidarDisplay::~LidarDisplay()
{
     RCLCPP_DEBUG(this->get_logger(), "Destroy the Lidar Display object");
}

void
LidarDisplay::lidarScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
     lidar_ = *msg;
}

void
LidarDisplay::tfTimerCallback()
{
     // 获取lidar2base tf
     try
     {
          std::string target_link = "base_footprint";
          std::string source_link = "base_scan";
          geometry_msgs::msg::TransformStamped tf_pose;
          tf_pose = tf_buffer_->lookupTransform(target_link, source_link, tf2::TimePointZero);

          tf2::Quaternion quat;
          tf2::fromMsg(tf_pose.transform.rotation, quat);
          double roll, pitch, yaw;
          tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
          double yaw_degrees = yaw * 180.0 / M_PI;
          tf_lidar2base_x_ = tf_pose.transform.translation.x;
          tf_lidar2base_y_ = tf_pose.transform.translation.y;
          tf_lidar2base_yaw_ = yaw;
          RCLCPP_INFO(this->get_logger(), "tf lidar----base: [x %lf, y %lf, yaw %lf(%lf°)]",
               tf_lidar2base_x_, tf_lidar2base_y_, tf_lidar2base_yaw_, yaw_degrees);
     }
     catch(const std::exception& e)
     {
          std::cerr << e.what() << '\n';
     }
     // 获取base2map tf
     try
     {
          std::string target_link = "map";
          std::string source_link = "base_footprint";
          geometry_msgs::msg::TransformStamped tf_pose;
          tf_pose = tf_buffer_->lookupTransform(target_link, source_link, tf2::TimePointZero);

          tf2::Quaternion quat;
          tf2::fromMsg(tf_pose.transform.rotation, quat);
          double roll, pitch, yaw;
          tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
          double yaw_degrees = yaw * 180.0 / M_PI;
          tf_base2map_x_ = tf_pose.transform.translation.x;
          tf_base2map_y_ = tf_pose.transform.translation.x;
          tf_base2map_yaw_ = yaw;
          RCLCPP_INFO(this->get_logger(), "tf base----map: [x %lf, y %lf, yaw %lf(%lf°)]",
               tf_base2map_x_, tf_base2map_y_, tf_base2map_yaw_, yaw_degrees);
     }
     catch(const std::exception& e)
     {
          std::cerr << e.what() << '\n';
     }

     lidarDisplay();
}

void
LidarDisplay::lidarDisplay()
{
     // 雷达坐标系下点位
     std::vector<geometry_msgs::msg::PointStamped> points;
     points.clear();
     for (size_t i = 0; i < lidar_.ranges.size(); i++)
     {
          float range = lidar_.ranges[i];
          if (std::isinf(range) || std::isnan(range))
          {
               range = lidar_.range_max;
          }
          float angle = lidar_.angle_min + lidar_.angle_increment * i;
          float x_lidar = range * std::cos(angle);
          float y_lidar = range * std::sin(angle);
          geometry_msgs::msg::PointStamped point_lidar;
          point_lidar.point.x = x_lidar;
          point_lidar.point.y = y_lidar;
          points.push_back(point_lidar);
     }

     // 变换至base
     std::vector<geometry_msgs::msg::PointStamped> points_base;
     points_base.clear();
     for (size_t i = 0; i < points.size(); i++)
     {
          float x_base = points[i].point.x + tf_lidar2base_x_;
          float y_base = points[i].point.y + tf_lidar2base_y_;
          float x_base_rotate = x_base * std::cos(tf_lidar2base_yaw_) - y_base * std::sin(tf_lidar2base_yaw_);
          float y_base_rotate = x_base * std::sin(tf_lidar2base_yaw_) + y_base * std::cos(tf_lidar2base_yaw_);
          geometry_msgs::msg::PointStamped point;
          point.point.x = x_base_rotate;
          point.point.y = y_base_rotate;
          points_base.push_back(point);
     }

     // 变换至map
     std::vector<geometry_msgs::msg::PointStamped> points_map;
     points_map.clear();
     for (size_t i = 0; i < points_base.size(); i++)
     {
          float x_map = points_base[i].point.x + tf_base2map_x_;
          float y_map = points_base[i].point.y + tf_base2map_y_;
          float x_map_rotate = x_map * std::cos(tf_base2map_yaw_) - y_map * std::sin(tf_base2map_yaw_);
          float y_map_rotate = x_map * std::sin(tf_base2map_yaw_) + y_map * std::cos(tf_base2map_yaw_);
          geometry_msgs::msg::PointStamped point;
          point.point.x = x_map_rotate;
          point.point.y = y_map_rotate;
          points_map.push_back(point);
     }
     
     // display
     auto msg = visualization_msgs::msg::Marker();
     // 设置消息的基本属性
     msg.header.frame_id = "map"; // 设置坐标系，根据你的需求更改
     msg.header.stamp = this->get_clock()->now();
     msg.id = 1; // 不同的ID
     msg.ns = "points";

     // 设置Marker类型为点
     msg.type = visualization_msgs::msg::Marker::POINTS;
     msg.action = visualization_msgs::msg::Marker::ADD;

     // 创建多个点并将它们添加到points字段中
     for (size_t i = 0; i < points_map.size(); i++)
     {
          geometry_msgs::msg::Point point;
          point.x = points_map[i].point.x;
          point.y = points_map[i].point.y;
          point.z = 0.0;
          msg.points.push_back(point);
     }

     // 设置点的颜色
     msg.color.r = 0.0; // 红色
     msg.color.g = 1.0; // 绿色
     msg.color.b = 0.0; // 蓝色
     msg.color.a = 1.0; // 不透明

     // 设置点的尺寸
     msg.scale.x = 0.025; // 点的大小
     msg.scale.y = 0.025; // 点的大小

     // 设置持续时间
     msg.lifetime = rclcpp::Duration::from_seconds(0.8); // 持续时间为1秒

     RCLCPP_INFO(this->get_logger(), "Publishing message multiple points");
     marker_pub_->publish(msg);
}



} // namespace lidar_display
