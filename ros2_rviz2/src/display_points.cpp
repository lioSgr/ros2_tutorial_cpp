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


#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ros2_rviz2/display_points.hpp"

namespace ros2_rviz2
{
DisplayPoints::DisplayPoints()
: Node("display_points")
{
    RCLCPP_DEBUG(this->get_logger(), "Construct a new Display Points object");
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "visualization_points",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DisplayPoints::timerCallback, this));
}

DisplayPoints::~DisplayPoints()
{
    RCLCPP_DEBUG(this->get_logger(), "Destroy the Display Points object");
}

void
DisplayPoints::timerCallback()
{
    static bool status = false;
    if (status)
    {
        status = !status;
        displaySinglePoint();
    }
    else
    {
        status = !status;
        displayMultiplePoints();
    }
}

void
DisplayPoints::displaySinglePoint()
{
    auto msg = visualization_msgs::msg::Marker();
    // 设置消息的基本属性
    msg.header.frame_id = "map"; // 设置坐标系，根据你的需求更改
    msg.header.stamp = this->get_clock()->now();
    msg.id = 0; // 每个Marker应该有唯一的ID
    msg.ns = "point";

    // 设置Marker类型为点
    msg.type = visualization_msgs::msg::Marker::POINTS;
    msg.action = visualization_msgs::msg::Marker::ADD;

    // 创建一个点
    geometry_msgs::msg::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;

    // 将点添加到points字段中
    msg.points.push_back(point);

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
    // msg.lifetime = rclcpp::Duration::from_seconds(0);  //  设置成0表示一直可见

    RCLCPP_INFO(this->get_logger(), "Publishing message single point");
    marker_pub_->publish(msg);
}

void
DisplayPoints::displayMultiplePoints()
{
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
    for (double x = -1.0; x <= 1.0; x += 0.1) {
        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = 0.0;
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


} // namespace ros2_rviz2


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ros2_rviz2::DisplayPoints>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
