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
#include "ros2_rviz2/display_lines.hpp"

namespace ros2_rviz2
{

DisplayLines::DisplayLines()
: Node("display_lines")
{
    RCLCPP_DEBUG(this->get_logger(), "Construct a new Display Lines object");
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "visualization_lines",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DisplayLines::timerCallback, this));
}

DisplayLines::~DisplayLines()
{
    RCLCPP_DEBUG(this->get_logger(), "Destroy the Display Lines object");
}

void
DisplayLines::timerCallback()
{
#if 0
    static bool status = false;
    if (status)
    {
        status = !status;
        displaySingleLine();
    }
    else
    {
        status = !status;
        displayMultipleLines();
    }
#endif
    displaySingleLine();
}

void
DisplayLines::displaySingleLine()
{
    auto msg = visualization_msgs::msg::Marker();
    msg.header.frame_id = "map"; // 设置坐标系
    msg.header.stamp = this->get_clock()->now();
    msg.ns = "line";
    msg.id = 0;
    msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    msg.action = visualization_msgs::msg::Marker::ADD;

    // 设置线条的颜色
    msg.color.r = 0.0;
    msg.color.g = 1.0;
    msg.color.b = 0.0;
    msg.color.a = 1.0;

    // 设置线条的尺寸
    msg.scale.x = 0.025; // 线宽

    // 创建线条的两个点
    geometry_msgs::msg::Point p1, p2;
    p1.x = 2.0;
    p1.y = 0.0;
    p1.z = 0.0;
    p2.x = -2.0;
    p2.y = 0.0;
    p2.z = 0.0;

    msg.points.push_back(p1);
    msg.points.push_back(p2);

    // 设置持续时间
    msg.lifetime = rclcpp::Duration::from_seconds(0.9); // 持续时间为1秒
    // msg.lifetime = rclcpp::Duration::from_seconds(0);  //  设置成0表示一直可见

    RCLCPP_INFO(this->get_logger(), "Publishing message single line");
    marker_pub_->publish(msg);
}

void
DisplayLines::displayMultipleLines()
{
    auto msg = visualization_msgs::msg::Marker();
    msg.header.frame_id = "map"; // 设置坐标系
    msg.header.stamp = this->get_clock()->now();
    msg.ns = "lines";
    msg.id = 1;
    msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    msg.action = visualization_msgs::msg::Marker::ADD;

    // 设置线条的颜色
    msg.color.r = 0.0;
    msg.color.g = 1.0;
    msg.color.b = 0.0;
    msg.color.a = 1.0;

    // 设置线条的尺寸
    msg.scale.x = 0.025; // 线宽

    // // 创建多条线的点
    // for (double x = -2.0; x <= 2.0; x += 0.5)
    // {
    //     geometry_msgs::msg::Point point;
    //     point.x = x;
    //     point.y = 2.0; // 顶部的y坐标
    //     point.z = 0.0;
    //     msg.points.push_back(point);

    //     point.y = -2.0; // 底部的y坐标
    //     msg.points.push_back(point);
    // }
    
    // 创建个正方形框
    geometry_msgs::msg::Point point;
    point.x = 2.0;
    point.y = 2.0;
    point.z = 0.0;
    for (size_t i = 0; i < 5; i++)
    {
        msg.points.push_back(point);
        if ((i % 2) == 0)
        {
            point.x = -point.x;
        }
        else
        {
            point.y = -point.y;
        }
    }

    // 设置持续时间
    msg.lifetime = rclcpp::Duration::from_seconds(0.8); // 持续时间为1秒
    // msg.lifetime = rclcpp::Duration::from_seconds(0);  //  设置成0表示一直可见

    RCLCPP_INFO(this->get_logger(), "Publishing message multiple lines");
    marker_pub_->publish(msg);
}


} // namespace ros2_rviz2


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ros2_rviz2::DisplayLines>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
