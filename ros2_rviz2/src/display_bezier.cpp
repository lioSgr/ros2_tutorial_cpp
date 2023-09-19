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
#include "ros2_rviz2/display_bezier.hpp"

namespace ros2_rviz2
{
DisplayBezier::DisplayBezier()
: Node("display_bezier")
{
    RCLCPP_DEBUG(this->get_logger(), "Construct a new Display Bezier object");
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "visualization_beziers",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DisplayBezier::timerCallback, this));
}

DisplayBezier::~DisplayBezier()
{
    RCLCPP_DEBUG(this->get_logger(), "Destroy the Display Bezier object");
}

void
DisplayBezier::timerCallback()
{
    static bool status = false;
    if (status)
    {
        status = !status;
        displaySingleBezier();
    }
    else
    {
        status = !status;
        displayMultipleBeziers();
    }
}

// 三阶 Bezier Curve 曲线上各点的参数方程
// P(t) = P0 * (1 − t)^3 + 3 * P1 * (1 − t)^2 * t + 3 * P2 * (1 − t) * t^2 + P3 * t^3

// 五阶 Bezier Curve 曲线上各点的参数方程
// P(t) = P0 * (1-t)^5 + P1 * 5 * (1-t)^4 * t + P2 * 10 * (1-t)^3 * t^2 + P3 * 10 * (1-u)^2 * t^3 + P4 * 5 * (1-t) * t^4 + P5 * t^5;

std::vector<geometry_msgs::msg::Point>
DisplayBezier::calculateBezierPoints(geometry_msgs::msg::Point p0, geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2, geometry_msgs::msg::Point p3, int num_points)
{
    std::vector<geometry_msgs::msg::Point> points;

    for (int i = 0; i <= num_points; ++i) 
    {
        double t = static_cast<double>(i) / static_cast<double>(num_points);
        double u = 1.0 - t;
        double x = u*u*u*p0.x + 3*u*u*t*p1.x + 3*u*t*t*p2.x + t*t*t*p3.x;
        double y = u*u*u*p0.y + 3*u*u*t*p1.y + 3*u*t*t*p2.y + t*t*t*p3.y;
        
        // double x = u*u*u*u*u*p0.x + 5*u*u*u*u*t*p1.x + 10*u*u*u*t*t*p2.x + 10*u*u*t*t*t*p3.x + 5*u*t*t*t*t*p4.x + t*t*t*t*t*p5.x;
        // double y = u*u*u*u*u*p0.y + 5*u*u*u*u*t*p1.y + 10*u*u*u*t*t*p2.y + 10*u*u*t*t*t*p3.y + 5*u*t*t*t*t*p4.y + t*t*t*t*t*p5.y;

        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        points.push_back(point);
    }

    return points;
}


void
DisplayBezier::displaySingleBezier()
{
    auto msg = visualization_msgs::msg::Marker();
    msg.header.frame_id = "map"; // 设置坐标系
    msg.header.stamp = this->get_clock()->now();
    msg.ns = "bezier";
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

    // 计算bezier全部点位
    std::vector<geometry_msgs::msg::Point> bezier_points;
    bezier_points.clear();
    // 三阶bezier控制点
    geometry_msgs::msg::Point p0, p1, p2, p3;
    p0.x = 0;
    p0.y = 0;
    p1.x = 3.0;
    p1.y = 0;
    p2.x = 0;
    p2.y = 3.0;
    p3.x = 3.0;
    p3.y = 3.0;
    bezier_points = calculateBezierPoints(p0, p1, p2, p3, 100);
    for (size_t i = 0; i < bezier_points.size(); i++)
    {
        msg.points.push_back(bezier_points.at(i));
    }
    // 设置持续时间
    msg.lifetime = rclcpp::Duration::from_seconds(2); // 持续时间为1秒

    RCLCPP_INFO(this->get_logger(), "Publishing message single bezier");
    marker_pub_->publish(msg);
}

void
DisplayBezier::displayMultipleBeziers()
{
    auto msg = visualization_msgs::msg::Marker();

    RCLCPP_INFO(this->get_logger(), "Publishing message multiple beziers");
    marker_pub_->publish(msg);
}





}  // namespace ros2_rviz2


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ros2_rviz2::DisplayBezier>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}