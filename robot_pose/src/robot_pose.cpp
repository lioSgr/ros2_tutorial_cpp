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

#include "robot_pose/robot_pose.hpp"
#include "tf2/utils.h"

namespace robot_pose
{

RobotPose::RobotPose()
: Node("robot_pose"),
tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
target_frame_("map"),
source_frame_("base_footprint")
{
    RCLCPP_INFO(this->get_logger(), "Construct a new Robot Pose object");

    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>(
        "pose_2d",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&RobotPose::timerCallback, this));

}

RobotPose::~RobotPose()
{
    RCLCPP_INFO(this->get_logger(), "Destroy the Robot Pose object");
}

void
RobotPose::timerCallback()
{
    double x, y, roll, pitch, yaw, yaw_degrees;
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);

        x = transform_stamped.transform.translation.x;
        y = transform_stamped.transform.translation.y;
        yaw = tf2::getYaw(transform_stamped.transform.rotation);
        // tf2::Quaternion quat;
        // tf2::fromMsg(transform_stamped.transform.rotation, quat);
        // tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        yaw_degrees = yaw / M_PI * 180.0f;
        auto msg = geometry_msgs::msg::Pose2D();
        msg.x = x;
        msg.y = y;
        msg.theta = yaw;
        pose_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "[x %.4lf, y %.4lf, yaw %.4lf (%.4lf°)]", x, y, yaw, yaw_degrees);
    } catch(tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    }

}


} // namespace robot_pose
