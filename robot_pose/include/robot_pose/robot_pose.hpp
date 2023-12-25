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


#ifndef ROBOT_POSE__ROBOT_POSE_HPP_
#define ROBOT_POSE__ROBOT_POSE_HPP_

#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace robot_pose
{
class RobotPose : public rclcpp::Node
{
public:
    RobotPose();
    ~RobotPose();

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::string target_frame_;
    
    std::string source_frame_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;

    void timerCallback();





};
}  // namespace robot_pose
#endif  // ROBOT_POSE__ROBOT_POSE_HPP_
