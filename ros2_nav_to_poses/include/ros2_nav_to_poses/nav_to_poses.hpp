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


#ifndef ROS2_NAV_TO_POSES__NAV_TO_POSES_HPP_
#define ROS2_NAV_TO_POSES__NAV_TO_POSES_HPP_

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace nav_to_poses
{
class NavToPoses : public rclcpp::Node
{
public:
    NavToPoses();
    ~NavToPoses();

private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_poses_cli_;

    rclcpp::TimerBase::SharedPtr timer_;

    // 目标点
    std::vector<geometry_msgs::msg::Pose> poses_;

    bool is_pose_arrived_;

    void timerCallback();

    void navToPosesHandle(geometry_msgs::msg::Pose & pose);

    void initParameters();

    using ClientGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
    void goal_response_callback(const ClientGoalHandle::SharedPtr & future);
    
    void feedback_callback(
        ClientGoalHandle::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
    
    void result_callback(const ClientGoalHandle::WrappedResult & result);




};
}  // namespace nav_to_poses
#endif  // ROS2_NAV_TO_POSES__NAV_TO_POSES_HPP_
