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


#include "ros2_nav_to_poses/nav_to_poses.hpp"
#include "tf2/utils.h"
#include "nav2_util/geometry_utils.hpp"

namespace nav_to_poses
{

NavToPoses::NavToPoses()
: Node("ros2_nav_to_poses"),
number_(0)
{
     nav_to_poses_cli_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this,
          "navigate_to_pose");
     
     timer_ = this->create_wall_timer(
          std::chrono::milliseconds(500),
          std::bind(&NavToPoses::timerCallback, this));
}  

NavToPoses::~NavToPoses()
{
     nav_to_poses_cli_->async_cancel_all_goals();
}

void
NavToPoses::initPoses(std::vector<geometry_msgs::msg::Pose> poses)
{
     poses_.clear();
     for (size_t i = 0; i < poses.size(); i++)
     {
          poses_.push_back(poses[i]);
     }
}

void
NavToPoses::timerCallback()
{
     static bool is_send_first_goal = false;
     if (!is_send_first_goal)
     {
          navToPosesHandle(poses_[0]);
          is_send_first_goal = true;
     }
}

void
NavToPoses::navToPosesHandle(geometry_msgs::msg::Pose & pose)
{
     while (!nav_to_poses_cli_->wait_for_action_server(std::chrono::seconds(1))) 
     {
          if (!rclcpp::ok()) 
          {
               RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the action server.");
               return;
          }
          RCLCPP_INFO(this->get_logger(), "Action server not available, waiting again...");
     }

     using std::placeholders::_1;
     using std::placeholders::_2;
     auto goal = nav2_msgs::action::NavigateToPose::Goal();
     goal.behavior_tree = "";
     goal.pose.header.frame_id = "map";
     goal.pose.header.stamp = rclcpp::Clock().now();
     goal.pose.pose = pose;
     auto send_goal_options =
          rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
     send_goal_options.goal_response_callback =
          std::bind(&NavToPoses::goal_response_callback, this, _1);
     send_goal_options.feedback_callback =
          std::bind(&NavToPoses::feedback_callback, this, _1, _2);
     send_goal_options.result_callback =
          std::bind(&NavToPoses::result_callback, this, _1);
     RCLCPP_INFO(this->get_logger(), "Sending goal: [x %lf, y %lf]", goal.pose.pose.position.x, goal.pose.pose.position.y);
     nav_to_poses_cli_->async_send_goal(goal, send_goal_options);
}

void
NavToPoses::goal_response_callback(const ClientGoalHandle::SharedPtr & future)
{
     auto goal_handle = future.get();
     if (!goal_handle) 
     {
          RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
     } 
     else 
     {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
     }
}

void
NavToPoses::feedback_callback(
     ClientGoalHandle::SharedPtr,
     const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
     double x, y, yaw, yaw_degress;
     x = feedback->current_pose.pose.position.x;
     y = feedback->current_pose.pose.position.y;
     yaw = tf2::getYaw(feedback->current_pose.pose.orientation);
     yaw_degress = yaw / M_PI * 180.0;
     RCLCPP_INFO(this->get_logger(),
          "robot pose: [x %.4lf, y %.4lf, yaw %.4lf(%.4lf°)]", 
          x, y, yaw, yaw_degress);
}

void
NavToPoses::result_callback(const ClientGoalHandle::WrappedResult & result)
{
     switch (result.code) 
     {
          case rclcpp_action::ResultCode::SUCCEEDED:
          {
               RCLCPP_INFO(this->get_logger(), "Server successfully executed goal");
               // delay some time
               rclcpp::sleep_for(std::chrono::milliseconds(1000));
               number_ ++;
               if (number_ == poses_.size())
               {
                    number_ = 0;
               }
               navToPosesHandle(poses_[number_]);
          }break;
          case rclcpp_action::ResultCode::ABORTED:
               RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
               return;
          case rclcpp_action::ResultCode::CANCELED:
               RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
               return;
          default:
               RCLCPP_ERROR(this->get_logger(), "Unknown result code");
               return;
     }
}




} // namespace nav_to_poses
