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

namespace nav_to_poses
{

NavToPoses::NavToPoses()
: Node("ros2_nav_to_poses"),
is_pose_arrived_(false)
{
     nav_to_poses_cli_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this,
          "navigate_to_pose");
     
     timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&NavToPoses::timerCallback, this));

     initParameters();

}  

NavToPoses::~NavToPoses()
{
     // 
}

void
NavToPoses::timerCallback()
{
     static bool is_pose_0 = true;
     static bool is_sended = false;
     if (!is_pose_arrived_)
     {
          if (is_pose_0)
          {
               if (!is_sended)
               {
                    is_sended = true;
                    navToPosesHandle(poses_[0]);
               }
          }
          else
          {
               if (!is_sended)
               {
                    is_sended = true;
                    navToPosesHandle(poses_[1]);
               }
          }
          
     }
     else
     {
          is_pose_arrived_ = false;
          is_pose_0 = !is_pose_0;
          is_sended = false;
     }
     
}

void
NavToPoses::initParameters()
{
     poses_.clear();
     geometry_msgs::msg::Pose pose;
     pose.position.x = 0.0;
     pose.position.y = 8.0;
     pose.position.z = 0.0;
     pose.orientation.x = 0.0;
     pose.orientation.y = 0.0;
     pose.orientation.z = 0.0;
     pose.orientation.w = 1.0;
     poses_.push_back(pose);

     pose.position.x = -8.0;
     pose.position.y = 0.0;
     pose.position.z = 0.0;
     pose.orientation.x = 0.0;
     pose.orientation.y = 0.0;
     pose.orientation.z = 0.0;
     pose.orientation.w = 1.0;
     poses_.push_back(pose);
}

void
NavToPoses::navToPosesHandle(geometry_msgs::msg::Pose & pose)
{
     while (!nav_to_poses_cli_->wait_for_action_server(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
               RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the action server.");
               return;
          }
          RCLCPP_INFO(this->get_logger(), "Action server not available, waiting again...");
     }

     using std::placeholders::_1;
     using std::placeholders::_2;
     auto goal = nav2_msgs::action::NavigateToPose::Goal();
     // set value
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
     if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
     } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
     }
}

void
NavToPoses::feedback_callback(
     ClientGoalHandle::SharedPtr,
     const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
     RCLCPP_INFO(this->get_logger(), "robot pose: [x %lf, y %lf]", 
          feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y);
}

void
NavToPoses::result_callback(const ClientGoalHandle::WrappedResult & result)
{
     switch (result.code) 
     {
          case rclcpp_action::ResultCode::SUCCEEDED:
          {
               is_pose_arrived_ = true;
               RCLCPP_INFO(this->get_logger(), "Server successfully executed goal");
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
