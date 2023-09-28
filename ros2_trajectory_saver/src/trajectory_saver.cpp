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

#include "ros2_trajectory_saver/trajectory_saver.hpp"

using std::placeholders::_1;

namespace trajectory_saver
{

TrajectorySaver::TrajectorySaver()
: Node("trajectory_saver")
{
     RCLCPP_INFO(this->get_logger(), "Construct a new Trajectory Saver object");
     trajectory_list_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
          "trajectory_node_list",
          10,
          std::bind(&TrajectorySaver::trajectoryListSubCallback, this, _1));
}

TrajectorySaver::~TrajectorySaver()
{
     saveToFile();
     RCLCPP_INFO(this->get_logger(), "Destroy the Trajectory Saver object");
}

void
TrajectorySaver::trajectoryListSubCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
     for (const auto &marker : msg->markers)
     {
          std::string key = marker.ns + "_" + std::to_string(marker.id);
          
          // 如果这是一个新的轨迹，初始化它
          if(trajectories_.find(key) == trajectories_.end())
          {
               trajectories_[key] = std::vector<std::tuple<float, float, float>>();
          }

          // 将marker中的points的每一个位置添加到轨迹中
          for (const auto& point : marker.points)
          {
               trajectories_[key].emplace_back(point.x, point.y, point.z);
          }
     }
   
}

void
TrajectorySaver::saveToFile()
{
     int index = 0;
     for (const auto &pair : trajectories_)
     {
          std::ofstream outfile("trajectory_" + std::to_string(index) + ".csv");
          for (const auto &point : pair.second)
          {
               outfile << std::get<0>(point) << "," << std::get<1>(point) << "," << std::get<2>(point) << "\n";
          }
          outfile.close();
          index++;
     }
}



    
} // namespace TrajectorySaver
