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


#ifndef ROS2_TRAJECTORY_SAVER__TRAJECTORY_SAVER_HPP_
#define ROS2_TRAJECTORY_SAVER__TRAJECTORY_SAVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <fstream>
#include <vector>
#include <string>

namespace trajectory_saver
{
class TrajectorySaver : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Trajectory Saver object
     * 
     */
    TrajectorySaver();
    /**
     * @brief Destroy the Trajectory Saver object
     * 
     */
    ~TrajectorySaver();

private:
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_list_sub_;
    void trajectoryListSubCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void saveToFile();
    std::map<std::string, std::vector<std::tuple<float, float, float>>> trajectories_;




};
}  // namespace trajectory_saver
#endif  // ROS2_TRAJECTORY_SAVER__TRAJECTORY_SAVER_HPP_
