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
#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "ros2_nav_to_poses/nav_to_poses.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/utils.h"
#include "nav2_util/geometry_utils.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav_to_poses::NavToPoses>();
    RCLCPP_INFO(node->get_logger(), "Executable name: %s", argv[0]);
    // 读取命令行参数
    std::vector<geometry_msgs::msg::Pose> poses;
    if (((argc - 1) % 3) != 0)
    {
        std::cout << "Parameters size is: " << argc << std::endl;
        std::cout << "The parameter is illegal. The parameter format is value 1 value 2 value 3." << std::endl;
        return -1;
    }
    std::size_t size = (argc - 1) / 3;
    poses.resize(size);
    for (size_t i = 0; i < poses.size(); i++)
    {
        poses[i].position.x = std::stof(argv[i * 3 + 1]);
        poses[i].position.y = std::stof(argv[i * 3 + 2]);
        poses[i].orientation = nav2_util::geometry_utils::orientationAroundZAxis(std::stof(argv[i * 3 + 3]));
    }
    node->initPoses(poses);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
