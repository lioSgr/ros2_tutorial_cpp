cmake_minimum_required(VERSION 3.8)
project(ros2_rviz2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  visualization_msgs
  geometry_msgs
)

# ----
add_library(display_points_lib SHARED src/display_points.cpp)
ament_target_dependencies(display_points_lib ${dependencies})

add_executable(display_points src/display_points.cpp)
ament_target_dependencies(display_points ${dependencies})

target_link_libraries(display_points display_points_lib)
# ----
add_library(display_lines_lib SHARED src/display_lines.cpp)
ament_target_dependencies(display_lines_lib ${dependencies})

add_executable(display_lines src/display_lines.cpp)
ament_target_dependencies(display_lines ${dependencies})

target_link_libraries(display_lines display_lines_lib)
# ----
add_library(display_bezier_lib SHARED src/display_bezier.cpp)
ament_target_dependencies(display_bezier_lib ${dependencies})

add_executable(display_bezier src/display_bezier.cpp)
ament_target_dependencies(display_bezier ${dependencies})

target_link_libraries(display_bezier display_bezier_lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS display_points_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS display_points
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS display_lines_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS display_lines
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS display_bezier_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS display_bezier
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_libraries(display_points_lib)
ament_export_libraries(display_lines_lib)
ament_export_libraries(display_bezier_lib)
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_package()
