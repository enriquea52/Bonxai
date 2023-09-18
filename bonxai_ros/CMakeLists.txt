cmake_minimum_required(VERSION 3.8)
project(bonxai_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)


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

add_executable(bonxai_server src/bonxai_server.cpp)
target_link_libraries(bonxai_server bonxai_map)
ament_target_dependencies(bonxai_server 
                          rclcpp 
                          rclcpp_components 
                          nav_msgs 
                          sensor_msgs 
                          tf2 
                          tf2_eigen 
                          tf2_geometry_msgs 
                          tf2_ros 
                          visualization_msgs
                          PCL
                          pcl_conversions
                          message_filters)


install(TARGETS
  bonxai_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()