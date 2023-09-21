cmake_minimum_required(VERSION 3.10)
project(bonxai_ros)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)

  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")

  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-fno-omit-frame-pointer -g")

#---- Add the subdirectory cmake ----
set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(PCL REQUIRED COMPONENTS common io)

find_package(benchmark QUIET)
find_package(octomap QUIET)
find_package(LZ4 QUIET)
find_package(Eigen3 QUIET)

#---- Use Address sanitizer if compiled in Debug mode ----
option(BONXAI_SANITIZE "Add address sanitizer when compiling in Debug mode" ON)

if(BONXAI_SANITIZE)
    set (CMAKE_CXX_DEBUG_FLAGS "${CMAKE_CXX_DEBUG_FLAGS}  -fsanitize=address")
    set (CMAKE_LINKER_DEBUG_FLAGS "${CMAKE_LINKER_DEBUG_FLAGS}  -fsanitize=address")
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer)
endif()

####################################################
add_subdirectory(bonxai_core)
add_subdirectory(bonxai_map)
####################################################

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
find_package(std_srvs REQUIRED)


include_directories(bonxai_ros/include 3rdparty)

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

add_executable(bonxai_server_node bonxai_ros/src/bonxai_server.cpp)
target_link_libraries(bonxai_server_node bonxai_map)
ament_target_dependencies(bonxai_server_node 
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
                          message_filters
                          std_srvs
                          )

add_executable(kitti_pub_node bonxai_ros/src/kitti_pub_node.cpp)
target_link_libraries(kitti_pub_node bonxai_map)
ament_target_dependencies(kitti_pub_node 
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
                          message_filters
                          std_srvs
                          )

install(TARGETS
  bonxai_server_node
  kitti_pub_node
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY bonxai_ros/launch 
            bonxai_ros/rviz 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()