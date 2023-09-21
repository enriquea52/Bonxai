#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <set>
#include <filesystem>
#include <chrono>
#include <Eigen/Geometry>

#include "bonxai_map/pcl_utils.hpp"
#include "bonxai_map/probabilistic_map.hpp"
#include "cxxopt/cxxopts.hpp"

#include "iostream"
#include <stdio.h>
#include <iterator>
#include <algorithm>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

#include <chrono>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std;


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("kitti_pub_node"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_in", 10);

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


  private:

    size_t count_;
};

namespace fs = std::filesystem;

long ToMsec(std::chrono::system_clock::duration const& dur)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
}

Eigen::Isometry3f ReadCalibration(const std::string& calibration_file)
{
  if(!std::filesystem::exists(calibration_file))
  {
    throw std::runtime_error("Calibration file not found");
  }

  std::ifstream input(calibration_file);
  std::string line;

  std::string header;

  Eigen::Isometry3f calib;

  while(std::getline(input, line))
  {
    Eigen::Matrix3f rot;
    Eigen::Vector3f pos;

    std::istringstream ss(line);
    ss  >> header
        >> rot(0,0) >> rot(0,1) >> rot(0,2)  >> pos(0)
        >> rot(1,0) >> rot(1,1) >> rot(1,2)  >> pos(1)
        >> rot(2,0) >> rot(2,1) >> rot(2,2)  >> pos(2);

    if(header == "Tr:")
    {
      calib = Eigen::Translation3f(pos) * Eigen::Quaternionf(rot);
      return calib;
    }
  }

  throw std::runtime_error("Calibration value not found");
}

std::vector<Eigen::Isometry3f> ReadPoses(const std::string& poses_file)
{
  if(!std::filesystem::exists(poses_file))
  {
    throw std::runtime_error("Calibration file not found");
  }

  std::ifstream input(poses_file);
  std::string line;

  std::vector<Eigen::Isometry3f> poses;


  while(std::getline(input, line))
  {
    Eigen::Matrix3f rot;
    Eigen::Vector3f pos;

    std::istringstream ss(line);
    ss  >> rot(0,0) >> rot(0,1) >> rot(0,2)  >> pos(0)
        >> rot(1,0) >> rot(1,1) >> rot(1,2)  >> pos(1)
        >> rot(2,0) >> rot(2,1) >> rot(2,2)  >> pos(2);

    poses.emplace_back(Eigen::Translation3f(pos) * Eigen::Quaternionf(rot));
  }
  return poses;
}

template <class PointCloudT>
void ReadPointcloud(const std::string& cloud_file,
                    PointCloudT& points)
{
  std::fstream input(cloud_file, std::ios::in | std::ios::binary);

  points.clear();
  while (input.good() && !input.eof())
  {
    Eigen::Vector3f point;
    float intensity;
    input.read((char *) &point.x(), sizeof(float));
    input.read((char *) &point.y(), sizeof(float));
    input.read((char *) &point.z(), sizeof(float));
    input.read((char *) &intensity, sizeof(float));

    // apply transform first
    const Eigen::Vector3f p = point;
    points.push_back( {p.x(), p.y(), p.z()} );
  }
}