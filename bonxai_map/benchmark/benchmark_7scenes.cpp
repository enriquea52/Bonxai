#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include <set>
#include <filesystem>
#include <chrono>
#include <Eigen/Geometry>
#include <fstream>
#include <string>

#include "bonxai_map/pcl_utils.hpp"
#include "bonxai_map/probabilistic_map.hpp"
#include "cxxopt/cxxopts.hpp"


#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace fs = std::filesystem;

long ToMsec(std::chrono::system_clock::duration const& dur)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
}

bool ends_with(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

void file2eigen(std::string path, Eigen::Matrix4d& pose)
{

    std::fstream file;
    std::string word, t, q, filename;
 
    // filename of the file
    filename = path;
 
    // opening file
    file.open(filename.c_str());
 
    // extracting words from the file
    int i = 0;
    while (file >> word)
    {
        double number = std::stod(word); 

        pose(i/4, i%4) = number;

        // displaying content
        i++;
    }
}

 void ReadData(const std::string& dataset, std::vector<std::string>& poses, 
                                           std::vector<std::string>& color_imgs, 
                                           std::vector<std::string>& depth_imgs)
{
  if(!std::filesystem::exists(dataset))
  {
    throw std::runtime_error("Calibration file not found");
  }
  
  for (const auto & entry : fs::directory_iterator(dataset))
  {
    if(ends_with(entry.path(), ".txt"))
    {
        poses.push_back(entry.path());

    }
    else if(ends_with(entry.path(), ".color.png"))
    {
        color_imgs.push_back(entry.path());

    }
    else if(ends_with(entry.path(), ".depth.png"))
    {
        depth_imgs.push_back(entry.path());
    }

  }

  std::sort(poses.begin(), poses.end());
  std::sort(color_imgs.begin(), color_imgs.end());
  std::sort(depth_imgs.begin(), depth_imgs.end());

}


std::vector<Eigen::Isometry3d> ReadPoses(const std::vector<std::string>& poses_files)
{

  std::vector<Eigen::Isometry3d> poses;

  for(const auto& pose_file: poses_files)
  {
    Eigen::Matrix4d pose;
    Eigen::Matrix3d rot;
    Eigen::Vector3d pos;

    file2eigen(pose_file, pose);

    pos = pose.block(0, 3, 3, 1);
    rot = pose.block(0, 0, 3, 3);

    poses.emplace_back(Eigen::Translation3d(pos) * Eigen::Quaterniond(rot));

  }

  return poses;
}


template <class PointCloudT, typename Real = double>
void ReadPointcloud(const std::string& rgb_file,
                    const std::string& depth_file,
                    const Eigen::Isometry3d& transform,
                    PointCloudT& points)
{

  // 7-Scenes Dataset's Kinect Intrinsic Parameters
  double fx = 585.0, fy = 585.0,
         cx = 320.0, cy = 240.0,
         factor = 1000.0;

  // 24-bit RGB image
  cv::Mat color_img = cv::imread(rgb_file, cv::IMREAD_COLOR);

  // 16-bit pixel image, the depth measurement is done in millimeters
  cv::Mat depth_img = cv::imread(depth_file, -1); 

  points.clear();
  for(int i = 0; i < depth_img.rows; i++)
  {
    for(int j = 0; j < depth_img.cols; j++)
    {

      uint16_t depth_mm = depth_img.at<uint16_t>(i,j);

      if (depth_mm == 65535 || depth_mm == 0)
      {
        continue;
      }
      else
      {
        Eigen::Vector3f point; // must be float

        point.z() = depth_mm/factor;
        point.x() = (j - cx)*point.z() / fx;
        point.y() = (i - cy)*point.z() / fy;

        // Add color dimensions when available
        // cv::Vec3b intensity = img.at<cv::Vec3b>(i, j);
        // uchar blue = intensity.val[0];
        // uchar green = intensity.val[1];
        // uchar red = intensity.val[2];

        // apply transform first
        const Eigen::Vector3d p = transform * point.cast<double>();
        points.push_back( {Real(p.x()), Real(p.y()), Real(p.z())} );
      }
    }
  }
}

//-----------------------------------------------------------
//-----------------------------------------------------------

int main(int argc, char** argv)
{
  cxxopts::Options options("7-Scenes benchmarks", "octomap VS Bonxai");

  options.add_options()
      ("dataset", "Dataset Sequence (seq-XX, where XX is the sequence number) folder path", cxxopts::value<std::string>())
      ("max_files", "Max files to process", cxxopts::value<size_t>()->default_value("1000"))
      ("max_dist", "Max distance in meters", cxxopts::value<double>()->default_value("25.0"))
      ("voxel_size", "Voxel size in meters", cxxopts::value<double>()->default_value("0.2"))
      ("skip_octree", "Do not compute the octree", cxxopts::value<bool>()->default_value("false"))
      ;


  const auto options_res = options.parse(argc, argv);

  const auto seq_path = options_res["dataset"].as<std::string>();
  auto voxel_size = options_res["voxel_size"].as<double>();
  const auto max_distance = options_res["max_dist"].as<double>();
  auto max_pointclouds = options_res["max_files"].as<size_t>();
  const auto skip_octree = options_res["skip_octree"].as<bool>();

  if (options_res.count("pc") || seq_path.empty() )
  {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  // Containers for storing the data to be processed
  std::vector<std::string> poses_files, color_imgs_files, depth_imgs_files;
  
  // Get all the file names for each type of data
  ReadData(seq_path, poses_files, color_imgs_files, depth_imgs_files); 

  // Get std::vector of Isometry3d poses
  const auto poses = ReadPoses(poses_files);

  long total_time_octree = 0;
  long total_time_bonxai = 0;

  max_pointclouds = std::min(max_pointclouds, poses_files.size());

  color_imgs_files.resize(max_pointclouds);
  depth_imgs_files.resize(max_pointclouds);

  //---------------------------------------
  octomap::OcTree octree(voxel_size);
  Bonxai::ProbabilisticMap bonxai_map(voxel_size);

  for (size_t count = 0; count < max_pointclouds; count++)
  {
    const auto& color_img_filename = color_imgs_files[count];
    const auto& depth_img_filename = depth_imgs_files[count];
    const Eigen::Isometry3d transform = poses[count];

    const Eigen::Vector3d origin(transform.translation());
    std::vector<Eigen::Vector3d> pointcloud;
    ReadPointcloud(color_img_filename, depth_img_filename, transform, pointcloud);

    const auto t1 = std::chrono::system_clock::now();

    bonxai_map.insertPointCloud(pointcloud, origin, max_distance);

    const auto diff = ToMsec(std::chrono::system_clock::now() - t1);
    std::cout << "[" << depth_img_filename << "] bonxai time: " << diff << " ms" << std::endl;
    total_time_bonxai += diff;
  }

  printf("average time bonxai: %.1f ms\n",
         double(total_time_bonxai) / double(max_pointclouds));

  std::vector<Eigen::Vector3d> bonxai_result;
  bonxai_map.getOccupiedVoxels(bonxai_result);
  std::vector<Bonxai::CoordT> free_coords;
  bonxai_map.getFreeVoxels(free_coords);
  std::cout << "free cells: " << free_coords.size() << std::endl;
  std::cout << "bonxai_result.pcd contains " << bonxai_result.size()
            << " points\n" << std::endl;

  if(!bonxai_result.empty())
  {
    Bonxai::WritePointsFromPCD("bonxai_result.pcd", bonxai_result);
  }

  std::cout << "//-----------------------------------------\n";
  //----------------------------------------------------

  if(!skip_octree)
  {
    for (size_t count = 0; count < max_pointclouds; count++)
    {
      const auto& color_img_filename = color_imgs_files[count];
      const auto& depth_img_filename = depth_imgs_files[count];
      const Eigen::Isometry3d transform = poses[count];

      const octomap::point3d origin(transform.translation().x(),
                                    transform.translation().y(),
                                    transform.translation().z());
      octomap::Pointcloud pointcloud;
      ReadPointcloud<octomap::Pointcloud, float>(color_img_filename, depth_img_filename, transform, pointcloud);

      const auto t1 = std::chrono::system_clock::now();

      octree.insertPointCloud(pointcloud, origin, max_distance, false, true);

      const auto diff = ToMsec(std::chrono::system_clock::now() - t1);
      std::cout << "[" << depth_img_filename << "] octree time: " << diff << " ms" << std::endl;
      total_time_octree += diff;
    }

    printf("average time octree: %.1f ms\n",
           double(total_time_octree) / double(max_pointclouds));

    std::vector<Eigen::Vector3d> octree_result;
    int free_cell_count = 0;
    for (auto it = octree.begin(), end = octree.end(); it != end; ++it)
    {
      if (octree.isNodeOccupied(*it)){
        octree_result.push_back( {it.getX(), it.getY(), it.getZ()} );
      }
      else {
        free_cell_count++;
      }
    }
    std::cout << "free cells: " << free_cell_count << std::endl;
    std::cout << "octomap_result.pcd contains " << octree_result.size()
              << " points\n" << std::endl;
    if(!octree_result.empty())
    {
      Bonxai::WritePointsFromPCD("octomap_result.pcd", octree_result);
    }

    std::cout << "\nMemory used. octree: " << int(octree.memoryUsage() / 1000)
              << " Kb / bonxai: " << int(bonxai_map.grid().memUsage() / 1000)
              << " Kb" << std::endl;
    printf("speed up: %.1f X\n", double(total_time_octree) / double(total_time_bonxai));
  }

  return 0;
}
