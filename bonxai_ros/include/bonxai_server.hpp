#ifndef BONXAI_SERVER__BONXAI_SERVER_HPP_
#define BONXAI_SERVER__BONXAI_SERVER_HPP_

#include "bonxai_map/pcl_utils.hpp"
#include "bonxai_map/pointcloud.hpp"
#include "bonxai/bonxai.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"


namespace bonxai_server
{

  using nav_msgs::msg::MapMetaData;
  using nav_msgs::msg::OccupancyGrid;
  using sensor_msgs::msg::PointCloud2;
  using std_msgs::msg::ColorRGBA;
  using visualization_msgs::msg::MarkerArray;


  class BonxaiServer : public rclcpp::Node
  {
    public:
    #ifdef COLOR_OCTOMAP_SERVER
      using PCLPoint = pcl::PointXYZRGB;
      using PCLPointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
      using OcTreeT = octomap::ColorOcTree;
    #else
      using PCLPoint = pcl::PointXYZ;
      using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;
      // using OcTreeT = octomap::OcTree;
    #endif
      // using ResetSrv = std_srvs::srv::Empty;

      explicit BonxaiServer(/*const rclcpp::NodeOptions & node_options*/);

      virtual void insertCloudCallback(const PointCloud2::ConstSharedPtr cloud);

      protected:

      rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
      // rclcpp::Publisher<Octomap>::SharedPtr binary_map_pub_;
      // rclcpp::Publisher<Octomap>::SharedPtr full_map_pub_;
      rclcpp::Publisher<PointCloud2>::SharedPtr point_cloud_pub_;
      rclcpp::Publisher<OccupancyGrid>::SharedPtr map_pub_;
      rclcpp::Publisher<MarkerArray>::SharedPtr fmarker_pub_;
      message_filters::Subscriber<PointCloud2> point_cloud_sub_;
      std::shared_ptr<tf2_ros::MessageFilter<PointCloud2>> tf_point_cloud_sub_;
      // rclcpp::Service<OctomapSrv>::SharedPtr octomap_binary_srv_;
      // rclcpp::Service<OctomapSrv>::SharedPtr octomap_full_srv_;
      // rclcpp::Service<BBoxSrv>::SharedPtr clear_bbox_srv_;
      // rclcpp::Service<ResetSrv>::SharedPtr reset_srv_;
      std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

      // std::unique_ptr<OcTreeT> octree_;
      // octomap::KeyRay key_ray_;  // temp storage for ray casting
      // octomap::OcTreeKey update_bbox_min_;
      // octomap::OcTreeKey update_bbox_max_;


      double max_range_;
      std::string world_frame_id_;  // the map frame
      std::string base_frame_id_;  // base of the robot for ground plane filtering
      bool use_height_map_;
      ColorRGBA color_;
      ColorRGBA color_free_;
      double color_factor_;

      bool latched_topics_;
      bool publish_free_space_;

      double res_;
      size_t tree_depth_;
      size_t max_tree_depth_;

      double point_cloud_min_x_;
      double point_cloud_max_x_;
      double point_cloud_min_y_;
      double point_cloud_max_y_;
      double point_cloud_min_z_;
      double point_cloud_max_z_;
      double occupancy_min_z_;
      double occupancy_max_z_;
      double min_x_size_;
      double min_y_size_;
      bool filter_speckles_;

      bool filter_ground_plane_;
      double ground_filter_distance_;
      double ground_filter_angle_;
      double ground_filter_plane_distance_;

      bool compress_map_;

      bool init_config_;

      // downprojected 2D map:
      bool incremental_2D_projection_;
      OccupancyGrid gridmap_;
      bool publish_2d_map_;
      bool map_origin_changed;
      // octomap::OcTreeKey padded_min_key_;
      unsigned multires_2d_scale_;
      bool project_complete_map_;
      bool use_colored_map_;

  };
}

#endif // BONXAI_SERVER__BONXAI_SERVER_HPP_