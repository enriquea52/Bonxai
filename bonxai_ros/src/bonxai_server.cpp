#include "bonxai_server.hpp"


namespace bonxai_server
{
  BonxaiServer::BonxaiServer(/*const rclcpp::NodeOptions & node_options*/)
  : Node("octomap_server"/*, node_options*/)
  {
      using std::chrono_literals::operator""s;
      point_cloud_sub_.subscribe(this, "cloud_in", rmw_qos_profile_sensor_data);
      tf_point_cloud_sub_ = std::make_shared<tf2_ros::MessageFilter<PointCloud2>>(
      point_cloud_sub_, *tf2_buffer_, world_frame_id_, 5, this->get_node_logging_interface(),
      this->get_node_clock_interface(), 5s);
      tf_point_cloud_sub_->registerCallback(&BonxaiServer::insertCloudCallback, this);

  }


  void BonxaiServer::insertCloudCallback(const PointCloud2::ConstSharedPtr cloud)
  {
    // To Fill
    std::cout << "Pointcloud Received" << std::endl;
    return;
    // const auto start_time = rclcpp::Clock{}.now();

    // //
    // // ground filtering in base frame
    // //
    // PCLPointCloud pc;  // input cloud for filtering and ground-detection
    // pcl::fromROSMsg(*cloud, pc);

    // geometry_msgs::msg::TransformStamped sensor_to_world_transform_stamped;
    // try {
    //   sensor_to_world_transform_stamped = tf2_buffer_->lookupTransform(
    //     world_frame_id_, cloud->header.frame_id, cloud->header.stamp,
    //     rclcpp::Duration::from_seconds(1.0));
    // } catch (const tf2::TransformException & ex) {
    //   RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    //   return;
    // }

    // Eigen::Matrix4f sensor_to_world =
    //   tf2::transformToEigen(sensor_to_world_transform_stamped.transform).matrix().cast<float>();

    // // set up filter for height range, also removes NANs:
    // pcl::PassThrough<PCLPoint> pass_x;
    // pass_x.setFilterFieldName("x");
    // pass_x.setFilterLimits(point_cloud_min_x_, point_cloud_max_x_);
    // pcl::PassThrough<PCLPoint> pass_y;
    // pass_y.setFilterFieldName("y");
    // pass_y.setFilterLimits(point_cloud_min_y_, point_cloud_max_y_);
    // pcl::PassThrough<PCLPoint> pass_z;
    // pass_z.setFilterFieldName("z");
    // pass_z.setFilterLimits(point_cloud_min_z_, point_cloud_max_z_);

    // PCLPointCloud pc_ground;  // segmented ground plane
    // PCLPointCloud pc_nonground;  // everything else

    // if (filter_ground_plane_) {
    //   geometry_msgs::msg::TransformStamped sensor_to_base_transform_stamped;
    //   geometry_msgs::msg::TransformStamped base_to_world_transform_stamped;
    //   try {
    //     tf2_buffer_->canTransform(
    //       base_frame_id_, cloud->header.frame_id, cloud->header.stamp,
    //       rclcpp::Duration::from_seconds(0.2));
    //     sensor_to_base_transform_stamped = tf2_buffer_->lookupTransform(
    //       base_frame_id_, cloud->header.frame_id, cloud->header.stamp,
    //       rclcpp::Duration::from_seconds(1.0));
    //     base_to_world_transform_stamped = tf2_buffer_->lookupTransform(
    //       world_frame_id_, base_frame_id_, cloud->header.stamp,
    //       rclcpp::Duration::from_seconds(1.0));
    //   } catch (const tf2::TransformException & ex) {
    //     RCLCPP_ERROR_STREAM(
    //       get_logger(),
    //       "Transform error for ground plane filter: " << ex.what() << ", quitting callback.\n"
    //         "You need to set the base_frame_id or disable filter_ground.");
    //   }


    //   Eigen::Matrix4f sensor_to_base =
    //     tf2::transformToEigen(sensor_to_base_transform_stamped.transform).matrix().cast<float>();
    //   Eigen::Matrix4f base_to_world =
    //     tf2::transformToEigen(base_to_world_transform_stamped.transform).matrix().cast<float>();

    //   // transform pointcloud from sensor frame to fixed robot frame
    //   pcl::transformPointCloud(pc, pc, sensor_to_base);
    //   pass_x.setInputCloud(pc.makeShared());
    //   pass_x.filter(pc);
    //   pass_y.setInputCloud(pc.makeShared());
    //   pass_y.filter(pc);
    //   pass_z.setInputCloud(pc.makeShared());
    //   pass_z.filter(pc);
    //   filterGroundPlane(pc, pc_ground, pc_nonground);

    //   // transform clouds to world frame for insertion
    //   pcl::transformPointCloud(pc_ground, pc_ground, base_to_world);
    //   pcl::transformPointCloud(pc_nonground, pc_nonground, base_to_world);
    // } else {
    //   // directly transform to map frame:
    //   pcl::transformPointCloud(pc, pc, sensor_to_world);

    //   // just filter height range:
    //   pass_x.setInputCloud(pc.makeShared());
    //   pass_x.filter(pc);
    //   pass_y.setInputCloud(pc.makeShared());
    //   pass_y.filter(pc);
    //   pass_z.setInputCloud(pc.makeShared());
    //   pass_z.filter(pc);

    //   pc_nonground = pc;
    //   // pc_nonground is empty without ground segmentation
    //   pc_ground.header = pc.header;
    //   pc_nonground.header = pc.header;
    // }

    // const auto & t = sensor_to_world_transform_stamped.transform.translation;
    // tf2::Vector3 sensor_to_world_vec3{t.x, t.y, t.z};
    // insertScan(sensor_to_world_vec3, pc_ground, pc_nonground);

    // double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
    // RCLCPP_DEBUG(
    //   get_logger(),
    //   "Pointcloud insertion in OctomapServer done (%zu+%zu pts (ground/nonground), %f sec)",
    //   pc_ground.size(), pc_nonground.size(), total_elapsed);

    // publishAll(cloud->header.stamp);
  }
}

int main(int argc, char * argv[])
{

  double voxel_resolution = 0.05;
  Bonxai::VoxelGrid<int> grid( voxel_resolution );

  std::cout << "Starting Node" << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bonxai_server::BonxaiServer>());
  rclcpp::shutdown();
  return 0;
}