#include "bonxai_server.hpp"

namespace
{
template<typename T>
bool update_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(
    p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
      return parameter.get_name() == name;
    });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}
}  // namespace

namespace bonxai_server
{
  BonxaiServer::BonxaiServer(const rclcpp::NodeOptions & node_options)
  : Node("bonxai_server_node", node_options)
  {
      using std::placeholders::_1;
      using std::placeholders::_2;

      {
        world_frame_id_ = declare_parameter("frame_id", "map");
        base_frame_id_ = declare_parameter("base_frame_id", "base_footprint");
        use_height_map_ = declare_parameter("use_height_map", false);
        use_colored_map_ = declare_parameter("colored_map", false);
        color_factor_ = declare_parameter("color_factor", 0.8);
      }
      {
        point_cloud_min_x_ = declare_parameter("point_cloud_min_x", -std::numeric_limits<double>::max());
        point_cloud_max_x_ = declare_parameter("point_cloud_max_x", std::numeric_limits<double>::max());
        point_cloud_min_y_ = declare_parameter("point_cloud_min_y", -std::numeric_limits<double>::max());
        point_cloud_max_y_ = declare_parameter("point_cloud_max_y", std::numeric_limits<double>::max());        
      }
      {
        rcl_interfaces::msg::ParameterDescriptor point_cloud_min_z_desc;
        point_cloud_min_z_desc.description = "Minimum height of points to consider for insertion";
        rcl_interfaces::msg::FloatingPointRange point_cloud_min_z_range;
        point_cloud_min_z_range.from_value = -100.0;
        point_cloud_min_z_range.to_value = 100.0;
        point_cloud_min_z_desc.floating_point_range.push_back(point_cloud_min_z_range);
        point_cloud_min_z_ = declare_parameter("point_cloud_min_z", -100.0, point_cloud_min_z_desc);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor point_cloud_max_z_desc;
        point_cloud_max_z_desc.description = "Maximum height of points to consider for insertion";
        rcl_interfaces::msg::FloatingPointRange point_cloud_max_z_range;
        point_cloud_max_z_range.from_value = -100.0;
        point_cloud_max_z_range.to_value = 100.0;
        point_cloud_max_z_desc.floating_point_range.push_back(point_cloud_max_z_range);
        point_cloud_max_z_ = declare_parameter("point_cloud_max_z", 100.0, point_cloud_max_z_desc);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor occupancy_min_z_desc;
        occupancy_min_z_desc.description =
          "Minimum height of occupied cells to consider in the final map";
        rcl_interfaces::msg::FloatingPointRange occupancy_min_z_range;
        occupancy_min_z_range.from_value = -100.0;
        occupancy_min_z_range.to_value = 100.0;
        occupancy_min_z_desc.floating_point_range.push_back(occupancy_min_z_range);
        occupancy_min_z_ = declare_parameter("occupancy_min_z", -100.0, occupancy_min_z_desc);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor occupancy_max_z_desc;
        occupancy_max_z_desc.description =
          "Maximum height of occupied cells to consider in the final map";
        rcl_interfaces::msg::FloatingPointRange occupancy_max_z_range;
        occupancy_max_z_range.from_value = -100.0;
        occupancy_max_z_range.to_value = 100.0;
        occupancy_max_z_desc.floating_point_range.push_back(occupancy_max_z_range);
        occupancy_max_z_ = declare_parameter("occupancy_max_z", 100.0, occupancy_max_z_desc);
      }
        min_x_size_ = declare_parameter("min_x_size", 0.0);
        min_y_size_ = declare_parameter("min_y_size", 0.0);

      {
        rcl_interfaces::msg::ParameterDescriptor filter_speckles_desc;
        filter_speckles_desc.description = "Filter speckle nodes (with no neighbors)";
        filter_speckles_ = declare_parameter("filter_speckles", false, filter_speckles_desc);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor filter_ground_plane_desc;
        filter_ground_plane_desc.description = "Filter ground plane";
        filter_ground_plane_ =
          declare_parameter("filter_ground_plane", false, filter_ground_plane_desc);
      }
      {
        // distance of points from plane for RANSAC
        rcl_interfaces::msg::ParameterDescriptor ground_filter_distance_desc;
        ground_filter_distance_desc.description =
          "Distance threshold to consider a point as ground";
        rcl_interfaces::msg::FloatingPointRange ground_filter_distance_range;
        ground_filter_distance_range.from_value = 0.001;
        ground_filter_distance_range.to_value = 1.0;
        ground_filter_distance_desc.floating_point_range.push_back(ground_filter_distance_range);
        ground_filter_distance_ =
          declare_parameter("ground_filter.distance", 0.04, ground_filter_distance_desc);
      }
      {
        // angular derivation of found plane:
        rcl_interfaces::msg::ParameterDescriptor ground_filter_angle_desc;
        ground_filter_angle_desc.description =
          "Angular threshold of the detected plane from the horizontal plane to be detected as ground";
        rcl_interfaces::msg::FloatingPointRange ground_filter_angle_range;
        ground_filter_angle_range.from_value = 0.001;
        ground_filter_angle_range.to_value = 15.0;
        ground_filter_angle_desc.floating_point_range.push_back(ground_filter_angle_range);
        ground_filter_angle_ = declare_parameter("ground_filter.angle", 0.15);
      }
      {
        // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
        rcl_interfaces::msg::ParameterDescriptor ground_filter_plane_distance_desc;
        ground_filter_plane_distance_desc.description =
          "Distance threshold from z=0 for a plane to be detected as ground";
        rcl_interfaces::msg::FloatingPointRange ground_filter_plane_distance_range;
        ground_filter_plane_distance_range.from_value = 0.001;
        ground_filter_plane_distance_range.to_value = 1.0;
        ground_filter_plane_distance_desc.floating_point_range.push_back(
          ground_filter_plane_distance_range
        );
        ground_filter_plane_distance_ =
          declare_parameter("ground_filter.plane_distance", 0.07, ground_filter_plane_distance_desc);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor max_range_desc;
        max_range_desc.description = "Sensor maximum range";
        rcl_interfaces::msg::FloatingPointRange max_range_range;
        max_range_range.from_value = -1.0;
        max_range_range.to_value = 100.0;
        max_range_desc.floating_point_range.push_back(max_range_range);
        max_range_ = declare_parameter("sensor_model.max_range", -1.0, max_range_desc);
      }

      res_ = declare_parameter("resolution", 0.1);

      rcl_interfaces::msg::ParameterDescriptor prob_hit_desc;
      prob_hit_desc.description =
        "Probabilities for hits in the sensor model when dynamically building a map";
      rcl_interfaces::msg::FloatingPointRange prob_hit_range;
      prob_hit_range.from_value = 0.5;
      prob_hit_range.to_value = 1.0;
      prob_hit_desc.floating_point_range.push_back(prob_hit_range);
      const double prob_hit = declare_parameter("sensor_model.hit", 0.7, prob_hit_desc);

      rcl_interfaces::msg::ParameterDescriptor prob_miss_desc;
      prob_miss_desc.description =
        "Probabilities for misses in the sensor model when dynamically building a map";
      rcl_interfaces::msg::FloatingPointRange prob_miss_range;
      prob_miss_range.from_value = 0.0;
      prob_miss_range.to_value = 0.5;
      prob_miss_desc.floating_point_range.push_back(prob_miss_range);
      const double prob_miss = declare_parameter("sensor_model.miss", 0.4, prob_miss_desc);

      rcl_interfaces::msg::ParameterDescriptor prob_min_desc;
      prob_min_desc.description =
        "Minimum probability for clamping when dynamically building a map";
      rcl_interfaces::msg::FloatingPointRange prob_min_range;
      prob_min_range.from_value = 0.0;
      prob_min_range.to_value = 1.0;
      prob_min_desc.floating_point_range.push_back(prob_min_range);
      const double thres_min = declare_parameter("sensor_model.min", 0.12, prob_min_desc);

      rcl_interfaces::msg::ParameterDescriptor prob_max_desc;
      prob_max_desc.description =
        "Maximum probability for clamping when dynamically building a map";
      rcl_interfaces::msg::FloatingPointRange prob_max_range;
      prob_max_range.from_value = 0.0;
      prob_max_range.to_value = 1.0;
      prob_max_desc.floating_point_range.push_back(prob_max_range);
      const double thres_max = declare_parameter("sensor_model.max", 0.97, prob_max_desc);

      {
        rcl_interfaces::msg::ParameterDescriptor compress_map_desc;
        compress_map_desc.description = "Compresses the map losslessly";
        compress_map_ = declare_parameter("compress_map", true, compress_map_desc);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor incremental_2D_projection_desc;
        incremental_2D_projection_desc.description = "Incremental 2D projection";
        incremental_2D_projection_ =
          declare_parameter("incremental_2D_projection", false, incremental_2D_projection_desc);
      }

      if (filter_ground_plane_ && (point_cloud_min_z_ > 0.0 || point_cloud_max_z_ < 0.0)) {
        RCLCPP_WARN_STREAM(
          get_logger(),
          "You enabled ground filtering but incoming pointclouds will be pre-filtered in [" <<
            point_cloud_min_z_ << ", " << point_cloud_max_z_ << "], excluding the ground level z=0. "
            "This will not work.");
      }

      // initialize bonxai object & params
      bonxai_ = std::make_unique<BonxaiT>(res_);
      BonxaiT::Options options = {bonxai_->logods(prob_miss), 
                                  bonxai_->logods(prob_hit), 
                                  bonxai_->logods(thres_min), 
                                  bonxai_->logods(thres_max)};
      bonxai_->setOptions(options);

      // check if tree depth information is available (lines 256 - 268)

      gridmap_.info.resolution = res_;

      color_.r = declare_parameter("color.r", 0.0);
      color_.g = declare_parameter("color.g", 0.0);
      color_.b = declare_parameter("color.b", 1.0);
      color_.a = declare_parameter("color.a", 1.0);

      publish_free_space_ = declare_parameter("publish_free_space", false);

      latched_topics_ = declare_parameter("latch", true);
      if (latched_topics_) {
        RCLCPP_INFO(
          get_logger(),
          "Publishing latched (single publish will take longer, "
          "all topics are prepared)");
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Publishing non-latched (topics are only prepared as needed, "
          "will only be re-published on map change");
      }

      auto qos = latched_topics_ ? rclcpp::QoS{1}.transient_local() : rclcpp::QoS{1};
      single_marker_pub_ = create_publisher<Marker>("occupied_voxel_vis_array", qos);
      point_cloud_pub_ = create_publisher<PointCloud2>("bonxai_point_cloud_centers", qos);

      tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
      tf2_buffer_->setCreateTimerInterface(timer_interface);
      tf2_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

      using std::chrono_literals::operator""s;
      point_cloud_sub_.subscribe(this, "cloud_in", rmw_qos_profile_sensor_data);
      tf_point_cloud_sub_ = std::make_shared<tf2_ros::MessageFilter<PointCloud2>>(
        point_cloud_sub_, *tf2_buffer_, world_frame_id_, 5, this->get_node_logging_interface(),
        this->get_node_clock_interface(), 5s);

      tf_point_cloud_sub_->registerCallback(&BonxaiServer::insertCloudCallback, this);

      // Add reset service

      // set parameter callback
      set_param_res_ =
        this->add_on_set_parameters_callback(std::bind(&BonxaiServer::onParameter, this, _1));

      // const auto filename = declare_parameter("bonxai_path", "");
      // if (!openFile(filename)) {
      //   RCLCPP_WARN(get_logger(), "Could not open file %s", filename.c_str());
      // }
  }

  void BonxaiServer::insertCloudCallback(const PointCloud2::ConstSharedPtr cloud)
  {

    const auto start_time = rclcpp::Clock{}.now();

    //
    // ground filtering in base frame
    //
    PCLPointCloud pc;  // input cloud for filtering and ground-detection
    pcl::fromROSMsg(*cloud, pc);

    // Sensor In Global Frames Coordinates
    geometry_msgs::msg::TransformStamped sensor_to_world_transform_stamped;
    try {
      sensor_to_world_transform_stamped = tf2_buffer_->lookupTransform(
        world_frame_id_, cloud->header.frame_id, cloud->header.stamp,
        rclcpp::Duration::from_seconds(1.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    Eigen::Matrix4f sensor_to_world =
      tf2::transformToEigen(sensor_to_world_transform_stamped.transform).matrix().cast<float>();

    // Transforming Points to Global Reference Frame
    pcl::transformPointCloud (pc, pc, sensor_to_world);

    // Getting the Translation from the sensor to the Global Reference Frame
    const auto & t = sensor_to_world_transform_stamped.transform.translation;

    const pcl::PointXYZ sensor_to_world_vec3(t.x, t.y, t.z);
    
    bonxai_->insertPointCloud(pc.points, sensor_to_world_vec3, 30.0);

    double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
    RCLCPP_DEBUG(
      get_logger(),
      "Pointcloud insertion in Bonxai done, %f sec)", total_elapsed);

    publishAll(cloud->header.stamp);
  }


  rcl_interfaces::msg::SetParametersResult BonxaiServer::onParameter(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    update_param(parameters, "point_cloud_min_z", point_cloud_min_z_);
    update_param(parameters, "point_cloud_max_z", point_cloud_max_z_);
    update_param(parameters, "occupancy_min_z", occupancy_min_z_);
    update_param(parameters, "occupancy_max_z", occupancy_max_z_);
    update_param(parameters, "filter_speckles", filter_speckles_);
    update_param(parameters, "filter_ground_plane", filter_ground_plane_);
    update_param(parameters, "compress_map", compress_map_);
    update_param(parameters, "incremental_2D_projection", incremental_2D_projection_);
    update_param(parameters, "ground_filter_distance", ground_filter_distance_);
    update_param(parameters, "ground_filter_angle", ground_filter_angle_);
    update_param(parameters, "ground_filter_plane_distance", ground_filter_plane_distance_);
    update_param(parameters, "sensor_model.max_range", max_range_);
    double sensor_model_min{get_parameter("sensor_model.min").as_double()};
    update_param(parameters, "sensor_model.min", sensor_model_min);
    double sensor_model_max{get_parameter("sensor_model.max").as_double()};
    update_param(parameters, "sensor_model.max", sensor_model_max);
    double sensor_model_hit{get_parameter("sensor_model.hit").as_double()};
    update_param(parameters, "sensor_model.hit", sensor_model_hit);
    double sensor_model_miss{get_parameter("sensor_model.miss").as_double()};
    update_param(parameters, "sensor_model.miss", sensor_model_miss);
    BonxaiT::Options options = {bonxai_->logods(sensor_model_miss), 
                                bonxai_->logods(sensor_model_hit), 
                                bonxai_->logods(sensor_model_min), 
                                bonxai_->logods(sensor_model_max)};

    bonxai_->setOptions(options);

    publishAll(now());

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
  }

  void BonxaiServer::publishAll(const rclcpp::Time & rostime)
  {
    const auto start_time = rclcpp::Clock{}.now();
    std::vector<Eigen::Vector3d> bonxai_result;
    bonxai_->getOccupiedVoxels(bonxai_result); // TODO: estimate num occ. voxels for size of arrays (reserve)
    
    if (bonxai_result.size() <= 1){
      RCLCPP_WARN(get_logger(), "Nothing to publish, octree is empty");
      return;
    }

    bool publishSingleMarker =
      (latched_topics_ ||
      single_marker_pub_->get_subscription_count() +
      single_marker_pub_->get_intra_process_subscription_count() > 0);
    bool publish_point_cloud =
      (latched_topics_ ||
      point_cloud_pub_->get_subscription_count() +
      point_cloud_pub_->get_intra_process_subscription_count() > 0);

    // init markers for occupied space:
    Marker occupied_nodes_vis;

    // init pointcloud for occupied space:
    pcl::PointCloud<PCLPointRGB> pcl_cloud;

    if (publishSingleMarker || publish_point_cloud)
    {

      double min_x{};
      double min_y{};
      double min_z{};
      double max_x{};
      double max_y{};
      double max_z{};


      if (use_height_map_) 
      {
        bonxai_->calcMinMax(bonxai_result, min_x, min_y, min_z, max_x, max_y, max_z);
      }

      for (const auto& voxel: bonxai_result)
      {

        if(publishSingleMarker)
        {
          geometry_msgs::msg::Point cube_center;
          cube_center.x = voxel.x();
          cube_center.y = voxel.y();
          cube_center.z = voxel.z();

          occupied_nodes_vis.points.push_back(cube_center);

          if (use_height_map_) 
          {
            double h = (1.0 - std::min(std::max((cube_center.z - min_z) / (max_z - min_z), 0.0), 1.0)) * color_factor_;
            occupied_nodes_vis.colors.push_back(heightMapColor(h));
          }

        }

        if(publish_point_cloud)
        {
          
          if (!use_height_map_) // If heightmap is not required send blue pointcloud
          {
            pcl_cloud.push_back(PCLPointRGB(voxel.x(), voxel.y(), voxel.z(), int(floor(255*color_.r)), int(floor(255*color_.g)), int(floor(255*color_.b))));
          }
          else 
          {
            double h = (1.0 - std::min(std::max((voxel.z() - min_z) / (max_z - min_z), 0.0), 1.0)) * color_factor_;
            ColorRGBA height_map = heightMapColor(h);
            pcl_cloud.push_back(PCLPointRGB(voxel.x(), voxel.y(), voxel.z(), int(floor(255*height_map.r)), int(floor(255*height_map.g)), int(floor(255*height_map.b))));
          }
          
        }
      }

      if(publishSingleMarker)
      {
        occupied_nodes_vis.header.frame_id = world_frame_id_;
        occupied_nodes_vis.header.stamp = rostime;
        occupied_nodes_vis.ns = "map";
        occupied_nodes_vis.id = 0;
        occupied_nodes_vis.type = visualization_msgs::msg::Marker::CUBE_LIST;
        occupied_nodes_vis.scale.x = res_;
        occupied_nodes_vis.scale.y = res_;
        occupied_nodes_vis.scale.z = res_;
        occupied_nodes_vis.color = color_;
        occupied_nodes_vis.action = visualization_msgs::msg::Marker::ADD;
        single_marker_pub_->publish(occupied_nodes_vis);
      }

      if(publish_point_cloud)
      {
        PointCloud2 cloud;
        pcl::toROSMsg(pcl_cloud, cloud);
        cloud.header.frame_id = world_frame_id_;
        cloud.header.stamp = rostime;
        point_cloud_pub_->publish(cloud);
      }

    }

  }

  ColorRGBA BonxaiServer::heightMapColor(double h)
  {
    ColorRGBA color;
    color.a = 1.0;
    // blend over HSV-values (more colors)

    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i{};
    double m{};
    double n{};
    double f{};

    i = floor(h);
    f = h - i;
    if (!(i & 1)) {
      f = 1 - f;  // if i is even
    }
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i) {
      case 6:
      case 0:
        color.r = v; color.g = n; color.b = m;
        break;
      case 1:
        color.r = n; color.g = v; color.b = m;
        break;
      case 2:
        color.r = m; color.g = v; color.b = n;
        break;
      case 3:
        color.r = m; color.g = n; color.b = v;
        break;
      case 4:
        color.r = n; color.g = m; color.b = v;
        break;
      case 5:
        color.r = v; color.g = m; color.b = n;
        break;
      default:
        color.r = 1; color.g = 0.5; color.b = 0.5;
        break;
    }

    return color;
  }

} // namespace bonxai_server

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(bonxai_server::BonxaiServer)