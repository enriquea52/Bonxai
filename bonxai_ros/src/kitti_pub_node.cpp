#include "kitti_pub_node.hpp"

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);

  MinimalPublisher  cloud_pub;



  const auto calibration_transform = ReadCalibration("/home/enrique/DATASET/kitti_sequence_04/calib.txt");
  const auto poses = ReadPoses("/home/enrique/DATASET/kitti_sequence_04/poses.txt");

  std::cout << "poses.size()" << poses.size() <<std::endl;


  std::vector<std::string> cloud_filenames;
  for (const auto& entry : fs::directory_iterator("/home/enrique/DATASET/kitti_sequence_04/velodyne"))
  {
    cloud_filenames.push_back(entry.path().generic_string());
  }

  std::sort(cloud_filenames.begin(), cloud_filenames.end());
  
  for (size_t count = 0; count < cloud_filenames.size(); count++)
  {
    const auto& filename = cloud_filenames[count];
    const Eigen::Isometry3f transform = Eigen::AngleAxisf(M_PI/2,  Eigen::Vector3f::UnitY())*  poses[count] * calibration_transform * Eigen::AngleAxisf(-M_PI/2,  Eigen::Vector3f::UnitX());

    const Eigen::Vector3f origin(transform.translation());
    std::vector<Eigen::Vector3f> pointcloud;
    ReadPointcloud(filename,  pointcloud);

    // // Prepare pointcloud 

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    for(const auto point: pointcloud)
    {
        cloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));

    }

    std::cout << "count: " << count <<  " pcl cloud points: " << cloud->size() << std::endl;

    sensor_msgs::msg::PointCloud2 cloud_out;
    pcl::toROSMsg(*cloud,cloud_out);  

    cloud_out.header.frame_id = "sensor";
    

    // // Prepare tf transform
    Eigen::Vector3f T = transform.translation();
    Eigen::Quaternionf q(transform.rotation());

    geometry_msgs::msg::TransformStamped tf_pose;

    // Read message content and assign it to
    // corresponding tf variables
    // tf_pose.header.stamp = rclcpp::Node::now();
    tf_pose.header.frame_id = "map";
    tf_pose.child_frame_id = "sensor";

    tf_pose.transform.translation.x = T.x();
    tf_pose.transform.translation.y = T.y();
    tf_pose.transform.translation.z = T.z();

    tf_pose.transform.rotation.x = q.x();
    tf_pose.transform.rotation.y = q.y();
    tf_pose.transform.rotation.z = q.z();
    tf_pose.transform.rotation.w = q.w();


    // publish both pointcloud and broadcast tf

    cloud_pub.publisher_->publish(cloud_out);

    cloud_pub.tf_broadcaster_->sendTransform(tf_pose);

    // sleep for x time
    this_thread::sleep_for(chrono::milliseconds(500));

  }

  std::cout << "cloud_filenames.size()" << cloud_filenames.size() <<std::endl;
  std::cout << "kitti ros2 parser" << std::endl;

//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MinimalPublisher>());
//   rclcpp::shutdown();
  return 0;
}