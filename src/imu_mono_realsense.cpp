#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/logging.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sstream>

#include <cv_bridge/cv_bridge.h>

// this is orb_slam3
#include "System.h"

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

class ImuMonoRealSense : public rclcpp::Node {
public:
  ImuMonoRealSense()
    : Node("imu_mono_realsense"),
      vocabulary_file_path(std::string(PROJECT_PATH) +
                           "/ORB_SLAM3/Vocabulary/ORBvoc.txt"),
      inertial_ba1_(false), inertial_ba2_(false)
  {

    // declare parameters
    declare_parameter("sensor_type", "imu-monocular");
    declare_parameter("use_pangolin", true);

    // get parameters

    sensor_type_param = get_parameter("sensor_type").as_string();
    bool use_pangolin = get_parameter("use_pangolin").as_bool();

    // define callback groups
    image_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    imu_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    slam_service_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    octomap_server_client_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions image_options;
    image_options.callback_group = image_callback_group_;
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_callback_group_;

    // set the sensor type based on parameter
    ORB_SLAM3::System::eSensor sensor_type;
    if (sensor_type_param == "monocular") {
      sensor_type = ORB_SLAM3::System::MONOCULAR;
      settings_file_path =
        std::string(PROJECT_PATH) + "/config/Monocular/RealSense_D435i.yaml";
    } else if (sensor_type_param == "imu-monocular") {
      sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
      settings_file_path = std::string(PROJECT_PATH) +
                           "/config/Monocular-Inertial/RealSense_D435i.yaml";
    } else {
      RCLCPP_ERROR(get_logger(), "Sensor type not recognized");
      rclcpp::shutdown();
    }

    RCLCPP_INFO_STREAM(get_logger(),
                       "vocabulary_file_path: " << vocabulary_file_path);

    // setup orb slam object
    orb_slam3_system_ = std::make_shared<ORB_SLAM3::System>(
      vocabulary_file_path, settings_file_path, sensor_type, use_pangolin, 0);

    // create publishers
    accumulated_pcl_cloud_msg_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("orb_point_cloud2", 10);
    pose_array_publisher_ =
      create_publisher<geometry_msgs::msg::PoseArray>("pose_array", 100);

    // create subscriptions
    rclcpp::QoS image_qos(rclcpp::KeepLast(10));
    image_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    image_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    image_sub = create_subscription<sensor_msgs::msg::Image>(
      "camera/camera/color/image_raw", image_qos,
      std::bind(&ImuMonoRealSense::image_callback, this, _1), image_options);

    rclcpp::QoS imu_qos(rclcpp::KeepLast(10));
    imu_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    imu_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    imu_sub = create_subscription<sensor_msgs::msg::Imu>(
      "camera/camera/imu", imu_qos,
      std::bind(&ImuMonoRealSense::imu_callback, this, _1), imu_options);

    // create services
    slam_service = create_service<std_srvs::srv::Empty>(
      "slam_service",
      std::bind(&ImuMonoRealSense::slam_service_callback, this, _1, _2),
      rmw_qos_profile_services_default, slam_service_callback_group_);

    // create clients
    octomap_server_client_ = create_client<std_srvs::srv::Empty>(
      "octomap_server/reset", rmw_qos_profile_services_default,
      octomap_server_client_callback_group_);

    // tf broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // create timer
    timer = create_wall_timer(
      1000ms, std::bind(&ImuMonoRealSense::timer_callback, this),
      timer_callback_group_);

    initialize_variables();
  }

  ~ImuMonoRealSense()
  {
    orb_slam3_system_->SavePCDASCII(std::string(PROJECT_PATH) + "/maps/");
  }

private:
  void initialize_variables()
  {
    pose_array_ = geometry_msgs::msg::PoseArray();
    pose_array_.header.frame_id = "point_cloud";

    accumulated_pcl_cloud_msg_ = sensor_msgs::msg::PointCloud2();
    accumulated_pcl_cloud_msg_.header.frame_id = "point_cloud";
  }

  std::string generate_timestamp_string()
  {
    std::time_t now = std::time(nullptr);
    std::tm *ptm = std::localtime(&now);

    std::ostringstream oss;

    oss << std::put_time(ptm, "%Y-%m-%d_%H-%M-%S") << ".mp4";

    return oss.str();
  }

  cv::Mat get_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0) {
      return cv_ptr->image.clone();
    } else {
      std::cerr << "Error image type" << std::endl;
      return cv_ptr->image.clone();
    }
  }

  void slam_service_callback(const std_srvs::srv::Empty::Request::SharedPtr,
                             const std_srvs::srv::Empty::Response::SharedPtr)
  {
    orb_slam3_system_->SavePCDBinary(std::string(PROJECT_PATH) + "/maps/");
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    img_buf_.push(msg);
    rclcpp::Time time_now = get_clock()->now();

    // begin to empty the img_buf_ queue, which is full of other queues
    while (!img_buf_.empty()) {
      // grab the oldest image
      auto imgPtr = img_buf_.front();
      img_buf_.pop();

      cv::Mat imageFrame = get_image(imgPtr);
      double tImage =
        imgPtr->header.stamp.sec + imgPtr->header.stamp.nanosec * 1e-9;

      vector<ORB_SLAM3::IMU::Point> vImuMeas;

      // package all the imu data for this image for orbslam3 to process
      buf_mutex_imu_.lock();
      while (!imu_buf_.empty()) {
        auto imuPtr = imu_buf_.front();
        imu_buf_.pop();
        double tIMU =
          imuPtr->header.stamp.sec + imuPtr->header.stamp.nanosec * 1e-9;

        cv::Point3f acc(imuPtr->linear_acceleration.x,
                        imuPtr->linear_acceleration.y,
                        imuPtr->linear_acceleration.z);
        cv::Point3f gyr(imuPtr->angular_velocity.x, imuPtr->angular_velocity.y,
                        imuPtr->angular_velocity.z);
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, tIMU));
      }

      buf_mutex_imu_.unlock();

      if (vImuMeas.empty() && sensor_type_param == "imu-monocular") {
        // RCLCPP_WARN(get_logger(),
        //             "No valid IMU data available for the current frame "
        //             "at time %.6f.",
        //             tImage);
        return;
      }

      try {
        if (sensor_type_param == "monocular") {
          orb_slam3_system_->TrackMonocular(imageFrame, tImage);
        } else {
          if (vImuMeas.size() > 1) {
            auto Tcw =
              orb_slam3_system_->TrackMonocular(imageFrame, tImage, vImuMeas);
            auto Twc = Tcw.inverse();
            if (orb_slam3_system_->isImuInitialized()) {

              tf2::Quaternion q_orig(
                Twc.unit_quaternion().x(), Twc.unit_quaternion().y(),
                Twc.unit_quaternion().z(), Twc.unit_quaternion().w());

              tf2::Matrix3x3 m(q_orig);
              double roll, pitch, yaw;
              m.getRPY(roll, pitch, yaw);

              tf2::Quaternion q_yaw;
              q_yaw.setRPY(0, 0, yaw);

              tf2::Quaternion q_rot_x, q_rot_z;
              // q_rot_x.setRPY(M_PI / 2.0, 0, 0);
              q_rot_z.setRPY(0, 0, M_PI / 2.0);

              tf2::Quaternion q_combined = q_rot_z * q_yaw;
              q_combined.normalize();

              geometry_msgs::msg::Pose pose;
              pose.position.x = Twc.translation().x();
              pose.position.y = Twc.translation().y();
              // pose.position.z = Twc.translation().z();
              pose.orientation.x = q_combined.x();
              pose.orientation.y = q_combined.y();
              pose.orientation.z = q_combined.z();
              pose.orientation.w = q_combined.w();
              pose_array_.header.stamp = time_now;
              pose_array_.poses.push_back(pose);

              geometry_msgs::msg::TransformStamped base_link_tf;
              base_link_tf.header.stamp = time_now;
              base_link_tf.header.frame_id = "point_cloud";
              base_link_tf.child_frame_id = "base_link";
              base_link_tf.transform.translation.x = Twc.translation().x();
              base_link_tf.transform.translation.y = Twc.translation().y();
              base_link_tf.transform.rotation.x = q_combined.x();
              base_link_tf.transform.rotation.y = q_combined.y();
              base_link_tf.transform.rotation.z = q_combined.z();
              base_link_tf.transform.rotation.w = q_combined.w();
              tf_broadcaster->sendTransform(base_link_tf);

              tf2::Quaternion q_rot_x2, q_rot_y2;
              q_rot_x2.setRPY(M_PI / 2.0, 0, 0);
              q_rot_y2.setRPY(0, -M_PI / 2.0, 0);
              tf2::Quaternion q_combined2 = q_rot_y2 * q_rot_x2;
              q_combined2.normalize();

              geometry_msgs::msg::TransformStamped scan_tf;
              scan_tf.header.stamp = time_now;
              scan_tf.header.frame_id = "base_link";
              scan_tf.child_frame_id = "scan";
              tf_broadcaster->sendTransform(scan_tf);

              geometry_msgs::msg::TransformStamped point_cloud_tf;
              point_cloud_tf.header.stamp = time_now;
              point_cloud_tf.header.frame_id = "map";
              point_cloud_tf.child_frame_id = "point_cloud";
              tf_broadcaster->sendTransform(point_cloud_tf);

              accumulated_pcl_cloud_ = orb_slam3_system_->GetMapPCL();

              pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_ptr =
                std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(
                  accumulated_pcl_cloud_);

              // voxel grid filter
              // pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(
              //   new pcl::PointCloud<pcl::PointXYZ>);
              // pcl::VoxelGrid<pcl::PointXYZ> vg;
              // vg.setInputCloud(accumulated_ptr);
              // vg.setLeafSize(0.05f, 0.05f, 0.05f);
              // vg.filter(*voxel_cloud);

              // statistical outlier removal
              pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(
                new pcl::PointCloud<pcl::PointXYZ>);
              pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
              sor.setInputCloud(accumulated_ptr);
              sor.setMeanK(100);
              sor.setStddevMulThresh(0.1);
              sor.filter(*sor_cloud);

              sor_cloud->width = sor_cloud->points.size();
              pcl::toROSMsg(*sor_cloud, accumulated_pcl_cloud_msg_);

              accumulated_pcl_cloud_msg_.header.frame_id = "point_cloud";
              accumulated_pcl_cloud_msg_.header.stamp = time_now;
            }
          }
        }

        if (!inertial_ba1_ && orb_slam3_system_->GetInertialBA1()) {
          inertial_ba1_ = true;
          pose_array_.poses.clear();
          pose_array_.header.stamp = time_now;
          RCLCPP_INFO(get_logger(), "Inertial BA1 complete");
        }

        if (!inertial_ba2_ && orb_slam3_system_->GetInertialBA2()) {
          inertial_ba2_ = true;
          pose_array_.poses.clear();
          pose_array_.header.stamp = time_now;
          RCLCPP_INFO(get_logger(), "Inertial BA2 complete");
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "SLAM processing exception: %s", e.what());
      }
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu &msg)
  {
    buf_mutex_imu_.lock();
    if (!std::isnan(msg.linear_acceleration.x) &&
        !std::isnan(msg.linear_acceleration.y) &&
        !std::isnan(msg.linear_acceleration.z) &&
        !std::isnan(msg.angular_velocity.x) &&
        !std::isnan(msg.angular_velocity.y) &&
        !std::isnan(msg.angular_velocity.z)) {
      const sensor_msgs::msg::Imu::SharedPtr msg_ptr =
        std::make_shared<sensor_msgs::msg::Imu>(msg);
      imu_buf_.push(msg_ptr);
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid IMU data - nan");
    }
    buf_mutex_imu_.unlock();
  }

  void timer_callback()
  {
    unique_lock<mutex> lock(orbslam3_mutex_);

    geometry_msgs::msg::Pose pose;
    if (orb_slam3_system_->isImuInitialized() ||
        orb_slam3_system_->GetInertialBA1() ||
        orb_slam3_system_->GetInertialBA2()) {
      if (pose_array_.poses.size() > 1000) {
        pose_array_.poses.erase(pose_array_.poses.begin());
      }

      // publish the variables I've been accumulating in the image callback
      pose_array_publisher_->publish(pose_array_);
      accumulated_pcl_cloud_msg_publisher_->publish(accumulated_pcl_cloud_msg_);

    } else {
      octomap_server_client_->async_send_request(
        std::make_shared<std_srvs::srv::Empty::Request>());
      initialize_variables();
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    accumulated_pcl_cloud_msg_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
    pose_array_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr slam_service;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octomap_server_client_;
  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::CallbackGroup::SharedPtr image_callback_group_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr slam_service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr octomap_server_client_callback_group_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  sensor_msgs::msg::Imu imu_msg;

  nav_msgs::msg::Odometry odom_msg_;

  std::string sensor_type_param;

  std::vector<geometry_msgs::msg::Vector3> vGyro;
  std::vector<double> vGyro_times;
  std::vector<geometry_msgs::msg::Vector3> vAccel;
  std::vector<double> vAccel_times;

  queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf_;
  queue<sensor_msgs::msg::Image::SharedPtr> img_buf_;
  std::mutex buf_mutex_imu_, buf_mutex_img_, orbslam3_mutex_;

  std::shared_ptr<ORB_SLAM3::System> orb_slam3_system_;
  std::string vocabulary_file_path;
  std::string settings_file_path;

  sensor_msgs::msg::PointCloud2 accumulated_pcl_cloud_msg_;
  pcl::PointCloud<pcl::PointXYZ> accumulated_pcl_cloud_;

  geometry_msgs::msg::PoseArray pose_array_;

  bool inertial_ba1_;
  bool inertial_ba2_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuMonoRealSense>());
  rclcpp::shutdown();
  return 0;
}
