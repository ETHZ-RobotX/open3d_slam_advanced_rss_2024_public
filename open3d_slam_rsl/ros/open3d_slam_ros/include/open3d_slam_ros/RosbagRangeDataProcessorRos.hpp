/*
 * RosbagRangeDataProcessorRos.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam/output.hpp"

#include <std_srvs/Empty.h>

#include "open3d_slam_ros/DataProcessorRos.hpp"

namespace o3d_slam {

struct SlamInputs {
  sensor_msgs::PointCloud2::ConstPtr pointCloud_;
  geometry_msgs::PoseStamped::ConstPtr odometryPose_;

  SlamInputs() = default;

  SlamInputs(sensor_msgs::PointCloud2::ConstPtr pointCloud, geometry_msgs::PoseStamped::ConstPtr odometryPose)
      : pointCloud_(pointCloud), odometryPose_(odometryPose) {}
};

class RosbagRangeDataProcessorRos : public DataProcessorRos {
  using BASE = DataProcessorRos;
  using o3dReplayBuffer = std::deque<std::unique_ptr<SlamInputs>>;

 public:
  RosbagRangeDataProcessorRos(ros::NodeHandlePtr nh);
  ~RosbagRangeDataProcessorRos() override = default;

  void initialize() override;
  void startProcessing() override;
  void processMeasurement(const PointCloud& cloud, const Time& timestamp) override;
  void poseStampedCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  bool iterateCloudAndPoseBuffers(o3dReplayBuffer& buffer);
  void drawLinesBetweenPoses(const nav_msgs::Path& path1, const nav_msgs::Path& path2, const ros::Time& stamp);
  bool validateTopicsInRosbag(const rosbag::Bag& bag, const std::vector<std::string>& mandatoryTopics);
  bool readCalibrationIfNeeded();
  // bool run();

  std::optional<visualization_msgs::Marker> generateMarkersForSurfaceNormalVectors(const open3d::geometry::PointCloud& o3d_pc,
                                                                                   const ros::Time& timestamp,
                                                                                   const o3d_slam::RgbaColorMap::Values& color);

  std::tuple<ros::WallDuration, ros::WallDuration, ros::WallDuration> usePairForRegistration();

 private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  bool processRosbag();

  std::string rosbagFilename_;

  //! Parameters.
  Parameters parameters_;

  nav_msgs::Path trackedPath_;
  nav_msgs::Path bestGuessPath_;
  std::ofstream poseFile_;
  std::ofstream imuFile_;
  std::string asyncOdometryFrame_;

  std::string buildUpLogFilename(const std::string& typeSuffix, const std::string& extension = ".txt");
  bool createOutputDirectory();
  visualization_msgs::MarkerArray convertPathToMarkerArray(const nav_msgs::Path& path);
  visualization_msgs::Marker createLineStripMarker();

  o3d_slam::PointCloud lineStripToPointCloud(const visualization_msgs::MarkerArray& marker_array, const int num_samples);

  void calculateSurfaceNormals(o3d_slam::PointCloud& cloud);

  void processRosbagForIMU();

  void exportIMUData();

  //! Publishers.
  ros::Publisher clockPublisher_;
  ros::Publisher inputPointCloudPublisher_;

  ros::Publisher odometryPosePublisher_;
  ros::Publisher registeredPosePublisher_;

  ros::ServiceServer sleepServer_;

  sensor_msgs::PointCloud2 registeredCloud_;

  //! Tf2.
  tf2_ros::TransformBroadcaster transformBroadcaster_;
  tf2_ros::StaticTransformBroadcaster staticTransformBroadcaster_;

  std::deque<geometry_msgs::PoseStamped> registeredPoses_;

  ros::Time tracker;

  //! Maximum processing rate.
  double maxProcessingRate_{-0.1};

  //! Tf topic name.
  std::string tfTopic_{"/tf"};

  //! Static Tf topic name.
  std::string tfStaticTopic_{"/tf_static"};

  //! ROS clock topic name.
  std::string clockTopic_{"/clock"};

  bool isFirstMessage_ = true;
  bool isStaticTransformFound_ = false;

  ros::Duration timeDiff_;
  rosbag::Bag outBag;

  geometry_msgs::TransformStamped baseToLidarTransform_;
  std::string odometryHeader_{"/bestHeaderThereis"};
  std::vector<geometry_msgs::TransformStamped> staticTransforms_;

  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;

  bool isBagReadyToPlay_ = false;

  o3d_slam::RgbaColorMap colorMap_;
};

}  // namespace o3d_slam
