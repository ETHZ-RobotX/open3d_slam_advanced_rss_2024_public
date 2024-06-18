/*
 * RosbagRangeDataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/RosbagRangeDataProcessorRos.hpp"
#include <open3d/io/PointCloudIO.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <filesystem>
#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam_ros/SlamWrapperRos.hpp"

#include <rosbag/view.h>
#include "open3d_slam/time.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"

namespace o3d_slam {

RosbagRangeDataProcessorRos::RosbagRangeDataProcessorRos(ros::NodeHandlePtr nh) : BASE(nh) {}

void RosbagRangeDataProcessorRos::initialize() {
  initCommonRosStuff();
  slam_ = std::make_shared<SlamWrapperRos>(nh_);
  slam_->loadParametersAndInitialize();
  rosbagFilename_ = nh_->param<std::string>("rosbag_filepath", "");
  ROS_INFO_STREAM("Reading from rosbag: " << rosbagFilename_);

  // Initialize the calibration Transform.
  baseToLidarTransform_.transform.rotation.w = 1.0;
  baseToLidarTransform_.transform.rotation.z = 0.0;
  baseToLidarTransform_.transform.rotation.y = 0.0;
  baseToLidarTransform_.transform.rotation.x = 0.0;

  baseToLidarTransform_.transform.translation.z = 0.0;
  baseToLidarTransform_.transform.translation.y = 0.0;
  baseToLidarTransform_.transform.translation.x = 0.0;

  tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
  tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);
  timeDiff_ = ros::Duration(0.0);

  // Provide questionare.
  ROS_INFO_STREAM("\033[33m"
                  << " Did you check the odometry topic frame? "
                  << "\033[39m");
  ROS_INFO_STREAM("\033[33m"
                  << " Did you check the check if loopclosure enabled? "
                  << "\033[39m");
  ROS_INFO_STREAM("\033[33m"
                  << " Is clock available in your bag? "
                  << "\033[39m");
  ROS_INFO_STREAM("\033[33m"
                  << " Did you pray this works? "
                  << "\033[39m");
  ROS_INFO_STREAM("\033[33m"
                  << " Did you check if tf_static exists in your bag? "
                  << "\033[39m");

  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(2.0)};
  ros::WallTime::sleepUntil(first);
}

bool RosbagRangeDataProcessorRos::createOutputDirectory() {
  // TODO(TT) This folder is currently not used.

  // Check if the output folder exists.
  if (std::filesystem::is_directory("log/slam_loggers")) {
    return true;
  }

  // If the folder doesn't exist, create it.
  try {
    return std::filesystem::create_directories("log/slam_loggers");
  } catch (const std::exception& exception) {
    ROS_ERROR_STREAM("Caught an exception trying to create output folder: " << exception.what());
  }

  return false;
}

std::string RosbagRangeDataProcessorRos::buildUpLogFilename(const std::string& typeSuffix, const std::string& extension) {
  ros::WallTime stamp = ros::WallTime::now();
  std::stringstream ss;
  ss << stamp.sec << "_" << stamp.nsec;

  // Add prefixes.
  // Not sure if adding time is the best thing to do since we dont keep a ring buffer.
  // std::string filename = slam_->mapSavingFolderPath_ + typeSuffix + ss.str() + extension;
  std::string filename = slam_->mapSavingFolderPath_ + typeSuffix + extension;

  return filename;
}

visualization_msgs::Marker RosbagRangeDataProcessorRos::createLineStripMarker() {
  visualization_msgs::Marker marker;
  marker.header.frame_id = slam_->frames_.mapFrame;
  marker.header.stamp = tracker;
  marker.ns = "path";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  marker.points.reserve(trackedPath_.poses.size());

  for (const auto& pose : trackedPath_.poses) {
    geometry_msgs::Point point;
    point.x = pose.pose.position.x;
    point.y = pose.pose.position.y;
    point.z = pose.pose.position.z;
    marker.points.push_back(point);
  }

  return marker;
}

o3d_slam::PointCloud RosbagRangeDataProcessorRos::lineStripToPointCloud(const visualization_msgs::MarkerArray& marker_array,
                                                                        const int num_samples) {
  // Create point stream
  std::vector<Eigen::Vector3d> points;

  // Iterate over markers in marker array to find line strip
  for (auto marker : marker_array.markers) {
    for (size_t j = 0; j < marker.points.size(); j++) {
      // Sample points between start and end of line strip
      float dx = (marker.points[j + 1].x - marker.points[j].x) / num_samples;
      float dy = (marker.points[j + 1].y - marker.points[j].y) / num_samples;
      float dz = (marker.points[j + 1].z - marker.points[j].z) / num_samples;

      for (int i = 0; i < num_samples; i++) {
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
        point.x() = marker.points[j].x + i * dx;
        point.y() = marker.points[j].y + i * dy;
        point.z() = marker.points[j].z + i * dz;

        points.push_back(point);
      }
    }
  }
  // Create point cloud
  o3d_slam::PointCloud cloud(points);
  return cloud;
}

visualization_msgs::MarkerArray RosbagRangeDataProcessorRos::convertPathToMarkerArray(const nav_msgs::Path& path) {
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(1);
  marker_array.markers[0] = createLineStripMarker();
  return marker_array;
}

void RosbagRangeDataProcessorRos::calculateSurfaceNormals(o3d_slam::PointCloud& cloud) {
  if (cloud.points_.size() == 0u) {
    ROS_ERROR_STREAM("Cloud is empty. Skipping the normal calculation.");
    return;
  }

  // Create a KD-Tree and estimate surface normals.
  open3d::geometry::KDTreeSearchParamHybrid param(5, 20);
  cloud.EstimateNormals(param);
  cloud.NormalizeNormals();
  cloud.OrientNormalsTowardsCameraLocation();
}

void RosbagRangeDataProcessorRos::exportIMUData() {
  // An auxiliary function to save the IMU data, useful for offline operations.
  std::string IMUFilename_ = buildUpLogFilename("imu");
  std::remove(IMUFilename_.c_str());

  const std::string imuFileHeader_ = "timestamp acc_x acc_y acc_z w_x w_y w_z";

  // Open file and set numerical precision to the max.
  imuFile_.open(IMUFilename_, std::ios_base::app);
  imuFile_.precision(std::numeric_limits<double>::max_digits10);
  // Save data to file.
  imuFile_ << imuFileHeader_ << std::endl;

  processRosbagForIMU();
  // Close file handle.
  imuFile_.close();
}

void RosbagRangeDataProcessorRos::processRosbagForIMU() {
  std::vector<std::string> topics;
  topics.push_back("/sensors/imu");

  Timer rosbagTimer;

  // Open ROS bag.
  rosbag::Bag bag;
  try {
    bag.open(rosbagFilename_, rosbag::bagmode::Read);
  } catch (const rosbag::BagIOException& e) {
    ROS_ERROR_STREAM("Error opening ROS bag: '" << rosbagFilename_ << "'");
    return;
  }
  ROS_INFO_STREAM("ROS bag '" << rosbagFilename_ << "' open.");

  if (!validateTopicsInRosbag(bag, topics)) {
    ROS_ERROR("FAILED TO SUCCESSFULLY VALIDATE THE ROSBAG.");
    bag.close();
    return;
  }

  ROS_INFO_STREAM("\033[92m"
                  << " Exporting IMU MSGs... "
                  << "\033[0m");
  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(2.0)};
  ros::WallTime::sleepUntil(first);

  // The bag view we iterate over.
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  for (const auto& messageInstance : view) {
    // If the node is shutdown, stop processing and do early return.
    if (!ros::ok()) {
      return;
    }

    if (messageInstance.getTopic() == topics[0]) {
      sensor_msgs::Imu::ConstPtr message = messageInstance.instantiate<sensor_msgs::Imu>();
      if (message != nullptr) {
        // msg->linear_acceleration.x
        sensor_msgs::Imu imuMsg = *message;

        const double stamp = imuMsg.header.stamp.toSec();
        imuFile_ << stamp << " ";
        imuFile_ << imuMsg.linear_acceleration.x << " " << imuMsg.linear_acceleration.y << " " << imuMsg.linear_acceleration.z << " ";
        imuFile_ << imuMsg.angular_velocity.x << " " << imuMsg.angular_velocity.y << " " << imuMsg.angular_velocity.z << std::endl;

      } else {
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }
  }
}

void RosbagRangeDataProcessorRos::startProcessing() {
  if (!createOutputDirectory()) {
    std::cout << "Couldn't create the directory "
              << "\n";
    return;
  }

  if (slam_->exportIMUdata_) {
    exportIMUData();
    std::cout << "IMU Exporting is complete"
              << "\n";

    const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(10.0)};
    ros::WallTime::sleepUntil(first);

    std::cout << "Sleeping 10 seconds.."
              << "\n";
  }

  std::string trackedPosesFilename_ = buildUpLogFilename("slam_poses");
  std::remove(trackedPosesFilename_.c_str());

  const std::string poseLogFileHeader_ = "# timestamp x y z q_x q_y q_z q_w";

  // Open file and set numerical precision to the max.
  poseFile_.open(trackedPosesFilename_, std::ios_base::app);
  poseFile_.precision(std::numeric_limits<double>::max_digits10);
  poseFile_ << poseLogFileHeader_ << std::endl;

  std::string outBagPath_ = buildUpLogFilename("processed_slam", ".bag");
  std::remove(outBagPath_.c_str());
  outBag.open(outBagPath_, rosbag::bagmode::Write);

  // Iterate and process the bag.
  if (processRosbag()) {
    // Create the magical tube.
    visualization_msgs::MarkerArray lineStrip = convertPathToMarkerArray(trackedPath_);

    o3d_slam::PointCloud samplecloud;
    // Magic number, points per circle strip. If you set it too high, cloud becomes mega heavy.
    int numPoints = 50;
    // Define the radius of the tube
    float tube_radius = 0.02;

    samplecloud = lineStripToPointCloud(lineStrip, numPoints);

    calculateSurfaceNormals(samplecloud);

    // Create a new point cloud for the tube
    o3d_slam::PointCloud tube_cloud;
    std::vector<Eigen::Vector3d> tube_cloud_normal_vector;
    std::vector<Eigen::Vector3d> tube_cloud_point_vector;

    tube_cloud_normal_vector.reserve(samplecloud.points_.size() * 36);
    tube_cloud_point_vector.reserve(samplecloud.points_.size() * 36);

    // Iterate over the points in the original point cloud
    for (int i = 0; i < samplecloud.points_.size() - numPoints; i++) {
      // Retrieve the surface normal
      Eigen::Vector3d normal = samplecloud.normals_[i].normalized();

      // Calculate the direction vector
      const Eigen::Vector3d direction = normal.unitOrthogonal().cross(normal).normalized();

      // Generate points around the current point
      const Eigen::Vector3d current_point = samplecloud.points_[i];
      const Eigen::Vector3d current_normal = samplecloud.normals_[i];
      for (float angle = 0; angle <= 360; angle += 10) {
        float x = current_point.x() + tube_radius * (direction.x() * cos(angle) + normal.x() * sin(angle));
        float y = current_point.y() + tube_radius * (direction.y() * cos(angle) + normal.y() * sin(angle));
        float z = current_point.z() + tube_radius * (direction.z() * cos(angle) + normal.z() * sin(angle));

        Eigen::Vector3d tube_point(x, y, z);

        tube_cloud_normal_vector.push_back(current_normal);
        tube_cloud_point_vector.push_back(tube_point);
      }
    }
    tube_cloud.normals_ = tube_cloud_normal_vector;
    tube_cloud.points_ = tube_cloud_point_vector;

    // Copy the surface normals to the new point cloud
    for (int i = 0; i < samplecloud.points_.size() - numPoints; i++) {
      for (int j = 0; j < 36; j++) {
        tube_cloud.normals_[i * 36 + j] = samplecloud.normals_[i];
      }
    }

    std::string nameWithCorrectSuffix = slam_->mapSavingFolderPath_ + "robotPathAsMesh.pcd";
    // size_t found = nameWithCorrectSuffix.find(".pcd");

    open3d::io::WritePointCloudToPCD(nameWithCorrectSuffix, tube_cloud, open3d::io::WritePointCloudOption());
    ROS_INFO_STREAM("Successfully saved the poses as point cloud. Waiting for user to terminate.");
  }

  // Close file handle.
  poseFile_.close();
  outBag.close();

  ros::spin();
  slam_->stopWorkers();
  return;
}

bool RosbagRangeDataProcessorRos::validateTopicsInRosbag(const rosbag::Bag& bag, const std::vector<std::string>& mandatoryTopics) {
  // Get a view on the data and check if all mandatory topics are present.
  bool areMandatoryTopicsInRosbag{true};

  // Iterate over the mandatory topics and check if they are in the bag.
  for (const auto& topic : mandatoryTopics) {
    rosbag::View topicView(bag, rosbag::TopicQuery(topic));
    if (topicView.size() == 0u) {
      if (topic == clockTopic_) {
        // This means this is our second time coming here. So actually the the alternative topic is not working either.
        if (topic == slam_->asyncOdometryTopic_) {
          ROS_ERROR_STREAM(clockTopic_ << " topic does not exist in the rosbag. This is breaking.");
          areMandatoryTopicsInRosbag = false;
        } else {
          ROS_ERROR_STREAM(clockTopic_ << " topic does not exist in the rosbag. Using alternative topic: " << slam_->asyncOdometryTopic_
                                       << " as clock.");
          clockTopic_ = slam_->asyncOdometryTopic_;
        }

      } else if (topic == tfTopic_) {
        ROS_WARN_STREAM("No data under the topic: " << topic << " was found. This was optional so okay.");
        continue;
      } else if (topic == tfStaticTopic_) {
        ROS_ERROR_STREAM("No data under the topic: "
                         << topic << " was found. This is NOT optional. But if you make tf_static available external its okay.");
        continue;
      } else {
        ROS_ERROR_STREAM("No data under the topic: " << topic << " was found.");
        areMandatoryTopicsInRosbag = false;
      }
    } else {
      if (topic == slam_->asyncOdometryTopic_) {
        // Get a view for the specific topic only
        rosbag::View view(bag, rosbag::TopicQuery(topic));
        for (const auto& messageInstance : view) {
          if (messageInstance.getDataType() == "geometry_msgs/PoseWithCovarianceStamped") {
            ROS_WARN_STREAM(" ' " << topic
                                  << "' topic does not support automatic frame detection. Msg Type: " << messageInstance.getDataType()
                                  << ". Assumed tracked odometry frame: " << slam_->frames_.assumed_external_odometry_tracked_frame);
            break;
          } else if (messageInstance.getDataType() == "nav_msgs/Odometry") {
            nav_msgs::Odometry::ConstPtr message = messageInstance.instantiate<nav_msgs::Odometry>();
            if (message != nullptr) {
              slam_->frames_.assumed_external_odometry_tracked_frame = message->child_frame_id;
              ROS_WARN_STREAM(topic << " frame_id is set to: " << slam_->frames_.assumed_external_odometry_tracked_frame);
              break;
            }  // if
          } else {
            ROS_ERROR_STREAM(topic << " msg type is NOT SUPPORTED");
            return false;
          }

        }  // for
      }    // if
    }      // if
  }        // for

  if (slam_->useSyncedPoses_) {
    ROS_WARN_STREAM("Sync poses are enabled. This does not support automatic frame detection. Assumed tracked odometry frame: "
                    << slam_->frames_.assumed_external_odometry_tracked_frame);
  }

  if (!areMandatoryTopicsInRosbag) {
    ROS_ERROR("All required topics are not within the rosbag. Replay operations are terminated. Waiting user to terminate.");
    return false;
  }

  return true;
}

void RosbagRangeDataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {
  /*bool success = slam_->addRangeScan(cloud, timestamp);
  std::tuple<PointCloud, Time, Transform> cloudTimePair = slam_->getLatestRegisteredCloudTimestampPair();
  const bool isCloudEmpty = std::get<0>(cloudTimePair).IsEmpty();
  if (isTimeValid(std::get<1>(cloudTimePair)) && !isCloudEmpty) {
    o3d_slam::publishCloud(std::get<0>(cloudTimePair), slam_->frames_.rangeSensorFrame, toRos(std::get<1>(cloudTimePair)), rawCloudPub_);
  }
  */
}

bool RosbagRangeDataProcessorRos::iterateCloudAndPoseBuffers(o3dReplayBuffer& buffer) {
  // This is as fast as the thread can go
  if (buffer.empty()) {
    ROS_DEBUG("Empty buffer");
    return false;
  }

  if (slam_->useSyncedPoses_) {
    // Sync poses are currently is only X-ICP type. Thus not well supported.
    // Provide the pose prior
    auto& odometryPose = buffer.front()->odometryPose_;
    geometry_msgs::Pose odomPose = odometryPose->pose;

    if (isFirstMessage_) {
      Eigen::Isometry3d eigenTransform = Eigen::Isometry3d::Identity();
      slam_->setExternalOdometryFrameToCloudFrameCalibration(eigenTransform);
    }

    if (isFirstMessage_ && isStaticTransformFound_) {
      geometry_msgs::Pose initialPose;
      initialPose.position = odomPose.position;
      initialPose.orientation.w = 1.0;
      initialPose.orientation.z = 0.0;
      initialPose.orientation.y = 0.0;
      initialPose.orientation.x = 0.0;
      slam_->setInitialTransform(o3d_slam::getTransform(initialPose).matrix());
      isFirstMessage_ = false;
    }

    // Add to the odometry buffer.
    if (!(slam_->addOdometryPoseToBuffer(o3d_slam::getTransform(odomPose), fromRos(odometryPose->header.stamp)))) {
      std::cout << "Couldn't Add pose to buffer" << std::endl;
      return false;
    }

  } else {
    // Buffer is empty, means the async odometry poses did not arrive before the point cloud.
    if (slam_->isOdometryPoseBufferEmpty()) {
      std::cout << "Odometry Buffer is empty!" << std::endl;
      return false;
    }

    if (!slam_->isInitialTransformSet()) {
      std::cout << "Initial transform not set yet! Popping the measurement." << std::endl;
      return false;
    }
  }

  auto& pointCloud = buffer.front()->pointCloud_;

  // Convert to o3d cloud
  open3d::geometry::PointCloud cloud;

  if (!open3d_conversions::rosToOpen3d(pointCloud, cloud, false, true)) {
    std::cout << "Couldn't convert the point cloud" << std::endl;
    return false;
  }

  const Time timestamp = fromRos(pointCloud->header.stamp);

  if (!slam_->doesOdometrybufferHasMeasurement(timestamp)) {
    std::cout << "RosbagReplayer:: Pointcloud is here, pose buffer is not empty but odometry with the right stamp not available yet. "
                 "Popping the measurement."
              << std::endl;
    buffer.pop_front();
    return false;
  }

  // Add the cloud to queue, internally checks if there is a matching odometry pose in the odometryBuffer
  if (slam_->addRangeScan(cloud, timestamp)) {
    // std::cout << "Adding cloud with stamp: " << o3d_slam::toString(timestamp) << std::endl;
    // Timer mapperOnlyTimer_;
    // mapperOnlyTimer_.startStopwatch();
    auto timeTuple = usePairForRegistration();
    // const double timeElapsed = mapperOnlyTimer_.elapsedMsecSinceStopwatchStart();
  } else {
    std::cout << "RosbagReplayer:: Couldn't add range scan. Popping this measurement from the buffer." << std::endl;
    buffer.pop_front();
    return false;
  }

  // Publish the tfs
  slam_->offlineTfWorker();

  // Publish Submap markers
  slam_->offlineVisualizationWorker();

  // Check the latest registered cloud buffer
  std::tuple<PointCloud, Time, Transform> cloudTimePair = slam_->getLatestRegisteredCloudTimestampPair();
  Transform calculatedTransform = std::get<2>(cloudTimePair);

  std::tuple<Time, Transform> bestGuessTimePair = slam_->getLatestRegistrationBestGuess();
  Transform bestGuessTransform = std::get<1>(bestGuessTimePair);

  geometry_msgs::PoseStamped bestGuessPoseStamped;
  Eigen::Quaterniond bestGuessRotation(bestGuessTransform.rotation());

  bestGuessPoseStamped.header.stamp = toRos(std::get<0>(bestGuessTimePair));
  bestGuessPoseStamped.pose.position.x = bestGuessTransform.translation().x();
  bestGuessPoseStamped.pose.position.y = bestGuessTransform.translation().y();
  bestGuessPoseStamped.pose.position.z = bestGuessTransform.translation().z();
  bestGuessPoseStamped.pose.orientation.w = bestGuessRotation.w();
  bestGuessPoseStamped.pose.orientation.x = bestGuessRotation.x();
  bestGuessPoseStamped.pose.orientation.y = bestGuessRotation.y();
  bestGuessPoseStamped.pose.orientation.z = bestGuessRotation.z();
  bestGuessPath_.poses.push_back(bestGuessPoseStamped);
  bestGuessPath_.header.stamp = toRos(std::get<0>(bestGuessTimePair));  // This guess supposed to be associated with the pose we give to it.
  bestGuessPath_.header.frame_id = slam_->frames_.mapFrame;

  if (offlineBestGuessPathPub_.getNumSubscribers() > 0u || offlineBestGuessPathPub_.isLatched()) {
    offlineBestGuessPathPub_.publish(bestGuessPath_);
  }

  geometry_msgs::PoseStamped poseStamped;
  Eigen::Quaterniond rotation(calculatedTransform.rotation());

  poseStamped.header.stamp = toRos(std::get<1>(cloudTimePair));
  poseStamped.pose.position.x = calculatedTransform.translation().x();
  poseStamped.pose.position.y = calculatedTransform.translation().y();
  poseStamped.pose.position.z = calculatedTransform.translation().z();
  poseStamped.pose.orientation.w = rotation.w();
  poseStamped.pose.orientation.x = rotation.x();
  poseStamped.pose.orientation.y = rotation.y();
  poseStamped.pose.orientation.z = rotation.z();
  trackedPath_.poses.push_back(poseStamped);

  outBag.write("/slam_optimized_poses", toRos(std::get<1>(cloudTimePair)), poseStamped);

  const double stamp = pointCloud->header.stamp.toSec();
  poseFile_ << stamp << " ";
  poseFile_ << calculatedTransform.translation().x() << " " << calculatedTransform.translation().y() << " "
            << calculatedTransform.translation().z() << " ";
  poseFile_ << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w() << std::endl;

  trackedPath_.header.stamp = toRos(std::get<1>(cloudTimePair));
  trackedPath_.header.frame_id = slam_->frames_.mapFrame;

  if (offlinePathPub_.getNumSubscribers() > 0u || offlinePathPub_.isLatched()) {
    offlinePathPub_.publish(trackedPath_);
  }

  drawLinesBetweenPoses(trackedPath_, bestGuessPath_, toRos(std::get<1>(cloudTimePair)));

  if (isTimeValid(std::get<1>(cloudTimePair)) && !(std::get<0>(cloudTimePair).IsEmpty())) {
    o3d_slam::publishCloud(std::get<0>(cloudTimePair), slam_->frames_.rangeSensorFrame, toRos(std::get<1>(cloudTimePair)),
                           registeredCloudPub_);
    ros::spinOnce();
  } else {
    ROS_ERROR_STREAM("Cloud is empty or time is invalid. Skipping the cloud.");
    return false;
  }

  if (surfaceNormalPub_.getNumSubscribers() > 0u || surfaceNormalPub_.isLatched()) {
    auto surfaceNormalLineMarker{
        generateMarkersForSurfaceNormalVectors(std::get<0>(cloudTimePair), toRos(std::get<1>(cloudTimePair)), colorMap_[ColorKey::kRed])};

    if (surfaceNormalLineMarker != std::nullopt) {
      // ROS_DEBUG("Publishing point cloud surface normals for publisher '%s'.", parameters_.pointCloudPublisherTopic_.c_str());
      surfaceNormalPub_.publish(surfaceNormalLineMarker.value());
    }
  }

  // Pop oldest input point cloud from the queue.
  buffer.pop_front();

  sensor_msgs::PointCloud2 outCloud;
  open3d_conversions::open3dToRos(std::get<0>(cloudTimePair), outCloud, slam_->frames_.rangeSensorFrame);
  outCloud.header.stamp = toRos(std::get<1>(cloudTimePair));
  outBag.write("/registered_cloud", toRos(std::get<1>(cloudTimePair)), outCloud);

  // Convert geometry_msgs::PoseStamped to geometry_msgs::TransformStamped
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = poseStamped.header.stamp;
  transformStamped.header.frame_id = slam_->frames_.mapFrame;
  transformStamped.child_frame_id = slam_->frames_.rangeSensorFrame;  // The child frame_id should be your robot's frame
  transformStamped.transform.translation.x = poseStamped.pose.position.x;
  transformStamped.transform.translation.y = poseStamped.pose.position.y;
  transformStamped.transform.translation.z = poseStamped.pose.position.z;
  transformStamped.transform.rotation = poseStamped.pose.orientation;

  // Encapsulate geometry_msgs::TransformStamped into tf2_msgs/TFMessage
  tf2_msgs::TFMessage tfMessage;
  tfMessage.transforms.push_back(transformStamped);

  // Write the TFMessage to the bag
  outBag.write("/tf", toRos(std::get<1>(cloudTimePair)), tfMessage);

  PointCloud& transformedCloud = std::get<0>(cloudTimePair);

  transformedCloud.Transform(std::get<2>(cloudTimePair).matrix());
  sensor_msgs::PointCloud2 transformedRosCloud;
  open3d_conversions::open3dToRos(transformedCloud, transformedRosCloud, slam_->frames_.rangeSensorFrame);
  transformedRosCloud.header.stamp = toRos(std::get<1>(cloudTimePair));
  outBag.write("/transformed_registered_cloud", toRos(std::get<1>(cloudTimePair)), transformedRosCloud);

  const ros::WallTime arbitrarySleep{ros::WallTime::now() + ros::WallDuration(slam_->relativeSleepDuration_)};
  ros::WallTime::sleepUntil(arbitrarySleep);

  return true;
}

std::optional<visualization_msgs::Marker> RosbagRangeDataProcessorRos::generateMarkersForSurfaceNormalVectors(
    const open3d::geometry::PointCloud& pointCloud, const ros::Time& timestamp, const o3d_slam::RgbaColorMap::Values& color) {
  if (pointCloud.IsEmpty()) {
    ROS_WARN("Point cloud is empty.");
    return {};
  }
  if (!pointCloud.HasNormals()) {
    ROS_WARN("Point cloud has no normals");
    return {};
  }

  std_msgs::ColorRGBA colorMsg;
  colorMsg.r = color[0];
  colorMsg.g = color[1];
  colorMsg.b = color[2];
  colorMsg.a = color[3];

  visualization_msgs::Marker vectorsMarker;
  vectorsMarker.header.stamp = timestamp;
  vectorsMarker.header.frame_id = slam_->frames_.rangeSensorFrame;
  vectorsMarker.ns = "surface_normals";
  vectorsMarker.action = visualization_msgs::Marker::ADD;
  vectorsMarker.type = visualization_msgs::Marker::LINE_LIST;
  vectorsMarker.pose.orientation.w = 1.0;
  vectorsMarker.id = 0;
  vectorsMarker.scale.x = 0.02;
  vectorsMarker.color = colorMsg;
  vectorsMarker.points.resize(pointCloud.points_.size() * 2);

  const auto& surfaceNormalsView = pointCloud.normals_;
  for (size_t i = 0; i < pointCloud.points_.size(); i += 2) {
    // The actual position of the point that the surface normal belongs to.
    vectorsMarker.points[i].x = pointCloud.points_[i][0];
    vectorsMarker.points[i].y = pointCloud.points_[i][1];
    vectorsMarker.points[i].z = pointCloud.points_[i][2];

    // End if arrow.
    vectorsMarker.points[i + 1].x = pointCloud.points_[i][0] + surfaceNormalsView[i][0] * 0.09;
    vectorsMarker.points[i + 1].y = pointCloud.points_[i][1] + surfaceNormalsView[i][1] * 0.09;
    vectorsMarker.points[i + 1].z = pointCloud.points_[i][2] + surfaceNormalsView[i][2] * 0.09;
  }

  return vectorsMarker;
}

void RosbagRangeDataProcessorRos::drawLinesBetweenPoses(const nav_msgs::Path& path1, const nav_msgs::Path& path2, const ros::Time& stamp) {
  if (!offlineDifferenceLinePub_.getNumSubscribers() > 0u && !offlineDifferenceLinePub_.isLatched()) {
    return;
  }

  if (path1.poses.size() != path2.poses.size()) {
    ROS_ERROR_STREAM("Path sizes are not equal. Skipping the line drawing.");
    return;
  }

  visualization_msgs::Marker line_list;
  line_list.header.frame_id = slam_->frames_.mapFrame;  // Change the frame_id as per your requirement
  line_list.header.stamp = stamp;
  line_list.ns = "paths";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.008;  // Line width

  // Setting color for the lines (you can change color as needed)
  line_list.color.r = 0.0;
  line_list.color.g = 1.0;
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;  // Alpha

  for (size_t i = 0; i < path1.poses.size(); i++) {
    geometry_msgs::Point p_start;
    p_start.x = path1.poses[i].pose.position.x;
    p_start.y = path1.poses[i].pose.position.y;
    p_start.z = path1.poses[i].pose.position.z;
    line_list.points.push_back(p_start);

    geometry_msgs::Point p_end;
    p_end.x = path2.poses[i].pose.position.x;
    p_end.y = path2.poses[i].pose.position.y;
    p_end.z = path2.poses[i].pose.position.z;
    line_list.points.push_back(p_end);
  }

  offlineDifferenceLinePub_.publish(line_list);
}

std::tuple<ros::WallDuration, ros::WallDuration, ros::WallDuration> RosbagRangeDataProcessorRos::usePairForRegistration() {
  const ros::WallTime odometryProcessingStartTime{ros::WallTime::now()};

  slam_->callofflineOdometryWorker();

  const auto odometryProcessingElapsed{ros::WallTime::now() - odometryProcessingStartTime};
  // ROS_INFO_STREAM("Odometry Operations took " << "\033[92m" << odometryProcessingElapsed.toNSec() / 1000000u << "ms" << "\033[0m");

  // Odometry is processed, now mapping
  const ros::WallTime mappingProcessingStartTime{ros::WallTime::now()};

  slam_->callofflineMappingWorker();

  const auto mappingProcessingElapsed{ros::WallTime::now() - mappingProcessingStartTime};
  ROS_INFO_STREAM("Mapping Operations took "
                  << "\033[92m" << mappingProcessingElapsed.toNSec() / 1000000u << "ms"
                  << "\033[0m");

  // Mapping is processed, now checking loop closures,
  const ros::WallTime loopclosureProcessingStartTime{ros::WallTime::now()};

  slam_->callofflineLoopClosureWorker();

  const auto loopclosureProcessingElapsed{ros::WallTime::now() - loopclosureProcessingStartTime};
  // ROS_INFO_STREAM("Loop closure Operations took " << "\033[92m" << loopclosureProcessingElapsed.toNSec() / 1000000u << "ms" <<
  // "\033[0m");

  return {std::make_tuple(odometryProcessingElapsed, mappingProcessingElapsed, loopclosureProcessingElapsed)};
}

bool RosbagRangeDataProcessorRos::readCalibrationIfNeeded() {
  if (slam_->useSyncedPoses_) {
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    baseToLidarTransform_ = tf2::eigenToTransform(transform);
    isStaticTransformFound_ = true;
    return true;
  }

  if (!isStaticTransformFound_ && (slam_->frames_.rangeSensorFrame != slam_->frames_.assumed_external_odometry_tracked_frame)) {
    try {
      auto transformation = tfBuffer_->lookupTransform(slam_->frames_.rangeSensorFrame,
                                                       slam_->frames_.assumed_external_odometry_tracked_frame, tracker, ros::Duration(0.0));
      ros::spinOnce();
      ROS_INFO_STREAM("\033[92m"
                      << "Found the transform between " << slam_->frames_.rangeSensorFrame << " and "
                      << slam_->frames_.assumed_external_odometry_tracked_frame << "\033[0m");
      ROS_INFO_STREAM("\033[92m"
                      << "You dont believe me? Here it is:\n " << transformation << "\033[0m");

      baseToLidarTransform_ = transformation;
      isStaticTransformFound_ = true;

    } catch (const tf2::TransformException& exception) {
      ROS_WARN_STREAM_THROTTLE(
          0.2, "Caught exception while looking for the transform Fingers Crossed it will appear soon: " << exception.what());
      return true;
    }
    return true;

  } else {
    ros::spinOnce();
    isStaticTransformFound_ = true;
    return true;
  }
}

bool RosbagRangeDataProcessorRos::processRosbag() {
  std::vector<std::string> topics;

  // Currently investigating if we really need clock or can we live without it.
  topics.push_back(clockTopic_);
  topics.push_back(cloudTopic_);
  topics.push_back(tfStaticTopic_);

  if (slam_->isUsingOdometryTopic()) {
    if (slam_->useSyncedPoses_) {
      // This is currently very specific
      topics.push_back(poseStampedTopic_);
    } else {
      // This is alternating between different common ROS msg types. I.e. poseStampedWithCovariance and Odometry
      topics.push_back(slam_->asyncOdometryTopic_);
    }
  }

  if (slam_->rePublishTf_) {
    ROS_INFO_STREAM(
        "\033[33m"
        << " So you like living risky? Tf will be republished. It is YOUR responsibility to ensure there is no frame duplication."
        << "\033[39m");
    topics.push_back(tfTopic_);
  }

  Timer rosbagTimer;

  // Open ROS bag.
  rosbag::Bag bag;
  try {
    bag.open(rosbagFilename_, rosbag::bagmode::Read);
  } catch (const rosbag::BagIOException& e) {
    ROS_ERROR_STREAM("Error opening ROS bag: '" << rosbagFilename_ << "'");
    return false;
  }
  ROS_INFO_STREAM("ROS bag '" << rosbagFilename_ << "' open.");

  if (!validateTopicsInRosbag(bag, topics)) {
    bag.close();
    return false;
  }

  ROS_INFO_STREAM("\033[92m"
                  << " Here wo go... "
                  << "\033[0m");
  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(2.0)};
  ros::WallTime::sleepUntil(first);

  // The bag view we iterate over.
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ros::Time stampLastIteration{view.getBeginTime()};
  ros::WallTime wallStampLastIteration{ros::WallTime::now()};
  const ros::WallTime wallStampStart{ros::WallTime::now()};

  o3dReplayBuffer o3dReplayBuffer;
  std::unique_ptr<SlamInputs> slamInputs;

  for (const auto& messageInstance : view) {
    // If the node is shutdown, stop processing and do early return.
    if (!ros::ok()) {
      return false;
    }

    if (!slamInputs) {
      slamInputs = std::make_unique<SlamInputs>();
    }

    bool isInvalidMessageInBag = false;

    // Update time registers.
    if ((ros::Time::now() - stampLastIteration) >= ros::Duration(1.0)) {
      stampLastIteration = ros::Time::now();
      wallStampLastIteration = ros::WallTime::now();
    }

    // Clock.
    if (messageInstance.getTopic() == clockTopic_) {
      if (messageInstance.getDataType() == "rosgraph_msgs/Clock") {
        rosgraph_msgs::Clock::ConstPtr clockMessage = messageInstance.instantiate<rosgraph_msgs::Clock>();
        if (clockMessage != nullptr) {
          clockPublisher_.publish(clockMessage);
          // Assign the current "time" to a global variable for async operations.
          tracker = clockMessage->clock;
        } else {
          ROS_ERROR("clockMsgs not initialized something is wrong.");
          isInvalidMessageInBag = true;
          return false;
        }

      } else if ((messageInstance.getDataType() == "nav_msgs/Odometry") ||
                 (messageInstance.getDataType() == "geometry_msgs/PoseWithCovarianceStamped")) {
        ROS_WARN_STREAM_ONCE("SHOWN ONCE: Using async odometry topic as clock master. This is sub-optimal. Topic: " << clockTopic_);

        rosgraph_msgs::Clock clockMessage;  // = messageInstance.instantiate<rosgraph_msgs::Clock>();
        clockMessage.clock = messageInstance.getTime();
        clockPublisher_.publish(clockMessage);
        // Assign the current "time" to a global variable for async operations.
        tracker = clockMessage.clock;

      } else {
        ROS_ERROR("This type of clock alternative is not supported");
        return false;
      }

      // rosgraph_msgs::Clock::ConstPtr clockMessage = messageInstance.instantiate<rosgraph_msgs::Clock>();
      // if (clockMessage != nullptr) {
      // clockPublisher_.publish(clockMessage);
      // Assign the current "time" to a global variable for async operations.
      // tracker = clockMessage->clock;

      // Spins once here.
      if (!readCalibrationIfNeeded()) {
        ROS_ERROR("Calibration failed to read. Exiting.");
        return false;
      }

      // Align the clock and the bag time.
      if (timeDiff_ == ros::Duration(0.0)) {
        timeDiff_ = tracker - view.getBeginTime();

        // If the bag time is ahead of the clock, we need to calculate the otherway around.
        if (timeDiff_ < ros::Duration(0.0)) {
          timeDiff_ = view.getBeginTime() - tracker;
        }

        ROS_INFO_STREAM("The calculated time difference between the clock and bag is: " << timeDiff_.toNSec() / 1000000u << "ms");
      }

      // Skip processes based on the required bag start time.
      if ((std::abs(((tracker + timeDiff_) - view.getBeginTime()).toSec() <= slam_->bagReplayStartTime_)) &&
          (slam_->bagReplayStartTime_ != 0.0)) {
        ROS_INFO_STREAM("Rosbag starting: " << std::abs(((tracker + timeDiff_) - view.getBeginTime()).toSec()) << " / "
                                            << slam_->bagReplayStartTime_ << " s");
        isBagReadyToPlay_ = false;
        continue;
      } else {
        isBagReadyToPlay_ = true;
      }

      // Process data in buffers.
      if (iterateCloudAndPoseBuffers(o3dReplayBuffer)) {
        const auto elapsedTimeSinceStart{ros::WallTime::now() - wallStampStart};
        ROS_INFO_STREAM("Replay run time: " << elapsedTimeSinceStart.toSec()
                                            << "s. ROS bag time: " << ((tracker + timeDiff_) - view.getBeginTime()).toSec() << " / "
                                            << (view.getEndTime() - view.getBeginTime()) << " s");

        // Play the bag until a certain time that the user decides.
        if (((tracker + timeDiff_) - view.getBeginTime()).toSec() >= slam_->bagReplayEndTime_) {
          break;
        }
      }
    }

    // Tf static.
    if (messageInstance.getTopic() == tfStaticTopic_) {
      tf2_msgs::TFMessage::ConstPtr message = messageInstance.instantiate<tf2_msgs::TFMessage>();
      if (message != nullptr) {
        staticTransformBroadcaster_.sendTransform(message->transforms);

      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    // Tf.
    if (messageInstance.getTopic() == tfTopic_) {
      tf2_msgs::TFMessage::ConstPtr message = messageInstance.instantiate<tf2_msgs::TFMessage>();
      if (message != nullptr) {
        transformBroadcaster_.sendTransform(message->transforms);

      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    if (!isBagReadyToPlay_) {
      continue;
    }

    // Pointcloud
    if (messageInstance.getTopic() == cloudTopic_) {
      sensor_msgs::PointCloud2::ConstPtr message = messageInstance.instantiate<sensor_msgs::PointCloud2>();
      if (message != nullptr) {
        /*ROS_ERROR_STREAM("message->fields[0].name: " << message->fields[0].name);
        ROS_ERROR_STREAM("message->fields[1].name: " << message->fields[1].name);
        ROS_ERROR_STREAM("message->fields[2].name: " << message->fields[2].name);
        ROS_ERROR_STREAM("message->fields[3].name: " << message->fields[3].name);
        ROS_ERROR_STREAM("message->fields[4].name: " << message->fields[4].name);
        ROS_ERROR_STREAM("message->fields[5].name: " << message->fields[5].name);
        ROS_ERROR_STREAM("message->fields[6].name: " << message->fields[6].name);
        ROS_ERROR_STREAM("message->fields[7].name: " << message->fields[7].name);
        ROS_ERROR_STREAM("message->fields[8].name: " << message->fields[8].name);
        ROS_ERROR_STREAM("message->fields[9].name: " << message->fields[9].name);
        ROS_ERROR_STREAM("message->fields[10].name: " << message->fields[10].name);
        ROS_ERROR_STREAM("message->fields[11].name: " << message->fields[11].name);
        ROS_ERROR_STREAM("message->fields[12].name: " << message->fields[12].name);
        */

        // Add point cloud to buffer.
        slamInputs->pointCloud_ = message;

        if (slamInputs->pointCloud_) {
          // Read the point cloud frame from the topic. Think whether having the option to change it makes sense.
          slam_->frames_.rangeSensorFrame = slamInputs->pointCloud_->header.frame_id;
          if (slam_->frames_.rangeSensorFrame != slamInputs->pointCloud_->header.frame_id) {
            ROS_ERROR("Tracked frame and PC frame are not same. This is not supported yet.");
            ROS_ERROR_STREAM("Tracked frame: " << slam_->frames_.rangeSensorFrame
                                               << " and PC frame: " << slamInputs->pointCloud_->header.frame_id);
            ROS_ERROR("Terminating the replayer node.");
            return false;
          }
        }

        if (!slam_->useSyncedPoses_) {
          if (slamInputs && slamInputs->pointCloud_) {
            // std::cout << " slamInputs IS HEALTHY adding to buffer  " << std::endl;
            o3dReplayBuffer.emplace_back(std::move(slamInputs));
          } else {
            std::cout << " slamInputs IS NOT HEALTHY  " << std::endl;
          }
        }

      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    if (slam_->useSyncedPoses_) {
      // Synced odometry poses. This expects the odometry poses exactly in the same timestamp as the pc msgs.
      if (messageInstance.getTopic() == poseStampedTopic_) {
        geometry_msgs::PoseStamped::ConstPtr message = messageInstance.instantiate<geometry_msgs::PoseStamped>();
        if (message != nullptr) {
          // Add pose to buffer.
          slamInputs->odometryPose_ = message;

        } else {
          isInvalidMessageInBag = true;
          ROS_WARN("Invalid message found in ROS bag.");
          return false;
        }
      }
    } else {
      // Async Msgs. Currently only supports geometry_msgs::PoseWithCovarianceStamped
      if (messageInstance.getTopic() == slam_->asyncOdometryTopic_) {
        if (messageInstance.getDataType() == "geometry_msgs/PoseWithCovarianceStamped") {
          geometry_msgs::PoseWithCovarianceStamped::ConstPtr message =
              messageInstance.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
          if (message != nullptr) {
            geometry_msgs::PoseStamped odomPose;
            odomPose.pose = message->pose.pose;

            geometry_msgs::PoseStamped odomPose_transformed;
            Eigen::Isometry3d eigenTransform = tf2::transformToEigen(baseToLidarTransform_);
            // geometry_msgs::TransformStamped inverseTransform = tf2::eigenToTransform(eigenTransform.inverse());
            slam_->setExternalOdometryFrameToCloudFrameCalibration(eigenTransform);
            tf2::doTransform(odomPose, odomPose_transformed, baseToLidarTransform_);

            if (isFirstMessage_ && isStaticTransformFound_) {
              geometry_msgs::PoseStamped initialPose = odomPose_transformed;

              // initialPose.position=odomPose.position;
              initialPose.pose.orientation.w = 1.0;
              initialPose.pose.orientation.z = 0.0;
              initialPose.pose.orientation.y = 0.0;
              initialPose.pose.orientation.x = 0.0;
              ROS_INFO("Initial Transform is set. Nice.");
              std::cout << " Initial Transform value (Rotation enforced to be Identity): "
                        << "\033[92m" << o3d_slam::asString(o3d_slam::getTransform(initialPose.pose)) << " \n"
                        << "\033[0m";
              slam_->setInitialTransform(o3d_slam::getTransform(initialPose.pose).matrix());
              isFirstMessage_ = false;
            }

            if (!(slam_->addOdometryPoseToBuffer(o3d_slam::getTransform(odomPose.pose), fromRos(message->header.stamp)))) {
              ROS_ERROR("Couldn't add pose to buffer. This is not unexpected, you should be concerned.");
              return false;
            }

          } else {
            isInvalidMessageInBag = true;
            ROS_WARN("Invalid message found in Rosbag, you are allowed to panic.");
            return false;
          }

        } else if (messageInstance.getDataType() == "nav_msgs/Odometry") {
          nav_msgs::Odometry::ConstPtr message = messageInstance.instantiate<nav_msgs::Odometry>();
          if (message != nullptr) {
            nav_msgs::Odometry odomPose;
            odomPose.pose.pose = message->pose.pose;

            nav_msgs::Odometry odomPose_transformed;
            Eigen::Isometry3d eigenTransform = tf2::transformToEigen(baseToLidarTransform_);
            // geometry_msgs::TransformStamped inverseTransform = tf2::eigenToTransform(eigenTransform.inverse());
            slam_->setExternalOdometryFrameToCloudFrameCalibration(eigenTransform);
            tf2::doTransform(odomPose.pose.pose, odomPose_transformed.pose.pose, baseToLidarTransform_);

            if (isFirstMessage_ && isStaticTransformFound_) {
              nav_msgs::Odometry initialPose = odomPose_transformed;

              initialPose.pose.pose.orientation.w = 1.0;
              initialPose.pose.pose.orientation.z = 0.0;
              initialPose.pose.pose.orientation.y = 0.0;
              initialPose.pose.pose.orientation.x = 0.0;
              ROS_INFO("Initial Transform is set. Nice.");
              slam_->setInitialTransform(o3d_slam::getTransform(initialPose.pose.pose).matrix());
              isFirstMessage_ = false;
            }

            if (!(slam_->addOdometryPoseToBuffer(o3d_slam::getTransform(odomPose.pose.pose), fromRos(message->header.stamp)))) {
              ROS_ERROR("Couldn't Add pose to buffer. This is not unexpected, you should be concerned.");
              return false;
            }

          } else {
            isInvalidMessageInBag = true;
            ROS_WARN("Invalid message found in Rosbag, you are allowed to panic.");
          }
        } else {
          ROS_WARN("This msg type is not supported yet. Exiting.");
          return false;
        }
      }  // else
    }

    if (slam_->isUsingOdometryTopic()) {
      if (slam_->useSyncedPoses_) {
        // Expecting sync odometry and PC.
        if (slamInputs && slamInputs->pointCloud_ && slamInputs->odometryPose_ &&
            slamInputs->pointCloud_->header.stamp == slamInputs->odometryPose_->header.stamp) {
          o3dReplayBuffer.emplace_back(std::move(slamInputs));
        }
      }
    }

    const ros::WallDuration processingWallDurationActual{ros::WallTime::now() - wallStampLastIteration};
    const ros::Duration processingDurationActual{tracker - stampLastIteration};
    const auto rosWallTimeRatio{processingDurationActual.toSec() / processingWallDurationActual.toSec()};

    if (maxProcessingRate_ <= 0) {
      continue;
    }

    // ROS time -> sleep in scaled wall time.
    if (rosWallTimeRatio > maxProcessingRate_) {
      // Check if we are reaching the desired processing rate. If not, sleep in scaled wall time.
      const auto desiredProcessingPeriod{ros::WallDuration(1.0 / maxProcessingRate_)};
      const auto scaledWallSleep = rosWallTimeRatio != 0 ? desiredProcessingPeriod.toSec() / rosWallTimeRatio : 0;
      // ROS_INFO_STREAM("Scaled sleep duration = " << scaledWallSleep);
      const ros::WallTime processingWallTimeExpected{ros::WallTime::now() + ros::WallDuration(scaledWallSleep)};
      ros::WallTime::sleepUntil(processingWallTimeExpected);

      // Update time register.
      wallStampLastIteration = ros::WallTime::now();
    }
  }

  iterateCloudAndPoseBuffers(o3dReplayBuffer);

  ROS_INFO("Finished running through the rosbag.");
  const ros::Time bag_begin_time = view.getBeginTime();
  const ros::Time bag_end_time = view.getEndTime();
  std::cout << "Rosbag processing finished. Rosbag duration: " << (bag_end_time - bag_begin_time).toSec()
            << " Time elapsed for running the rosbag: " << rosbagTimer.elapsedSec() << " sec. \n \n";

  bag.close();

  slam_->offlineFinishProcessing();

  return true;
}

}  // namespace o3d_slam
