/*
 * Odometry.cpp
 *
 *  Created on: Oct 15, 2021
 *      Author: jelavice
 */
#include "open3d_slam/Odometry.hpp"
#include "open3d_slam/CloudRegistration.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/time.hpp"

#include <iostream>

namespace o3d_slam {

LidarOdometry::LidarOdometry() {
  cropper_ = std::make_shared<CroppingVolume>();
  cloudRegistration_ = cloudRegistrationFactory(params_.scanMatcher_);
}

PointCloudPtr LidarOdometry::preprocess(const PointCloud& in) const {
  auto croppedCloud = cropper_->crop(in);
  o3d_slam::voxelize(params_.scanProcessing_.voxelSize_, croppedCloud.get());
  cloudRegistration_->estimateNormalsOrCovariancesIfNeeded(croppedCloud.get());
  return croppedCloud->RandomDownSample(params_.scanProcessing_.downSamplingRatio_);
}

bool LidarOdometry::addRangeScan(const open3d::geometry::PointCloud& cloud, const Time& timestamp) {
  // If the previous cloud is empty then this is the first measurement.
  if (cloudPrev_.IsEmpty()) {
    auto preProcessed = preprocess(cloud);
    cloudPrev_ = *preProcessed;
    odomToRangeSensorBuffer_.push(timestamp, odomToRangeSensorCumulative_);
    lastMeasurementTimestamp_ = timestamp;
    return true;
  }

  // We return early if we don't need to employ scan2scan odometry.
  if (params_.useOdometryTopic_) {
    // std::cout << "Already an odometry measurement for this timestamp. Skipping" << std::endl;
    return true;
  }

  if (timestamp < lastMeasurementTimestamp_) {
    std::cerr << "\n\n !!!!! LIDAR ODOMETRY WARNING: Measurements came out of order!!!! This might happen at the beginning once. \n\n";
    return false;
  }

  const o3d_slam::Timer timer;
  auto preProcessed = preprocess(cloud);

  const auto result = cloudRegistration_->registerClouds(cloudPrev_, *preProcessed, Transform::Identity());

  // I want to prevent big jumps in odometry and make isOdomOkay false based on result.transformation
  Transform resT = Transform(result.transformation_);

  if (resT.translation().norm() > 0.8) {
    std::cout << "\033[33m"
              << "LiDAR Odometry jumped more than 80cm. Skipping this estimation."
              << "\033[0m" << std::endl;
    return false;
  }

  // todo magic
  const bool isOdomOkay = result.fitness_ > 0.1;
  if (!isOdomOkay) {
    std::cout << "Odometry failed!!!!! \n";
    std::cout << "Size of the odom buffer: " << odomToRangeSensorBuffer_.size() << std::endl;
    std::cout << "Scan matching time elapsed: " << timer.elapsedMsec() << " msec \n";
    std::cout << "Fitness: " << result.fitness_ << "\n";
    std::cout << "RMSE: " << result.inlier_rmse_ << "\n";
    std::cout << "Transform: \n" << asString(Transform(result.transformation_)) << "\n";
    std::cout << "target size: " << cloud.points_.size() << std::endl;
    std::cout << "reference size: " << cloudPrev_.points_.size() << std::endl;
    std::cout << "\n \n";
    if (!preProcessed->IsEmpty()) {
      cloudPrev_ = std::move(*preProcessed);
    }
    return isOdomOkay;
  }

  if (isInitialTransformSet_) {
    odomToRangeSensorCumulative_.matrix() = initialTransform_;
    isInitialTransformSet_ = false;
  } else {
    odomToRangeSensorCumulative_.matrix() *= result.transformation_.inverse();
  }

  cloudPrev_ = std::move(*preProcessed);
  odomToRangeSensorBuffer_.push(timestamp, odomToRangeSensorCumulative_);
  lastMeasurementTimestamp_ = timestamp;
  return isOdomOkay;
}

const Transform LidarOdometry::getOdomToRangeSensor(const Time& t) const {
  return getTransform(t, odomToRangeSensorBuffer_);
}

const open3d::geometry::PointCloud& LidarOdometry::getPreProcessedCloud() const {
  return cloudPrev_;
}

const TransformInterpolationBuffer& LidarOdometry::getBuffer() const {
  return odomToRangeSensorBuffer_;
}

bool LidarOdometry::hasProcessedMeasurements() const {
  return !odomToRangeSensorBuffer_.empty();
}

void LidarOdometry::setParameters(const OdometryParameters& p) {
  params_ = p;
  cropper_ = croppingVolumeFactory(params_.scanProcessing_.cropper_);
  cloudRegistration_ = cloudRegistrationFactory(params_.scanMatcher_);
}

void LidarOdometry::setInitialTransform(const Eigen::Matrix4d& initialTransform) {
  // todo decide what to do
  //  if I uncomment stuff below the odom jumps but starts from the pose you specified
  //  if I leave it like this it is always continuous, but starts always from the
  //  origin
  if (isInitialTransformSet_) {
    std::cout << "\033[31m"
              << "Open3d_slam odometry initial transform already set. Skipping. OK to see in the beginning."
              << "\033[0m" << std::endl;
    return;
  }

  // Set to global variables.
  initialTransform_ = initialTransform;
  odomToRangeSensorCumulative_ = Transform(initialTransform);
  isInitialTransformSet_ = true;
}

}  // namespace o3d_slam
