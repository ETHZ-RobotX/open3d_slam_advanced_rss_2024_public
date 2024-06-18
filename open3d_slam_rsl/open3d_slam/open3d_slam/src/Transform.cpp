/*
 * Transform.cpp
 *
 *  Created on: Nov 8, 2021
 *      Author: jelavice
 */

#include "open3d_slam/Transform.hpp"
#include <glog/logging.h>
#include <iostream>
#include <string>
#include "Eigen/Geometry"

namespace o3d_slam {

TimestampedTransform interpolate(const TimestampedTransform& start, const TimestampedTransform& end, const Time& time) {
  TimestampedTransform new_start;
  TimestampedTransform new_end;

  if (start.time_ > end.time_) {
    std::cout << "Interpolator: \n";
    std::cout << "Start time: " << toSecondsSinceFirstMeasurement(start.time_) << std::endl;
    std::cout << "End time: " << toSecondsSinceFirstMeasurement(end.time_) << std::endl;
    // throw std::runtime_error("transform interpolate:: start time is greater than end time");

    /*const Eigen::Vector3d origin = start.transform_.translation();
    const Eigen::Quaterniond rotation = Eigen::Quaterniond(start.transform_.rotation());
    Transform transform(rotation);
    transform.translation() = origin;

    return TimestampedTransform{time, transform};*/

    std::cout << "###############################################################################: \n";
    std::cout << "####################################### REPORT TO TURCAN ######################: \n";
    std::cout << "###############################################################################: \n";

    new_start.time_ = end.time_;
    new_end.time_ = start.time_;
    new_start.transform_ = end.transform_;
    new_end.transform_ = start.transform_;

  } else {
    new_start.time_ = start.time_;
    new_start.transform_ = start.transform_;
    new_end.time_ = end.time_;
    new_end.transform_ = end.transform_;
  }

  if (time > new_end.time_ || time < new_start.time_) {
    std::cout << "Interpolator: \n";
    std::cout << "Start time: " << toSecondsSinceFirstMeasurement(new_start.time_) << std::endl;
    std::cout << "End time: " << toSecondsSinceFirstMeasurement(new_end.time_) << std::endl;
    std::cout << "Query time: " << toSecondsSinceFirstMeasurement(time) << std::endl;
    throw std::runtime_error("transform interpolate:: query time is not between start and end time");
  }

  const double duration = toSeconds(new_end.time_ - new_start.time_);
  const double factor = toSeconds(time - new_start.time_) / (duration + 1e-6);  // avoid zero division
  const Eigen::Vector3d origin =
      new_start.transform_.translation() + (new_end.transform_.translation() - new_start.transform_.translation()) * factor;
  const Eigen::Quaterniond rotation =
      Eigen::Quaterniond(new_start.transform_.rotation()).slerp(factor, Eigen::Quaterniond(new_end.transform_.rotation()));
  Transform transform(rotation);
  transform.translation() = origin;

  return TimestampedTransform{time, transform};
}

TimestampedTransform extrapolate(const TimestampedTransform& start, const TimestampedTransform& end, const Time& future_time) {
  if (start.time_ > end.time_) {
    std::cerr << "Error: Start time is greater than end time.\n";
    throw std::runtime_error("transform extrapolate:: start time is greater than end time");
  }

  // Calculate the time difference between start and end
  const double duration = toSeconds(end.time_ - start.time_);
  if (duration <= 0) {
    std::cerr << "Error: Invalid time duration between start and end.\n";
    throw std::runtime_error("transform extrapolate:: invalid time duration between start and end");
  }

  // Calculate the velocities
  const Eigen::Vector3d start_translation = start.transform_.translation();
  const Eigen::Vector3d end_translation = end.transform_.translation();
  const Eigen::Vector3d translation_velocity = (end_translation - start_translation) / duration;

  const Eigen::Quaterniond start_rotation(start.transform_.rotation());
  const Eigen::Quaterniond end_rotation(end.transform_.rotation());
  const Eigen::Quaterniond rotation_diff = end_rotation * start_rotation.inverse();
  const Eigen::AngleAxisd rotation_velocity(rotation_diff);

  // Calculate the time difference to the future time
  const double future_duration = toSeconds(future_time - end.time_);
  if (future_duration < 0) {
    // std::cerr << "Error: Future time is before the end time.\n";
    // throw std::runtime_error("transform extrapolate:: future time is before end time");

    return end;
  }

  // Extrapolate the translation and rotation
  const Eigen::Vector3d future_translation = end_translation + translation_velocity * future_duration;
  const Eigen::Quaterniond future_rotation =
      end_rotation * Eigen::Quaterniond(Eigen::AngleAxisd(rotation_velocity.angle() * future_duration, rotation_velocity.axis()));

  Transform future_transform(future_rotation);
  future_transform.translation() = future_translation;

  return TimestampedTransform{future_time, future_transform};
}

Transform makeTransform(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
  Transform transform(q);
  transform.translation() = p;
  return transform;
}

} /* namespace o3d_slam */
