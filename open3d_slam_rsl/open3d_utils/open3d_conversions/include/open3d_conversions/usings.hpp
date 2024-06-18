#pragma once

#include <pointmatcher_ros/PmTf.h>
#include <pointmatcher_ros/StampedPointCloud.h>

#include <string_view>

namespace open3d_conversions {

// Point cloud name as view
constexpr std::string_view normalStringView{"normals"};

// Pointmatcher types
using PM = PointMatcher<float>;
using PmDataPoints = PM::DataPoints;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using PmIcp = pointmatcher_ros::PmIcp;
using PmMatches = pointmatcher_ros::PmMatches;
using PmMatrix = pointmatcher_ros::PmMatrix;
using PmStampedPointCloud = pointmatcher_ros::StampedPointCloud;
using PmTf = pointmatcher_ros::PmTf;
using PmTfParameters = pointmatcher_ros::PmTfParameters;
using PmPointCloudFilters = pointmatcher_ros::PmPointCloudFilters;
typedef typename std::vector<Eigen::Matrix<float, -1, -1>, Eigen::aligned_allocator<Eigen::Matrix<float, -1, -1>>> vectorMat;

}  // namespace open3d_conversions