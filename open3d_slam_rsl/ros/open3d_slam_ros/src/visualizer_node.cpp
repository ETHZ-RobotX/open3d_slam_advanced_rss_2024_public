/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <open3d/Open3D.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// open3d_conversions
#include "open3d_conversions/open3d_conversions.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_visualizer");
  ros::NodeHandle nh("~");

  // Get the PCD file path from the parameter server
  std::string pcd_file_path;
  nh.param<std::string>("pcd_file_path", pcd_file_path, "");

  if (pcd_file_path.empty()) {
    ROS_ERROR("No PCD file path provided. Please set the 'pcd_file_path' parameter.");
    return -1;
  }

  // Publisher for the point cloud
  ros::Publisher pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("loaded_map", 1);
  auto cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();

  // Convert to ROS message
  sensor_msgs::PointCloud2 ros_cloud;
  if (open3d::io::ReadPointCloud(pcd_file_path, *cloud_ptr)) {
    ROS_INFO("Successfully loaded PCD file: %s", pcd_file_path.c_str());

    open3d_conversions::open3dToRos(*cloud_ptr, ros_cloud, "map");
    ros_cloud.header.stamp = ros::Time::now();

    ROS_INFO("Point cloud published.");
  } else {
    ROS_ERROR("Failed to load PCD file: %s", pcd_file_path.c_str());
  }

  ros::Duration(2).sleep();

  ros::Rate rate(0.1);
  while (ros::ok()) {
    // Publish the point cloud
    pointcloud_pub.publish(ros_cloud);
    ROS_INFO_THROTTLE(0.1, "Point cloud published.");

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
