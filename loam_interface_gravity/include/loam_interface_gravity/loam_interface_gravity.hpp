#ifndef LOAM_INTERFACE_G_HPP_
#define LOAM_INTERFACE_G_HPP_

#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace loam_interface_g
{

class LoamInterfaceGNode : public rclcpp::Node
{
public:
  explicit LoamInterfaceGNode(const rclcpp::NodeOptions & options);

private:
  inline tf2::Transform gravityAlign(tf2::Transform tf_notAlign);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string state_estimation_topic_;
  std::string registered_scan_topic_;
  std::string odom_frame_;
  std::string lidar_frame_;
  std::string base_frame_;

  bool base_frame_to_lidar_initialized_;
  tf2::Transform tf_odom_to_lidar_odom_;
};

}  // namespace loam_interface_g

#endif  // LOAM_INTERFACE_G_HPP_
