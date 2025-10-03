// Copyright 2025 LemperorD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KISS_MATCHER_RELOCALIZATION_HPP
#define KISS_MATCHER_RELOCALIZATION_HPP

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <kiss_matcher/KISSMatcher.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include "pcl/io/pcd_io.h"
#include "pcl/memory.h"

#include <pcl/filters/voxel_grid.h>
#include <memory>

namespace kiss_matcher_relocalization
{

class KissMatcherRelocalizationNode : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit KissMatcherRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  std::vector<Eigen::Vector3f> convertCloudToVec(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  void publishTransform();
  void performMatcher();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

  // int num_threads_;
  // int num_neighbors_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;
  float resolution_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  std::string current_scan_frame_id_;
  rclcpp::Time last_scan_time_;
  Eigen::Isometry3d result_t_;

  kiss_matcher::KISSMatcherConfig config_;
  std::unique_ptr<kiss_matcher::KISSMatcher> matcher_;
  kiss_matcher::RegistrationSolution solution_;

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_global_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_local_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_pcl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_pcl;
  std::vector<Eigen::Vector3f> target_vec;
  std::vector<Eigen::Vector3f> source_vec;
  std::vector<int> src_indices;
  std::vector<int> tgt_indices;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

}  //namespace kiss_matcher_relocalization_node

#endif //KISS_MATCHER_RELOCALIZATION_HPP