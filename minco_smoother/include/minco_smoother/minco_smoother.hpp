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

#ifndef NAV2_SMOOTHER__MINCO_SMOOTHER_HPP_
#define NAV2_SMOOTHER__MINCO_SMOOTHER_HPP_

#include <nav2_core/smoother.hpp>
#include <minco_smoother/traj_representation.hpp>
#include <minco_smoother/minco.hpp>

#include <tf2/utils.hpp>
#include <minco_smoother/minco_utils.hpp>

namespace minco_smoother
{

/**
 * @class minco_smoother::minco_smoother
 * @brief use minco to optimize the path
 */
class MincoSmoother : public nav2_core::Smoother
{
public:
  /**
   * @brief Constructor for minco_smoother::minco_smoother
   */
  MincoSmoother() = default;

  /**
   * @brief Destructor for minco_smoother::minco_smoother
   */
  ~MincoSmoother() override = default;

  /**
   * @brief Configure smoother state machine
   * 
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap2d_sub Subscriber for costmap2d
   * @param footprint_sub Subscriber for footprint
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap2d_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  /**
   * @brief Cleanup smoother state machine
   */
  void cleanup() override;

  /**
   * @brief Activate smoother state machine
   */
  void activate() override;

  /**
   * @brief Deactivate smoother state machine
   */
  void deactivate() override;
  
  /**
   * @brief Method to smooth given path
   *
   * @param path In-out path to be smoothed
   * @param max_time Maximum duration smoothing should take
   * @return If smoothing was completed (true) or interrupted by time limit (false)
   */
  bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) override;

protected:
  /**
   * @brief representation trajectory data from path
   * 
   * @param path Input path
   */
  FlatTrajData getTrajDataFromPath(const nav_msgs::msg::Path & path);

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  rclcpp::Logger logger_{rclcpp::get_logger("MincoSmoother")};

  std::vector<Eigen::VectorXd> Unoccupied_sample_trajs_; // x y theta dtheta ds
  std::vector<Eigen::VectorXd> cut_Unoccupied_sample_trajs_; // x y theta dtheta ds
  FlatTrajData flat_traj_;
  minco::Minco minco_solver_;

  //---parameters---
  double safe_dis_;
  double max_jps_dis_;
  double distance_weight_; double yaw_weight_;
  double trajCutLength_;
  double max_vel_; double max_acc_;
  double max_omega_; double max_domega_;
  double sampletime_;
  int mintrajNum_;


};

} // namespace minco_smoother

#endif //NAV2_SMOOTHER__MINCO_SMOOTHER_HPP_