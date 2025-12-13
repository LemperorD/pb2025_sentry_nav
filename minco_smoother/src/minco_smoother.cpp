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

#include "minco_smoother/minco_smoother.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <nav2_smoother/smoother_utils.hpp>

namespace minco_smoother
{
  using nav2_util::declare_parameter_if_not_declared;

  void MincoSmoother::cleanup(){

  }

  void MincoSmoother::activate(){

  }

  void MincoSmoother::deactivate(){
    
  }

  void MincoSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub){

  }

  bool MincoSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time){
    flat_traj_ = getTrajDataFromPath(path);

  }

  FlatTrajData MincoSmoother::getTrajDataFromPath(const nav_msgs::msg::Path & path){
    bool reversing_segment;
    smoother_utils::updateApproximatePathOrientations(const_cast<nav_msgs::msg::Path &>(path), reversing_segment);

    Unoccupied_sample_trajs_.clear();
    double cur_theta;

  }

}; // namespace minco_smoother

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  minco_smoother::MincoSmoother, nav2_core::Smoother)