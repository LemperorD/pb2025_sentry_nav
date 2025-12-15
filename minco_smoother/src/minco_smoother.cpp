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

namespace minco_smoother
{
  using nav2_util::declare_parameter_if_not_declared;

  void MincoSmoother::cleanup(){

  }

  void MincoSmoother::activate(){
    RCLCPP_INFO(logger_, "Activating Minco Smoother");
  }

  void MincoSmoother::deactivate(){
    RCLCPP_INFO(logger_, "Deactivating Minco Smoother");
  }

  void MincoSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub){
    auto node = parent.lock();
    node_ = parent;
    if (!node) {
      throw nav2_core::PlannerException("Unable to lock node!");
    }

    costmap_sub_ = costmap_sub;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();

    //---declare parameters---
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_velocity", rclcpp::ParameterValue(0.9));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_acc", rclcpp::ParameterValue(0.9));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_omega", rclcpp::ParameterValue(0.9));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_domega", rclcpp::ParameterValue(0.9));
    declare_parameter_if_not_declared(node, plugin_name_ + ".safe_distance", rclcpp::ParameterValue(0.9));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_jps_dis", rclcpp::ParameterValue(0.9));
    declare_parameter_if_not_declared(node, plugin_name_ + ".distance_weight", rclcpp::ParameterValue(0.9));
    declare_parameter_if_not_declared(node, plugin_name_ + ".yaw_weight", rclcpp::ParameterValue(0.9));
    declare_parameter_if_not_declared(node, plugin_name_ + ".traj_cut_length", rclcpp::ParameterValue(0.9));
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_traj_num", rclcpp::ParameterValue(0.9));
    declare_parameter_if_not_declared(node, plugin_name_ + ".sample_time", rclcpp::ParameterValue(0.9));
    
    //---get parameters---
    node->get_parameter(plugin_name_ + ".max_velocity", max_vel_);
    node->get_parameter(plugin_name_ + ".max_acc", max_acc_);
    node->get_parameter(plugin_name_ + ".max_omega", max_omega_);
    node->get_parameter(plugin_name_ + ".max_domega", max_domega_);
    node->get_parameter(plugin_name_ + ".safe_distance", safe_dis_);
    node->get_parameter(plugin_name_ + ".max_jps_dis", max_jps_dis_);
    node->get_parameter(plugin_name_ + ".distance_weight", distance_weight_);
    node->get_parameter(plugin_name_ + ".yaw_weight", yaw_weight_);
    node->get_parameter(plugin_name_ + ".traj_cut_length", trajCutLength_);
    node->get_parameter(plugin_name_ + ".min_traj_num", mintrajNum_);
    node->get_parameter(plugin_name_ + ".sample_time", sampletime_);
  }

  bool MincoSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time){
    flat_traj_ = getTrajDataFromPath(path);



    return true;
  }

  FlatTrajData MincoSmoother::getTrajDataFromPath(const nav_msgs::msg::Path & path){
    //-------add Yaw and pathlength to each traj point---------
    Unoccupied_sample_trajs_.clear();
    double cur_theta;

    Eigen::VectorXd state5d; state5d.resize(5);// x y theta dtheta ds 
    state5d << path.poses[0].pose.position.x, path.poses[0].pose.position.y, 0, 0, 0;
    Unoccupied_sample_trajs_.push_back(state5d); 
    
    cur_theta = atan2(path.poses[1].pose.position.y - path.poses[0].pose.position.y, path.poses[1].pose.position.x - path.poses[0].pose.position.x);
    normalizeAngle(0, cur_theta);
    state5d << path.poses[0].pose.position.x, path.poses[0].pose.position.y, cur_theta , cur_theta, 0;
    Unoccupied_sample_trajs_.push_back(state5d);

    cur_theta = atan2(path.poses[0].pose.position.y - path.poses[1].pose.position.y, path.poses[0].pose.position.x - path.poses[1].pose.position.x) + M_PI;
    normalizeAngle(0, cur_theta);
    state5d << path.poses[0].pose.position.x, path.poses[0].pose.position.y, cur_theta , cur_theta, 0;
    Unoccupied_sample_trajs_.push_back(state5d); // 2
    
    int path_size = path.poses.size();
    geometry_msgs::msg::PoseStamped pt;
    for(int i = 1; i<path_size-1; i++){ 
      pt = path.poses[i];

      state5d << pt.pose.position.x, pt.pose.position.y, Unoccupied_sample_trajs_.back()[2], 0, 
                 hypot(pt.pose.position.x - Unoccupied_sample_trajs_.back()[0], pt.pose.position.y - Unoccupied_sample_trajs_.back()[1]);
      Unoccupied_sample_trajs_.push_back(state5d);
      cur_theta = atan2(path.poses[i+1].pose.position.y - path.poses[i+1].pose.position.y, 
                        path.poses[i].pose.position.x - path.poses[i].pose.position.x);
      normalizeAngle(Unoccupied_sample_trajs_.back()[2], cur_theta);
      state5d << pt.pose.position.x, pt.pose.position.y, cur_theta, cur_theta - Unoccupied_sample_trajs_.back()[2], 0;
      Unoccupied_sample_trajs_.push_back(state5d);
    }

    pt = path.poses.back();
    state5d << pt.pose.position.x, pt.pose.position.y, Unoccupied_sample_trajs_.back()[2], 0, 
               hypot(pt.pose.position.x - Unoccupied_sample_trajs_.back()[0], pt.pose.position.y - Unoccupied_sample_trajs_.back()[1]);
    Unoccupied_sample_trajs_.push_back(state5d);

    cur_theta = tf2::getYaw(pt.pose.orientation);
    normalizeAngle(Unoccupied_sample_trajs_.back()[2], cur_theta);
    state5d << pt.pose.position.x, pt.pose.position.y, cur_theta, cur_theta - Unoccupied_sample_trajs_.back()[2], 0;
    Unoccupied_sample_trajs_.push_back(state5d);
    //-------add Yaw and pathlength to each traj point finish---------

    std::vector<double> Unoccupied_thetas;
    std::vector<double> Unoccupied_pathlengths; 
    std::vector<double> Unoccupied_Weightpathlengths; 

    double Unoccupied_AllWeightingPathLength_ = 0; 
    double Unoccupied_AllPathLength = 0;

    int PathNodeNum = Unoccupied_sample_trajs_.size();
    Unoccupied_thetas.push_back(Unoccupied_sample_trajs_[0][2]);
    Unoccupied_pathlengths.push_back(0);    
    Unoccupied_Weightpathlengths.push_back(0);

    int pathnodeindex = 1;
    for(; pathnodeindex<PathNodeNum; pathnodeindex++){
      Eigen::VectorXd pathnode = Unoccupied_sample_trajs_[pathnodeindex];
      Unoccupied_thetas.push_back(pathnode[2]);
      Unoccupied_AllPathLength += pathnode[4];
      Unoccupied_pathlengths.push_back(Unoccupied_AllPathLength); 
      Unoccupied_AllWeightingPathLength_ += yaw_weight_ * abs(pathnode[3]) + distance_weight_ * abs(pathnode[4]);
      Unoccupied_Weightpathlengths.push_back(Unoccupied_AllWeightingPathLength_);
    }

    double totalTrajTime_ = evaluateDuration(Unoccupied_AllWeightingPathLength_, current_state_VAJ_.x(),0.0,max_vel_,max_acc_);
    std::vector<Eigen::Vector3d> Unoccupied_traj_pts; // Store the sampled coordinates yaw, s, t
    std::vector<Eigen::Vector3d> Unoccupied_positions; // Store the sampled coordinates x, y, yaw

    double Unoccupied_totalTrajTime_ = totalTrajTime_;
    double Unoccupied_sampletime;
    int Unoccupied_PathNodeIndex = 1;

    Unoccupied_sampletime = Unoccupied_totalTrajTime_ / std::max(int(Unoccupied_totalTrajTime_ / sampletime_ + 0.5), mintrajNum_);

    double tmparc = 0;

    for(double samplet = Unoccupied_sampletime; samplet<Unoccupied_totalTrajTime_-1e-3; samplet+=Unoccupied_sampletime){
        double arc = evaluateLength(samplet, Unoccupied_AllWeightingPathLength_, Unoccupied_totalTrajTime_, current_state_VAJ_.x(), 0.0, max_vel_, max_acc_);
        for (int k = Unoccupied_PathNodeIndex; k<PathNodeNum; k++){
            Eigen::VectorXd pathnode = Unoccupied_sample_trajs_[k];
            Eigen::VectorXd prepathnode = Unoccupied_sample_trajs_[k-1];
            tmparc = Unoccupied_Weightpathlengths[k];
            if(tmparc >= arc){
                Unoccupied_PathNodeIndex = k; 
                double l1 = tmparc-arc;
                double l = Unoccupied_Weightpathlengths[k]-Unoccupied_Weightpathlengths[k-1];
                double interp_s = Unoccupied_pathlengths[k-1] + (l-l1)/l*(pathnode[4]);
                double interp_yaw = Unoccupied_sample_trajs_[k-1][2] + (l-l1)/l*(pathnode[3]);
                Unoccupied_traj_pts.emplace_back(interp_yaw, interp_s, samplet);

                double interp_x = l1/l*prepathnode[0] + (l-l1)/l*(pathnode[0]);
                double interp_y = l1/l*prepathnode[1] + (l-l1)/l*(pathnode[1]);
                Unoccupied_positions.emplace_back(interp_x, interp_y, interp_yaw);
                break;
            }
        }
    }

    Eigen::MatrixXd startS;
    Eigen::MatrixXd endS;
    startS.resize(2,3);
    endS.resize(2,3);  
    Eigen::Vector2d startP(cut_Unoccupied_sample_trajs_[0][2],0);
    Eigen::Vector2d finalP(cut_Unoccupied_sample_trajs_[PathNodeNum-1][2],Unoccupied_pathlengths[PathNodeNum-1]);
    startS.col(0) = startP;
    startS.block(0,1,1,2) = current_state_OAJ_.transpose().head(2);
    startS.block(1,1,1,2) = current_state_VAJ_.transpose().head(2);
    endS.col(0) = finalP;
    endS.col(1).setZero();
    endS.col(2).setZero();
    flat_traj_.UnOccupied_traj_pts = Unoccupied_traj_pts;
    flat_traj_.UnOccupied_initT = Unoccupied_sampletime;
    flat_traj_.UnOccupied_positions = Unoccupied_positions;
  
    flat_traj_.start_state = startS;
    flat_traj_.final_state = endS;
    flat_traj_.start_state_XYTheta = start_state_;
    flat_traj_.final_state_XYTheta = end_state_;
    flat_traj_.if_cut = false;
    
    return flat_traj_;
  }

  void MincoSmoother::minco_plan(FlatTrajData & flat_traj){
    //---minco smoother plan---

  }

}; // namespace minco_smoother

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  minco_smoother::MincoSmoother, nav2_core::Smoother)