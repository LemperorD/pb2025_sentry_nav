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
#include "minco_smoother/lbfgs.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace minco_smoother
{

using nav2_util::declare_parameter_if_not_declared;

void MincoSmoother::cleanup(){
  costmap_sub_.reset();
  RCLCPP_INFO(logger_, "Cleaning up Minco Smoother");
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
  if (!node) { throw nav2_core::PlannerException("Unable to lock node!"); }

  costmap_sub_ = costmap_sub; tf_ = tf;
  plugin_name_ = name; logger_ = node->get_logger();

  current_state_VAJ_ << 0.0, 0.0, 0.0;
  current_state_OAJ_ << 0.0, 0.0, 0.0;

  //---declare parameters---
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_velocity", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_acc", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_omega", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_domega", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".safe_distance", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_jps_dis", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".distance_weight", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".yaw_weight", rclcpp::ParameterValue(0.9));
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
  node->get_parameter(plugin_name_ + ".min_traj_num", mintrajNum_);
  node->get_parameter(plugin_name_ + ".sample_time", sampletime_);
}

bool MincoSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time){
  start_pose_xytheta_ << path.poses.front().pose.position.x,
                         path.poses.front().pose.position.y,
                         tf2::getYaw(path.poses.front().pose.orientation);
  end_pose_xytheta_ << path.poses.back().pose.position.x,
                       path.poses.back().pose.position.y,
                       tf2::getYaw(path.poses.back().pose.orientation);

  flat_traj_ = getTrajDataFromPath(path);
  minco_plan(flat_traj_);
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
    cur_theta = atan2(path.poses[i+1].pose.position.y - path.poses[i].pose.position.y, 
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

  double totalTrajTime_ = evaluateDuration(Unoccupied_AllWeightingPathLength_, current_state_VAJ_.x(), 0.0, max_vel_, max_acc_);
  std::vector<Eigen::Vector3d> Unoccupied_traj_pts; // Store the sampled coordinates yaw, s, t
  std::vector<Eigen::Vector3d> Unoccupied_positions; // Store the sampled coordinates x, y, yaw

  double Unoccupied_totalTrajTime_ = totalTrajTime_;
  double Unoccupied_sampletime;
  int Unoccupied_PathNodeIndex = 1;

  Unoccupied_sampletime = Unoccupied_totalTrajTime_ / std::max(int(Unoccupied_totalTrajTime_ / sampletime_ + 0.5), mintrajNum_);

  double tmparc = 0;

  for(double samplet = Unoccupied_sampletime; samplet<Unoccupied_totalTrajTime_-1e-3; samplet+=Unoccupied_sampletime){
    double arc = evaluateLength(samplet, Unoccupied_AllWeightingPathLength_,
                                Unoccupied_totalTrajTime_, current_state_VAJ_.x(), 0.0, max_vel_, max_acc_);
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
  Eigen::Vector2d startP(Unoccupied_sample_trajs_[0][2],0);
  Eigen::Vector2d finalP(Unoccupied_sample_trajs_[PathNodeNum-1][2],Unoccupied_pathlengths[PathNodeNum-1]);
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
  flat_traj_.start_state_XYTheta = start_pose_xytheta_;
  flat_traj_.final_state_XYTheta = end_pose_xytheta_;;
    
  return flat_traj_;
}

bool MincoSmoother::minco_plan(FlatTrajData & flat_traj){
  bool final_collision = false;
  int replan_num_for_coll = 0;

  double start_safe_dis = map_->getDistanceReal(flat_traj.start_state_XYTheta.head(2))*0.85;
  safeDis = std::min(start_safe_dis, safeDis_);
  
  for(; replan_num_for_coll < safeReplanMaxTime; replan_num_for_coll++){
    if(get_state(flat_traj))
        RCLCPP_INFO(logger_, "\033[40;36m get_state time:%f \033[0m", (ros::Time::now()-current).toSec());
    else
        return false;
    current = ros::Time::now();
    if(optimizer())
        RCLCPP_INFO(logger_, "\033[41;37m minco optimizer time:%f \033[0m", (ros::Time::now()-current).toSec());
    else
        return false;

    Minco.getTrajectory(optimizer_traj_);
    final_collision = check_final_collision(optimizer_traj_, iniStateXYTheta);
    if(final_collision){
        penaltyWt.time_weight *= 0.75;
        // safeDis *= 1.2;
    }
    else{
        break;
    }
  }
  Collision_point_Pub();
  penaltyWt_.time_weight = penaltyWt.time_weight_backup_for_replan;
  safeDis = safeDis_;
  if(replan_num_for_coll == safeReplanMaxTime){
    ROS_ERROR("\n\n\n final traj Collision!!!!!!!!!!!\n\n\n\n");
    return false;
  }

  final_traj_ = optimizer_traj_;
  final_initStateXYTheta_ = iniStateXYTheta_;
  final_finStateXYTheta_ = finStateXYTheta_;
  Collision_point_Pub();

  return true;
}

bool MincoSmoother::get_state(const FlatTrajData &flat_traj){
  TrajNum = flat_traj.UnOccupied_traj_pts.size()+1;

  Innerpoints.resize(2,TrajNum-1);
  for(u_int i=0; i<flat_traj.UnOccupied_traj_pts.size(); i++){
      Innerpoints.col(i) = flat_traj.UnOccupied_traj_pts[i].head(2);
  }

  inner_init_positions = flat_traj.UnOccupied_positions;
  inner_init_positions.push_back(flat_traj.final_state_XYTheta);

  iniState = flat_traj.start_state;
  finState = flat_traj.final_state;

  pieceTime.resize(TrajNum); pieceTime.setOnes();
  pieceTime *= flat_traj.UnOccupied_initT;

  iniStateXYTheta = flat_traj.start_state_XYTheta;
  finStateXYTheta = flat_traj.final_state_XYTheta;
  
  return true;
}

bool MincoSmoother::optimizer(){
  EqualLambda = init_EqualLambda_;
  EqualRho = init_EqualRho_; 

  // 2*(N-1) intermediate points, 1 relaxed S, N times
  int variable_num_ = 3 * TrajNum_ - 1;
  // ROS_INFO_STREAM("iniStates: \n" << iniState_);
  // ROS_INFO_STREAM("finStates: \n" << finState_);
  // ROS_INFO("TrajNum: %d", TrajNum_);
  minco_.setConditions(iniState_, finState_, TrajNum_, energyWeights_);

  // ROS_INFO_STREAM("init Innerpoints: \n" << Innerpoints_);
  // ROS_INFO_STREAM("init pieceTime: " << pieceTime_.transpose());

  minco_.setParameters(Innerpoints_, pieceTime_);   
  minco_.getTrajectory(init_final_traj_);
  mincoPathPub(init_final_traj_, iniStateXYTheta_, mincoinitPath); 
  mincoPointPub(init_final_traj_, iniStateXYTheta_, mincoinitPoint, Eigen::Vector3d(173, 127, 168));
  Eigen::VectorXd x;
  x.resize(variable_num_);
  int offset = 0;
  memcpy(x.data()+offset,Innerpoints_.data(), Innerpoints_.size() * sizeof(x[0]));
  offset += Innerpoints_.size();
  x[offset] = finState_(1,0);
  ++offset;
  Eigen::Map<Eigen::VectorXd> Vt(x.data()+offset, pieceTime_.size());
  offset += pieceTime_.size();
  RealT2VirtualT(pieceTime_,Vt);

  double cost;
  int result;
  Eigen::VectorXd g;
  g.resize(x.size());
  iter_num_ = 0;

  auto start = std::chrono::high_resolution_clock::now();
  // Handle cases where the path is too short to converge
  if (fabs(finState_(1, 0)) < path_lbfgs_params_.shot_path_horizon) {
      path_lbfgs_params_.path_lbfgs_params.past = path_lbfgs_params_.shot_path_past;
  } else {
      path_lbfgs_params_.path_lbfgs_params.past = path_lbfgs_params_.normal_past;
  }

  ifprint = false;
  result = lbfgs::lbfgs_optimize(x,
                              cost,
                              MincoSmoother::costFunctionCallbackPath,
                              NULL,
                              NULL,
                              this,
                              path_lbfgs_params_.path_lbfgs_params);

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

  // Output computation time
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "pre_process";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 11.5;
  marker.pose.position.y = 7;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  double search_time = duration / 1000.0;
  std::ostringstream out;
  out << std::fixed << "Pre-process: \n"<< std::setprecision(2) << search_time<<" ms";
  marker.text = out.str();
  recordTextPub.publish(marker);

  ifprint = true;
  MincoSmoother::costFunctionCallbackPath(this,x,g);
  ifprint = false;

  ROS_INFO_STREAM("Pre-processing optimizer:" << duration / 1000.0 << " ms");
  ROS_INFO("Pre-processing finish! result:%d   finalcost:%f   iter_num_:%d", result, cost, iter_num_);
  offset = 0;
  Eigen::Map<Eigen::MatrixXd> PathP(x.data() + offset, 2, TrajNum_ - 1);
  offset += 2 * (TrajNum - 1);
  finalInnerpoints = PathP;
  finState(1, 0) = x[offset];
  ++offset;

  Eigen::Map<const Eigen::VectorXd> Patht(x.data() + offset, TrajNum_);
  offset += TrajNum;
  VirtualT2RealT(Patht, finalpieceTime);
  minco_.setTConditions(finState);
  minco_.setParameters(finalInnerpoints, finalpieceTime);
  minco_.getTrajectory(final_traj_);
  mincoPathPub(final_traj_, iniStateXYTheta, pathmincoinitPath);
  mincoPointPub(final_traj_, iniStateXYTheta, pathmincoinitPoint, Eigen::Vector3d(114, 159, 207));

  // ROS_INFO_STREAM("Path final pieces time: " << finalpieceTime.transpose());
  // ROS_INFO_STREAM("Path final Innerpoints: \n" << finalInnerpoints);
  // ROS_INFO_STREAM("Path final finState: \n" << finState);

  // Formal optimization
  // ROS_INFO("---------------------------------------------------------------------init---------------------------------------------------------------");
  // ifprint = true;
  // costFunctionCallback(this, x, g);
  // ifprint = false;
  ROS_INFO("-------------------------------------------------------------------optimize---------------------------------------------------------------");
  iter_num_ = 0;
  
  ros::Time current = ros::Time::now();

  while(ros::ok()){
      result = lbfgs::lbfgs_optimize(x,
                                      cost,
                                      MSPlanner::costFunctionCallback,
                                      NULL,
                                      MSPlanner::earlyExit,
                                      this,
                                      lbfgs_params_);
      if (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_CANCELED ||
          result == lbfgs::LBFGS_STOP||result == lbfgs::LBFGSERR_MAXIMUMITERATION){
          ROS_INFO("optimizer finish! result:%d   finalcost:%f   iter_num_:%d ",result,cost,iter_num_);
      } 
      else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH){
          ROS_WARN("Lbfgs: The line-search routine reaches the maximum number of evaluations.");
      }
      else{
          ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
      }
      // ALM update
      if(!ifCutTraj_){

          // ROS_INFO_STREAM("EqualLambda: " << EqualLambda.transpose() << "  EqualRho: " << EqualRho.transpose() << "  current hx cost:" << FinalIntegralXYError.transpose() << "  XYError.norm():" << FinalIntegralXYError.norm());
          if(FinalIntegralXYError.norm() < EqualTolerance_[0]){
              break;
          }
          EqualLambda[0] += EqualRho[0] * FinalIntegralXYError.x();
          EqualLambda[1] += EqualRho[1] * FinalIntegralXYError.y();
          EqualRho[0] = std::min((1 + EqualGamma_[0]) * EqualRho[0], EqualRhoMax_[0]);
          EqualRho[1] = std::min((1 + EqualGamma_[1]) * EqualRho[1], EqualRhoMax_[1]);
      }
      else{
          // ROS_INFO_STREAM("EqualLambda: " << EqualLambda.transpose() << "  EqualRho: " << EqualRho.transpose() << "  current hx cost:" << FinalIntegralXYError.transpose() << "  XYError.norm():" << FinalIntegralXYError.norm());
          if(FinalIntegralXYError.norm() < Cut_EqualTolerance_[0]){
              break;
          }
          EqualLambda[0] += EqualRho[0] * FinalIntegralXYError.x();
          EqualLambda[1] += EqualRho[1] * FinalIntegralXYError.y();
          EqualRho[0] = std::min((1 + Cut_EqualGamma_[0]) * EqualRho[0], Cut_EqualRhoMax_[0]);
          EqualRho[1] = std::min((1 + Cut_EqualGamma_[1]) * EqualRho[1], Cut_EqualRhoMax_[1]);

      }

  }

  double mincotime = (ros::Time::now()-current).toSec();
  ROS_INFO("\033[40;36m minco optimizer time:%f \033[0m", mincotime);
  ROS_INFO_STREAM("--------------------------------------------------------------------final------------------------------------------------------");

  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "minco";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 11.5;
  marker.pose.position.y = 6;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  search_time = mincotime*1000.0;
  std::ostringstream out2;
  out2 << std::fixed <<"Optimization: \n"<< std::setprecision(2) << search_time << " ms";
  marker.text = out2.str();
  recordTextPub.publish(marker);

  // Output optimization infomation
  // ifprint = true;
  // costFunctionCallback(this,x,g);
  
  offset = 0;
  Eigen::Map<Eigen::MatrixXd> P(x.data()+offset, 2, TrajNum - 1);
  offset += 2 * (TrajNum - 1);
  finalInnerpoints = P;

  finState(1,0) = x[offset];
  ++offset;

  Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, TrajNum);
  offset += TrajNum;
  VirtualT2RealT(t,finalpieceTime);
  minco_.setTConditions(finState);
  minco_.setParameters(finalInnerpoints, finalpieceTime);

  // std::cout<<"finalInnerpoints: \n"<<finalInnerpoints<<std::endl;
  // std::cout<<"finalpieceTime: \n"<<finalpieceTime.transpose()<<std::endl;

  ROS_INFO("\n\n optimizer finish! result:%d   finalcost:%f   iter_num_:%d\n\n ",result,cost,iter_num_);

  return true;
}

}; // namespace minco_smoother

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(minco_smoother::MincoSmoother, nav2_core::Smoother)