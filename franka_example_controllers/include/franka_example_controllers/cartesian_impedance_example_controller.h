// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/TwistStamped.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_example_controllers {

class CartesianImpedanceExampleController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double nullspace_stiffness_{10.0};
  double nullspace_stiffness_target_{10.0};
  double dt{0.001};
  double time_start;
  int alpha;
  int filter_step{0};
  int filter_step_;
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d vel_d;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 6, 1> force_torque;
  Eigen::Matrix<double, 6, 1> force_torque_old;
  Eigen::Matrix<float, 7, 1> stiff_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config,
                               uint32_t level);

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  // Configuration pose subscriber
  ros::Subscriber sub_equilibrium_config_;
  void equilibriumConfigurationCallback( const std_msgs::Float32MultiArray::ConstPtr& joint);

  // Velocity subscriber
  ros::Subscriber sub_equilibrium_velocity_;
  void equilibriumVelocityCallback( const geometry_msgs::TwistStampedConstPtr& velocity);

  // Multi directional stiffness stiffnes 
  ros::Subscriber sub_stiffness_;
  void equilibriumStiffnessCallback(const std_msgs::Float32MultiArray::ConstPtr& stiffness_);

  
  ros::Publisher pub_stiff_update_;                                                
  ros::Publisher pub_cartesian_pose_;
  ros::Publisher pub_force_torque_;
  
                                                
                                                  

};


}  // namespace franka_example_controllers
