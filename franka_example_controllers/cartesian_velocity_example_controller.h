// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_example_controllers {

class CartesianVelocityExampleController : public controller_interface::MultiInterfaceController<
                                               franka_hw::FrankaVelocityCartesianInterface,
                                               franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;
  ros::Duration elapsed_time_;
  bool tactile_exploration;
  bool direction_down;
  bool direction_right;
  bool direction_left;
  bool direction_up;
  bool direction_forward;
  bool direction_forward_2;
  bool direction_back;
  double v_y_to_init_pose;
  double v_z_to_init_pose;
  double v_x;
  double v_y;
  double v_z;
  double a_x;
  double a_y;
  double a_z;
  double d_x;
  double d_y;
  double d_z;
  double v_x_final;
  double v_y_final;
  double v_z_final;
  double internal_time;
  int count;
  int total_count;
  int phase;
  int repetition;
  int total_repetitions;
  bool changing_lanes;
  bool go_initial;
  int waiting_counter;
};

}  // namespace franka_example_controllers
