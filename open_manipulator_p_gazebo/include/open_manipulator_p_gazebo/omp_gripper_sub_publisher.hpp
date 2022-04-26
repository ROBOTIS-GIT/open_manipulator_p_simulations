// Copyright 2019 ROBOTIS CO., LTD.
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
//
// Authors: Ashe Kim

#ifndef OPEN_MANIPULATOR_P_GAZEBO__OMP_GRIPPER_SUB_PUBLISHER_HPP_
#define OPEN_MANIPULATOR_P_GAZEBO__OMP_GRIPPER_SUB_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class OpenManipulatorPGazebo : public rclcpp::Node
{
public:
  OpenManipulatorPGazebo();
  ~OpenManipulatorPGazebo();

private:
  // ROS topic publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_joint_sub_pub;

  // ROS topic subscribers
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_joint_sub;


  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Function prototypes
  void gripperJointCallback(const std_msgs::msg::Float64::SharedPtr msg);
};
#endif  // OPEN_MANIPULATOR_P_GAZEBO__OMP_GRIPPER_SUB_PUBLISHER_HPP_