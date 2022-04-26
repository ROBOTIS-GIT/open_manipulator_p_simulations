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

#include "open_manipulator_p_gazebo/omp_gripper_sub_publisher.hpp"
#include <memory>
using namespace std::chrono_literals;


OpenManipulatorPGazebo::OpenManipulatorPGazebo()
: Node("open_manipulator_p_gazebo_node")
{
  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  gripper_joint_sub_pub = this->create_publisher<std_msgs::msg::Float64>("gripper_sub_position/command", qos);

  // Initialise subscribers
  gripper_joint_sub = this->create_subscription<std_msgs::msg::Float64>(
    "gripper_position/command", qos, std::bind(&OpenManipulatorPGazebo::gripperJointCallback, this, std::placeholders::_1));
}

OpenManipulatorPGazebo::~OpenManipulatorPGazebo()
{
  RCLCPP_INFO(this->get_logger(), "Open manipulator p gazebo node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void OpenManipulatorPGazebo::gripperJointCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  std_msgs::msg::Float64 pub_msg;
  pub_msg.data = msg->data;
  gripper_joint_sub_pub->publish(pub_msg);
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenManipulatorPGazebo>());
  rclcpp::shutdown();

  return 0;
}
