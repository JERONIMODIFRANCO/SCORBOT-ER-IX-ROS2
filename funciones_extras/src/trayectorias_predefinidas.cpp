/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

// #include <moveit_visual_tools/moveit_visual_tools.h>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("Planeador");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "arm";

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // // Now, we call the planner to compute the plan and visualize it.
  // // Note that we are just planning, not asking move_group
  // // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success;
  bool within_bounds;

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  //
  // Next get the current set of joint values for the group.
  std::vector<double> pose_1;
  std::vector<double> pose_2;
  std::vector<double> pose_3;
  std::vector<double> pose_4;
  std::vector<double> pose_5;
  std::vector<double> pose_6;
  current_state->copyJointGroupPositions(joint_model_group, pose_1);
  current_state->copyJointGroupPositions(joint_model_group, pose_2);
  current_state->copyJointGroupPositions(joint_model_group, pose_3);
  current_state->copyJointGroupPositions(joint_model_group, pose_4);
  current_state->copyJointGroupPositions(joint_model_group, pose_5);
  current_state->copyJointGroupPositions(joint_model_group, pose_6);
  
  move_group.setMaxVelocityScalingFactor(0.75);
  move_group.setMaxAccelerationScalingFactor(0.75);

  
  // Pose 1
  pose_1[0] = 0;  // radians
  pose_1[1] = 0;  // radians
  pose_1[2] = 0;  // radians
  pose_1[3] = 0;  // radians
  pose_1[4] = 0;  // radians

  pose_2[0] = 0;  // radians
  pose_2[1] = -1.57;  // radians
  pose_2[2] = 1.57;  // radians
  pose_2[3] = 0;  // radians
  pose_2[4] = 0;  // radians

  pose_3[0] = 1.57;  // radians
  pose_3[1] = 0.262;  // radians
  pose_3[2] = -0.698;  // radians
  pose_3[3] = 0;  // radians
  pose_3[4] = 0;  // radians

  pose_4[0] = -1.57;  // radians
  pose_4[1] = 0.262;  // radians
  pose_4[2] = -0.698;  // radians
  pose_4[3] = 0;  // radians
  pose_4[4] = 0;  // radians

  pose_5[0] = 0;  // radians
  pose_5[1] = 0;  // radians
  pose_5[2] = 0;  // radians
  pose_5[3] = -1.483;  // radians
  pose_5[4] = -3.1415;  // radians

  pose_6[0] = 0;  // radians
  pose_6[1] = 0;  // radians
  pose_6[2] = 0;  // radians
  pose_6[3] = 1.483;  // radians
  pose_6[4] = 3.1415;  // radians

  // Pose 1
  within_bounds = move_group.setJointValueTarget(pose_1);
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }
  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Ejecutando la Pose 1");
  move_group.move();
  static auto start_time_ = rclcpp::Clock().now();
  while(start_time_.seconds() > rclcpp::Clock().now().seconds() - 2){}

  // Pose 2
  within_bounds = move_group.setJointValueTarget(pose_2);
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }
  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Ejecutando la Pose 2");
  move_group.move();
  start_time_ = rclcpp::Clock().now();
  while(start_time_.seconds() > rclcpp::Clock().now().seconds() - 2){}

  // Pose 3
  within_bounds = move_group.setJointValueTarget(pose_3);
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }
  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Ejecutando la Pose 3");
  move_group.move();
  start_time_ = rclcpp::Clock().now();
  while(start_time_.seconds() > rclcpp::Clock().now().seconds() - 2){}

  // Pose 2
  within_bounds = move_group.setJointValueTarget(pose_2);
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }
  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Ejecutando la Pose 2");
  move_group.move();
  start_time_ = rclcpp::Clock().now();
  while(start_time_.seconds() > rclcpp::Clock().now().seconds() - 2){}

  // Pose 4
  within_bounds = move_group.setJointValueTarget(pose_4);
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }
  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Ejecutando la Pose 4");
  move_group.move();
  start_time_ = rclcpp::Clock().now();
  while(start_time_.seconds() > rclcpp::Clock().now().seconds() - 2){}

  // Pose 1
  within_bounds = move_group.setJointValueTarget(pose_1);
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }
  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Ejecutando la Pose 1");
  move_group.move();
  start_time_ = rclcpp::Clock().now();
  while(start_time_.seconds() > rclcpp::Clock().now().seconds() - 2){}

  // Pose 5
  within_bounds = move_group.setJointValueTarget(pose_5);
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }
  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Ejecutando la Pose 5");
  move_group.move();

  // Pose 6
  within_bounds = move_group.setJointValueTarget(pose_6);
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }
  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Ejecutando la Pose 6");
  move_group.move();

  // Pose 1
  within_bounds = move_group.setJointValueTarget(pose_1);
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }
  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Ejecutando la Pose 1");
  move_group.move();


  rclcpp::shutdown();
  return 0;
}