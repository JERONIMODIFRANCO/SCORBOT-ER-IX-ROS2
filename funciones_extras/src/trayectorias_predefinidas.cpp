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
// #include <geometry_msgs/msg/pose.hpp>
// #include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

// #include <moveit_visual_tools/moveit_visual_tools.h>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("Planeador");
bool pose_juntas = 1; // Indica si voy a planear con el estado de las juntas
bool pose_end_effector = 0; // Indica si voy a planear con poses del end_effector

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
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

  // ##############  Planning to a joint-space goal  ###################
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  if(pose_juntas){
    // Let's set a joint space goal and move towards it.  This will replace the
    // pose target we set above.
    //
    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    
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
    pose_2[1] = 0;  // radians
    pose_2[2] = 0;  // radians
    pose_2[3] = 0;  // radians
    pose_2[4] = 3.14;  // radians

    pose_3[0] = 0;  // radians
    pose_3[1] = 0;  // radians
    pose_3[2] = 0;  // radians
    pose_3[3] = 0;  // radians
    pose_3[4] = -3.14;  // radians

    pose_4[0] = -1.57;  // radians
    pose_4[1] = 0;  // radians
    pose_4[2] = -0;  // radians
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
    RCLCPP_INFO(LOGGER, "Ejecutando la Pose 1 %d", success);
    move_group.move();
    static auto start_time_ = rclcpp::Clock().now();
    while(start_time_.seconds() > rclcpp::Clock().now().seconds() - 2){}

    for(int i = 0; i<10; i++){
    // Pose 2
    within_bounds = move_group.setJointValueTarget(pose_2);
    if (!within_bounds)
    {
      RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    }
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Ejecutando la Pose 2");
    move_group.move();

    // Pose 3
    within_bounds = move_group.setJointValueTarget(pose_3);
    if (!within_bounds)
    {
      RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    }
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Ejecutando la Pose 3");
    move_group.move();
    }

    // // Pose 2
    // within_bounds = move_group.setJointValueTarget(pose_2);
    // if (!within_bounds)
    // {
    //   RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    // }
    // success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // RCLCPP_INFO(LOGGER, "Ejecutando la Pose 2");
    // move_group.move();
    // start_time_ = rclcpp::Clock().now();
    // while(start_time_.seconds() > rclcpp::Clock().now().seconds() - 2){}

    // // Pose 3
    // within_bounds = move_group.setJointValueTarget(pose_3);
    // if (!within_bounds)
    // {
    //   RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    // }
    // success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // RCLCPP_INFO(LOGGER, "Ejecutando la Pose 3");
    // move_group.move();
    // start_time_ = rclcpp::Clock().now();
    // while(start_time_.seconds() > rclcpp::Clock().now().seconds() - 2){}

    // // Pose 2
    // within_bounds = move_group.setJointValueTarget(pose_2);
    // if (!within_bounds)
    // {
    //   RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    // }
    // success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // RCLCPP_INFO(LOGGER, "Ejecutando la Pose 2");
    // move_group.move();
    // start_time_ = rclcpp::Clock().now();
    // while(start_time_.seconds() > rclcpp::Clock().now().seconds() - 2){}

    // // Pose 4
    // within_bounds = move_group.setJointValueTarget(pose_3);
    // if (!within_bounds)
    // {
    //   RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    // }
    // success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // RCLCPP_INFO(LOGGER, "Ejecutando la Pose 4");
    // move_group.move();
    // start_time_ = rclcpp::Clock().now();
    // while(start_time_.seconds() > rclcpp::Clock().now().seconds() - 2){}

    // // Pose 1
    // within_bounds = move_group.setJointValueTarget(pose_2);
    // if (!within_bounds)
    // {
    //   RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    // }
    // success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // RCLCPP_INFO(LOGGER, "Ejecutando la Pose 1");
    // move_group.move();
    // start_time_ = rclcpp::Clock().now();
    // while(start_time_.seconds() > rclcpp::Clock().now().seconds() - 2){}

    // // Pose 5
    // within_bounds = move_group.setJointValueTarget(pose_3);
    // if (!within_bounds)
    // {
    //   RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    // }
    // success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // RCLCPP_INFO(LOGGER, "Ejecutando la Pose 5");
    // move_group.move();

    // // Pose 6
    // within_bounds = move_group.setJointValueTarget(pose_2);
    // if (!within_bounds)
    // {
    //   RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    // }
    // success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // RCLCPP_INFO(LOGGER, "Ejecutando la Pose 6");
    // move_group.move();

    // Pose 1
    within_bounds = move_group.setJointValueTarget(pose_1);
    if (!within_bounds)
    {
      RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    }
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Ejecutando la Pose 1");
    move_group.move();
  }

  // ##############  Imprimir por pantalla el estado de las juntas actual  ###################
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // current_state = move_group.getCurrentState(10);
  // std::vector<double> pose_x;
  // current_state->copyJointGroupPositions(joint_model_group, pose_x);
  // for (size_t i = 0; i < pose_1.size(); ++i) {
  //   RCLCPP_INFO(LOGGER, "Junta %ld: %f",i,pose_x[i]);
  // }
 
  // // Imprimir por pantalla la pose del end effector
  // moveit::core::RobotModelConstPtr robot_model = move_group.getRobotModel();
  // moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  // robot_state->setToDefaultValues();

  // robot_state->setJointGroupActivePositions(joint_model_group, pose_x);
  // robot_state->updateLinkTransforms();

  // const Eigen::Isometry3d& end_effector_state = robot_state->getFrameTransform(move_group.getEndEffectorLink());
  // geometry_msgs::msg::PoseStamped planned_eef_pose;
  // // tf2::convert(end_effector_state, planned_eef_pose.pose);
  // planned_eef_pose.pose = tf2::toMsg(end_effector_state);
  // RCLCPP_INFO(LOGGER, "Posición - x: %f, y: %f, z: %f", planned_eef_pose.pose.position.x, planned_eef_pose.pose.position.y, planned_eef_pose.pose.position.z);
  // RCLCPP_INFO(LOGGER, "Orientación - x: %f, y: %f, z: %f, w: %f", planned_eef_pose.pose.orientation.x, planned_eef_pose.pose.orientation.y, planned_eef_pose.pose.orientation.z, planned_eef_pose.pose.orientation.w);

  // ################### Poses utilizando ubicación del end effector ####################
  if(pose_end_effector){
    move_group.setMaxVelocityScalingFactor(0.75);
    move_group.setMaxAccelerationScalingFactor(0.75);
    geometry_msgs::msg::Pose target_pose1;
    // Alargado
    // target_pose1.orientation.x = 0.12;
    // target_pose1.orientation.y = 0.73;
    // target_pose1.orientation.z = -0.1;
    // target_pose1.orientation.w = 0.66;
    // target_pose1.position.x = 0.477;
    // target_pose1.position.y = 0;
    // target_pose1.position.z = 0.37;
    // Agarrando 1
    // target_pose1.orientation.x = 0;
    // target_pose1.orientation.y = 1;
    // target_pose1.orientation.z = 0;
    // target_pose1.orientation.w = 0;
    // target_pose1.position.x = 0.3;
    // target_pose1.position.y = 0;
    // target_pose1.position.z = 0.2;
    // Pre Juntando horizontal
    target_pose1.orientation.x = 0.374513;
    target_pose1.orientation.y = 0.659620;
    target_pose1.orientation.z = -0.320785;
    target_pose1.orientation.w = 0.567220;
    target_pose1.position.x = 0.136978;
    target_pose1.position.y = -0.480721;
    target_pose1.position.z = 0.188224;
    //Pre Juntando Vertical
    // target_pose1.orientation.x = -0.592450;
    // target_pose1.orientation.y = 0.805556;
    // target_pose1.orientation.z = -0.005388;
    // target_pose1.orientation.w = -0.00730;
    // target_pose1.position.x = 0.048063;
    // target_pose1.position.y = 0.378895;
    // target_pose1.position.z = 0.313686;


    geometry_msgs::msg::Pose target_pose2;
    // target_pose2.orientation.x = 0;
    // target_pose2.orientation.y = 1;
    // target_pose2.orientation.z = 0;
    // target_pose2.orientation.w = 0;
    // target_pose2.position.x = 0.45;
    // target_pose2.position.y = 0;
    // target_pose2.position.z = 0.25;
    // Juntando horizontal
    target_pose2.orientation.x = 0.395373;
    target_pose2.orientation.y = 0.698154;
    target_pose2.orientation.z = -0.293266;
    target_pose2.orientation.w = 0.519861;
    target_pose2.position.x = 0.122871; // 0.11 +-0.01
    target_pose2.position.y = -0.455016; //-0.46 +-0.01
    target_pose2.position.z = 0.123510; // 0.15 +-0.01
    //Juntando veritical
    // target_pose2.orientation.x = -0.592459;
    // target_pose2.orientation.y = 0.805569;
    // target_pose2.orientation.z = 0.004237;
    // target_pose2.orientation.w = 0.005734;
    // target_pose2.position.x = 0.060707; //0.09 +-0.01
    // target_pose2.position.y = 0.419651; //0.415 +-0.005
    // target_pose2.position.z = 0.228712; //0.235 +-0.005


    // Ejecución Pose 1
    move_group.setPoseTarget(target_pose1);
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan 1 (pose goal) %s", success ? "" : "FAILED");
    if(success){
      RCLCPP_INFO(LOGGER, "Ejecutando la Pose 1");
      move_group.move();
    }

    // Ejecución Pose 2
    move_group.setPoseTarget(target_pose2);
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan 2 (pose goal) %s", success ? "" : "FAILED");
    if(success){
      RCLCPP_INFO(LOGGER, "Ejecutando la Pose 2");
      move_group.move();
    }
  }

  // ################### Fin poses utilizando ubicación del end effector ####################


  rclcpp::shutdown();
  return 0;
}