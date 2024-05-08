// Nodo de planeamiento y ejecución de trayectorias predefinidas.
// Es llamado mediante el archivo "ejecutar_trayectorias.launch.py" de este mismo paquete

// Posee 2 modos de funcionamientos (Poses en el espacio de las juntas o en el del end_effector)
// los cuales pueden ser seleccionados mediante los booleanos "pose_juntas" o "pose_end_effector"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("Planeador");
bool pose_juntas = 0; // Indica si voy a planear con el estado de las juntas
bool pose_end_effector = 1; // Indica si voy a planear con poses del end_effector

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
  auto move_gripper_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  executor.add_node(move_gripper_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "arm";
  static const std::string PLANNING_GRIPPER = "gripper";

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(move_gripper_node, PLANNING_GRIPPER);

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  const moveit::core::JointModelGroup* gripper_model_group =
      move_group_gripper.getCurrentState()->getJointModelGroup(PLANNING_GRIPPER);

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  // RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  // std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
  //           std::ostream_iterator<std::string>(std::cout, ", "));

  // RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  // std::copy(move_group_gripper.getJointModelGroupNames().begin(), move_group_gripper.getJointModelGroupNames().end(),
  //           std::ostream_iterator<std::string>(std::cout, ", "));


  // // Now, we call the planner to compute the plan and visualize it.
  // // Note that we are just planning, not asking move_group
  // // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success;
  bool error=false;
  bool within_bounds;
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

  
  std::vector<double> home;
  current_state->copyJointGroupPositions(joint_model_group, home);
  // Home
  home[0] = 0;  // radians
  home[1] = 0;  // radians
  home[2] = 0;  // radians
  home[3] = 0;  // radians
  home[4] = 0;  // radians

  moveit::core::RobotStatePtr gripper_state = move_group_gripper.getCurrentState(10);
  std::vector<double> gripper_cerrado;
  gripper_state->copyJointGroupPositions(gripper_model_group, gripper_cerrado);
  // gripper_cerrado
  gripper_cerrado[0] = 0.024;  // radians
  gripper_cerrado[1] = 0.024;  // radians

  std::vector<double> gripper_abierto;
  gripper_state->copyJointGroupPositions(gripper_model_group, gripper_abierto);
  // gripper_abierto
  gripper_abierto[0] = 0;  // radians
  gripper_abierto[1] = 0;  // radians

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
    pose_1[1] = -1.57;  // radians
    pose_1[2] = 1.57;  // radians
    pose_1[3] = 0;  // radians
    pose_1[4] = 0;  // radians

    pose_2[0] = 1.57;  // radians
    pose_2[1] = -1.57;  // radians
    pose_2[2] = 1.57;  // radians
    pose_2[3] = 0;  // radians
    pose_2[4] = 0;  // radians

    pose_3[0] = -1.57;  // radians
    pose_3[1] = -1.57;  // radians
    pose_3[2] = 1.57;  // radians
    pose_3[3] = 0;  // radians
    pose_3[4] = 0;  // radians

    pose_4[0] = -1.57;  // radians
    pose_4[1] = 0;  // radians
    pose_4[2] = 0;  // radians
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
  // for (size_t i = 0; i < pose_x.size(); ++i) {
  //   RCLCPP_INFO(LOGGER, "Junta %ld: %f",i,pose_x[i]);
  // }
  //
  // // Imprimir por pantalla la pose del end effector
  // moveit::core::RobotModelConstPtr robot_model = move_group.getRobotModel();
  // moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  // robot_state->setToDefaultValues();
  //
  // robot_state->setJointGroupActivePositions(joint_model_group, pose_x);
  // robot_state->updateLinkTransforms();
  //
  // const Eigen::Isometry3d& end_effector_state = robot_state->getFrameTransform(move_group.getEndEffectorLink());
  // geometry_msgs::msg::PoseStamped planned_eef_pose;
  // // tf2::convert(end_effector_state, planned_eef_pose.pose);
  // planned_eef_pose.pose = tf2::toMsg(end_effector_state);
  // RCLCPP_INFO(LOGGER, "Posición - x: %f, y: %f, z: %f", planned_eef_pose.pose.position.x, planned_eef_pose.pose.position.y, planned_eef_pose.pose.position.z);
  // RCLCPP_INFO(LOGGER, "Orientación - x: %f, y: %f, z: %f, w: %f", planned_eef_pose.pose.orientation.x, planned_eef_pose.pose.orientation.y, planned_eef_pose.pose.orientation.z, planned_eef_pose.pose.orientation.w);

  // ################### Poses utilizando ubicación del end effector ####################
  if(pose_end_effector){
    geometry_msgs::msg::Pose target_pose1;
    // Alargado
    // target_pose1.position.x = 0.15656;
    // target_pose1.position.y = -0.31299;
    // target_pose1.position.z = 0.31405;
    // target_pose1.orientation.x = 0.38108;
    // target_pose1.orientation.y = 0.9245;
    // target_pose1.orientation.z = 0.0032479;
    // target_pose1.orientation.w = -0.0079372;
    // Agarrando 1
    // target_pose1.orientation.x = 0;
    // target_pose1.orientation.y = 1;
    // target_pose1.orientation.z = 0;
    // target_pose1.orientation.w = 0;
    // target_pose1.position.x = 0.3;
    // target_pose1.position.y = 0;
    // target_pose1.position.z = 0.2;
    // Pre Juntando horizontal
    target_pose1.orientation.x = 0.35773;
    target_pose1.orientation.y = 0.62747;
    target_pose1.orientation.z = -0.34101;
    target_pose1.orientation.w = 0.60168;
    target_pose1.position.x = 0.089135;
    target_pose1.position.y = -0.4032;
    target_pose1.position.z = 0.059295;


    geometry_msgs::msg::Pose target_pose3;
    //Pre Juntando Vertical
    target_pose3.orientation.x = -0.58892;
    target_pose3.orientation.y = 0.80782;
    target_pose3.orientation.z = -0.014447;
    target_pose3.orientation.w = -0.019772;
    target_pose3.position.x = 0.055426;
    target_pose3.position.y = 0.39006;
    target_pose3.position.z = 0.35525;


    geometry_msgs::msg::Pose target_pose2;
    // target_pose2.position.x = 0.209;
    // target_pose2.position.y = -0.348;
    // target_pose2.position.z = 0.2611;
    // target_pose2.orientation.x = 0.36728;
    // target_pose2.orientation.y = 0.92754;
    // target_pose2.orientation.z = -0.025406;
    // target_pose2.orientation.w = 0.064195;
    // Juntando horizontal
    target_pose2.orientation.x = 0.39371;
    target_pose2.orientation.y = 0.69051;
    target_pose2.orientation.z = -0.29914;
    target_pose2.orientation.w = 0.52792;
    target_pose2.position.x = 0.11216;
    target_pose2.position.y = -0.44183;
    target_pose2.position.z = 0.08458;

    geometry_msgs::msg::Pose target_pose4;
    //Juntando veritical
    target_pose4.orientation.x = -0.58899;
    target_pose4.orientation.y = 0.80797;
    target_pose4.orientation.z = 0.009739;
    target_pose4.orientation.w = 0.013318;
    target_pose4.position.x = 0.063818; //0.09 +-0.01
    target_pose4.position.y = 0.41624; //0.415 +-0.005
    target_pose4.position.z = 0.24826; //0.235 +-0.005


    move_group.setMaxVelocityScalingFactor(1);
    move_group.setMaxAccelerationScalingFactor(1);
    std::vector<double> gripper_estado;
    gripper_state->copyJointGroupPositions(gripper_model_group, gripper_estado);
    std::vector<double> arm_estado;
    gripper_state->copyJointGroupPositions(gripper_model_group, arm_estado);
    
    
    
    if((arm_estado[0]!=0) || (arm_estado[1]!=0) || (arm_estado[2]!=0) ||
       (arm_estado[3]!=0) || (arm_estado[4]!=0)){
      // Ejecución Home
      move_group.setJointValueTarget(home);
      success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(LOGGER, "Home (pose goal) %s", success ? "" : "FAILED");
      if(success){
        RCLCPP_INFO(LOGGER, "Ejecutando Home");
        move_group.move();
      }
    }

    // Sleep for 3 second
    rclcpp::sleep_for(std::chrono::seconds(10));

    if(gripper_estado[0] != 0){
      // Ejecución Gripper Abierto
      move_group_gripper.setJointValueTarget(gripper_abierto);
      success = (move_group_gripper.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(LOGGER, "Abrir Gripper (pose goal) %s", success ? "" : "FAILED");
      if(success){
        RCLCPP_INFO(LOGGER, "Ejecutando Abrir Gripper");
        move_group_gripper.move();
      }
    // Sleep for 3 second
    // rclcpp::sleep_for(std::chrono::seconds(3));
    }


    // Ejecución Pose 3
    move_group.setPoseTarget(target_pose3);
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan 3 (pose goal) %s", success ? "" : "FAILED");
    if(success){
      RCLCPP_INFO(LOGGER, "Ejecutando la Pose 3");
      move_group.move();
    }else{error = true;}
    // Sleep for 3 second
    // rclcpp::sleep_for(std::chrono::seconds(2));

    if(!error){
      // Ejecución Pose 4
      move_group.setMaxVelocityScalingFactor(0.25);
      move_group.setMaxAccelerationScalingFactor(0.25);
      move_group.setPoseTarget(target_pose4);
      success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(LOGGER, "Plan 4 (pose goal) %s", success ? "" : "FAILED");
      if(success){
        RCLCPP_INFO(LOGGER, "Ejecutando la Pose 4");
        move_group.move();
      }else{error = true;}
    }
    // Sleep for 3 second
    // rclcpp::sleep_for(std::chrono::seconds(2));

    if(!error){
      // Ejecución Gripper Cerrado
      move_group_gripper.setJointValueTarget(gripper_cerrado);
      success = (move_group_gripper.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(LOGGER, "Cerrar Gripper (pose goal) %s", success ? "" : "FAILED");
      if(success){
        RCLCPP_INFO(LOGGER, "Ejecutando Cerrar Gripper");
        move_group_gripper.move();
      }else{error = true;}
      // Sleep for 3 second
      // rclcpp::sleep_for(std::chrono::seconds(1));
    }

    if(!error){
      // Ejecución Pose 3
      move_group.setPoseTarget(target_pose3);
      success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(LOGGER, "Plan 3 (pose goal) %s", success ? "" : "FAILED");
      if(success){
        RCLCPP_INFO(LOGGER, "Ejecutando la Pose 3");
        move_group.move();
      }else{error = true;}
      // Sleep for 3 second
      // rclcpp::sleep_for(std::chrono::seconds(2));
    }

    move_group.setMaxVelocityScalingFactor(1);
    move_group.setMaxAccelerationScalingFactor(1);

    if(!error){
      // Ejecución Pose 1
      move_group.setPoseTarget(target_pose1);
      success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(LOGGER, "Plan 1 (pose goal) %s", success ? "" : "FAILED");
      if(success){
        RCLCPP_INFO(LOGGER, "Ejecutando la Pose 1");
        move_group.move();
      }else{error = true;}
      // Sleep for 3 second
      // rclcpp::sleep_for(std::chrono::seconds(5));
    }

    if(!error){
      // Ejecución Pose 2
      move_group.setMaxVelocityScalingFactor(0.25);
      move_group.setMaxAccelerationScalingFactor(0.25);
      move_group.setPoseTarget(target_pose2);
      success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(LOGGER, "Plan 2 (pose goal) %s", success ? "" : "FAILED");
      if(success){
        RCLCPP_INFO(LOGGER, "Ejecutando la Pose 2");
        move_group.move();
      }else{error = true;}
      // Sleep for 3 second
      // rclcpp::sleep_for(std::chrono::seconds(3));
    }

    if(!error){
      // Ejecución Gripper Abierto
      move_group_gripper.setJointValueTarget(gripper_abierto);
      success = (move_group_gripper.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(LOGGER, "Abrir Gripper (pose goal) %s", success ? "" : "FAILED");
      if(success){
        RCLCPP_INFO(LOGGER, "Ejecutando Abrir Gripper");
        move_group_gripper.move();
      }else{error = true;}
      // Sleep for 3 second
      rclcpp::sleep_for(std::chrono::seconds(3));
    }


    if(!error){
      // Ejecución Pose 1
      move_group.setPoseTarget(target_pose1);
      success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(LOGGER, "Plan 1 (pose goal) %s", success ? "" : "FAILED");
      if(success){
        RCLCPP_INFO(LOGGER, "Ejecutando la Pose 1");
        move_group.move();
      }else{error = true;}
      // Sleep for 3 second
      // rclcpp::sleep_for(std::chrono::seconds(3));
    }

    move_group.setMaxVelocityScalingFactor(1);
    move_group.setMaxAccelerationScalingFactor(1);

    if(!error){
      // Ejecución Home
      move_group.setJointValueTarget(home);
      success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(LOGGER, "Home (pose goal) %s", success ? "" : "FAILED");
      if(success){
        RCLCPP_INFO(LOGGER, "Ejecutando Home");
        move_group.move();
      }
      // Sleep for 3 second
      // rclcpp::sleep_for(std::chrono::seconds(3));
    }else{error = true;}
  }

  // ################### Fin poses utilizando ubicación del end effector ####################


  rclcpp::shutdown();
  return 0;
}