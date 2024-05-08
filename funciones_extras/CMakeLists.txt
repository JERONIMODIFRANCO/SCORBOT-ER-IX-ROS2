cmake_minimum_required(VERSION 3.8)
project(funciones_extras)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ros_ign REQUIRED)

# Dependencias necesarias para la función de movimiento
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  Boost
  control_msgs
  Eigen3
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  tf2_ros
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(listener src/nodo_de_comunicacion.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
add_executable(logger src/loggeo_de_datos.cpp)
ament_target_dependencies(logger rclcpp std_msgs moveit_core moveit_msgs)
add_executable(planeador src/trayectorias_predefinidas.cpp)
ament_target_dependencies(planeador ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
add_executable(logger_gz src/loggeo_de_gazebo.cpp)
ament_target_dependencies(logger_gz rclcpp std_msgs moveit_core moveit_msgs)

install(TARGETS
  listener
  logger
  planeador
  logger_gz
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
