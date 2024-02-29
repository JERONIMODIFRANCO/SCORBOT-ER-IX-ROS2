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
# uncomment the following section in order to fill in
# further dependencies manually.

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
  # moveit_servo
  # moveit_task_constructor_core
  # moveit_visual_tools
  pluginlib
  rclcpp
  rclcpp_action
  # rviz_visual_tools
  tf2_geometry_msgs
  tf2_geometry_msgs
  tf2_ros
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
add_executable(logger src/guarda_comandos_estados.cpp)
ament_target_dependencies(logger rclcpp std_msgs moveit_core moveit_msgs)
add_executable(planeador src/planeamiento_moveit.cpp)
ament_target_dependencies(planeador ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS
  talker
  listener
  logger
  planeador
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

  
# find_path(SBOT_INCLUDE_DIR sbot_hi/usb_functions.h
#   HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../.. # Del directorio del proyecto actual, salgo 2 para atrás
#   sbot_hi/hardware/include/ # Busco acá adentro
# )
# if(NOT SBOT_INCLUDE_DIR)
# message(FATAL_ERROR "No se encontró usb_functions.h" CMAKE_CURRENT_SOURCE_DIR) 
# endif()
# set(SBOT_INCLUDE_DIRS "${SBOT_INCLUDE_DIR}" CACHE INTERNAL "")
# include_directories(${SBOT_INCLUDE_DIR})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
