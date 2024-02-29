cmake_minimum_required(VERSION 3.16)
project(sbot_hi LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  sbot_hi
  SHARED
  hardware/rrbot.cpp
)
target_compile_features(sbot_hi PUBLIC cxx_std_17)
target_include_directories(sbot_hi PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/sbot_hi>
)
ament_target_dependencies(
  sbot_hi PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "sbot_hi_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface sbot_hi.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/sbot_hi
)

install(TARGETS sbot_hi
  EXPORT export_sbot_hi
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()


## EXPORTS
ament_export_targets(export_sbot_hi HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
