cmake_minimum_required(VERSION 3.8)
project(nfr_target_finder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(nfr_msgs REQUIRED)
find_package(apriltag_msgs)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(nfr_target_finder_node src/nfr_target_finder_node.cpp)
add_library(nfr_target_finder::nfr_target_finder_node ALIAS nfr_target_finder_node)
ament_target_dependencies(nfr_target_finder_node rclcpp rclcpp_components nfr_msgs apriltag_msgs sensor_msgs cv_bridge message_filters)
target_compile_features(nfr_target_finder_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(nfr_target_finder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_nodes(nfr_target_finder_node "nfr::NFRTargetFinderNode")
set(node_plugins "${node_plugins}nfr::NFRTargetFinderNode;$<TARGET_FILE:nfr_target_finder_node>\n")

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(nfr_target_finder_node PRIVATE "NFR_TARGET_FINDER_BUILDING_LIBRARY")

install(
  TARGETS nfr_target_finder_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_export_include_directories(
  include
)
ament_export_libraries(
  nfr_target_finder_node
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
