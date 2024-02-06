cmake_minimum_required(VERSION 3.8)
project(nfr_apriltag_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nfr_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(apriltag_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(nfr_apriltag_localization_node src/nfr_apriltag_localization_node.cpp)
add_library(nfr_apriltag_localization::nfr_apriltag_localization_node ALIAS nfr_apriltag_localization_node)
ament_target_dependencies(nfr_apriltag_localization_node rclcpp rclcpp_components ament_index_cpp tf2 tf2_ros tf2_geometry_msgs nfr_msgs geometry_msgs
  sensor_msgs cv_bridge apriltag_msgs)
target_compile_features(nfr_apriltag_localization_node PUBLIC c_std_99 cxx_std_23)  # Require C99 and C++17
target_include_directories(nfr_apriltag_localization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_nodes(nfr_apriltag_localization_node "nfr::NFRAprilTagLocalizatioNode")
set(node_plugins "${node_plugins}nfr::NFRAprilTagLocalizationNode;$<TARGET_FILE:nfr_apriltag_localization_node>\n")

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(nfr_apriltag_localization_node PRIVATE "NFR_APRILTAG_LOCALIZATION_BUILDING_LIBRARY")

install(
  TARGETS nfr_apriltag_localization_node
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
  nfr_apriltag_localization_node
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
