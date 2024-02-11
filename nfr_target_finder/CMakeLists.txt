cmake_minimum_required(VERSION 3.8)
project(nfr_target_finder)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nfr_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(apriltag 3.2 REQUIRED)
find_package(OpenCV REQUIRED)
add_library(nfr_target_finder_node SHARED src/nfr_target_finder_node.cpp)
target_include_directories(nfr_target_finder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(nfr_target_finder_node PUBLIC c_std_99 cxx_std_23)  # Require C99 and C++20
ament_target_dependencies(nfr_target_finder_node tf2 tf2_ros geometry_msgs tf2_geometry_msgs ament_index_cpp rclcpp rclcpp_components apriltag_msgs image_transport cv_bridge
  nfr_msgs sensor_msgs)
target_link_libraries(nfr_target_finder_node apriltag::apriltag ${OpenCV_LIBS})
rclcpp_components_register_nodes(nfr_target_finder_node "nfr::NFRTargetFinderNode")
set(node_plugins "${node_plugins}nfr::NFRTargetFinderNode;$<TARGET_FILE:nfr_target_finder_node>\n")
install(TARGETS nfr_target_finder_node DESTINATION lib)
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