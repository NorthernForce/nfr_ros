cmake_minimum_required(VERSION 3.8)
project(nfr_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(wpiutil REQUIRED)
find_package(wpinet REQUIRED)
find_package(ntcore REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nfr_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(nfr_bridge_node src/nfr_bridge_node.cpp)
target_include_directories(nfr_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "/usr/local/include/ntcore"
  "/usr/local/include/wpiutil"
)
target_compile_features(nfr_bridge_node PUBLIC c_std_99 cxx_std_23)  # Require C99 and C++17
ament_target_dependencies(nfr_bridge_node rclcpp rclcpp_action geometry_msgs tf2_geometry_msgs nav_msgs nav2_msgs sensor_msgs nfr_msgs)
target_link_libraries(nfr_bridge_node wpinet ntcore wpiutil)
install(TARGETS nfr_bridge_node
  DESTINATION lib/${PROJECT_NAME})

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
