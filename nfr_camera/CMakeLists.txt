cmake_minimum_required(VERSION 3.8)
project(nfr_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(wpiutil REQUIRED)
find_package(wpinet REQUIRED)
find_package(cscore REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(nfr_camera_node src/nfr_camera_node.cpp)
target_include_directories(nfr_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "/usr/local/include/cscore"
  "/usr/local/include/wpiutil")
target_link_libraries(nfr_camera_node cscore wpinet wpiutil)
target_compile_features(nfr_camera_node PUBLIC c_std_99 cxx_std_23)  # Require C99 and C++17
ament_target_dependencies(nfr_camera_node rclcpp sensor_msgs cv_bridge)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(TARGETS nfr_camera_node
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
