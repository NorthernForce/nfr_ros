cmake_minimum_required(VERSION 3.8)
project(nfr_note_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(wpiutil REQUIRED)
find_package(wpinet REQUIRED)
find_package(cscore REQUIRED)
find_package(ntcore REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(nfr_note_detector_node src/nfr_note_detector_node.cpp)
target_include_directories(nfr_note_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "/usr/local/include/cscore"
  "/usr/local/include/wpiutil"
  "/usr/local/include/ntcore")
target_link_libraries(nfr_note_detector_node cscore wpinet wpiutil ntcore)
target_compile_features(nfr_note_detector_node PUBLIC c_std_99 cxx_std_23)  # Require C99 and C++17
ament_target_dependencies(nfr_note_detector_node rclcpp sensor_msgs cv_bridge)

install(TARGETS nfr_note_detector_node
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
