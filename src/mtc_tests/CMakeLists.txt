cmake_minimum_required(VERSION 3.8)
project(mtc_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(generate_parameter_library REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)

set(DEPENDENCIES
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  rclcpp
  tf2_eigen
)

add_executable(mtc_pick_place src/mtc_pick_place.cpp)
ament_target_dependencies(
  mtc_pick_place
  ${DEPENDENCIES}
)

install(
  TARGETS mtc_pick_place
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
