cmake_minimum_required(VERSION 3.5)
project(go2_recovery_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(
  go2_recovery_manager_node
  src/main.cpp
)
ament_target_dependencies(
  go2_recovery_manager_node
  rclcpp
  unitree_go
)

target_include_directories(
  go2_recovery_manager_node
  PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(
  go2_recovery_manager_node
  ${OpenCV_LIBS}
)
install(
  TARGETS go2_recovery_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
