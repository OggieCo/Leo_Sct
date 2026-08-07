cmake_minimum_required(VERSION 3.8)
project(leo_image)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

# OpenCV and CV Bridge dependencies
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(image_processor src/image_processor.cpp)
target_include_directories(image_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(image_processor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(image_processor
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  OpenCV
  cv_bridge
  message_filters
)


install(TARGETS image_processor
  DESTINATION lib/${PROJECT_NAME})

# ---- Specialised human detector (depth-based person blob) ----
add_executable(image_human_processor src/image_human_processor.cpp)
target_include_directories(image_human_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(image_human_processor PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(image_human_processor
  rclcpp
  sensor_msgs
  std_msgs
  OpenCV
  cv_bridge
)

install(TARGETS image_human_processor
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
