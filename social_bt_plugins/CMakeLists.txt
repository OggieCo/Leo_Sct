cmake_minimum_required(VERSION 3.8)
project(social_bt_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# ---- custom BT plugins (loaded by nav2 bt_navigator via plugin_lib_names) ----
add_library(social_bt_plugins SHARED
  src/is_human_close.cpp
)
target_compile_features(social_bt_plugins PUBLIC c_std_99 cxx_std_17)

# needed so BT_REGISTER_NODES exports the plugin symbol
target_compile_definitions(social_bt_plugins PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(social_bt_plugins
  rclcpp
  std_msgs
  behaviortree_cpp_v3
)

install(TARGETS social_bt_plugins
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
