cmake_minimum_required(VERSION 3.20)
project(robotic_4dof_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(DIRECTORY urdf launch worlds config
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS 
  scripts/pick_and_place.py
  scripts/world_publisher.py
  scripts/planning_scene_manager.py
  DESTINATION lib/${PROJECT_NAME}
)
# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_params test/test_params.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(test_scene_manager test/test_scene_manager.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

ament_package()
