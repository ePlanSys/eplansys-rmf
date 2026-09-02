cmake_minimum_required(VERSION 3.8)
project(eplansys_rmf_probe)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(eplansys_rmf_bridge REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(state_probe src/state_probe.cpp)
target_link_libraries(state_probe
  rclcpp::rclcpp
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(state_probe eplansys_rmf_bridge)

add_executable(submit_probe src/submit_probe.cpp)
target_link_libraries(submit_probe
  rclcpp::rclcpp
  nlohmann_json::nlohmann_json
  ${rmf_task_msgs_TARGETS}
)

install(TARGETS state_probe submit_probe
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
