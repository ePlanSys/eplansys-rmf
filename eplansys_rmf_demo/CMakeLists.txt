cmake_minimum_required(VERSION 3.8)
project(eplansys_rmf_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# mission_check asks the epistemic state what the run came out to, so it needs
# eplansys. The launch files and the task map do not, and a machine with
# Open-RMF alone should still be able to install them, so this is QUIET for the
# same reason the bridge's plansys2 lookup is.
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(nlohmann_json QUIET)
find_package(plansys2_epistemic_executor QUIET)

# radio needs no planner: it is one agent's subscriptions and a transcript, and
# it is what makes the private channel checkable. mission_check needs both, so
# they are guarded separately.
if(rclcpp_FOUND AND std_msgs_FOUND AND nlohmann_json_FOUND)
  add_executable(radio src/radio.cpp)
  ament_target_dependencies(radio rclcpp std_msgs)
  target_link_libraries(radio nlohmann_json::nlohmann_json)
  install(TARGETS radio DESTINATION lib/${PROJECT_NAME})
endif()

if(rclcpp_FOUND AND std_msgs_FOUND AND nlohmann_json_FOUND AND
   plansys2_epistemic_executor_FOUND)
  add_executable(mission_check src/mission_check.cpp)
  ament_target_dependencies(mission_check rclcpp std_msgs plansys2_epistemic_executor)
  target_link_libraries(mission_check nlohmann_json::nlohmann_json)
  install(TARGETS mission_check DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS
    "eplansys not found: installing the demo without mission_check")
endif()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
