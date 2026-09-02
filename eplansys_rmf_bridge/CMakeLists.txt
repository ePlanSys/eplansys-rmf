cmake_minimum_required(VERSION 3.8)
project(eplansys_rmf_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# The library below is the RMF half of the bridge and needs no planner. Only
# the performer executable does, so plansys2 is looked for and not required:
# that lets the library, its tests and the probes build on a machine with
# Open-RMF and no eplansys, which is what CI has.
find_package(plansys2_executor QUIET)
find_package(plansys2_msgs QUIET)
find_package(rmf_task_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/TaskMapping.cpp
  src/RmfTaskClient.cpp
  src/WebsocketFeed.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  nlohmann_json::nlohmann_json
  Boost::system
  pthread
  ${rmf_task_msgs_TARGETS}
  ${rmf_fleet_msgs_TARGETS}
)

if(plansys2_executor_FOUND AND plansys2_msgs_FOUND)
  add_executable(rmf_action_node src/rmf_action_node.cpp)
  target_link_libraries(rmf_action_node ${PROJECT_NAME})
  ament_target_dependencies(rmf_action_node plansys2_executor plansys2_msgs)
  install(TARGETS rmf_action_node DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS
    "plansys2 not found: building the RMF library without the performers")
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_task_mapping test/test_task_mapping.cpp)
  target_link_libraries(test_task_mapping ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
