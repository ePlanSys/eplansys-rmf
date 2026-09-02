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
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(rmf_websocket REQUIRED)
find_package(nlohmann_json REQUIRED)

# rmf_websocket's exported interface names the schema validator as a bare
# -lnlohmann_json_schema_validator, and the vendor package ships no imported
# target for it, so the library has to be located by hand.
find_package(nlohmann_json_schema_validator_vendor REQUIRED)
find_library(JSON_SCHEMA_VALIDATOR_LIB
  NAMES nlohmann_json_schema_validator
  REQUIRED
)
get_filename_component(JSON_SCHEMA_VALIDATOR_DIR
  "${JSON_SCHEMA_VALIDATOR_LIB}" DIRECTORY)
link_directories("${JSON_SCHEMA_VALIDATOR_DIR}")

add_library(${PROJECT_NAME} SHARED
  src/TaskMapping.cpp
  src/RmfTaskClient.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  rmf_websocket::rmf_websocket
  nlohmann_json::nlohmann_json
  ${JSON_SCHEMA_VALIDATOR_LIB}
  ${rmf_task_msgs_TARGETS}
)

add_executable(rmf_action_node src/rmf_action_node.cpp)
target_link_libraries(rmf_action_node ${PROJECT_NAME})
ament_target_dependencies(rmf_action_node plansys2_executor plansys2_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS rmf_action_node DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
