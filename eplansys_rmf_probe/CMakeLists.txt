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
message(STATUS "schema validator: ${JSON_SCHEMA_VALIDATOR_LIB}")

add_executable(state_probe src/state_probe.cpp)
target_link_libraries(state_probe
  rclcpp::rclcpp
  rmf_websocket::rmf_websocket
  nlohmann_json::nlohmann_json
  ${JSON_SCHEMA_VALIDATOR_LIB}
)

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
