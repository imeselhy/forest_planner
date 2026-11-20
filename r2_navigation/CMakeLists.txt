cmake_minimum_required(VERSION 3.8)
project(r2_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Custom messages ───────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/R2Step.msg"
  "msg/R2Path.msg"
  DEPENDENCIES std_msgs
)

# ── path_receiver node ────────────────────────────────────────────────────────
# third_party/ must contain the single-header libs httplib.h and json.hpp
add_executable(path_receiver_node src/path_receiver_node.cpp)
target_include_directories(path_receiver_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)
ament_target_dependencies(path_receiver_node rclcpp std_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(path_receiver_node "${cpp_typesupport_target}")

install(TARGETS path_receiver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
