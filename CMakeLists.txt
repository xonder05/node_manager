cmake_minimum_required(VERSION 3.8)
project(node_manager)

# -------------------- Custom interfaces --------------------

find_package(rosidl_default_generators REQUIRED)
ament_export_dependencies(rosidl_default_runtime)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Entry.msg"
  "msg/DictionarySerialized.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# -------------------- Build --------------------

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread json)

# what to build
add_executable(node_manager_node src/node_manager.cpp)

# local linker
target_include_directories(node_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# library linker
target_link_libraries(node_manager_node 
    ${cpp_typesupport_target}
    ament_index_cpp::ament_index_cpp
    Boost::thread
    Boost::json
)

# ros linker
ament_target_dependencies(node_manager_node
    rclcpp
    std_msgs
)

# fix name collision error in cmake
set_target_properties(node_manager_node PROPERTIES
  OUTPUT_NAME node_manager
)

# -------------------- Install --------------------

# binary
install(TARGETS node_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# headers
install(DIRECTORY include
  DESTINATION include
)

# other files
install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

# -------------------- End --------------------

ament_package()
