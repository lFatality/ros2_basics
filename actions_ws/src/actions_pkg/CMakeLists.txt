cmake_minimum_required(VERSION 3.5)
project(actions_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
    "action/Fibonacci.action"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(action_server_node src/server.cpp)
ament_target_dependencies(action_server_node
    rclcpp
    rclcpp_action
)

install(TARGETS
    action_server_node
    DESTINATION lib/${PROJECT_NAME})

# --- use messages from same package ---
rosidl_target_interfaces(action_server_node ${PROJECT_NAME}
    "rosidl_typesupport_cpp")


if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
