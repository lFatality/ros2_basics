# ROS2 Actions

## How to run


## Useful documentation
https://index.ros.org/doc/ros2/Tutorials/Actions/Creating-an-Action/  
https://index.ros.org/doc/ros2/Tutorials/Actions/Writing-an-Action-Client-CPP/  
https://design.ros2.org/articles/actions.html

___

## Steps necessary

## 1.) Create your own action

### `.action` file

Create a folder called `action` in your package and add a `YourActionName.action` file with the following format:

```
# Request
---
# Result
---
# Feedback
```

Example:
```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

### `CMakeLists.txt`

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/YourActionName.action"
)

ament_export_dependencies(rosidl_default_runtime) (?)
```

### `package.xml`
```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

### Test that it worked

```
source install/setup.bash
ros2 interface show action_tutorials/action/Fibonacci
```

## ROS2 Action server

### `server.cpp`

To include your action:  
`#include "your_pkg_name/action/your_action_name.hpp"`  
Note: You name the `.action` file in CamelCase, the `.hpp` will be in snake_case.

### `CMakeLists.txt`

```
find_package(rclcpp_action REQUIRED)

add_executable(action_server_node src/server.cpp)
ament_target_dependencies(action_server_node 
    rclcpp
    rclcpp_action
)

install(TARGETS
    action_server_node
    DESTINATION lib/${PROJECT_NAME})

# --- to use messages in a node in the same package ---
rosidl_target_interfaces(action_server_node ${PROJECT_NAME}
    "rosidl_typesupport_cpp")
```

## ROS2 Action client