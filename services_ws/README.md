# ROS2 Services Example

Workspace to give an overview how to use ROS2 services.

### How to run

- Build the workspace  
`colcon build`

- Source the workspace  
`source install/setup.bash`

- Start the service   
`rosrun services_example service_node`

- Start a client  
`rosrun services_example service_client_node`  


## Steps necessary

## 1.) Create your service

It's a good practice to create your interfaces (topic messages, services, actions) in a separate package with the name `your_code_pkg_msg`. This is so that other users can build your messages isolated without having to compile anything else. You see this reflected in the `package_name_msgs` packages that you can find for well-known ROS packages (e.g. `turtlebot_msgs`, https://github.com/ROBOTIS-GIT/turtlebot3_msgs). That being said, sometimes it can be convenient to put the interfaces in the same package, e.g. when the package is not planned to be used in production code and only used for learning / testing purposes. If you want to use the interfaces from the same package, you have to adjust your `CMakeLists.txt` file (add this line `rosidl_target_interfaces(your_executable_name ${PROJECT_NAME} "rosidl_typesupport_cpp")`).

### `.srv` file

Create a folder `srv` and create a `.srv` file inside with the following format:

```
# request
---
# response
```

Example:
```
int64 a
int64 b
---
int64 sum
```

### `CMakeLists.txt`

Make ROS compile the headers for your service.

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/YourService.srv"
)
```

### `package.xml`

These dependencies are required for interface generation.

```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

### Test that it worked

```
source install/setup.bash
ros2 interface show your_package_msgs/srv/YourService
```

## 2.) Create the service

The code for the service and client do not go into the `_msgs` package but in the main package.
The `_msgs` package does not contain any code, only interface definitions.

### `.cpp` file

To include the service (notice that snake_case is used here)
```
#include "your_package_msgs/srv/your_service.hpp"
```
To get access to the definition (here you use CamelCase like you used for the `.service` file):
```
your_package_msgs::srv::YourService
```

### `CMakeLists.txt`

Be careful with your executable name. Calling it `service` or `client` might not be possible (interference with other definitions of CMake).

```
# to find the interfaces from the _msgs package
find_package(your_package_msgs REQUIRED)

add_executable(your_service_executable_name src/your_service.cpp)
ament_target_dependencies(your_service_executable_name
    rclcpp
    your_package_msgs # the executable depends on the _msgs package
)

install(TARGETS
  your_service_executable_name
  DESTINATION lib/${PROJECT_NAME})

# --- to use interfaces in a node in the same package ---
rosidl_target_interfaces(your_service_executable_name ${PROJECT_NAME}
    "rosidl_typesupport_cpp")
```

### Test that it worked
Start the service:
```
source install/setup.bash
ros2 run services_example minimal_service
```

Then make a call to it (generic example, find a specific below. Don't forget the space after the `:` symbol)
```
ros2 service call /service_name your_package_msgs/srv/YourService "{your_parameter: your_int_value, your_other_parameter: 'your string value'}"
```

Example:
```
ros2 service call /add_two_ints services_example_msgs/srv/AddTwoInts "{a: 1, b: 3}"
```

## 3.) Create the client

The adjustments for the `CMakeLists.txt` are basically the same as for the service.

### `CMakeLists.txt`

```
# to find the interfaces from the _msgs package
find_package(your_package_msgs REQUIRED)

add_executable(your_client_executable_name src/your_service.cpp)
ament_target_dependencies(your_client_executable_name
    rclcpp
    your_package_msgs # the executable depends on the _msgs package
)

install(TARGETS
  your_service_executable_name # note: you can just add both executables in a single call like this
  your_client_executable_name
  DESTINATION lib/${PROJECT_NAME})

# --- to use interfaces in a node in the same package ---
rosidl_target_interfaces(your_client_executable_name ${PROJECT_NAME}
    "rosidl_typesupport_cpp")
```

### Test that it worked

Start the service, then call it with the client
```
source install/setup.bash
ros2 run services_example minimal_service
ros2 run services_example minimal_client
```