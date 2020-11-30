# ROS2 Actions

Workspace to give an overview how to use ROS2 actions.

## How to run

- Build the workspace  
`colcon build`

- Source the workspace  
`source install/setup.bash`

- Start the action server  
`rosrun actions_example action_server_node`

- Start a client  
`rosrun actions_example action_client_node`  
`rosrun actions_example action_client_feedback_node`  
`rosrun actions_example action_client_cancel_node`

## Useful documentation
https://index.ros.org/doc/ros2/Tutorials/Actions/Creating-an-Action/  
https://index.ros.org/doc/ros2/Tutorials/Actions/Writing-an-Action-Client-CPP/  
https://design.ros2.org/articles/actions.html

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

## 2.) ROS2 Action server

### `server.cpp`

To include your action:  
```
#include "your_pkg_name/action/your_action_name.hpp"
```  
Note: You name the `.action` file in CamelCase, the `.hpp` will be in snake_case.

To access your message definition:
```
your_package_name::action::YourActionName
```
Note: Here you use the case you used to name the `.action` file (CamelCase).

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

# --- to use interfaces in a node in the same package ---
rosidl_target_interfaces(action_server_node ${PROJECT_NAME}
    "rosidl_typesupport_cpp")
```

Note: It's good practice to put your interface files (messages, services, actions)
into a separate package. This enables other users to build only the interfaces without
having to build anything else. You see this reflected in the `package_name_msgs` packages
that you can find for well-known ROS packages (e.g. `turtlebot_msgs`, https://github.com/ROBOTIS-GIT/turtlebot3_msgs). That being said, sometimes it can be convenient to put the interfaces in the same package, e.g. when the package is not planned to be used in production code and only used for learning / testing purposes.

### Test that it worked
Start the action server: 
``` 
source install/setup.bash
rosrun actions_example action_server_node
```

Then publish a goal to it with (generic example, find a specific below. Don't forget the space after the `:` symbol):

```
ros2 action send_goal /published_action_name your_package_name/action/YourActionName "{your_parameter: your_int_value, your_other_parameter: 'your_string_value'}"
```

The `published_action_name` is set as a String name when you create the server.

To also be able to see the feedback, use the `-f` or `--feedback` option.
```
ros2 action send_goal -f /published_action_name your_package_name/action/YourActionName "{your_parameter: your_int_value, your_other_parameter: 'your_string_value'}"
```

Example:
```
ros2 action send_goal --feedback /fibonacci actions_example/action/Fibonacci order:\ 10
```

Note: Cancelling the goal with `Ctrl+C` will cause a RCLError exception on the server.
Using a client written in code and cancelling the goal does not result in the same error.

## 3.) ROS2 Action client

### `CMakeLists.txt`

```
add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client
    rclcpp
    rclcpp_action
)

# --- use interfaces from same package ---
rosidl_target_interfaces(action_client ${PROJECT_NAME}
    "rosidl_typesupport_cpp")
```

#### Problem 1:  
Compiling the `action_client.cpp` results in:
```
no type named ‘type’ in ‘struct std::enable_if<false, std::function<void(std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci> > >)>&>
```

#### Solution 1:  
Your include path for the action or the action definition are probably incorrect.
Did you use your package name and the correct case?
```
#include "your_pkg_name/action/your_action_name.hpp"
your_package_name::action::YourActionName
```

#### Problem 2:  
Compiling the `action_client.cpp` results in:
```
no match for ‘operator=’ (operand types are ‘rclcpp_action::Client<actions_example::action::Fibonacci>::GoalResponseCallback’ 

{aka ‘std::function<void(std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<actions_example::action::Fibonacci> > >)>’} 

and ‘std::_Bind_helper<false, void (MinimalActionClient::*)(std::shared_ptr<rclcpp_action::ClientGoalHandle<actions_example::action::Fibonacci> >), MinimalActionClient*, const std::_Placeholder<1>&>::type’ 

{aka ‘std::_Bind<void (MinimalActionClient::*(MinimalActionClient*, std::_Placeholder<1>))(std::shared_ptr<rclcpp_action::ClientGoalHandle<actions_example::action::Fibonacci> >)>’})
```

#### Solution 2:
You're probably on the wrong branch. Select the branch for your ROS distro.

Side note:
This was caused because it was attempted to bind a function with an `GoalHandleFibonacci::SharedPtr>` argument to a function with an `std::shared_future<GoalHandleFibonacci::SharedPtr>` argument. To get the `GoalHandleFibonacci::SharedPtr>` argument you can use the `get()` function on the future object.

#### Problem 3:
```
undefined reference to `rosidl_action_type_support_t const* rosidl_typesupport_cpp::get_action_type_support_handle<example_interfaces::action::Fibonacci>()'
```

#### Solution 3:
Your include path for the action or the action definition are probably incorrect.
Did you use your package name and the correct case?
```
#include "your_pkg_name/action/your_action_name.hpp"
your_package_name::action::YourActionName
```
