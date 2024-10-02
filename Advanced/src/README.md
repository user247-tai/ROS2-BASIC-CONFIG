# ROS2 Actions, Lifecycle Nodes, Executors, Components, and More.

Tips:
  1. Action:
  - Get a list of actions: "ros2 action list"
  - Get a action info: "ros2 action info /{action_name}". Ex: "ros2 action info /count_until"
  - Get a list of topics and services (hidden): "ros2 topic list --include-hidden-topics", "ros2 service list --include-hidden-services"
  - Send a goal: "ros2 action send_goal /{action_name} {interface_path} {param: value}". Ex: 'ros2 action send_goal /count_until my_robot_interfaces/action/CountUntil "{target_number: 4, period: 1.3}" '
  2. Lifecycle Node:
  - Get a list of lifecycle nodes: "ros2 lifecycle nodes"
  - Get current state of a lifecycle node: "ros2 lifecycle get /{lifecycle node name}". Ex: ros2 lifecycle get /robotA
  - Get list of all states that a lifecycle node (in current state) can reach to: "ros2 lifecycle list /{lifecycle node name}". Ex: ros2 lifecycle list /robotA
  - Change a lifecycle node current state to a future state: "ros2 lifecycle set /{lifecycle node name} {state}". Ex: ros2 lifecycle set /robotA activate  
