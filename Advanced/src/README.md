# ROS2 Actions, Lifecycle Nodes, Executors, Components, and More.

Tips:
  1. Action:
  - Get a list of actions: "ros2 action list"
  - Get a action info: "ros2 action info /{action_name}". Ex: "ros2 action info /count_until"
  - We can also check topics & services (Run in actions): "ros2 topic list --include-hidden-topics", "ros2 service list --include-hidden-services"
  - Send a goal: "ros2 action send_goal /{action_name} {interface_path} {param: value}". Ex: 'ros2 action send_goal /count_until my_robot_interfaces/action/CountUntil "{target_number: 4, period: 1.3}" '
