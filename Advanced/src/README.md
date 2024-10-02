# ROS2 Actions, Lifecycle Nodes, Executors, Components

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
  3. Component:
  - Start the ComponentManager node (component_container) (or component_container_isolated, component_container_mt) node: "ros2 run rclcpp_components component_container"
  - Get a list of components: "ros2 component list"
  - Load a node to the "component_container": "ros2 component load /ComponentManager {components workspace} {namespace::class name}". Ex: "ros2 component load /ComponentManager components_cpp my_namespace::NumberPublisher"
  - Unload a node from the "component_container": "ros2 component unload /ComponentManager {number of node in ComponentManager}". Ex: "ros2 component unload /ComponentManager 1"
  - For component debugging: "ros2 component standalone {components workspace} {namespace::class name}". Ex: "ros2 component standalone components_cpp my_namespace::NumberPublisher"
