# TFs, URDF, Gazebo

Tips:

0. Set up environment for TFs, URDF, Gazebo:
  - Install Gazebo
      +  Run this command: "sudo apt install ros-humble-gazebo*" (don't forget the *). Maybe also run: "sudo apt install gazebo"
  - Visualize TFs in Rviz2
      +  Run this command: "sudo apt install ros-humble-urdf-tutortial"
      +  Launch an example: "ros2 launch urdf_tutortial display.launch.py model:=08-macroed.urdf.xacro"
  - Install tools for debugging TFs:
      + Run this command: "sudo apt install ros-humble-tf2-tools"
      + Run an example: "ros2 run tf2_tools view_frames". Then a pdf file will be generated.
      
1. Colcon:
  - Build a single package: colcon build --packages-select {packages name}
  - Don't need to rebuild colcon once the code is changed: colcon build --symlink-install (but if we create a new file, we have to build again).
    
2. Node:
  -  Run a Node normally: "ros2 run {execute_table} {node_file}"
  -  Rename a Node at Runtime: "ros2 run {execute_table} {node_file} --ros-args -r {node_name (when init)}:={rename_node}"
    
3. Interface:
  -  Get a list of interfaces in ROS2: ros2 interface list
  -  Get a list of interfaces in a package: ros2 interface package {package_name}. Ex: "ros2 interface package sensor_msgs"  
  -  Show interface types: ros2 interface show {interface_path}. Ex: "ros2 interface show example_interfaces/msg/String"
    
4. Topic
  - Get a list of topics: "ros2 topic list"
  - Listen to a topic: "ros2 topic echo /{topic_name}". Ex: "ros2 topic echo /robot_news"
  - Publish a topic: "ros topic pub {topic_name} {interface_path} "{variable: value}" ". Ex:  "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}" "
  - Get a topic info: "ros2 topic info /{topic_name}". Ex: "ros2 topic info /robot_news"
  - Get topic's rate: "ros2 topic hz /{topic_name}". Ex: "ros2 topic hz /robot_news"
  - Remap topic at Runtime: "ros2 run {execute table} {node_file} --ros-args -r {topic_name}:={new_name}". Ex: "ros2 run my_py_pkg smartphone --ros-args -r robot_news:=my_news"

5. Service
  - Get a list of services: "ros2 service list"
  - Get a topic type: "ros2 service type /{service_name}". Ex: "ros2 service type /add_two_ints"
  - Send request to a service: "ros2 service call /{service_name} {interface_path} "{param: value}" ". Ex: "ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 5}" "
  - Remap service at Runtime: "ros2 run {execute table} {node_file} --ros-args -r {service_name}:={new_name}". Ex: "ros2 run my_cpp_pkg add_two_ints_server --ros-args -r add_two_ints:=new_name"
    
6. Parameter
  - Get a list of parameters: "ros2 param list"
  - Get parameters value: "ros2 param get /{node_name} {parameter_name}". Ex: "ros2 param get /number_publisher use_sim_time"
  - Run a node with a parameter value: "ros2 run {execute_table} {node_file} --ros-args -p {parameter_name}:={value}". Ex: "ros2 run my_py_pkg number_publisher --ros-args -p test123:=3"
  - Run a node with multiple parameters values: "ros2 run {execute_table} {node_file} --ros-args -p {parameter_name}:={value} -p {parameter_name}:={value}". Ex: "ros2 run my_py_pkg number_publisher --ros-args -p test123:=3 -p another_param:=3.14"

- Reference:
  - ROS Link: http://wiki.ros.org/urdf/XML/link
  - ROS Joint: http://wiki.ros.org/urdf/XML/joint
  - URDF-Inertia-Documentation: http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model
  - List-of-Inertia-Matrices-Wikipedia: https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors
  - Gazebo-plugins-doc: https://classic.gazebosim.org/tutorials?tut=ros_gzplugins
  - Gazebo-plugins-GitHub: https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins/include/gazebo_plugins

 
