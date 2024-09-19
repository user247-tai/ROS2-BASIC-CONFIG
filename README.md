# ROS2
Set up ROS2 Humble Environment:
  1. Set up development tools:
     
  - Install Terminator:  "sudo apt install terminator"
  - Install python3:  "sudo apt install python3-pip"
  - Install VSCode:  "sudo snap install code --classic"
  - Install Extensions:  C/C++, Python, CMake, ROS, URDF, XML
  2. Install ROS2 on Unbuntu 22.04:
    
  - Follow this:  https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
  3. Set up environment for ROS2:
    
    + Step 1:  Open .bashrc file in home (~) directory using gedit: "gedit ~/.bashrc"
    + Step 2:  Add "source /opt/ros/{ros_distribution}/setup.bash" line at the bottom of .bashrc file
  4. Install & Set up Colcon:

    + Step 1:  Install colcon: "sudo apt install python3-colcon-common-extensions"
      
    + Step 2:  Set up colcon: Open .bashrc file in home (~) directory using gedit: "gedit ~/.bashrc" and Add "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" line at the bottom of .bashrc file
  5. Create a ROS2 Workspace:

    + Step 1: Make a ROS2 Workspace directory in home (~) directory: "mkdir ros2_ws"
      
    + Step 2: Go to ros2_ws directory and then build it with colcon: "cd ros2_ws" & "colcon build". Now we will have 4 directory inside ros2_ws dir: build, install, log and src
      
    + Step 3: Open .bashrc file in home (~) directory using gedit: "gedit ~/.bashrc" and Add "source ~/ros2_ws/install/setup.bash" line at the bottom of .bashrc file
  6. Create a package in 'ros2_ws/src' path(option):

    + Use this command to create a package: "ros2 pkg create {pkg_name}". Ex: ros2 pkg create my_robot_interfaces
      
    + And then remove "include" and "src" dir inside the package created, and then we can use it. 
  7. Create a python package in 'ros2_ws/src' path:

    + Use this command to create a python package:  "ros2 pkg create {py_pkg_name} --build-type ament_python --dependencies rclpy"
      
    + Then try to build with colcon in ros2_ws dir: "colcon build" or "colcon build --packages-select {pkg_name}"
  8. Create a C++ package in 'ros2_ws/src' path:

    + Use this command to create a C++ package: "ros2 pkg create {cpp_pkg_name} --build-type ament_cmake --dependencies rclcpp"
      
    + Then try to build with colcon in ros2_ws dir: "colcon build" or "colcon build --packages-select {cpp_pkg_name}"
 
-  Some tools that can be used frequenly: rqt, rqt_graph, turtlesim, rviz, gazebo.
