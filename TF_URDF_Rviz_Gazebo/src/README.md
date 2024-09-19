# TFs, URDF, Gazebo

Tips:

1. Set up environment for TFs, URDF, Gazebo:
  - Install Gazebo
      +  Run this command: "sudo apt install ros-humble-gazebo*" (don't forget the *). Maybe also run: "sudo apt install gazebo"
  - Visualize TFs in Rviz2
      +  Run this command: "sudo apt install ros-humble-urdf-tutortial"
      +  Launch an example: "ros2 launch urdf_tutortial display.launch.py model:=08-macroed.urdf.xacro"
  - Install tools for debugging TFs:
      + Run this command: "sudo apt install ros-humble-tf2-tools"
      + Run an example: "ros2 run tf2_tools view_frames". Then a pdf file will be generated.
        
2. Reference:
  - ROS Link: http://wiki.ros.org/urdf/XML/link
  - ROS Joint: http://wiki.ros.org/urdf/XML/joint
  - URDF-Inertia-Documentation: http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model
  - List-of-Inertia-Matrices-Wikipedia: https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors
  - Gazebo-plugins-doc: https://classic.gazebosim.org/tutorials?tut=ros_gzplugins
  - Gazebo-plugins-GitHub: https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins/include/gazebo_plugins

 
