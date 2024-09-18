Tips:
1. Colcon:
  - Don't need to rebuild colcon once the code is changed: colcon build --symlink-install (but if we create a new file, we have to build again). 
3. Run a Node:
  -  Run a Node normally: "ros2 run {execute table} {node}"
  -  Rename a Node at Runtime: "ros2 run {execute table} {node} --ros-args {node_name (when init)}:={rename_node}"
  -  
