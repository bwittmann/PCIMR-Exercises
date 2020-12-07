## Tutorial 04: Mapping and SLAM

The creation of this package was part of an optional task in the practical course 'Intelligent Mobile Robots with ROS' and is based on the topics of the fourth lecture.


In order to launch the necessary nodes (simple_sim_node, occu_grid_map_node) the following line should be entered after a ROS master has been launched:

        roslaunch pcimr_mapping mapping.launch 

Furthermore, run the key_robot_mover to be able to change the robots position with your keyboard:

        rosrun pcimr_navigation key_robot_mover

The current map does not only get published on the topic /map, but does also get displayed in the terminal. This is for demonstration reasons.
Keep in mind that the map the robot moves in (origin: bottom left) is actually flipped to the numpy array representation (origin: top left). Therefore, north and south are switched.
The robot is visualized in the terminal output as a '5'.


