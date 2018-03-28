# Integrated-Group-Robotics-Project
Repository includes the python file for the potential fields navigation, the Rviz file used whilst testing, the Ros launch files and a sample of the map files generated.

Mapping proceedure for gazebo environment (omit step 1 if using Turtlebot)

1)	Launch gazebo - roslaunch turtlebot_gazebo turtlebot_mw_office.launch  (or other gazebo environment)
2)	Launch gmapping – rosmake gmapping
3)	Launch slam - rosrun gmapping slam_gmapping scan:=scan
4)	Launch rviz with appropriate settings - rviz
5)  Launch navigation code - Potential.py
6)  Assign goal co-ordinates in rviz until environment is suitably mapped
7)	Save map - rosrun map_server map_saver -f name of map

Navigating a map for gazebo environment

1)	Launch gazebo - roslaunch turtlebot_gazebo turtlebot_mw_office.launch  (or other gazebo environment)
2)  Launch rviz with appropriate settings - rviz
3)  launch acml - roslaunch turtlebot_navigation acml_demo.launch FILE:= map address 
4)  Give the robot its pose estimate
5)  Select a co-ordinate to navigate to
