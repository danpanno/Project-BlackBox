### Terminal 1  
#### $ cd ~/catkin_NETID  
#### $ catkin_make  
#### $ source devel/setup.bash  
#### $ roslaunch ur3_driver ur3_gazebo.launch  

### Terminal 2  
#### $ source devel/setup.bash  
#### $ rosrun lab5pkg_py lab5_coordinate_converter.py  

### Terminal 3  
#### $ source devel/setup.bash  
#### $ rosrun lab5pkg_py lab5_exec.py --missing False --image 0  
