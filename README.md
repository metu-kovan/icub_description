# icub_description

This package consists of a urdf model of iCub humanoid robot and a yarp-ros bridge to publish joint states of the robot to ros.

### Requirements

1. ROS : http://wiki.ros.org/
2. Yarp : http://www.yarp.it/
3. iCub : http://wiki.icub.org/

### Install

1. `cd ~/catkin_ws/src`  
2. `git clone https://github.com/tkelestemur/icub_description.git`  
3. `cd ..`  
4. `catkin_make`  
5. `cd /src/icub_description/yarp_ros_bridge`  
6. `mkdir build && cd build`  
7. `cmake ../`  
8. `make`

### Run
1. `roscore`
2. `yarp server --ros`
3. `iCub_SIM`
4. `cd /src/icub_description/yarp_ros_bridge/build`
5. run `./icub_state_publisher`
6. `roslaunch icub_description icub.launch`
