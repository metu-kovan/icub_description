# icub_description

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
3. `cd /src/icub_description/yarp_ros_bridge/build`
4. run `./icub_state_publisher`
5. `roslaunch icub_description icub.launch`
