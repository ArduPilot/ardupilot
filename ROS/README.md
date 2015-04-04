# ROS port

This prototype aims to port the APM autopilot code to the Robot Operating System (ROS) as a set of nodes that publish/subscribe to the relevant topics enhancing robot application development.

------

This port is being done using [Erle-Brain](https://erlerobotics.com/blog/product/erle-brain/) Linux autopilot.

-------

### Milestones

 - [x] Prototype with AP_InertialSensor
 - [ ] Prototype AP_AHRS + AP_InertialSensor
 - [ ] Rover prototype
 - [ ] Convert msgs to ROS standarts

### Try it yourself

Create a ROS catkin workspace:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

Get the sources
```bash
git clone -b ros http://github.com/erlerobot/ardupilot
cd ..
```

Source environment variables and compile the code
```bash
# Now you should be in ~/catkin_ws
source devel/setup.bash
catkin_make
```
