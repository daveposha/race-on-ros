# race-on-ros
Race On code ported to ROS. This is the main respository for race-on. 

# Setting Up ROS for Your Car

To run the ros nodes, run `roslaunch raceon raceon.launch speed:=140`
To add argument to change the ros node parameters, you can add `<arg name="xxx" default="xx"/>` to `raceon.launch`

# Branch you need to know
- ```Master```: The release code for race day. Everything is *tested* and will released from staging branch
- ```Staging```: All *tested* codes on race-on-ros
- Other branches: development branches and merge request needs to be submitted to *staging* for merge.

# Things to do
- [x] Run the basic codes
- [ ] Finish a lap with the basic codes
- [ ] Add PID to ros code (see [ros tutorial](https://raceon.io/workshops/1_ros/))
- [ ] Add gain scheduling (see [gain scheduling tutorial](https://raceon.io/workshops/2_gain_scheduling/))

# Useful link
[race on website](https://raceon.io/)
