# SAPHARI-FINAL-REVIEW
Configurations, launch files, and source code for the final SAPHARI review.

## Installation

This installation was tested for Ubuntu 14.04 with ROS Indigo.

Please install the following debian deps:
* ```sudo apt-get install libcdk5-dev libncurses5-dev automake autoconf```

To speed-up development, you will need three workspaces which overlay sequently: First, there is a ```catkin``` workspace for ```knowrob```, then comes another ```catkin``` workspace for all other ```catkin``` packages, and finally there is a ```rosbuild``` workspace for the ```dlr_action_bridge```. For convenience, we have created several ```rosinstall``` files.

First, create the workspaces in the above described order:
* ```cd && mkdir -p ros/saphari_knowrob/src && mkdir -p ros/saphari_catkin/src && mdir ros/saphari_rosbuild```
* ```cd ~/ros/saphari_knowrob/src && wstool init```
* ```cd ~/ros/saphari_knowrob && source /opt/ros/indigo/setup.bash && catkin_make```
* ```cd ~/ros/saphari_catkin/src && wstool init```
* ```cd ~/ros/saphari_catkin && source ~/ros/catkin_knowrob/devel/setup.bash && catkin_make```
* ```cd ~/ros/saphari_rosbuidl && wstool init```
* ```echo source ~/ros/saphari_catkin/devel/setup.bash >> ~/.bashrc```
* ```echo export ROS_PACKAGE_PATH=~/ros/saphari_rosbuild:${ROS_PACKAGE_PATH} >> ~/.bashrc```
* ```source ~/.bashrc```

Then, initialize and build the knowrob workspace:
* ```cd ~/ros/saphari_knowrob/src```
* ```wstool merge https://raw.githubusercontent.com/code-iai/saphari_final_review/master/saphari_final_review/rosinstall/saphari_knowrob.rosinstall```
* ```wstool update```
* ```rosdep install --ignore-src --from-paths stacks/```
* ```cd ~/ros/saphari_knowrob```
* ```catkin_make```

Afterwards, init and build the remaining catkin packages in the 2nd overlay. Unfortunately, you will need one of 'em precious ```CATKIN_IGNORE``` to make your everything compile:
* ```cd ~/ros/saphari_catkin/src```
* ```wstool merge https://raw.githubusercontent.com/code-iai/saphari_final_review/master/saphari_final_review/rosinstall/saphari_catkin.rosinstall```
* ```wstool update```
* ```roscd iai_boxy_hw```
* ```touch CATKIN_IGNORE```
* ```cd ~/ros/saphari_catkin```
* ```catkin_make```

Finally, checkout and build the ```dlr_action_bridge``` in the ```rosbuild``` workspace:
* ```cd ~/ros/saphari_rosbuild```
* ```wstool merge https://raw.githubusercontent.com/code-iai/saphari_final_review/master/saphari_final_review/rosinstall/saphari_rosbuild.rosinstall```
* ```wstool update```
* ```rosmake dlr_action_bridge```

To conclude the setup, your user needs the rights to write to shared memory and set high real-time priorities. Using sudo, add the following lines to the file ```/etc/security/limits.conf```:
* ```YOUR-USER           -       rtprio          99```
* ```YOUR-USER           -       memlock         250000```


## Running 

### Simulation
Start the simulated environment:
* ```roslaunch saphari_robot_bringup robot_sim.launch```

## Debugging/Testing

### URDF 
To play with the ```urdf``` of the robot in ```rviz``` launch:
* ```roslaunch saphari_robot_description display.launch```

### SIMULATION
Start the simulation:
* ```roslaunch saphari_robot_bringup robot_sim.launch```
 
Use ```beastypy``` to verify that we can move the arm around:
* ```roscd dlr_beastypy_examples```
* ```./src/rcu_test_sim.py```
* ```>>> start_on_table(beasty)```

Use the command-line tools to check that we can command the gripper:
* ```rostopic pub -r 10 /gripper/position_goal iai_wsg_50_msgs/PositionCmd '{pos: 90, speed: 20.0, force: 10.0}'```
* ```rostopic pub -r 10 /gripper/position_goal iai_wsg_50_msgs/PositionCmd '{pos: 10, speed: 20.0, force: 10.0}'```
