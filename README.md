# SAPHARI-FINAL-REVIEW
Configurations, launch files, and source code for the final SAPHARI review.

## Installation

This installation was tested for Ubuntu 14.04 with ROS Indigo.

Please install the following debian deps:
* ```sudo apt-get install libcdk5-dev```

For building the sources, you will need a ```catkin``` workspace overlayed with a ```rosbuild``` workspace.

In your ```catkin``` workspace, please add the following PUBLICLY available repos:
* ```git clone git@github.com:code-iai/saphari_final_review.git```
* ```git clone git@github.com:code-iai/iai_robots.git```

In your ```catkin``` workspace, please add the following PRIVATELY available repos:
* ```git clone gitolite@kif.ai.uni-bremen.de:iai_drivers_private.git```

In your ```rosbuild``` workspace, please add the following PRIVATELY available repos:
* ```git clone ssh://gitolite@kif.ai.uni-bremen.de:2023/dlr_lwr.git```

First, build the ```catkin``` workspace:
* ```catkin_make```

Then, build the ```dlr_action_bridge``` in the ```rosbuild``` workspace:
* ```rosmake dlr_action_bridge```

Finally, your user needs the rights to write to shared memory and set high real-time priorities. Using sudo, add the following lines to the file ```/etc/security/limits.conf```:
* ```YOUR-USER           -       rtprio          99```
* ```YOUR-USER           -       memlock         250000```


## Running 

### Simulation
Start the simulated environment:
* ```roslaunch saphari_robot_bringup sim.launch```

## Debugging/Testing

### URDF 
To play with the ```urdf``` of the robot in ```rviz``` launch:
* ```roslaunch saphari_robot_description display.launch```

### SIMULATION
Start the simulation:
* ```roslaunch saphari_robot_bringup sim.launch```
 
Use ```beastypy``` to verify that we can move the arm around:
* ```roscd dlr_beastypy_examples```
* ```./src/rcu_test_sim.py```
* ```>>> start_on_table(beasty)```
