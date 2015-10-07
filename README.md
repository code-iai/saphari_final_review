# SAPHARI-FINAL-REVIEW
Configurations, launch files, and source code for the final SAPHARI review.

## Installation

Navigate to the source-directory of a new empty catkin workspace:
```roscd && cd ../src```

Checkout this repository:
```git clone git@github.com:code-iai/saphari_final_review.git```

Get public source dependendies:
* ```git clone git@github.com:code-iai/iai_robots.git```

## Running 

### Simulation
Start the simulated environment:
```roslaunch saphari_robot_bringup sim.launch```

## Debugging/Testing

### URDF 
To play with the ```urdf``` of the robot in ```rviz``` launch:
```roslaunch saphari_robot_description display.launch```
