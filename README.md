# Description
This is a ros project containing a simple wheeled robot capable of localization and navigation.

# Build
```
catkin_make
```

# Usage

## Using launch file
```
roslaunch main main.launch

# Open new terminal
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Using shell script
```
source main.sh
```

# Packages used

## Home service
The add_markers and pick_objects packages were created to provide the robot goal destinations and mark them in rviz. This simulates the robot picking and placing virtual objects.

## Robot
A robot was created from scratch in gazebo. It has a sensors such as camera and lidar and has actuators to move around the environment.

## Localization
The AMCL package is used for localization. It uses a particle filter to determine the robot pose with the help of a map and laser measurements.

## Mapping
A map was created with pgm map creator and provided to the robot

## Navigation
The move_base package is used for navigation. It allows the robot to reach a goal with the help of planners and cost maps.

