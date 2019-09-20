source devel/setup.bash

gnome-terminal -- bash -c "roslaunch my_robot world.launch";
sleep 5
gnome-terminal -- bash -c "roslaunch ball_chaser ball_chaser.launch";
sleep 5
gnome-terminal -- bash -c "roslaunch main amcl.launch";
sleep 5
gnome-terminal -- bash -c "roslaunch main rviz.launch";
sleep 5
gnome-terminal -- bash -c "rosrun add_markers add_markers";
sleep 5
gnome-terminal -- bash -c "rosrun pick_objects pick_objects";
sleep 5
