#!/bin/bash

# this file should be placed in the catkin_ws or ros_ws directory to run it to generate data record sequences of Baxter pushing a button
for i in {1..150}
  path_to_save="non_static_button/record_"
  rosrun arm_scenario_experiments button_babbler here
  mv here/record_0 path_to_save"${i}"
  echo "new record in "path_to_save"${i}"
done
