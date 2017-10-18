#!/bin/bash

# this file should be placed in the catkin_ws or ros_ws directory to run it to generate data record sequences of Baxter pushing a button
# THIS FILE SHOULD BE PLACED IN catkin_ws or ros_ws to be run to generate baxter data
# See execution instructions in arm_scenario_experiments README https://github.com/NataliaDiaz/arm_scenario_experiments

mkdir generated_data
for i in {1..150}  # assumes some will fail, we need at least 75
do
  #path_to_save="non_static_button/record_"  # takes name of var? TODO Fix!
  rosrun arm_scenario_experiments button_babbler here
  mv here/record_0 generated_data/record_"${i}"
  printf "\n\n\n +++++++++++++++++ New record in data/record_${i} !\n "
done
