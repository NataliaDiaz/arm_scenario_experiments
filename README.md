# Install 

Clone this repository into your catkin workspace "src" folder  
In the catkin_ws folder, run 
```
catkin_make
```

In arm_scenario_experiments run : 
```
./after_install.sh
```
1. If needed, check in after_install.sh file, if  'ros_ws' in the script should be changed to 'catkin_ws' because of the path names. This may be needed just once.
2. In general,  open three terminals at ~/catkin_ws, each launching first
./baxter.sh sim
then 
a) roslaunch arm_scenario_simulator baxter_world.launch   
in the first terminal
Wait to see baxter and for the message: 
[ INFO] [1507731135.345473470, 109.942000000]: Simulator is loaded and started successfully
[ INFO] [1507731135.351378551, 109.944000000]: Robot is disabled
[ INFO] [1507731135.351602182, 109.944000000]: Gravity compensation was turned off


b) rosrun arm_scenario_simulator spawn_objects_example 
in the second terminal, and wait for [INFO] [WallTime: 1507731557.483257] [1447.362000] creation of cylinder2 successful
Running. Ctrl-c to quit


c) For more than 1 sequence:
./run.sh
for one sequence: 
rosrun arm_scenario_experiments button_babbler here 
in the third terminal (if you use a script, you need to change 'here' in each iteration). 
NOTE 1: If you run two times 'roslaunch arm_scenario_simulator baxter_world.launch' there will be errors.
NOTE 2: In order to use relative positions of the button, the RELATIVE option in the baxter_representation_learning_3D/supervised.lua file is used. 
But other changes need to be made in files like function.lua to read the button positions from data files. I should have added some code but I'm not sure that it will work.



# Description

This package contains the code needed to carry out data collection and experiments with the `arm_scenario_simulator` package.
Basically, this fork is only used to generate baxter pushing button data.

# How to augment data a little bit (needed)

You can modify color of table in : `catkin_ws/src/arm_scenario_simulator/models/DREAM_table/model.sdf`
just change line 36 : 

`<name>Gazebo/SkyBlue</name>`
to 
`<name>Gazebo/Red</name>`  for example

See  `/usr/share/gazebo-2.2/media/materials/scripts/gazebo.material` for other material available (you can try to create your own if you want)

The color of button is selected randomly

If you want to change button position see `catkin_ws/src/arm_scenario_experiments/scripts/button_pressing/button_babbler` line 46

# Instruction to record sequences :

```
cd ~/catkin_ws/src/arm_scenario_experiments
./process
```


Wait for gazebo and open a new terminal window
```
rosrun arm_scenario_simulator spawn_objects_example
```

open a new terminal window
```
rosrun arm_scenario_experiments button_babbler here
```

The script will launch, creating sequences of images in record_X directory in 'here'

#Error you can get (because sometimes, ros hates you)

`OSError: [Errno 110] Failed to get robot state on robot/state`
Just relauch the script a few times, it will work

`Service IK error`
Meaning : Moving the arm is not possible for an unknown reason...
Usulally, restarting ros and gazebo seems to do the trick, but ugly ...
