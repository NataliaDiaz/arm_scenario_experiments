# Description

This folder contains scripts needed to do some babbling around a push-button, record signals into bag files, extract the latter's data to the disk or providing a RL interface for learning to press a button.

The order in which the different scripts have to be launched :

1. `roslaunch arm_scenario_simulator baxter.world` (opens gazebo, creates a rosmaster node, spawns baxter and a table)

2. `rosrun ann4smc button_spawn_environment` (this is the script that spawns some objects and implements the consequences of actions on objects (e.g a pression on buttonA switches a lightB on) )

3. `rosrun ann4smc button_init_pose` (this scripts enables the robot and puts its arms in a good position to start manipulating stuff on the table)


**A) In order to perform babbling :**
`rosrun ann4smc button_babbler` (this scripts contains baxter's controller and defines what to record, so that records can be synchronized with the controler).
Alternatively, if you don't need to record in synchronization with the controller, recording can be performed in a separate ROS node (process) like `ann4smc recorder`

Once data have been recorded to bag files, one can use scripts such `button_bag_to_disk` to extract messages in topics to disk as text or jpgs (you have to edit the script accordingly to extract what you have previously recorded, the way you want)


**B) In order to provide a ROS Reinforcement Learning interface* to the simulator:**
`rosrun ann4smc button_RL_environment`
This will advertise two services corrsponding to the start and step functions.

*In most online RL algorithms, it is assumed the agent (the learner) can interact with the environment through 2 functions:
```
observation = start([options])
reward, observation, terminal = step(action)
```

**More details in the doscstrings and comments of each script**





**If all data generated is just .bag files, you can manually convert them records of images and .txt data with:
rosrun arm_scenario_experiments extract_bags 
