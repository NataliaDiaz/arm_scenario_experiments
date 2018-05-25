#!/usr/bin/env python
''' This script controls baxter by randomly choosing position for the left-limb end effector and using the Inverse Kinematic service (hence IK) to reach this point.

The script also controls what is recorded, so that a snapshot of the world is recorded after every 3D position of the end-effector is reached (and nothing is recording in between)
This is usefull as the recorded messages are already corresponding to states between 2 actions hae been chosen.
If permanent recording had been used instead, it would have been harder (while using the data) to pick up the states corresponding to moments between two actions.

## Absolute and Relative positions:
object.get_state(): gives absolute positions
utils.change_CS(): transforms into coordinates relative to baxter

TODO: perhaps add suffxes to position variables to make it clear
TODO: In order to generate a test set, try to make the robot move uniformly. And when it approaches the button, its delta should be small.
TODO: 1. add a fixed point for the robot
TODO: 2. add data augmentation, colors table and button, change each time!

'''
import time
import math
import random
import argparse
import subprocess

import std_msgs.msg
from std_msgs.msg import String
import rospy
from tf import transformations
np = transformations.numpy
quat_conj = transformations.quaternion_conjugate
from geometry_msgs.msg import Point, Quaternion, Vector3, Vector3Stamped
from std_msgs.msg import Header
import baxter_interface
import arm_scenario_simulator as arm_sim
from arm_scenario_experiments import Recorder, utils, baxter_utils

# ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
IK_seed_positions = [-1.535, 1.491, -0.038, 0.194, 1.546, 1.497, -0.520]
z_table = 0.76092828353
button_pos_initial = [0.6, 0.30, 0.77] # abosulte
_in_every_seq = [0.6, 0.30, 0.10] # reference point present in every sequence

use_hardcoded_trajectory = False  # Does not reach button
change_table_color_per_sequence = True
change_button_position_per_sequence = True

def main(path):
    # this is the expert controller. Given the actual position of the ee, it gives the optimal action to perform to get closer to the button
    def action_to_goal(end_point_position, button_pos_relative, delta=None):
        # TODO: use parameter to control output
        # print "end_point_position",end_point_position
        # delta = np.random.uniform(0.03, 0.07)
        if delta==None:
            delta = np.random.uniform(0.03, 0.07)
        action = np.sign(button_pos_relative-end_point_position)*delta
        print "action",action
        return action

    def adapt_delta(end_point_position, button_pos_relative):
        # choosing small deltas when near the button

        delta = 0
        dist = np.linalg.norm(end_point_position - button_pos_relative)
        print("Distance now is", dist)
        if dist <= 0.1:
            delta = np.random.uniform(0.02, 0.03)
        elif dist > 0.1 and dist <= 0.2:
            delta = np.random.uniform(0.03, 0.05)
        elif dist >  0.3:
            delta = np.random.uniform(0.05, 0.09)
        return delta

    def reset_button(button_pos_absolute):
            if np.linalg.norm(button_pos_absolute-utils.point2array(button.get_state().pose.position))>0.0005:
                button.set_state(position = Point(*button_pos_absolute), orientation = button_orientation)

    # Function that randomly put the button on the table (useful to initialize a new sequence)
    def move_button():
        z_fixed = 0.76092828353
        # comment this block to fix button position
        #<<<<<<<<<<<<<<<<<<<<<<<<<<<<< BLOCK NON STATIC BUTTON>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   #########################################
        while True:
            new_position = np.random.uniform(mins,maxs)

        #theoretically button should be at
        # #============ FIXED BUTTON POSITION =================
        # print "Warning, button position is fixed, you can change that in the button_babbler file"
        # new_position[0] = 0.50
        # new_position[1] = 0.30
        # #====================================================
            new_position[2] = 0 #Spawn above the ground to avoid collision, other wise the button move all the time
            button.set_state( Point(*utils.change_CS(new_position, -baxter_position, quat_conj(baxter_orientation)) ), orientation = button_orientation )
            time.sleep(1) #Wait a little to let the button fall, to get true position
            real_position = utils.point2array(button.get_state().pose.position)
            # sometimes the button falls downs the table or lands on other objects.. so add this test!
            if (real_position[0] < maxs[0] and real_position[0] > mins[0]) and (real_position[1] < maxs[1] and real_position[1] > mins[1]):
                if (abs(real_position[2] - z_fixed) < 0.01):
                    # pass
                    break
                else:
                    print("Button not properly on the table")
                    print("z value is ", real_position[2])
            else:
                print("Button not properly on the table")
                print("real position is", real_position)
        #<<<<<<<<<<<<<<<<<<<<<< BLOCK FOR NON STATIC BUTTON >>>>>>>>>>>>>>>>>>>>>>>>>>>>> #########################################



        # self.add_object( arm_sim.Button('button1').spawn( Point(x=0.6, y=0, z=0.78) ) )
        # FIXING button position
        # button_pos_initial = Point(x=0.6, y=0, z=0.78)
        # if np.linalg.norm(button_pos_initial - utils.point2array(button.get_state().pose.position)) > 0.1:
            # print("Button position changed, replacing it back")
        button.set_state(position = Point(*button_pos_initial) )
        # print "button.get_state().pose.position", button.get_state().pose.position
        button_pos_from_topic = button.get_state().pose.position
        button_pos_absolute = utils.point2array(button_pos_from_topic)
        button_pos_relative = utils.change_CS( button_pos_absolute, baxter_position, baxter_orientation)
        button_pos_pub.publish( Point(*button_pos_relative) )
        return button_pos_relative, button_pos_absolute
    def restore_lever():
        # sometimes the lever falls down. So try to replace it on the table
        def r(maxVar):
            return random.random()*2*maxVar - maxVar
        lever_position = utils.point2array(lever.get_state().pose.position)
        if abs(lever_position[2] - z_table) > 0.1:
            lever.set_state( Point(x = 0.6 + r(0.1), y = 0 + r(0.3), z = 0.78)  )
            print("lever restored to table")

    def move_limb_to_init():
        joints = None
        while not joints:
            # position = np.random.uniform(mins,maxs)
            # position[2] = mins[2] + 7*delta
            if use_hardcoded_trajectory:
                position = ref_point_in_every_seq
            else:
                # position = np.array([0.45,  0.55, 0.28]) #TODO: FIX, DOES NOT WORK WHEN NOT FOUND! and crashes the simulator provideing TIMEDOUT error
                position = np.array([0.3,  0.3, 0.3])
            joints = baxter_utils.IK(limb, position, ee_orientation, IK_seed_positions)
        limb.move_to_joint_positions(joints)
        return position

    # TODO right hand not hangling out of sight DONE
    def move_limb_right_to_init():
        joints = None
        # print(utils.point2array(limb_right.endpoint_pose()['position']))
        # [ 0.26033143 -0.74212844  0.11364821] initial position
        # [short side of table, long side, height]
        # when increaseing, [forward ,left, high]
        while not joints:
            if use_hardcoded_trajectory:
                position = np.array([0.70, -0.60, 0.10]) + [np.random.uniform(-0.05,0.05), np.random.uniform(-0.10,0.10), np.random.uniform(-0.1,0.1)]  #TODO Why this little difference?
            else:
                position = np.array([0.70, -0.62, 0.10]) + [np.random.uniform(-0.05,0.05), np.random.uniform(-0.10,0.07), np.random.uniform(-0.08,0.08)]
            joints = baxter_utils.IK(limb_right, position, ee_orientation_right, IK_seed_positions)
        limb_right.move_to_joint_positions(joints)
        return position

    def wait_for_messages(excepts=[]):
        while not recorder.all_buffers_full(excepts=excepts):
            print('waiting')
            #print('\n'.join([(key, 'None' if value is None else 'ok') for (key,value) in recorder.lastMessages.iteritems()]))
            rospy.sleep(0.01)



    ## ACTUAL PROGRAM #####################################################
    # communication
    pub = rospy.Publisher('babbler', String , queue_size = 10)
    loop_rate = rospy.Rate(10)

    limb = baxter_interface.Limb('left')
    limb_right = baxter_interface.Limb('right')
    ee_orientation = baxter_utils.get_ee_orientation(limb)
    ee_orientation_right = baxter_utils.get_ee_orientation(limb_right)

    # those are publishers specially made to be listenned by the recorder, so that we can publish and thus record exactly what we want when we want
    action_pub = rospy.Publisher('/robot/limb/left/endpoint_action', Vector3Stamped, queue_size=1)
    button_pos_pub = rospy.Publisher('/button1/position', Point, queue_size=1)

    # here we define what topics the recorder will listen to (which is necessary to be able to record some message from them)
    recorder = Recorder(path, prefix='/recorded', topics = [ '/cameras/head_camera_2/image/compressed',
                                                            '/cameras/left_hand_camera/image/compressed',  #see http://sdk.rethinkrobotics.com/wiki/API_Reference#Cameras_2
                                                            '/robot/joint_states',
                                                            '/robot/limb/left/endpoint_state',
                                                            '/robot/limb/left/endpoint_action',
                                                            '/button1/is_pressed',
                                                            '/button1/position'])


    # Here we define some parameters for the babbling
    # DONE: change this delta randomly in the main loop
    delta = 0.05
    possible_deltas = [i*delta for i in xrange(-1,2)]

    # Relative values
    mins = np.array([0.42, -0.1, -0.11])# near left low
    maxs = np.array([0.75, 0.60, 0.35]) #  far rightmost high
        # [short side of table, long side, height]
        # when increasing, [forward ,left, high]
    # mins_resetting = np.array([0.40, -0.1, -0.11])
    # maxs_resetting = np.array([0.80, 0.60, 0.35])


    nstep = 250  # THIS IS THE NUMBER OF FRAMES = N OF ACTIONS PER DATA SEQUENCE (EPISODE)
    if use_hardcoded_trajectory: # DOES NOT END UP PUSHING BUTTON
        nstep = 50
        targets = []
        # postion = np.array([0.45,  0.55, 0.32])
        targets.append([0.42, 0.1, 0.15])
        # targets.append([0.45, -0.05, 0.12])
        # targets.append([0.45,  0.55, 0.12])
        targets.append([0.45,  0.55, -0.09])
        # targets.append([0.45, -0.05, -0.09])

        # targets.append([0.58, -0.05,  0.15])
        targets.append([0.58,  0.55, -0.09])
        targets.append([0.58, -0.05,  0.15])
        targets.append([0.58,  0.55,  0.28])
        # targets.append([0.58, -0.05, -0.09])
        # targets.append([0.58, -0.05,  0.32])
        # targets.append([0.58,  0.55,  0.32])

        targets.append([0.72,  0.55,  0.28])
        # targets.append([0.77, -0.05,  0.15])
        targets.append([0.72, -0.05,  0.15])
        # targets.append([0.72,  0.55,  0.15])
        targets.append([0.72,  0.55, -0.09])
        # targets.append([0.77,  0.55, 0.15])
        print 'Total target points for hard-coded trajectory of left arm: ',len(targets)



        # self.objects['button1'].set_base_color(rgba = [30,20,220])
    # lever test
    lever = arm_sim.Lever('lever1')
    # lever_position = utils.point2array(lever.get_state().pose.position)
    # lever_position_relative = utils.change_CS(lever_position, baxter_position, baxter_orientation)
    # print("lever absolute", lever_position)
    # print("lever relative", lever_position_relative)
    button = arm_sim.Button('button1')
    button_orientation = button.get_state().pose.orientation
    baxter = arm_sim.Button('baxter')
    baxter_pose = baxter.get_state().pose
    baxter_position = utils.point2array(baxter_pose.position)
    baxter_orientation = utils.quat2array(baxter_pose.orientation)
    button_pos_relative, button_pos_absolute = move_button()


    # Actually starts the babbling
    nrecords = 1
    for nb_button_pos in range(nrecords):
        if change_table_color_per_sequence:
            pub.publish("change_color")
        if change_button_position_per_sequence:
            pub.publish("change position")
        loop_rate.sleep()
        subprocess.call(["rosrun","arm_scenario_experiments","button_init_pose"])

        recorder.new_bag('record_'+str(nb_button_pos))
        if change_button_position_per_sequence:
            button_pos_relative, button_pos_absolute = move_button()
            print("button absolute", button_pos_absolute, ' button relative pos: ', button_pos_relative)
        end_point_position = move_limb_to_init()
        # move left limb into field of view
        move_limb_right_to_init()
        restore_lever()
        if not use_hardcoded_trajectory:
            reset_button(button_pos_absolute)
        else:  # TODO merge these two options in reset_button
            button_pos_relative, button_pos_absolute = move_button()
            button_pos_test = utils.change_CS_reverse(button_pos_relative, baxter_position, baxter_orientation)
            print(" button_pos_absolute", button_pos_absolute)
            print(" button_pos_relative", button_pos_relative)
            print(" button_pos_test", button_pos_test)

        exit_position_relative = None
        follow_another_trajectory = False
        attempt_button_relative = None

        k, k_success = 0, 0
        recently_pressed = False
        expert_control = True
        random_motion = True # generating a uniform test set TODO: not used, decrease exploration rate
        buffer_action = []
        if not simple_steps:
            count_exit = 0
            count_stupid = 0
            is_stupid = False  # if doing random movements not directed to push button.
            is_normal = False
            is_ref = False
            pWrongButton = 0.85  # the prob of touching somewhere where the button is not placed, the "babbling" itself
        while k_success<nstep and k<nstep*1.2:  # TODO WHY this 1.2?
            delta = np.random.uniform(0.03, 0.07)
            possible_deltas = [i*delta for i in xrange(-1,2)]
            # communication
            k = k+1
            actual_mins = np.array(mins)
            if np.linalg.norm(button_pos_relative[0:2]-utils.point2array(limb.endpoint_pose()['position'])[0:2])<=delta:
                actual_mins[2] -= delta
            if is_stupid:
                actual_mins[2] -= 0.07 # so it can touch the table surface

            # control
            if button.is_pressed():
                min_max_choice = [mins,maxs]

                # TODO change this to make the robot arm move to more positions
                exit_position_relative = np.random.uniform(mins,maxs)
                rand_id = np.random.randint(2)
                rand_min_or_max = np.random.randint(2)

                # change this so that the arm moves to more positions
                exit_position_relative[rand_id] = min_max_choice[rand_min_or_max][rand_id]
                print "Button is pressed. Now after",exit_position_relative

                # lifting up the arm while moving randomly
                # for _ in range(6):
                    # buffer_action.append(np.concatenate((np.random.choice(possible_deltas,2),[delta])))
                for _ in range(3):
                    if use_hardcoded_trajectory:
                        buffer_action.append(np.concatenate((np.random.choice(possible_deltas,2),[delta])))
                    else:
                        buffer_action.append([delta, np.random.choice([delta,-delta]), delta])
                # buffer_action = [np.concatenate((np.random.choice(possible_deltas,2),[delta])) for _ in xrange(6)]
                follow_another_trajectory = True
                is_stupid = False
                is_normal = False

            elif not buffer_action:
                if use_hardcoded_trajectory and random_motion and i_target < len(targets):
                    print("Following next hard-coded target", i_target)
                    target = targets[i_target]
                    buffer_action.append( action_to_goal(end_point_position, target, adapt_delta(end_point_position, button_pos_relative)) )
                    print("Dist to target", np.linalg.norm(end_point_position - target))
                    if np.linalg.norm(end_point_position - target) < 0.2:
                        i_target += 1
                else:
                    prob_essai = np.random.uniform(0,1) > pWrongButton  # pWrongButton is the prob of touching somewhere where the button is not placed
                    # if k == 2:
                    #     print "moving to reference point"
                    #     buffer_action.append(action_to_goal(end_point_position, , 0.003))
                    #     is_ref = True
                    # elif is_ref:
                    #     buffer_action.append(action_to_goal(end_point_position, , 0.003))
                    #     if np.linalg.norm(end_point_position - ) < 0.003:
                    #         print "ref point get"
                    #         is_ref = False
                    if follow_another_trajectory:
                        print "Restarting"
                        #During some steps, try to reach somewhere else on table, to simulate a kind of "goal babbling"
                        # buffer_action.append( action_to_goal(end_point_position, exit_position_relative, adapt_delta(end_point_position, exit_position_relative)) )
                        buffer_action.append( action_to_goal(end_point_position, exit_position_relative ))
                        # print "end_point_position",end_point_position
                        # print "exit_position_relative",exit_position_relative
                        count_exit += 1

                        if end_point_position[rand_id] <= mins[rand_id] or  end_point_position[rand_id] >= maxs[rand_id] or count_exit > 20:
                            follow_another_trajectory = False
                            count_exit = 0

                    elif is_stupid: # following random movements
                        print "Following trajectory to push on the table in a position where the button is not placed"
                        # print "hand position relative", end_point_position
                        # print "attempt_button_relative", attempt_button_relative
                        # buffer_action.append( action_to_goal(end_point_position, attempt_button_relative, adapt_delta(end_point_position, attempt_button_relative)) )
                        buffer_action.append( action_to_goal(end_point_position, attempt_button_relative))
                        # if abs(end_point_position[2] - button_pos_relative[2]) < 0.08:
                        count_stupid += 1
                        if np.linalg.norm(end_point_position - attempt_button_relative) < delta or count_stupid > 5:
                            count_stupid = 0
                            is_stupid = False
                            print("Wrong button terminated")
                            buffer_action.append(np.concatenate((np.random.choice(possible_deltas,2),[- delta])))
                            for _ in range(2):
                                buffer_action.append(np.concatenate((np.random.choice(possible_deltas,2),[delta])))

                    elif prob_essai and not is_normal:
                        print("Start following the wrong button")
                        # making mistakes while trying to press the button
                        # attempt_button_relative = utils.change_CS( attempt_button_absolute, baxter_position, baxter_orientation)
                        attempt_button_relative = [np.random.uniform(mins[0],maxs[0]), np.random.uniform(mins[1],maxs[1]), -0.17] # try to touch the table surface
                        # print("button absolute", attempt_button_absolute)
                        print("button relative", attempt_button_relative)
                        # buffer_action.append( action_to_goal(end_point_position, attempt_button_relative, adapt_delta(end_point_position, attempt_button_relative)) )
                        buffer_action.append( action_to_goal(end_point_position, attempt_button_relative))
                        is_stupid = True

                    else:
                        if use_hardcoded_trajectory:
                            buffer_action.append( action_to_goal(end_point_position, button_pos_relative, adapt_delta(end_point_position, button_pos_relative)) )
                        else:
                            buffer_action.append( action_to_goal(end_point_position, button_pos_relative))
                        is_normal = True
                        print("Acting normaly")

            action = buffer_action.pop(0)
            # TODO change this to make the robot's arm move to more positions
            print ''
            end_point_position_candidate = (end_point_position+action).clip(actual_mins, maxs)
            action = end_point_position_candidate - end_point_position

            try:
                joints = baxter_utils.IK(limb, end_point_position_candidate, ee_orientation)
            except:
                print "end_point_position_candidate", end_point_position_candidate
                raise

            if joints:
                action_pub.publish( Vector3Stamped(Header(stamp=rospy.Time.now()), Vector3(*action)) )
                end_point_position = end_point_position_candidate
                k_success +=1
                reset_button(button_pos_absolute)
                wait_for_messages(excepts=['/button1/position'])
                recorder.dump_all()
                limb.move_to_joint_positions(joints, timeout = 3)
                reset_button(button_pos_absolute) # TODO: WHY THIS IS DONE TWICE?

        wait_for_messages(excepts=['/button1/position','/robot/limb/left/endpoint_action']) # TODO: add here also left hand camera? why except these two topics only?
        recorder.dump_all()
        recorder.close_bag()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('path', type=str, help="path to bag file in which data will be recorded")
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing button_babbler node... ")
    rospy.init_node("learning_to_press_node")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    try: main(args.path)
    except rospy.ROSInterruptException:  pass

    subprocess.call(["rosrun","arm_scenario_experiments","button_bag_to_disk","here"])

    # subprocess.call(["rm","here/record_0.bag"])
    # subprocess.call(["rm","here/record_1.bag"])
