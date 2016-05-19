#!/usr/bin/env python

import math
import random
import struct
import argparse

import rospy

from tf import transformations as tft
np = tft.numpy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (Header, String, Empty)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
from baxter_interface import CHECK_VERSION

min_ranges = {'left_s0':-1.7016, 'left_s1':-2.147, 'left_e0':-3.0541, 'left_e1':-0.05,
                     'left_w0':-3.059, 'left_w1':-1.5707, 'left_w2':-3.059,
            'right_s0':-1.7016, 'right_s1':-2.147, 'right_e0':-3.0541, 'right_e1':-0.05,
                     'right_w0':-3.059, 'right_w1':-1.5707, 'right_w2':-3.059 }


max_ranges = {'left_s0':1.7016, 'left_s1':1.047, 'left_e0':3.0541, 'left_e1':2.618,
                     'left_w0':3.059, 'left_w1':2.094, 'left_w2':3.059,
            'right_s0':1.7016, 'right_s1':1.047, 'right_e0':3.0541, 'right_e1':2.618,
                                 'right_w0':3.059, 'right_w1':2.094, 'right_w2':3.059 }

magnitudes = {name:max_ranges[name]-min_ranges[name] for name in max_ranges.keys()}


all_names = ['head_pan', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint',
            'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2',
            'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint',
            'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']

start_joint_angles1 = [0, 0, 0, -1.597, 1.751, -0.394, 0.050, -0.185, 0.027, -0.013, 0, 0, 1.552, 0.939, -1.049, -0.055, 0.303, 2.094, -1.424]
start_joint_angles2 = [0.345, 0.020, -0.020, -1.457, 1.406, -0.344, -0.119, -0.177, 0.405, -2.764, 0, -0.020, 1.545, 0.937, -1.044, -0.035, 0.301, 2.085, -1.417]
start_pose1, start_pose2 = {}, {}
for k in range(len(start_joint_angles1)): start_pose1[all_names[k]] = start_joint_angles1[k]
for k in range(len(start_joint_angles2)): start_pose2[all_names[k]] = start_joint_angles2[k]

start_left_position1 = {
    'x': 0.636,
    'y': -0.386,
    'z': 0.408
    }
start_left_orientation1 = {
    'x': 0.148,
    'y': -0.680,
    'z': 0.718,
    'w': 0.0181,
    }

start_left_position2 = {
    'x': 0.80,
    'y': -0.30,
    'z': 0.50
    }
start_left_orientation2 = {
    'x': 0.729,
    'y': -0.033,
    'z': 0.173,
    'w': 0.660
    }

'''
middle of the screen
'''
start_left_position3 = { 
    'x': 0.877365622927,
    'y': -0.00359175100893,
    'z': 0.552679248842
}
start_left_orientation3 = {
    'x': -0.0443491357961,
    'y': -0.669163533888,
    'z': 0.735201780726,
    'w': -0.0986491798571}

start_left_position4 = { 
    'x': 0.793022441528,
    'y': -0.142764098196,
    'z': 0.565026046693
}
start_left_orientation4 = { 
    'x': -0.0147827824125,
    'y': -0.691505856462,
    'z': 0.703128036524,
    'w': -0.164960856195
}



start_pose = start_pose2
start_left_position = start_left_position4
start_left_orientation = start_left_orientation4

class Atomic_Babbler(object):

    def __init__(self, pan_head):
        # Command Current Joint Positions first
        self.left_arm = baxter_interface.Limb('left')
        self.right_arm = baxter_interface.Limb('right')
        #self.babbling_arm = self.left_arm if side is 'left' else self.right_arm
        self.head = baxter_interface.Head()
        self.pan_head = pan_head
        self.right_arm.move_to_joint_positions({name:start_pose[name] for name in self.right_arm.joint_names()})
        self.left_arm.move_to_joint_positions({name:start_pose[name] for name in self.left_arm.joint_names()})

        self.joint_publisher = rospy.Publisher('/atomic_babbler/status/joint_name', String, latch = True, queue_size=1)
        self.pose_publisher = rospy.Publisher('/atomic_babbler/status/new_pose', Empty, queue_size=1)

    def go_and_move(self):
        joint_angles=None
        while joint_angles == None:
            x = start_left_position['x'] + 2*(random.random()-0.5)*0.15
            y = start_left_position['y'] + 2*(random.random()-0.5)*0.15
            z = start_left_position['z'] + 2*(random.random()-0.5)*0.10
            o = slight_d(start_left_orientation,0.1,0.1)
            joint_angles = IK(self.left_arm, {'x':x,'y':y,'z':z}, o)
        self.left_arm.move_to_joint_positions(joint_angles)
        self.head.set_pan(random.uniform(-0.3,0.3))

        self.pose_publisher.publish(Empty())
        self.move_around()


	

    def move_around(self):
        rospy.sleep(1)
        if self.pan_head: self.turn_head(speed= 70, pause = 0.5)
        self.jiggle('left_s0', [-0.2,0.2], speed= 0.4, pause = 0.5, margin = 0.2)
        self.jiggle('left_s1', [-0.1,0.2], speed= 0.4, pause = 0.5, margin = 0.2)
        self.jiggle('left_e0', [-0.1,0.1], speed= 0.4, pause = 0.5, margin = 0.2)
        self.jiggle('left_e1', [-0.2,0.2], speed= 0.4, pause = 0.5, margin = 0.2)



    def turn_head(self, speed, pause):
        init_pan = self.head.pan()
        self.joint_publisher.publish(String('head_pan'))
        self.head.set_pan(-0.8, speed=speed)
        rospy.sleep(pause)
        self.head.set_pan(0.8, speed=speed)
        rospy.sleep(pause)
        self.head.set_pan(init_pan, speed=speed)
        self.joint_publisher.publish(String(''))
        rospy.sleep(pause)


    def jiggle(self, joint_name, normalized_targets, speed, pause = 0, margin = 0):
        '''
        Make the joint move to the targets defined by their deltas w.r.t the current pose,
        deltas being the ratio of the angular difference and the magnitude
        Make a pause between each movement
        '''
        self.joint_publisher.publish(String(joint_name))
        init_pos = self.left_arm.joint_angles()[joint_name]
        prev_target = init_pos
        for nt in normalized_targets:
            target = init_pos+nt*magnitudes[joint_name]
            target = clip(target, joint_name, margin)
            move(self.left_arm, {joint_name: target}, length = abs(target-prev_target)/speed)
            prev_target = target
            rospy.sleep(pause)
        move(self.left_arm, {joint_name: init_pos}, length = abs(init_pos-prev_target)/speed )
        self.joint_publisher.publish(String(''))
        rospy.sleep(pause)


def slight_d(quat,stdvec,stda):    
    v = [quat['x'],quat['y'],quat['z'],quat['w']]
    vec = tft.unit_vector(v[:3])
    a = 2*math.acos(v[3]) + random.random()*stda
    vec += 2*(np.random.random((3,))-0.5)*stdvec
    vec =  math.sin(a/2)*vec
    return {'w':math.cos(a/2), 'x':vec[0], 'y':vec[1], 'z':vec[2]}

def move(limb, target_positions, length, uprate=100):
    '''
    Move linearly all the joints from their current position to the position specified in the target_positions dictionary
    The length of the movement has to be given
    '''
    length = max(0.5,length)
    rate = rospy.Rate(uprate)
    tstart = rospy.get_time()
    start_joints = limb.joint_angles()
    names = target_positions.keys()
    while not rospy.is_shutdown():
        t = rospy.get_time() - tstart
        if t>length:break
        targets = {name: target_positions[name]*t/length + (length-t)*start_joints[name]/length for name in names}
        limb.set_joint_positions(targets)
        rate.sleep()


def clip(x, name, margin_min=0, margin_max=None):
    '''
    clip the joint_angle between the min_value and max_value (the valid interval) of this joint_angle
    margins (in [0,1]) can be provided to addtionally reduce the valid interval. Limits are augmented/reduced by margin*magnitude(joint)
    '''
    margin_max = margin_max if margin_max else margin_min
    return min(max_ranges[name]-margin_max*magnitudes[name] , max(x, min_ranges[name]+margin_min*magnitudes[name]))



def IK(limb, position, orientation):
    ns = "ExternalTools/" + limb.name + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    pose = PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id='base'),
            pose=Pose(
                position=Point(
                    x=position['x'],
                    y=position['y'],
                    z=position['z'],
                ),
                orientation=Quaternion(
                    x=orientation['x'],
                    y=orientation['y'],
                    z=orientation['z'],
                    w=orientation['w'],
                ),
            ),
        )

    ikreq.pose_stamp.append(pose)
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" % (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp
        return limb_joints
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    #parser.add_argument('side', dest='side', required=True, choices=['left','right'], help="side to use to babble")
    parser.add_argument( '-ph', '--pan-head', dest='pan_head', default=False, action='store_true', help="do the head_pan (should not be done in simulator)")
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing babbler node... ")
    print('with do head :', args.pan_head)
    rospy.init_node("node_clement")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    babbler = Atomic_Babbler(args.pan_head)
    for k in range(100):
        babbler.go_and_move()
