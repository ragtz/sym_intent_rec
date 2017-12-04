from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory
from vector_msgs.msg import GripperCmd
from sym_intent_rec.msg import WorldState
from std_msgs.msg import String
from numpy.random import choice
from collections import deque
from threading import Lock
from copy import deepcopy
import numpy as np
import actionlib
import rospy
import pickle
import time
import sys
import os

n = 3 # number of conseutive intent inputs to trigger
#n = 1 # number of conseutive intent inputs to trigger
AND = lambda l: reduce(lambda x, y: x and y, list(l))

def move_arm(traj):
    client = actionlib.SimpleActionClient('/jaco_trajectory_controller/trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj

    client.send_goal(goal)
    client.wait_for_result()

def move_gripper(pub, cmd):
    pub.publish(cmd)

def play_seq(manip_seq, gripper_pub, objs, obj_idx, bin_name): 
    seq = manip_seq[objs][obj_idx][bin_name]
    for action in seq:
        if action['type'] == 'arm':
            move_arm(action['msg'])
        else:
            move_gripper(gripper_pub, action['msg'])
            time.sleep(3)

def update_intent(msg, args):
    intent_history, lock = args
    with lock:
        intent_history.append(msg.data)

def update_state(msg, args):
    state, lock = args
    with lock:
        state.left = msg.left
        state.right = msg.right

if __name__ == "__main__":
    manip_seq = pickle.load(open(sys.argv[1]))

    rospy.init_node('manip_playback')

    state = rospy.wait_for_message('/world_state', WorldState, timeout=5)
    intent_history = deque([], n)

    state_lock = Lock()
    intent_lock = Lock()

    intent_pub = rospy.Publisher('/detected_intent', String, queue_size=10)
    gripper_pub = rospy.Publisher('/vector/right_gripper/cmd', GripperCmd, queue_size=10)
    rospy.Subscriber('/world_state', WorldState, update_state, (state, state_lock), queue_size=10)
    rospy.Subscriber('/intent', String, update_intent, (intent_history, intent_lock), queue_size=10)

    num_objs = sum(state.right) + sum(state.left)

    # Say can start (e.g. "Start")
    os.system("aplay ~/vector_ws/src/chime.wav")
    
    while not rospy.is_shutdown() and num_objs > 0:
        intent = None
        with intent_lock:
            if len(intent_history) == n:
                if AND(np.array(list(intent_history)) == 'left'):
                    intent = 'left'
                    intent_pub.publish(intent)
                elif AND(np.array(list(intent_history)) == 'right'):
                    intent = 'right'
                    intent_pub.publish(intent)

        if not intent is None:
            if intent == 'left':
                print 'LEFT'
                objs = 'right_objs'
                with state_lock:
                    objs_state = deepcopy(state.right)
            elif intent == 'right':
                print 'RIGHT'
                objs = 'left_objs'
                with state_lock:
                    objs_state = deepcopy(state.left)
                    
            obj_idx = choice([i for i, v in enumerate(objs_state) if v]) # random from available objects
            bin_name = choice(['left_bin', 'right_bin']) # random bin

            play_seq(manip_seq, gripper_pub, objs, obj_idx, bin_name)
            time.sleep(2)

            # Say can continue (e.g. "Next")
            os.system("aplay ~/vector_ws/src/Shutter-01.wav")

        with state_lock:
            num_objs = sum(state.right) + sum(state.left)
            
