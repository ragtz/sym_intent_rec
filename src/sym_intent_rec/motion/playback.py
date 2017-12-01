from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory
from vector_msgs.msg import GripperCmd
from sym_intent_rec.msg import WorldState
from std_msgs.msg import String
from numpy.random import choice
import actionlib
import rospy
import pickle
import time
import sys

running = False

def move_arm(traj):
    client = actionlib.SimpleActionClient('/jaco_trajectory_controller/trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj

    client.send_goal(goal)
    client.wait_for_result()

def move_gripper(pub, cmd):
    pub.publish(cmd)

def play_seq(msg, args):
    global running
    manip_seq, state, gripper_pub = args

    intent = msg.data
    if not running and intent != 'none':
        running = True

        if intent == 'left':
            objs = 'right_objs'
            objs_state = state.right
        else:
            objs = 'left_objs'
            objs_state = state.left

        obj_idx = choice([i for i, v in enumerate(objs_state) if v]) # random from available objects
        bin_name = choice(['left_bin', 'right_bin']) # random bin

        print objs, obj_idx, bin_name
        
        seq = manip_seq[objs][obj_idx][bin_name]
        for action in seq:
            if action['type'] == 'arm':
                move_arm(action['msg'])
            else:
                move_gripper(gripper_pub, action['msg'])
                time.sleep(3)
    else:
        running = False

def update_state(msg, state):
    state.left = msg.left
    state.right = msg.right

if __name__ == "__main__":
    manip_seq = pickle.load(open(sys.argv[1]))

    rospy.init_node('manip_playback')

    state = rospy.wait_for_message('/world_state', WorldState, timeout=5)

    gripper_pub = rospy.Publisher('/vector/left_gripper/cmd', GripperCmd, queue_size=10)
    rospy.Subscriber('/world_state', WorldState, update_state, state)
    rospy.Subscriber('/intent', String, play_seq, (manip_seq, state, gripper_pub), queue_size=1)
    rospy.spin()

