from vector_msgs.msg import GripperCmd
from sym_intent_rec.motion.playback import *
import rospy
import pickle
import time
import sys

def play_seq(manip_seq, gripper_pub, objs, obj_idx, bin_name): 
    seq = manip_seq[objs][obj_idx][bin_name]
    for action in seq:
        if action['type'] == 'arm':
            move_arm(action['msg'])
        else:
            move_gripper(gripper_pub, action['msg'])
            time.sleep(3)

if __name__ == "__main__":
    manip_seq = pickle.load(open(sys.argv[1]))
    objs = sys.argv[2]
    obj_idx = int(sys.argv[3])
    bin_name = sys.argv[4]

    rospy.init_node('manip_playback')
    gripper_pub = rospy.Publisher('/vector/left_gripper/cmd', GripperCmd, queue_size=10)
    play_seq(manip_seq, gripper_pub, objs, obj_idx, bin_name)

