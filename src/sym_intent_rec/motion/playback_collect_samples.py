from vector_msgs.msg import GripperCmd
from sym_intent_rec.motion.playback import *
from copy import deepcopy
import numpy as np
import rospy
import pickle
import time
import sys

num_samples = 10

if __name__ == "__main__":
    print sys.argv[1]
    manip_seq = pickle.load(open(sys.argv[1]))
    samples_file = sys.argv[2]

    num_left_objs = len(manip_seq['left_objs'])
    num_right_objs = len(manip_seq['right_objs'])
    time_data = {'left_objs': [{'left_bin': [], 'right_bin': []} for i in range(num_left_objs)],
                 'right_objs': [{'left_bin': [], 'right_bin': []} for i in range(num_right_objs)]} 

    rospy.init_node('manip_playback')
    gripper_pub = rospy.Publisher('/vector/right_gripper/cmd', GripperCmd, queue_size=10)

    for i in range(num_samples):
        print '------------------------------'
        print 'Sample:', i
        print '------------------------------'
        for objs in manip_seq:
            for obj_idx in range(len(manip_seq[objs])):
                for bin_name in manip_seq[objs][obj_idx]:
                    times = play_seq(manip_seq, gripper_pub, objs, obj_idx, bin_name)
                    time_data[objs][obj_idx][bin_name].append(times)

    for objs in time_data:
        for obj_idx in range(len(time_data[objs])):
            for bin_name in time_data[objs][obj_idx]:
                time_data[objs][obj_idx][bin_name] = np.array(time_data[objs][obj_idx][bin_name])

    pickle.dump(time_data, open(samples_file, 'w'))

