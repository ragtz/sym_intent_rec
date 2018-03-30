from vector_msgs.msg import GripperCmd
from sym_intent_rec.motion.playback import *
from copy import deepcopy
import numpy as np
import rospy
import pickle
import time
import sys

durations = np.array([5, 3, 3.5, 5, 5.5, 5.5, 9.5, 5, 5.5, 6.5, 4, 3.5, 5, 5])
#durations = 10*np.array([5, 3, 3.5, 5, 5.5, 5.5, 9.5, 5, 5.5, 6.5, 4, 3.5, 5, 5])

if __name__ == "__main__":
    print sys.argv[1]
    manip_seq = pickle.load(open(sys.argv[1]))
    ts_file = sys.argv[2]

    manip_seq_ts = deepcopy(manip_seq)

    rospy.init_node('manip_playback')
    gripper_pub = rospy.Publisher('/vector/right_gripper/cmd', GripperCmd, queue_size=10)

    for objs in manip_seq_ts:
        for obj_idx in range(len(manip_seq_ts[objs])):
            for bin_name in manip_seq_ts[objs][obj_idx]:
                times = play_seq(manip_seq_ts, gripper_pub, objs, obj_idx, bin_name)
                time_scale_factors = durations/times

                for i, action in enumerate(manip_seq_ts[objs][obj_idx][bin_name]):
                    action['time_scale'] = time_scale_factors[i]

    pickle.dump(manip_seq_ts, open(ts_file, 'w'))

