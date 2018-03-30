from copy import deepcopy
import numpy as np
import pickle
import rospy
import sys

manip_var_action_idx = 5

def norm_time(pred, leg):
    leg_p = deepcopy(pred)

    for objs in leg_p:
        for obj_idx in range(len(leg_p[objs])):
            for bin_name in leg_p[objs][obj_idx]:
                leg_p[objs][obj_idx][bin_name][manip_var_action_idx] = deepcopy(leg[objs][obj_idx][bin_name][manip_var_action_idx])

    return pred, leg_p

if __name__ == "__main__":
    pred_file = sys.argv[1]
    leg_file = sys.argv[2]

    pred = pickle.load(open(pred_file))
    leg = pickle.load(open(leg_file))

    pred, leg = norm_time(pred, leg)

    pickle.dump(pred, open(pred_file[:-4]+'_nt.pkl', 'w'))
    pickle.dump(leg, open(leg_file[:-4]+'_nt.pkl', 'w'))

