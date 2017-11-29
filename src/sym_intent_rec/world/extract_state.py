#!/usr/bin/env python

from sym_intent_rec.world.utils import *
from sym_intent_rec.msg import WorldState
from hlpr_perception_msgs.msg import SegClusters
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from tf import TransformListener
import sensor_msgs.point_cloud2 as pc2 
import numpy as np
import rospy
import sys

d_thresh = 0.06 # distance threshold
t_thresh = 0.5 # time threshold

def check_present(i, t, t_seen, p_check, p_curr):
    near = np.linalg.norm(np.array(p_check) - p_curr, axis=1) < d_thresh
    seen = reduce(lambda x, y: x or y, near)
    
    if not seen:
        present = t - t_seen[i] > t_thresh
    else:
        present = True
        t_seen[i] = t

    return present

def publish_state(msg, args):
    pub, tf, p_check, t_seen = args
    t = msg.header.stamp.to_sec()
    state = WorldState()

    p_curr = []
    for pc in msg.clusters:
        p = get_avg_position(pc, tf)
        p_curr.append(p)

    for i, p in enumerate(p_check['right']):
        state.right[i] = check_present(i, t, t_seen['right'], p, p_curr)
   
    for i, p in enumerate(p_check['left']):
        state.left[i] = check_present(i, t, t_seen['left'], p, p_curr)

    pub.publish(state)

if __name__ == "__main__":
    p_check = yaml.load(open(sys.argv[1]))
    t_seen = {'left': len(p_check['left'])*[-float('inf')],
              'right': len(p_check['right'])*[-float('inf')]}

    rospy.init_node('extract_state')

    tf = TransformListener(True, rospy.Duration(50))
    pub = rospy.Publisher('/world_state', WorldState, queue_size=10)
    rospy.Subscriber('/beliefs/clusters', SegClusters, publish_state, (pub, tf, p_check, t_seen))

    rospy.spin()

