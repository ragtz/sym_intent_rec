#!/usr/bin/env python

from sym_intent_rec.world.utils import *
from hlpr_perception_msgs.msg import SegClusters
from tf import TransformListener
import rospy
import time
import yaml
import sys

# assumes base_link frame
def get_left_right(positions):
    left = filter(lambda p: p[1] > 0, positions)
    right = filter(lambda p: p[1] < 0, positions)
    return left, right 

def save(msg, args):
    tf = args[0]
    filename = args[1]
    positions = []

    for pc in msg.clusters:
        position = get_avg_position(pc, tf)
        positions.append(position)

    left, right = get_left_right(positions)

    data = {'frame': '/base_link',
            'left': left,
            'right': right}

    yaml.dump(data, open(filename, 'w'))

    print "n:", len(positions)
    print "left:", len(left)
    print "right:", len(right)
    
if __name__ == "__main__":
    filename = sys.argv[1]

    rospy.init_node('save_state')
    cluster_topic = '/beliefs/clusters'
    tf = TransformListener(True, rospy.Duration(50))

    time.sleep(5)
    msg = rospy.wait_for_message(cluster_topic, SegClusters, timeout=5)
    save(msg, (tf, filename))

