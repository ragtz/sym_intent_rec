#!/usr/bin/env python

from geometry_msgs.msg import Point, PointStamped
from tf import TransformListener
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import rospy

nan = float('nan')

def get_avg_position(pc, tf, frame='/base_link'):
    position = np.zeros(3)
    n = 0

    for point in pc2.read_points(pc):
        position += [point[0], point[1], point[2]]
        n += 1

    position /= n

    ps = PointStamped()
    ps.header.frame_id = pc.header.frame_id
    ps.point.x = position[0]
    ps.point.y = position[1]
    ps.point.z = position[2]
    ps_tf = tf.transformPoint(frame, ps)

    return [ps_tf.point.x, ps_tf.point.y, ps_tf.point.z]

