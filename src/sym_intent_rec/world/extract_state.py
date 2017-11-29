#!/usr/bin/env python
#Scenario:(try to make it as simple as possible)
#1. The number of object is set to NUM=2.
#2. Always have $NUM objects on table, which means no object will disappear.
#3. Order the objects: 
#   0) Order from left-hand side to right-hand side.
#   1) A is always on left-hand side, B is always on right-hand side.
#      If there's a C, it should be on B's right-hand side, which means B is in the middle.
#   2) After moving A or B, A is still on B's left-hand side.
#4. Assume kinetic is perfect and always can correctly track all objects, which means if there're only two objects
#   on the table, kinetic shows only two objects on the monitor, not one object or more than two objects.
from hlpr_perception_msgs.msg import SegClusters
from geometry_msgs.msg import Point, PointStamped
from tf import TransformListener
import sensor_msgs.point_cloud2 as pc2 
import numpy as np
import rospy
from std_msgs.msg import String

tf = None
# from lest to right: A, B, C, ... => index 0 for A, index 1 for B, ...
# x_position for A is x[0], y_position for B is y[1]
x = dict()
y = dict()
z = dict()
pos = np.array( [[-0.35682135, 0.2453447, 1.0417007 ],
                 [-0.11894836, 0.26739127, 1.05216055],
                 [ 0.17182488, 0.3151547, 1.01269372]])

# used to map objects
name = {'0': 'obj1', '1': 'obj2', '2': 'obj3'}

# used to calculate object's location
d = 0.06#need to be modified

# used to store the counter for each object.
# the counter indicates how many times this object is out of distance
otime = {'0': 0, '1': 0, '2': 0}
isThere = {'0': 0, '1': 0, '2': 0}
MAX = 3

pub = None

'''
def talker(clusters, frame='/base_link'):
    for pc in clusters:
        position = np.array([0.0, 0.0, 0.0])
        n = 0 

        for point in pc2.read_points(pc, skip_nans=True):
            position += [point[0], point[1], point[2]]
            n += 1

        position = position/n

    #pub = rospy.Publisher('target_obj', String, queue_size=10)
    #rate = rospy.Rate(10) # 10hz
    #hello_str = str(position[0]) + "," + str(position[1]) + "," + str(position[2])
    #rospy.loginfo(hello_str)
    #pub.publish(hello_str)
    #rate.sleep()
'''

def check_objects(idx, clusters, frame='/base_link'):
    isThere[str(idx)] = 0
    for pc in clusters:
        position = np.array([0.0, 0.0, 0.0])
        n = 0 

        for point in pc2.read_points(pc, skip_nans=True):
            position += [point[0], point[1], point[2]]
            n += 1

        position = position/n
        #print position
        tx, ty, tz = position
        if abs(tx - pos[idx][0]) < d and abs(ty - pos[idx][1]) < d and abs(tz - pos[idx][2]) < d:
            isThere[str(idx)] = 1
            if otime[str(idx)] > MAX:
                print "Object stays in position " + str(idx) + " !"
            else:
                otime[str(idx)] += 1
                print "Object appeared a while in position " + str(idx) + "."

    if isThere[str(idx)] == 0:
        otime[str(idx)] = 0
        print "No object in position " + str(idx) + "."
        #talker(clusters, frame=None)
        pub.publish(name[str(idx)])


def print_positions(msg):
    #index = 0
    #for pc in msg.clusters:
    #    print get_avg_position(index, pc, frame=None)
    #    index += 1
    for i in range(0, 3):
        check_objects(i, msg.clusters, frame=None)
    print '-------------------'

if __name__ == "__main__":
    rospy.init_node('pc_position')

    tf = TransformListener(True, rospy.Duration(50))
    pub = rospy.Publisher('target_obj', String, queue_size=10)
    rospy.Subscriber('/beliefs/clusters', SegClusters, print_positions)

    rospy.spin()
