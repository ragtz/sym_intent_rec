from cartesian_trajectory_planner.trajopt_planner import *
from cartesian_trajectory_planner.openrave_utils import *
from sym_intent_rec.motion.plan import *
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from vector_msgs.msg import GripperCmd
from itertools import combinations
from copy import deepcopy
import numpy as np
import pickle
import yaml
import sys

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

cart_planner = None

urdf = 'package://cartesian_trajectory_planner/urdf/jaco_7dof.urdf'
srdf = 'package://cartesian_trajectory_planner/urdf/jaco_7dof.srdf'

joint_names = ['j2s7s300_joint_1', 'j2s7s300_joint_2', 'j2s7s300_joint_3', 'j2s7s300_joint_4', 'j2s7s300_joint_5', 'j2s7s300_joint_6', 'j2s7s300_joint_7']

eta = 1e3
n_wp = 20
n_iter = 250

def array_to_joint_traj(joint_traj):
    joint_traj_msg = JointTrajectory()
    joint_traj_msg.joint_names = joint_names
    joint_traj_msg.points = []

    for wp in joint_traj:
        pt = JointTrajectoryPoint()
        pt.positions = wp
        joint_traj_msg.points.append(pt)

    return joint_traj_msg

def move_gripper(pos):
    cmd = GripperCmd()
    cmd.position = pos
    return cmd

def open_gripper():
    return {'type': 'gripper', 'msg': move_gripper(1.0)}

def close_gripper():
    return {'type': 'gripper', 'msg': move_gripper(0.0)}

def goto_legible(start, objs, g, n_wp=n_wp):
    objs = objs[:]
    for i, obj in enumerate(objs):
        objs[i] = obj + start['tf'][3:]

    traj = plan(np.array(start['tf']), np.array(objs), g, eta, n_wp, n_iter)

    #fig = plt.figure()
    #ax = fig.add_subplot(111, projection='3d')
    #ax.plot(traj[:,0], traj[:,1], traj[:,2])
    #plt.show()

    pose_traj = []
    for i, wp in enumerate(traj):
        q = quaternion_from_euler(wp[3], wp[4], wp[5])
        
        pose = Pose()
        pose.position.x = wp[0]
        pose.position.y = wp[1]
        pose.position.z = wp[2]
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        pose_traj.append(pose)

    cart_planner.setDOFs(start['joints'])
    joint_traj_arr = cart_planner.plan('right_arm', pose_traj, link='right_ee_link')
    joint_traj = array_to_joint_traj(joint_traj_arr)

    return {'type': 'arm', 'msg': joint_traj}, {'joints': list(joint_traj_arr[-1]), 'tf': list(traj[-1])}

def goto_predictable(start, goal, n_wp=n_wp):
    return goto_legible(start, [goal], 0, n_wp)

def pickup(start, dy=0.25):
    cs = start['tf']
    return goto_predictable(start, (np.array(cs[:3]) + np.array([0,dy,0])).tolist())

def putdown(start, dy=0.20):
    cs = start['tf']
    return goto_predictable(start, (np.array(cs[:3]) - np.array([0,dy,0])).tolist())

def backup(start, dz=0.3):
    cs = start['tf']
    return goto_predictable(start, (np.array(cs[:3]) - np.array([0,0,dz])).tolist())

def pickup_and_backup(start, dy=0.25, dz=0.3):
    cs = start['tf']
    return goto_predictable(start, (np.array(cs[:3]) + np.array([0,dy,-dz])).tolist())

def pregrasp(start, obj, dz=0.15):
    return goto_predictable(start, (np.array(obj) - np.array([0,0,dz])).tolist())

def preplace(start, obj, dy=0.20):
    return goto_predictable(start, (np.array(obj) + np.array([0,dy,0])).tolist())

def goto(start, goal, objs=None, n_wp=n_wp):
    if objs is None:
        return goto_predictable(start, goal, n_wp)
    else:
        return goto_legible(start, [goal]+objs, 0, n_wp)

def goto_joints(start, goal):
    joint_traj = array_to_joint_traj([start['joints'], goal])
    return {'type': 'arm', 'msg': joint_traj} 

def pour(start, goal_bin_idx, n_wp=2):
    full_pour = 2.967
    step = full_pour/n_wp
    wp = []

    for i in range(n_wp):
        g = deepcopy(start['joints'])
        if goal_bin_idx == 0:
            g[-1] -= step
        else:
            g[-1] += step
        wp.append(g)

    wp_rev = deepcopy(wp)[:-1]
    wp_rev.reverse()

    wp.extend(wp_rev)

    joint_traj = array_to_joint_traj([start['joints']] + wp + [start['joint']])
    return {'type': 'arm', 'msg': joint_traj}, start

def generate_seq(start, obj, bins, goal_bin_idx, m_type='leg'):
    a0, es = pregrasp(start, obj)
    a1, es = goto(es, obj, n_wp=5)
    a2 = close_gripper()
    #a3, es = pickup(es)
    #a4, es = backup(es)
    a3, es = pickup_and_backup(es, dz=0.4)
    a5 = goto_joints(es, start['joints'])

    if m_type == 'leg':
        a6, es = goto(start, bins[goal_bin_idx], bins)
    else:
        a6, es = goto(start, bins[goal_bin_idx])

    a7, es = pour(es, goal_bin_idx)
    a8, es = backup(es)
    a9 = goto_joints(es, start['joints'])
    a10, es = preplace(start, obj)
    a11, es = putdown(es)
    a12 = open_gripper()
    a13, es = pickup_and_backup(es, dz=0.4)
    a14 = goto_joints(es, start['joints'])

    return [a0, a1, a2, a3, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14]

if __name__ == "__main__":
    in_file = sys.argv[1]
    out_file = sys.argv[2]
    m_type = sys.argv[3] # pred or leg
 
    world_file = yaml.load(open(in_file))
    start = world_file['start']
    left_objs = world_file['left_objs']
    right_objs = world_file['right_objs']
    left_bin = world_file['left_bin']
    right_bin = world_file['right_bin']
    manip_seqs = {'left_objs': [{'left_bin': [], 'right_bin': []} for i in range(len(left_objs))],
                  'right_objs':[{'left_bin': [], 'right_bin': []} for i in range(len(right_objs))]}

    cart_planner = TrajOptCartesianPlanner(urdf, srdf, viz=False)

    for i, obj in enumerate(left_objs):
        print '-------------------- left_obj', str(i), 'left_bin --------------------'
        left_seq = generate_seq(start, obj, [left_bin, right_bin], 0, m_type)
        print '-------------------- left_obj', str(i), 'right_bin --------------------' 
        right_seq = generate_seq(start, obj, [left_bin, right_bin], 1, m_type)

        manip_seqs['left_objs'][i]['left_bin'] = left_seq
        manip_seqs['left_objs'][i]['right_bin'] = right_seq

    for i, obj in enumerate(right_objs):
        print '-------------------- right_obj', str(i), 'left_bin --------------------'
        left_seq = generate_seq(start, obj, [left_bin, right_bin], 0, m_type)
        print '-------------------- right_obj', str(i), 'right_bin --------------------'
        right_seq = generate_seq(start, obj, [left_bin, right_bin], 1, m_type)

        manip_seqs['right_objs'][i]['left_bin'] = left_seq
        manip_seqs['right_objs'][i]['right_bin'] = right_seq

    pickle.dump(manip_seqs, open(out_file, 'w'))

