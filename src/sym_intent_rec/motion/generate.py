from cartesian_trajectory_planner.trajopt_planner import *
from cartesian_trajectory_planner.openrave_utils import *
from sym_intent_rec.motion.plan import *
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from vector_msgs.msg import GripperCmd
from itertools import combinations
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

def goto_legible(start, objs, g):
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

def goto_predictable(start, goal):
    return goto_legible(start, [goal], 0)

    #cs = start['tf']
    #q = quaternion_from_euler(cs[3], cs[4], cs[5])

    #goal_pose = Pose()
    #goal_pose.position.x = goal[0]
    #goal_pose.position.y = goal[1]
    #goal_pose.position.z = goal[2]
    #goal_pose.orientation.x = q[0]
    #goal_pose.orientation.y = q[1]
    #goal_pose.orientation.z = q[2]
    #goal_pose.orientation.w = q[3]
 
    #cart_planner.setDOFs(start['joints'])
    #joint_traj_arr = cart_planner.plan('right_arm', [goal_pose], link='right_ee_link')
    #joint_traj = array_to_joint_traj(joint_traj_arr)

    #return {'type': 'arm', 'msg': joint_traj}, {'joints': joint_traj_arr[-1], 'tf': goal+cs}

def pick_up(start, dy=0.3):
    cs = start['tf']
    return goto_predictable(start, (np.array(cs[:3]) + np.array([0,dy,0])).tolist())

def back_up(start, dz=0.3):
    cs = start['tf']
    return goto_predictable(start, (np.array(cs[:3]) - np.array([0,0,dz])).tolist())

def goto(start, goal, objs=None):
    if objs is None:
        return goto_predictable(start, goal)
    else:
        return goto_legible(start, [goal]+objs, 0)

def goto_joints(start, goal):
    joint_traj = array_to_joint_traj([start['joints'], goal])
    return {'type': 'arm', 'msg': joint_traj} 

def generate_seq(start, objs, obj_bin, obj_pair, m_type='leg'):
    h_obj, r_obj = obj_pair
    o_objs = list(set(objs.keys()) - set(obj_pair))

    if m_type == 'leg':
        a0, es = goto(start, objs[r_obj], [objs[o] for o in o_objs])
    else:
        a0, es = goto(start, objs[r_obj])
    a1 = close_gripper()
    a2, es = pick_up(es)
    a3, es = goto(es, obj_bin)
    a4 = open_gripper()
    a5, es = back_up(es)
    a6 = goto_joints(es, start['joints'])

    return [a0, a1, a2, a3, a4, a5, a6]

if __name__ == "__main__":
    in_file = sys.argv[1]
    out_file = sys.argv[2]
 
    world_file = yaml.load(open(in_file))
    start = world_file['start']
    objs = world_file['objects']
    obj_bin = world_file['bin']
    manip_seqs = {'objects': objs.keys()}

    cart_planner = TrajOptCartesianPlanner(urdf, srdf, viz=False)

    for obj_pair in combinations(objs.keys(), 2):
        seq = generate_seq(start, objs, obj_bin, obj_pair, 'pred')   
        manip_seqs[obj_pair] = seq
        manip_seqs[(obj_pair[1], obj_pair[0])] = seq

    pickle.dump(manip_seqs, open(out_file, 'w'))

