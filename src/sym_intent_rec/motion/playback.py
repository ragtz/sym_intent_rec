from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory
from vector_msgs.msg import GripperCmd
from std_msgs.msg import String
import actionlib
import rospy
import pickle
import time
import sys

manip_seq = None
gripper_pub = None
done = False

def move_arm(traj):
    client = actionlib.SimpleActionClient('/jaco_trajectory_controller/trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj

    client.send_goal(goal)
    client.wait_for_result()

def move_gripper(cmd):
    gripper_pub.publish(cmd)

def play_seq(msg):
    global done
    if not done:
        objs = manip_seq['objects']
        h_obj = msg.data
        r_obj = list(set(objs) - set([h_obj]))[0]
        print r_obj

        seq = manip_seq[(h_obj, r_obj)]
        for action in seq:
            if action['type'] == 'arm':
                move_arm(action['msg'])
            else:
                move_gripper(action['msg'])
                time.sleep(3)

        done = True

if __name__ == "__main__":
    manip_seq = pickle.load(open(sys.argv[1]))

    rospy.init_node('manip_playback')
    gripper_pub = rospy.Publisher('/vector/left_gripper/cmd', GripperCmd, queue_size=10)
    rospy.Subscriber('target_obj', String, play_seq)
    rospy.spin()

