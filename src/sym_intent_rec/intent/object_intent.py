#!/usr/bin/env python

from sym_intent_rec.msg import WorldState
from std_msgs.msg import String
import rospy
import time

n = 5

def detect_change(s1, s2):
    return s1.left != s2.left, s1.right != s2.right

def publish_intent(msg, args):
    state, pub = args
    left_change, right_change = detect_change(state, msg)

    if left_change:
        intent = 'left'
    elif right_change:
        intent = 'right'
    else:
        intent = 'none'

    pub.publish(intent)
    if intent != 'none':
        for i in range(n):
            pub.publish(intent)
            #time.sleep(0.2)
            
    state.left = msg.left
    state.right = msg.right

if __name__ == "__main__":
    rospy.init_node('intent_recognition')
    time.sleep(5)

    state_topic = '/world_state'
    intent_topic = '/intent'

    state = rospy.wait_for_message(state_topic, WorldState, timeout=5)
    pub = rospy.Publisher(intent_topic, String, queue_size=10)
    rospy.Subscriber(state_topic, WorldState, publish_intent, (state, pub))

    rospy.spin()

