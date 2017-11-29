#!/usr/bin/env python

from sym_intent_rec.msg import WorldState
from std_msgs.msg import String
import rospy
import time

def detect_change(s1, s2):
    return s1.left == s2.left, s1.right == s2.right

def publish_intent(msg, args):
    state, pub = args
    left_change, right_change = detect_change(state, msg)

    if left_change:
        intent = 'left'
    elif right_change:
        intent = 'right'
    else:
        intent = None

    if not intent is None:
        pub.publish(intent)

    state = msg

if __name__ == "__main__":
    rospy.init_node('intent_recognition')
    time.sleep(5)

    state_topic = '/world_state'
    intent_topic = '/intent'

    state = rospy.wait_form_message(state_topic, WorldState, timeout=5)
    pub = rospy.Publisher(intent_topic, String, queue_size=10)
    rospy.Subscriber(state_topic, publish_intent, (state, pub))

