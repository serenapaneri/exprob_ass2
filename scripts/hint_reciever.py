#!/usr/bin/env python

import rospy
import time
from exprob_ass2.msg import ErlOracle

hint_sub = None
hints = [[] for _ in range(6)]

def hint_callback(msg):

    global hints
    print(msg.ID)
    print(msg.key)
    print(msg.value)
    if msg.key == '' or msg.value == '' or msg.key == 'when' or msg.value == -1:
        print('Malformed hint, the robot will discard this')
    else:
        print('Hint collected: {}, {}, {}'.format(msg.ID, msg.key, msg.value))
        hints[msg.ID].append(msg.value)
        # print(hints)
        return hints
      
def main():
    global hints, hint_sub
    rospy.init_node('hint_reciever', anonymous = True)
    hint_sub = rospy.Subscriber('/oracle_hint', ErlOracle, hint_callback)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.wait_for_message('/oracle_hint', ErlOracle)
        print(hints)
        rate.sleep()
    # rospy.spin()

if __name__ == '__main__':
    main()
