#!/usr/bin/env python

import rospy
import time
from exprob_ass2.msg import ErlOracle

hint_sub = None
percieved = False
hints = []

def hint_callback(msg):

    global hints, percieved
    hints.append(msg.ID)
    hints.append(msg.key)
    hints.append(msg.value)
    percieved = True
    print(hints)
    print(percieved)
    return hints
    
def main():
    global hints, hint_sub, percieved
    rospy.init_node('hint_reciever', anonymous = True)
    # while percieved == True:
    hint_sub = rospy.Subscriber('/oracle_hint', ErlOracle, hint_callback)
    rospy.wait_for_message('/oracle_hint', ErlOracle)
    print(hints)
    print(percieved)
    rospy.spin()

if __name__ == '__main__':
    main()
