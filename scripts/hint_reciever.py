#!/usr/bin/env python

import rospy
import time
from exprob_ass2.msg import ErlOracle

hint_sub = None
hint = ErlOracle

def hint_callback(msg):

    global hint
    hint = msg
    return hint
      
def main():
    global hint, hint_sub
    rospy.init_node('hint_reciever', anonymous = True)
    hints = [[] for _ in range(6)]
    hint_sub = rospy.Subscriber('/oracle_hint', ErlOracle, hint_callback)
    rospy.wait_for_message('/oracle_hint', ErlOracle)
    ID = hint.ID
    key = hint.key
    value = hint.value
    print(ID)
    print(key)
    print(value)
    if key == '' or value == '' or key == 'when' or value == -1:
        print('Malformed hint, the robot will discard this')
    else:
        print('Hint collected: {}, {}, {}'.format(ID, key, value))
        hints[ID].append(value)
        print(hints)
    rospy.spin()

if __name__ == '__main__':
    main()
