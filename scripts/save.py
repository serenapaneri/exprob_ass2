#!/usr/bin/env python

import rospy
import time
from armor_msgs.srv import * 
from armor_msgs.msg import * 

def main():
    global armor_interface
    rospy.init_node('save')
    
    # armor client
    rospy.wait_for_service('armor_interface_srv')
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    
    req = ArmorDirectiveReq()
    req.client_name = 'plan_execution'
    req.reference_name = 'cluedontology'
    req.command = 'SAVE'
    req.primary_command_spec = 'INFERENCE'
    req.secondary_command_spec = ''
    req.args = ['/root/ros_ws/src/exprob_ass2/gggggggggggg.owl']
    msg = armor_interface(req)
    res = msg.armor_response
    print('The new ontology has been saved under the name final_ontology_inferred.owl')
    
    rospy.spin()
    
if __name__ == '__main__':
    main()
