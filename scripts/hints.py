#!/usr/bin/env python

import rospy
import time
from armor_msgs.srv import * 
from armor_msgs.msg import * 
from exprob_ass2.msg import ErlOracle
from exprob_ass2.srv import HypoFound, HypoFoundResponse
from exprob_ass2.srv import Complete, CompleteResponse

ID = 0
key = ''
value = ''
IDs = 0
hint_sub = None
# armor client
armor_interface = None
# hypo found service
hypo_found_srv = None
# complete service
complete_srv = None
complete = False
complete_hypotheses = [] 


def hint_callback(msg):

    global ID, key, value
    ID = msg.ID
    key = msg.key
    value = msg.value
    return ID, key, value 


def hypo_found_handle(req):
    global IDs
    res.IDs = IDs
    return res
    
def complete_handle(req):
    global complete
    res.complete = complete
    return res


##
# \brief This function search an element in a list.
# \param: list_, element
# \return: True, False
#
# This functions checks if a specific element is present (True) or not (False) in a list.     
def search(list_, element):
    """
      This function check if an element is present or not into a list
    """
    for i in range(len(list_)):
        if list_[i] == element:
            return True
    return False

   
def upload_hint(ID_, key_, value_):

    req = ArmorDirectiveReq()
    req.client_name = 'hints'
    req.reference_name = 'cluedontology'

    if key_ == 'who':  
        req.command = 'ADD'
        req.primary_command_spec = 'OBJECTPROP'
        req.secondary_command_spec = 'IND'
        req.args = ['who','Hypothesis' + str(ID_), value_]
        msg = armor_interface(req)
        res = msg.armor_response 
    
    elif key_ == 'what':
        req.command = 'ADD'
        req.primary_command_spec = 'OBJECTPROP'
        req.secondary_command_spec = 'IND'
        req.args = ['what','Hypothesis' + str(ID_), value_]
        msg = armor_interface(req)
        res = armor_interface(req)
    
    elif key_ == 'where':
        req.command = 'ADD'
        req.primary_command_spec = 'OBJECTPROP'
        req.secondary_command_spec = 'IND'
        req.args = ['where','Hypothesis' + str(ID_), value_]
        msg = armor_interface(req)
        res = msg.armor_response 
    
    print("The hint has been uploaded")

    
##
# \brief The reasoner of the ontology.
# \param: None
# \return: None
#
# This function implements the reasoner of the ontology that needs to be started in order to update
# the knowledge of the ontology.
def reasoner():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'REASON'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    msg = armor_interface(req)
    res = msg.armor_response
    
    
##
# \brief It is the query command to retrieve an individual from a class.
# \param: None
# \return: res
#
# This functions returns, if there are any, the individuals of the class COMPLETED of the ontology.
def complete():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['COMPLETED']
    msg = armor_interface(req)
    res = msg.armor_response
    return res

      
def main():
    global ID, key, value, IDs, hint_sub, armor_interface, complete_hypotheses, hypo_found_srv, complete_srv, complete
    rospy.init_node('hints', anonymous = True)
    
    rospy.wait_for_service('armor_interface_srv')
    print('Waiting for the armor service')
    # armor client
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    # hints subscriber
    hint_sub = rospy.Subscriber('/oracle_hint', ErlOracle, hint_callback)
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        rospy.wait_for_message('/oracle_hint', ErlOracle)
        # complete service
        complete = False
        complete_srv = rospy.Service('complete', Complete, complete_handle)
        if key == '' or value == '' or key == 'when' or value == '-1':
            print('Malformed hint, the robot will discard this')
        else:
            print('Hint collected: {}, {}, {}'.format(ID, key, value))
            # uploading the hint in the ontology 
            upload_hint(ID, key, value)
            # reason
            reasoner()
            # check if completed hypotheses have been loaded in the ontology 
            iscomplete = complete()
            print('Checking now the completeness')
            if len(iscomplete.queried_objects) == 0:
                print('There are not complete hypotheses yet')
                complete = False
            elif len(iscomplete.queried_objects) != 0:
                complete_hypotheses.append(iscomplete.queried_objects)
                print(complete_hypotheses)
                print(complete_hypotheses[-1])
                if len(complete_hypotheses) < 2 :
                else:
                    if complete_hypotheses[-1] == complete_hypotheses[-2]:
                        print('No new complete hypotheses')
                        complete = False
                    else:
                        print('A new complete hypotheses has been added')
                        IDs = ID
                        # hypo found service
                        hypo_found_srv = rospy.Service('hypo_ID', HypoFound, hypo_found_handle)
                        print(IDs)
                        complete = True
                
        IDs = 0    
        rate.sleep()

if __name__ == '__main__':
    main()
