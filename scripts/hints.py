#!/usr/bin/env python

## @package exprob_ass2
#
# \file hints.py
# \brief script that recieves all the hints and load them in to the ontology, moreover the completeness is evaluated.
#
# \author Serena Paneri
# \version 1.0
# \date 5/1/2023
# \details
#
# Subscribes to: <BR>
#     /oracle_hint
#
# Publishes to: <BR>
#     None
#
# Serivces: <BR>
#     hypo_ID
#     complete
#
# Client Services: <BR>
#     armor_interface_srv
#
# Action Services: <BR>
#     None
#
# Description: <BR>
#     In this node are handled the client of the ARMOR service, the hint subscriber, the complete service and the hypo_found service.
#     It menages the flow of hints recieved, evaluating if they are malformed hints, and in this case they will be discarded and not
#     uploaded in the ontology, and check the completeness of the current hypothesis that have been found till that moment.
#     With the complete service it communicates with the complete client in the check_complete node to see if a replanning is needed
#     Indeed at every waypoint the completeness is checked and if there are not complete hypotheses yet, or new complete hypotheses 
#     the rosplan perform a replanning. Instead if a new complete hypothesis is found it keeps going with the current plan. 
#     There are also the reasoner and the apply function that directly communicate with the ontology in order to apply the changes and
#     update the knowledge everytime that a new hint is uploaded.


import rospy
import time
from armor_msgs.srv import * 
from armor_msgs.msg import * 
from exprob_ass2.msg import ErlOracle
from exprob_ass2.srv import HypoFound, HypoFoundResponse
from exprob_ass2.srv import Complete, CompleteResponse

# hints subscriber
hint_sub = None
# armor client
armor_interface = None
# hypo found service
hypo_found_srv = None
# complete service
complete_srv = None

ID = 0
key = ''
value = ''
IDs = 0
collected = False 
complete_ = False
complete_hypotheses = [] 


##
# \brief Callback function of the hint_sub subscriber.
# \param: msg
# \return: None
#
# This function is the callback function of the subscriber hint_sub. It collects and store all the fields
# of the message recieved and also set the global variable "collected" to true when a new message is recieved.
def hint_callback(msg):

    global ID, key, value, collected 
    ID = msg.ID
    key = msg.key
    value = msg.value
    collected = True 


##
# \brief Callback function of the hypo_found_srv service.
# \param: req, HypoFoundRequest
# \return: res, HypoFoundResponse
#
# This function is the callback function of the service hypo_found_srv.
# When called it simply send the ID of the hint just recieved.
def hypo_found_handle(req):

    global IDs
    res = HypoFoundResponse()
    res.IDs = IDs
    return res
  

##
# \brief Callback function of the complete_srv service.
# \param: req, CompleteRequest
# \return: res, CompleteResponse
#
# This function is the callback function of the complete_srv service.
# It simply send a boolean value everytime a new hint is recieved. It will be set equal to 
# true onlty when a new complete hypothesis is found, advertising the client node check_complete.cpp
# to see if a replanning is needed or not. 
def complete_handle(req):

    global complete_
    res = CompleteResponse()
    res.complete = complete_
    return res
    

##
# \brief Function to upload the hints recieved in the cluedo_ontology.
# \param: ID_, key_, value_
# \return: None
#
# This function is used to upload every hints recieved from the environment in the cluedo_ontology.    
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
    
    apply_()
    reasoner()
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
    req.client_name = 'hints'
    req.reference_name = 'cluedontology'
    req.command = 'REASON'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    msg = armor_interface(req)
    res = msg.armor_response


##
# \brief It is the apply command of armor.
# \param: None
# \return: None
#
# This function apply the changes done to the ontology.  
def apply_():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'APPLY'
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
    req.client_name = 'hints'
    req.reference_name = 'cluedontology'
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['COMPLETED']
    msg = armor_interface(req)
    res = msg.armor_response
    return res


##
# \brief The main function initializes the node and handles the client of the ARMOR service, the hint subscriber, the complete service and the hypo_found service.
# \param: None
# \return: None
#
# This is the main function of the node where the node is initialized and handles the client of the ARMOR service,
# the hint subscriber, the complete service and the hypo_found service.
# Moreover it menages the flow of hints recieved, evaluating if they are malformed hints, and in this case they will be discarded
# and not uploaded in the ontology, and check the completeness of the current hypothesis that have been found till that moment.
# With the complete service it communicates with the complete client in the check_complete node to see if a replanning
# is needed. Indeed at every waypoint the completeness is checked and if there are not complete hypotheses yet, or new
# complete hypotheses the rosplan perform a replanning. Instead if a new complete hypothesis is found it keeps going
# with the actual plan.
def main():
    global ID, key, value, IDs, hint_sub, armor_interface, complete_hypotheses, hypo_found_srv, complete_srv, complete_, collected
    rospy.init_node('hints', anonymous = True)
    
    # armor client
    rospy.wait_for_service('armor_interface_srv')
    print('Waiting for the armor service')
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    # hints subscriber
    hint_sub = rospy.Subscriber('/oracle_hint', ErlOracle, hint_callback)
    # complete service
    complete_srv = rospy.Service('complete', Complete, complete_handle)
    
    # hypo found service
    hypo_found_srv = rospy.Service('hypo_ID', HypoFound, hypo_found_handle)
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        # if a new hint has been collected
        if collected == True:
            # if the hint is a malformed hint the robot will recognise and discard it
            if key == '' or value == '' or key == 'when' or value == '-1':
                print('Malformed hint, the robot will discard this')
            # instead if the hint is not a malformed one
            else:
                print('Hint collected: {}, {}, {}'.format(ID, key, value))   
                complete_ = False 
                IDs = 0
                # uploading the hint in the ontology
                upload_hint(ID, key, value)
                # check if completed hypotheses have been loaded in the ontology 
                iscomplete = complete()
                print('Checking now the completeness')
                print(iscomplete.queried_objects)
                # if there are not elements in the COMPLETED class
                if len(iscomplete.queried_objects) == 0:
                    print('There are not complete hypotheses yet')
                    # complete variable set to false
                    complete_ = False
                # if instead there are already elements in the COMPLETED class    
                elif len(iscomplete.queried_objects) != 0:
                    complete_hypotheses.append(iscomplete.queried_objects)
                    # if there is only one element in the COMPLETED class
                    if len(complete_hypotheses) < 2 :
                        print('The fist complete hypothesis has been found')
                        IDs = ID
                        complete_ = True
                    # if there are more element in the COMPLETED class
                    else:
                        # if there are not new elements in the COMPLETED class
                        if complete_hypotheses[-1] == complete_hypotheses[-2]:
                            print('No new complete hypotheses')
                            complete_ = False
                        # insted when a new element is found in the COMPLETED class
                        else:
                            print('A new complete hypotheses has been added')
                            IDs = ID
                            complete_ = True 

            rospy.sleep(5)
            collected = False
            rate.sleep()
        
        # if there are not new hint collected    
        elif collected == False:
            # the complete variable is set to false
            complete_ = False
            rate.sleep()

if __name__ == '__main__':
    main()
