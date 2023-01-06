#!/usr/bin/env python

## @package exprob_ass2
#
# \file check_hypothesis.py
# \brief script that checks the consistency of an hypothesis.
#
# \author Serena Paneri
# \version 1.0
# \date 5/1/2023
# \details
#
# Subscribes to: <BR>
#     None
#
# Publishes to: <BR>
#     None
#
# Serivces: <BR>
#    comm 
#    winhypo
#    consistent
#
# Client Services: <BR>
#     armor_interface_srv
#     hypo_ID
#
# Action Services: <BR>
#     None
#
# Description: <BR>
#     In this node are handled the armor client, the hypo_found_client, the comm_service, the win_hypo_srv and cons_service.
#     In this node the consistency of the current hypothesis is checked and this is done first retrieving the ID of the current
#     complete hypothesis from the hypo_found service and then retrieving the list of object properties of a certain individual
#     (that in this case is the current hypothesis), and evaluating the number of elements that composes that specific hypothesis
#     to see if the hypothesis is consistent or not.
#     If the hypothesis is complete and consistent the node check_consistency is advertise and those elements are sent to the oracle
#     node, if insead the hypothesis is incosistent, the check_consistency node is advertise in order to perform a replanning.

import rospy
import time
from armor_msgs.srv import *
from armor_msgs.msg import *
from exprob_ass2.srv import Command, CommandResponse
from exprob_ass2.srv import HypoFound, HypoFoundRequest
from exprob_ass2.srv import WinHypo, WinHypoResponse
from exprob_ass2.srv import Consistent, ConsistentResponse

# armor client
armor_interface = None
# command service
comm_service = None
# hypo_found client
hypo_found_client = None
# consistent service
cons_service = None
# win_hypo service
win_hypo_srv = None

start = False
consistent_ = False
IDs = 0
who = ''
what = ''
where = ''

##
# \brief Callback of the comm_service.
# \param: req, CommandRequest
# \return: start
#
# This function recieves the command from the client in the check_consistency.cpp node. When the client commands to
# start then the start variable is set to True and when it commands to stop the start variable is set to False. 
def com(req):

    global start
    if (req.command == 'start'):
        start = True
    elif (req.command == 'stop'):
        start = False
    return start


##
# \brief Callback of the cons_service service.
# \param: req, ConsistentRequest
# \return: res, ConsistentResponses
#
# This function is the callback function of the cons_service service. 
# It simply send a boolean value everytime the consistency of an hypothesis is evaluated.
# If a new consisten hypothesis is found the value is set to true, instead when the hypothesis
# is inconsistent is set to false, communicating to the check_consistency node that a replanning
# is needed. 
def cons_handle(req):

    global consistent_
    res = ConsistentResponse()
    res.consistent = consistent_
    return res


##
# \brief Callback of the win_hypo_srv service.
# \param: req, WinHypotRequest
# \return: res, WinHypotResponse
#
# This function is the callback function of the win_hypo_srv service.
# When a new complete and consistent hypothesis is found then it sends to the oracle node the information
# concerning the possible person, the weapon and the place of the murder. 
def winhypo_handle(req):

    global who, what, where
    res = WinHypoResponse()
    res.who = who
    res.what = what
    res.where = where
    return res   


##
# \brief Function that retrieves the object properties of an individual from the cluedo_ontology.
# \param: prop_name, ind_name
# \return: res
#
# This function retrives the object property of an individual from the cluedo_ontology. The ind_name stands for
# the name of the individuals whose objects propetries (prop_name) you want collect. 
def retrieve_hypo(prop_name, ind_name):

    req = ArmorDirectiveReq()
    req.client_name = 'check_hypothesis'
    req.reference_name = 'cluedontology'
    req.command = 'QUERY'
    req.primary_command_spec = 'OBJECTPROP'
    req.secondary_command_spec = 'IND'
    req.args = [prop_name, ind_name]
    msg = armor_interface(req)
    res = msg.armor_response
    return res


##
# \brief Function that "cleans" the url recieved by the cluedo ontology.
# \param: obj_query
# \return: res
#
# This function takes the string that is written as an url and remove that parl only leaving a part of
# the string that is the string of interest.    
def find_string(obj_query):

    url_ind = '<http://www.emarolab.it/cluedo-ontology#'
    obj_query = obj_query.replace(url_ind, '')
    obj_query = obj_query.replace('>', '')
    return obj_query


##
# \brief Main function of the check_hypothesis node where the node is initialized and the armor client, the hypo_found_client, the 
#        comm_service, the win_hypo_srv and cons_service are implemented.
# \param: None
# \return: None
#
# This is the main function of the check_hypothesis node is initialized and the armor client, the hypo_found_client, the comm_service, 
# the win_hypo_srv and cons_service are implemented. Here, when the comm client in the check_consistency node gives the command to start,
# the node retrieves the ID of the current hypothesis and so evaluate the consistency of the current hypothesis. If the hypothesis is 
# complete and consistent than the current plan can goes on, insted if the hypothesis found is inconsistent, the service consistent 
# advertise the client in the check_consistent node that a replanning is needed.
def main():

    global armor_interface, comm_service, cons_service, hypo_found_client, win_hypo_srv, start, IDs, who, what, where, consistent_
    rospy.init_node('check_consistency')
    
    # armor client
    rospy.wait_for_service('armor_interface_srv')
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    
    # service to retrieve the ID of the current complete hypothesis just found
    rospy.wait_for_service('hypo_ID')
    hypo_found_client = rospy.ServiceProxy('hypo_ID', HypoFound)
    
    # command service
    comm_service = rospy.Service('comm', Command, com)
    # win_hypo service
    win_hypo_srv = rospy.Service('winhypo', WinHypo, winhypo_handle)
    # consistent service
    cons_service = rospy.Service('consistent', Consistent, cons_handle)  
    
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        # if the command chosen is 'start'
        if start == True:
            # the hypo_found client recieves the ID of the last hint found (that is the hint of the new complete hypothesis)
            res = hypo_found_client()
            IDs = res.IDs
            
            # we retrieve the who, what and where from the hypothesis of the current ID
            who_url = retrieve_hypo('who', 'Hypothesis' + str(IDs))
            what_url = retrieve_hypo('what', 'Hypothesis' + str(IDs))
            where_url = retrieve_hypo('where', 'Hypothesis' + str(IDs))
            
            # taking the list of the queried_objects
            who_queried = who_url.queried_objects
            what_queried = what_url.queried_objects
            where_queried = where_url.queried_objects
    
            # if the element of each list are more than one the hypothesis is inconsistent
            if len(who_queried) > 1 or len(what_queried) > 1 or len(where_queried) > 1:
                print('The Hypothesis{} is inconsistent'.format(IDs)) 
                # set the variable consistent to false
                consistent_ = False
            # if instead there is only ONE elements per cathegory 
            else:
                print('The Hypothesis{} is complete and consistent'.format(IDs))
                # taking only the string of interest from the url        
                who = find_string(who_queried[0])
                what = find_string(what_queried[0])
                where = find_string(where_queried[0])
                # set consistent to true
                consistent_ = True 

            rospy.sleep(2)
            rate.sleep()
    
        # if the command chosen is 'stop'
        elif start == False:
            rate.sleep()
    
    
if __name__ == '__main__':
    main()
