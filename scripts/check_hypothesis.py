#!/usr/bin/env python

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
# \return: None
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


def cons_handle(req):
    global consistent_
    res = ConsistentResponse()
    res.consistent = consistent_
    return res

    
def winhypo_handle(req):
    global who, what, where
    res = WinHypoResponse()
    res.who = who
    res.what = what
    res.where = where
    return res   
    
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

    
def find_string(ind_query):
    url_ind = '<http://www.emarolab.it/cluedo-ontology#'
    ind_query = ind_query.replace(url_ind, '')
    ind_query = ind_query.replace('>', '')
    return ind_query


##
# \brief Main function of the check_hypothesis node where the node is initialized and the armor client, the hypo_found_client and the comm_service are implemented.
# \param: None
# \return: None
#
# This is the main function of the check_hypothesis node 
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
            res = hypo_found_client()
            IDs = res.IDs
            
            who_url = retrieve_hypo('who', 'Hypothesis' + str(IDs))
            what_url = retrieve_hypo('what', 'Hypothesis' + str(IDs))
            where_url = retrieve_hypo('where', 'Hypothesis' + str(IDs))
            
            who_queried = who_url.queried_objects
            what_queried = what_url.queried_objects
            where_queried = where_url.queried_objects
    
            if len(who_queried) > 1 or len(what_queried) > 1 or len(where_queried) > 1:
                print('The Hypothesis{} is inconsistent'.format(IDs)) 
                consistent_ = False
            else:
                print('The Hypothesis{} is complete and consistent'.format(IDs))          
                who = find_string(who_queried[0])
                what = find_string(what_queried[0])
                where = find_string(where_queried[0])
                consistent_ = True 

            rospy.sleep(2)
            rate.sleep()
    
        # if the command chosen is 'stop'
        elif start == False:
            rate.sleep()
    
    
if __name__ == '__main__':
    main()
