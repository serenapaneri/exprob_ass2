#!/usr/bin/env python

import rospy
import time
from armor_msgs.srv import *
from armor_msgs.msg import *
from exprob_ass2.srv import Command, CommandResponse
from exprob_ass2.srv import HypoFound, HypoFoundRequest
from exprob_ass2.srv import WinHypo, WinHypoResponse

# armor client
armor_interface = None
# command service
comm_service = None
# hypo_found client
hypo_found_client = None
# win_hypo service
win_hypo_srv = None

start = False
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
# \brief It is the query command to retrieve an individual from a class.
# \param: None
# \return: res
#
# This functions returns, if there are any, the individuals of the class INCONSISTENT of the ontology.  
def inconsistent():
    req = ArmorDirectiveReq()
    req.client_name = 'check_hypothesis'
    req.reference_name = 'cluedontology'
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['INCONSISTENT']
    msg = armor_interface(req)
    res = msg.armor_response
    return res


##
# \brief Main function of the check_hypothesis node where the node is initialized and the armor client, the hypo_found_client and the comm_service are implemented.
# \param: None
# \return: None
#
# This is the main function of the check_hypothesis node 
def main():

    global armor_interface, comm_service, hypo_found_client, win_hypo_srv, start, IDs, who, what, where
    rospy.init_node('check_consistency')
    
    rospy.wait_for_service('armor_interface_srv')
    print('Waiting for the armor service')
    # armor client
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    
    # service to retrieve the ID of the current complete hypothesis just found
    rospy.wait_for_service('hypo_ID')
    hypo_found_client = rospy.ServiceProxy('hypo_ID', HypoFound)
    
    # command service
    comm_service = rospy.Service('comm', Command, com)
    
    # win_hypo service
    win_hypo_srv = rospy.Service('winhypo', WinHypo, winhypo_handle)
    
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        # if the command recieved is 'start'
        if start == True: 
            res = hypo_found_client()
            IDs = res.IDs 
            print(IDs)
            
            # response = CommandResponse()
            
            reasoner()
            
            who_url = retrieve_hypo('who', 'Hypothesis' + str(IDs))
            what_url = retrieve_hypo('what', 'Hypothesis' + str(IDs))
            where_url = retrieve_hypo('where', 'Hypothesis' + str(IDs))
            
            who_queried = who_url.queried_objects
            what_queried = what_url.queried_objects
            where_queried = where_url.queried_objects
            
            print(who_queried)
            print(what_queried)
            print(where_queried)
            
            response = CommandResponse()
            
            url_ind = '<http://www.emarolab.it/cluedo-ontology#'
            
            if len(who_queried) > 1 or len(what_queried) > 1 or len(where_queried) > 1:
                print('The Hypothesis{} is inconsistent'.format(IDs)) 
                response.answer = False
            else:
                print('The Hypothesis{} is complete and consistent'.format(IDs))
                _who_ = who_queried[0]
                who_ = _who_.replace(url_ind, '')
                who = who_.replace('>', '')
                    
                _what_ = what_queried[0]
                what_ = _what_.replace(url_ind, '')
                what = what_.replace('>', '')
                    
                _where_ = where_queried[0]
                where_ = _where_.replace(url_ind, '')
                where = where_.replace('>', '')
                
                response.answer = True
            
            print(who)
            print(what)
            print(where)
                
            
            rospy.sleep(2)
            rate.sleep()
        
        elif start == False:
            rate.sleep()
    
    
if __name__ == '__main__':
    main()
