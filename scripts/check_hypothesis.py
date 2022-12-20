#!/usr/bin/env python2

import rospy
from armor_msgs.srv import *
from armor_msgs.msg import *
from exprob_ass2.srv import Command, CommandResponse

armor_interface = None
comm_service = None
start = False

def com(req):
    global start
    if (req.command == 'start'):
        start = True
    elif (req.command == 'stop'):
        start = False
    return start


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
    

##
# \brief It is the query command to retrieve an individual from a class.
# \param: None
# \return: res
#
# This functions returns, if there are any, the individuals of the class INCONSISTENT of the ontology.  
def inconsistent():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['INCONSISTENT']
    msg = armor_interface(req)
    res = msg.armor_response
    return res
    

def main():

    global armor_interface, comm_service
    rospy.init_node('check_hypothesis')
    rospy.wait_for_service('armor_interface_srv')
    print('Waiting for the armor service')
    # armor client
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    # command service
    comm_service = rospy.Service('comm', Command, com)
    
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
    
        if start == True: 
    
            # for num in range(6):
            # url = '<http://www.emarolab.it/cluedo-ontology#Hypothesis{}>'.format(num)
    
            print('Check for a complete and consistent hypothesis')
            iscomplete = complete()
            print(iscomplete.queried_objects)
            print('Checking now the completeness')
            if len(iscomplete.queried_objects) == 0:
                print('None of the founded hypotheses is complete')
                CommandResponse == False
                # server that advertise the node check_hypothesis to execute leave_home action
            elif len(iscomplete.queried_objects) != 0:
                print('Checking now the consistency')
        
                isinconsistent = inconsistent()
                print(isinconsistent.queried_objects)
        
                    # if len(isinconsistent.queried_objects) == 0:
        
                    # elif len(isinconsistent.queried_objects) != 0:
                    CommandResponse == True
                    rate.sleep()
        
        # implement a server that advertise the node check_hypothesis that a complete hypothesis has been found and it can go to the oracle.
        
        elif start == False:
            time.sleep(5)
            rate.sleep()
    
    
if __name__ == '__main__':
    main()
