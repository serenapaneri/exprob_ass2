#!/usr/bin/env python2

import rospy
import time
from armor_msgs.srv import *
from armor_msgs.msg import *
from exprob_ass2.srv import Command, CommandResponse
from exprob_ass2.srv import HypoFound, HypoFoundRequest

armor_interface = None
comm_service = None
hypo_found_client = None
start = False
success = False

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

    global armor_interface, comm_service, hypo_found_client, start, success
    rospy.init_node('check_hypothesis')
    rospy.wait_for_service('armor_interface_srv')
    print('Waiting for the armor service')
    # armor client
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    
    # command service
    comm_service = rospy.Service('comm', Command, com)
    
    # IDs client
    hypo_found_client = rospy.ServiceProxy('hypo_ID', HypoFound)
    
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():

        if start == True: 
        
            print('problema qui')

            # service to retrieve the list of ID that has been already taken
            rospy.wait_for_service('hypo_ID')
            
            print('nono problema qui')
            req = HypoFoundRequest()
            print('ma nono problema qui')
            res = hypo_found_client(req)
            print('ti dico invece che il problema è qui')
            IDs = res.IDs 
            print('non credo sia qui ma proviamo')
            print(IDs)
            
            # for every ID already uploaded in the ontology check completeness and consistency
            for num in IDs:
                print('dubito seriamente')
                url = '<http://www.emarolab.it/cluedo-ontology#Hypothesis{}>'.format(num)
                
                iscomplete = complete()
                print(iscomplete.queried_objects)
                print('Checking now the completeness of the Hypothesis{}'.format(num))
                
                if len(iscomplete.queried_objects) == 0:
                    print('The Hypothesis{} is not complete'.format(num))
                    success == False
                    # CommandResponse == False
                    
                elif len(iscomplete.queried_objects) != 0:
                    if url not in iscomplete.queried_objects:
                        print('The Hypothesis{} is not complete'.format(num))
                        success == False
                        # CommandResponse == False
                        
                    elif url in iscomplete.queried_objects:
                        print('The Hypothesis{} is complete'.format(num))
                        print('Checking now the consistency of the Hypothesis{}'.format(num))
        
                        isinconsistent = inconsistent()
                        print(isinconsistent.queried_objects)
        
                        if len(isinconsistent.queried_objects) != 0:
                            if url in isinconsistent.queried_objects:
                                print('The Hypothesis{} is inconsistent'.format(num)) 
                                success == False
                                # CommandResponse == False
                            elif url not in isinconsistent.queried_objects:
                                print('The Hypothesis{} is complete and consistent'.format(num))
                                success == True
                                # CommandResponse == True
                        elif len(isinconsistent.queried_objects) == 0:
                            print('The Hypothesis{} is complete and consistent'.format(num))
                            # success == True
            CommandResponse == True
            rate.sleep()
        
        # implement a server that advertise the node check_hypothesis that a complete hypothesis has been found and it can go to the oracle.
        
        elif start == False:
            time.sleep(5)
            rate.sleep()
    
    
if __name__ == '__main__':
    main()
