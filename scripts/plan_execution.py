#!/usr/bin/env python

## @package exprob_ass2
#
# \file plan_execution.py
# \brief script that menages the execution of the rosplan.
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
#     None
#
# Client Services: <BR>
#     armor_interface_srv
#     /rosplan_problem_interface/problem_generation_server
#     /rosplan_planner_interface/planning_server
#     /rosplan_parsing_interface/parse_plan
#     /rosplan_plan_dispatcher/dispatch_plan
#
# Action Services: <BR>
#     None
#
# Description: <BR>
#     In this node all the services of the rosplan are called in order to execute the current plan generated
#     automatically, thanks to those services, from the domain and the problem files provided. Since all those
#     services are called within a loop, everytime that the plan_dispatcher service returns that the plan has
#     failed, the replanning is automatically started. 

import rospy
import time
from armor_msgs.srv import * 
from armor_msgs.msg import * 
from rosplan_knowledge_msgs.srv import *
from rosplan_dispatch_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
from diagnostic_msgs.msg import KeyValue

upd_knowledge = None
rosplan_success = False
rosplan_goal = False


##
# \brief It saves the changes on a new ontology file.
# \param: None
# \return: None
#
# This functions saves the ontology in a new file, also saving the inferences.
def save():

    req = ArmorDirectiveReq()
    req.client_name = 'plan_execution'
    req.reference_name = 'cluedontology'
    req.command = 'SAVE'
    req.primary_command_spec = 'INFERENCE'
    req.secondary_command_spec = ''
    req.args = ['/root/ros_ws/src/exprob_ass2/final_ontology_inferred.owl']
    msg = armor_interface(req)
    res = msg.armor_response
    print('The new ontology has been saved under the name final_ontology_inferred.owl')
    

##
# \brief Main function of the node in which the node is initialized and all the rosplan services are called.
# \param: None
# \return: None
#
# This is the main function of the node, where the node is initialized and all the rosplan services are called.
# In case of the fail of the plan, the replanning is automatically started.
def main():

    global rosplan_success, rosplan_goal, armor_interface
    rospy.init_node('plan_execution')
    
    # calling all the rosplan services 
    print ("Calling all rosplan services ..")
    # problem interface service
    rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
    problem_interface = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
    # planning interface service
    rospy.wait_for_service('/rosplan_planner_interface/planning_server')
    planning_interface = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
    # parsing interface service
    rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
    parsing_interface = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
    # dispatch plan
    rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
    plan_dispatcher = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
    # armor client
    rospy.wait_for_service('armor_interface_srv')
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    print("Start planning")
    
    rosplan_success = False
    rosplan_goal = False
    
    while (rosplan_success == False or rosplan_goal == False):
        
        # print('Problem interface service')
        problem_interface()        
        # print('Planning interface service')
        planning_interface()
        # print('Parsing interface service')
        parsing_interface()
        # print('Plan dispatcher')
        feedback = plan_dispatcher()
        print(feedback)
        
        rosplan_success = feedback.success
        rosplan_goal = feedback.goal_achieved
        
        print('Replanning')
    
    save() 
    print('The plan has finished!')


if __name__ == '__main__':
    main()
