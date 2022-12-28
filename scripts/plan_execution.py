#!/usr/bin/env python

import rospy
import time
import random
from rosplan_knowledge_msgs.srv import *
from rosplan_dispatch_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
from diagnostic_msgs.msg import KeyValue
# from rosplan_interface_mapping.srv import CreatePRM
# from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
# from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse, PlanningService, PlanningServiceResponse
# from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest


update_knowledge = None


#   This function initializes all of the needed services, then it calculates a new plan
#   until the robot is able to fuòòy complete one. 
def main():

    global update_knowledge
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
    # update the knowledge base
    rospy.wait_for_service('/rosplan_knowledge_base/update')
    update_knowledge = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
    print("Start planning")
    

    while not rospy.is_shutdown():
        
        problem_interface()
        
        print('problem generates')
        time.sleep(1)
        
        # generate the plan
        planning_interface()
        
        print('plan generates')
        time.sleep(1)
        
        # parse the plan 
        parsing_interface()
        
        print('parse generates')
        time.sleep(1)
        
        # read the feedback
        feedback = plan_dispatcher()
        print(feedback)

        # update the knowledge base
        update_knowledgebase()
        time.sleep(1)
        
        print ( 'SUCCESS')
        

#   This function calls the knowledge base server to delete the predicate (hint_taken)
#   for one waypoint, the one recognized by the parameter name

def waypoint_visited(wp0):
    global update_knowledge
    req = KnowledgeUpdateServiceRequest()
    req.update_type = 2
    req.knowledge.is_negative = False
    req.knowledge.knowledge_type = 1
    req.knowledge.attribute_name= 'hint_percieved'
    req.knowledge.values.append(diagnostic_msgs.msg.KeyValue('waypoint', wp0))	
    res = update_knowledge(req)


#   This function calls the knowledge base server to delete the predicate
#   (hypothesis_complete)

def update_check_complete():
    global update_knowledge
    req = KnowledgeUpdateServiceRequest()
    req.update_type = 2
    req.knowledge.is_negative = False
    req.knowledge.knowledge_type = 1
    req.knowledge.attribute_name= 'complete_hypo'
    res = update_knowledge(req)
    print("funziona update_check_complete")


#   This function looks at the parameter in the ros param server random.
#   if random is set to true it finds a random waypoint ( different from the previously
#   calculated one) and it proceeds to delete the (hint_taken) for that waypoint.
#   In case the random parameter is set to false it deletes the fact (hint_taken) for
#   all waypoints. After that it deletes the (hypothesis_complete) fact.

def update_knowledgebase():

    waypoint_visited('wp1')
    waypoint_visited('wp2')
    waypoint_visited('wp3')
    waypoint_visited('wp4')
    
    print("funziona update_knowledgebase")

    update_check_complete()

if __name__ == '__main__':
    main()