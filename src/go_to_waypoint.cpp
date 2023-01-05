/** @package exprob_ass2
*
* \file go_to_waypoint.cpp
* \brief this node implements the go_to_waypoint action.
*
* \author Serena Paneri
* \version 1.0
* \date 5/1/2023
*
* \details
*
* Subscribes to: <BR>
*     None
*
* Publishes to: <BR>
*     None
*
* Serivces: <BR>
*     None
*
* Client Services: <BR>
*     None
*
* Action Client: <BR>
*     /go_to_point
*
* Description: <BR>
* In this node is implemented the go_to_waypoint action of the rosplan.
* This node makes the robot move from one waypoint to the following one in the simulation environment,
* and the movement is menaged thanks to an action client. 
*/

#include "exprob_ass2/go_to_waypoint.h"
#include <unistd.h>
#include <math.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprob_ass2/TargetAction.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace KCL_rosplan {

    GoToWaypointInterface::GoToWaypointInterface(ros::NodeHandle &nh) {
        // here the initialization
    }


/**
* \brief Callback of the go_to_waypoint action.
* \param msg: message from the plan_dispatcher
* \return true
*
* This function implements the beahvior of the robot when the action go_to_waypointy is executed. This action allows
* the robot to move within the environment, in particular from one waypoint to another one.
*/     
    bool GoToWaypointInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        
        moveit::planning_interface::MoveGroupInterface group("arm");
        group.setEndEffectorLink("cluedo_link");
        group.setPoseReferenceFrame("base_link");
        group.setPlannerId("RRTstar");
        group.setNumPlanningAttempts(10);
        group.setPlanningTime(10.0);
        group.allowReplanning(true);
        group.setGoalJointTolerance(0.0001);
        group.setGoalPositionTolerance(0.0001);
        group.setGoalOrientationTolerance(0.001);
        
        group.setNamedTarget("default");
	group.move();
	sleep(3.0);

        std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
        
        actionlib::SimpleActionClient<exprob_ass2::TargetAction> ac("/go_to_point", true);
        exprob_ass2::TargetGoal goal;
        ac.waitForServer();
        if(msg->parameters[2].value == "wp1"){
            goal.x = 2.2;
            goal.y = 0.0;
            goal.theta = 0.0;
        }
        else if (msg->parameters[2].value == "wp2"){
            goal.x = 0.0;
            goal.y = 2.2;
            goal.theta = M_PI/2;
        }
        else if (msg->parameters[2].value == "wp3"){
            goal.x = - 2.2;
            goal.y = 0.0;
            goal.theta = M_PI;
        }
        else if (msg->parameters[2].value == "wp4"){
            goal.x = 0.0;
            goal.y = - 2.2;
            goal.theta = - M_PI/2;
        }
        
        ac.sendGoal(goal);
        ac.waitForResult();
        
        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
        return true;
    }
}


/**
* \brief Main function of the go_to_waypoint action. 
* \param None
* \return 0
*
* This is the main function of the go_to_waypoint action, where the node is initialized. Moreover there is the 
* GoToWaypointInterface to execute the real action as an action of the rosplan.
*/
    int main(int argc, char **argv) {
        ros::init(argc, argv, "go_to_waypoint_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::GoToWaypointInterface go_to_waypoint(nh);
        go_to_waypoint.runActionInterface();
        ros::AsyncSpinner spinner(1);
        spinner.start();
        sleep(2.0);
        return 0;
    }

