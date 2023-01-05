/** @package exprob_ass2
*
* \file go_home.cpp
* \brief this node implements the go_home action.
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
* In this node is implemented the go_home action of the rosplan.
* This node makes the robot move from  one of the four waypoints in the simulation environement to home
* at the point (0.0, 0.0), and the movement is menaged thanks to an action client. 
*/

#include "exprob_ass2/go_home.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprob_ass2/TargetAction.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace KCL_rosplan {

    GoHomeInterface::GoHomeInterface(ros::NodeHandle &nh) {
        // here the initialization
    }


/**
* \brief Callback of the go_home action
* \param msg: message from the plan_dispatcher
* \return true
*
* This function implements the beahvior of the robot when the action go_home is executed. This action allows
* the robot to move within the environment, in particular from one of the four waypoints to home.
*/    
    bool GoHomeInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
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
        
        std::cout << "The robot is going " << msg->parameters[2].value << " from " << msg->parameters[1].value << std::endl;
        
        actionlib::SimpleActionClient<exprob_ass2::TargetAction> ac("/go_to_point", true);
        exprob_ass2::TargetGoal goal;
        ac.waitForServer();

        goal.x = 0.0;
        goal.y = 0.0;
        goal.theta = 0.0;
        
        ac.sendGoal(goal);
        ac.waitForResult();
        
        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
        return true;
        
    }
}


/**
* \brief Main function of the go_home action. 
* \param None
* \return 0
*
* This is the main function of the go_home action, where the node is initialized. Moreover there is the 
* GoHomeInterface to execute the real action as an action of the rosplan.
*/
    int main(int argc, char **argv) {
    
        ros::init(argc, argv, "go_home_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::GoHomeInterface go_home(nh);
        go_home.runActionInterface();
        ros::AsyncSpinner spinner(1);
        spinner.start();
        sleep(2.0);
        return 0;
    }
