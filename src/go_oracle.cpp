/** @package exprob_ass2
*
* \file go_oracle.cpp
* \brief this node implements the go_oracle action.
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
* In this node is implemented the go_oracle action of the rosplan.
* This node makes the robot move from one of the four waypoints in the simulation environment
* to the oracle room in (2.5, -2.5), and the movement is menaged thanks to an action client. 
*/

#include "exprob_ass2/go_oracle.h"
#include <unistd.h>
#include <math.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprob_ass2/TargetAction.h>

namespace KCL_rosplan {

    GoOracleInterface::GoOracleInterface(ros::NodeHandle &nh) {
        // here the initialization
    }


/**
* \brief Callback of the go_oracle action
* \param msg: message from the plan_dispatcher
* \return true
*
* This function implements the beahvior of the robot when the action go_oracle is executed. This action allows
* the robot to move within the environment, in particular from one of the four waypoints to the oracle room.
*/    
    bool GoOracleInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        std::cout << "The robot is going to the " << msg->parameters[2].value << " room from " << msg->parameters[1].value << std::endl;
        
        actionlib::SimpleActionClient<exprob_ass2::TargetAction> ac("/go_to_point", true);
        exprob_ass2::TargetGoal goal;
        ac.waitForServer();
        
        goal.x = 2.5;
        goal.y = - 2.5;
        goal.theta = - M_PI/4;
        
        ac.sendGoal(goal);
        ac.waitForResult();
        
        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
        return true;
    }
}


/**
* \brief Main function of the go_oracle action. 
* \param None
* \return 0
*
* This is the main function of the go_oracle action, where the node is initialized. Moreover there is the 
* GoOracleInterface to execute the real action as an action of the rosplan.
*/
    int main(int argc, char **argv) {
        ros::init(argc, argv, "go_oracle_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::GoOracleInterface go_oracle(nh);
        go_oracle.runActionInterface();
        return 0;
    }
