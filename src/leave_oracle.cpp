/** @package exprob_ass2
*
* \file leave_oracle.cpp
* \brief this node implements the leave_oracle action.
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
* In this node is implemented the leave_oracle action of the rosplan.
* This node makes the robot move from the (2.5., -2.5) point to one of the four waypoints in the simulation
* environment, and the movement is menaged thanks to an action client. 
*/

#include "exprob_ass2/leave_oracle.h"
#include <unistd.h>
#include <math.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprob_ass2/TargetAction.h>

namespace KCL_rosplan {

    LeaveOracleInterface::LeaveOracleInterface(ros::NodeHandle &nh) {
        // here the initialization
    }
    
/**
* \brief Callback of the leave_oracle action
* \param msg: message from the plan_dispatcher
* \return true
*
* This function implements the beahvior of the robot when the action leave_oracle is executed. This action allows
* the robot to move within the environment, in particular from the oracle, to one of the four waypoints.
*/      
    bool LeaveOracleInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        std::cout << "Leaving the " << msg->parameters[1].value << " room for going to " << msg->parameters[2].value << std::endl;
        
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
* \brief Main function of the leave_oracle action. 
* \param None
* \return 0
*
* This is the main function of the leave_oracle action, where the node is initialized. Moreover there is the 
* LeaveOracleInterface to execute the real action as an action of the rosplan.
*/
    int main(int argc, char **argv) {
        ros::init(argc, argv, "leave_oracle_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::LeaveOracleInterface leave_oracle(nh);
        leave_oracle.runActionInterface();
        return 0;
    }
