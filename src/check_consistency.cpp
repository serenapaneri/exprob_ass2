/** @package exprob_ass2
*
* \file check_consistency.cpp
* \brief this node implements the check_consistency action.
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
*     consistent
*     comm
*
* Action Client: <BR>
*     None
*
* Description: <BR>
* In this node is implemented the check_consistency action of the rosplan.
* This node allows to know if an hypotheis is consistent or not and this is perfomed thanks
* to the consistent service that advertise this node. If the hypothesis is complete and consistent
* then the plan can proceed, if instead it is inconsistent then a replanning is needed. 
*/

#include "exprob_ass2/check_consistency.h"
#include <unistd.h>
#include <exprob_ass2/Command.h>
#include <exprob_ass2/Consistent.h>

namespace KCL_rosplan {

    CheckConsistencyInterface::CheckConsistencyInterface(ros::NodeHandle &nh) {
        // here the initialization
    }


/**
* \brief Callback of the check_consistency action
* \param msg: message from the plan_dispatcher
* \return true
*
* This function implements the beahvior of the robot when the action check_consistency is executed. This action 
* is performed after a new complete hypothesis has been found and the robot reaches home in order to verify the
* consistency of the current complete hypothesis. In particular this is done by two clients to the services 
* comm and consistent, in particular this last service is composed by a boolean response that is set to true when
* the compelete hypothesis is also consistent and is set to false when it is not. Of course if the response is 
* false then a replanning is neeeded. 
*/     
    bool CheckConsistencyInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        std::cout << "Checking the consistency " << std::endl;
        
        ros::NodeHandle n;
        ros::ServiceClient comm_client = n.serviceClient<exprob_ass2::Command>("comm");
        exprob_ass2::Command comm_srv;
        
        ros::NodeHandle hn;
        ros::ServiceClient cons_client = hn.serviceClient<exprob_ass2::Consistent>("consistent");
        exprob_ass2::Consistent cons_srv;
        
        comm_srv.request.command = "start";
        
        comm_client.call(comm_srv);
        sleep(2.0);
        cons_client.call(cons_srv);
        
        if (cons_srv.response.consistent == true) {
        
            ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
            comm_srv.request.command = "stop";
            comm_client.call(comm_srv);
            return true; 
        }
        
        else if (cons_srv.response.consistent == false) {
            ROS_INFO("Action (%s) not performed!", msg->name.c_str());
            comm_srv.request.command = "stop";
            comm_client.call(comm_srv);
            return false;
        }
        
    }
}


/**
* \brief Main function of the check_consistency action. 
* \param None
* \return 0
*
* This is the main function of the check_consistency action, where the node is initialized. Moreover there is the 
* CheckConsistencyInterface to execute the real action as an action of the rosplan.
*/
    int main(int argc, char **argv) {
        ros::init(argc, argv, "check_hypothesis_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::CheckConsistencyInterface check_cons(nh);
        check_cons.runActionInterface();
        return 0;
    }
