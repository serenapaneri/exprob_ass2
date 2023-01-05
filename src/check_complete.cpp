/** @package exprob_ass2
*
* \file check_complete.cpp
* \brief this node implements the check_complete action.
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
*     complete
*
* Action Client: <BR>
*     None
*
* Description: <BR>
* In this node is implemented the check_complete action of the rosplan.
* This node allows to know if a new complete hypothesis has been found, and this is perfomed
* thanks to the complete service that advertise this node. If a new complete hypothesis has
* been found, then the plan can proceed, if instead there is no new complete hypothesis then
* the action return false and a replanning is needed. 
*/

#include "exprob_ass2/check_complete.h"
#include <unistd.h>
#include <exprob_ass2/Complete.h>

namespace KCL_rosplan {

    CheckCompleteInterface::CheckCompleteInterface(ros::NodeHandle &nh) {
        // here the initialization
    }


/**
* \brief Callback of the check_complete action
* \param msg: message from the plan_dispatcher
* \return true
*
* This function implements the beahvior of the robot when the action check_complete is executed. This action 
* is performed at every waypoint after the move_arm action. To know if there are new complete hypothesis a 
* client to the complete service is implemented. If the information given by the client is true then the plan
* can go on and can proceed with the check of the consistency, if insted the information given by the client
* is false, then a replanning is needed, and the robot will continue to explore the waypoints while collecting
* new hints.
*/     
    bool CheckCompleteInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        std::cout << "Checking the completeness " << std::endl;
        
        ros::NodeHandle n;
        ros::ServiceClient complete_client = n.serviceClient<exprob_ass2::Complete>("complete");
        
        exprob_ass2::Complete srv;

        complete_client.call(srv);

        
        if (srv.response.complete == true) {
            ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
            return true; 
        }
        
        else {
            ROS_INFO("Action (%s) not performed!", msg->name.c_str());
            return false;
        }
        
    }
}


/**
* \brief Main function of the check_complete action. 
* \param None
* \return 0
*
* This is the main function of the check_complete action, where the node is initialized. Moreover there is the 
* CheckCompleteInterface to execute the real action as an action of the rosplan.
*/
    int main(int argc, char **argv) {
        ros::init(argc, argv, "check_hypothesis_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::CheckCompleteInterface check_complete(nh);
        check_complete.runActionInterface();
        return 0;
    }
