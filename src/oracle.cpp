/** @package exprob_ass2
*
* \file oracle.cpp
* \brief this node implements the oracle action.
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
*     /oracle_solution
*     hypo_ID
*     winhypo
*
* Action Client: <BR>
*     None
*
* Description: <BR>
* In this node is implemented the oracle action of the rosplan.
* This node allows to knwo if the current complete and consistent hypothesis is the winning one or not.
* This is done by comparing the value contained in the client of oracle service and the client of the 
* hypo_found service, that contains the ID of the current complete and consistent hypothesis.
* The client of the service winhypo is only use to know the who, what and where associated to the current ID.
* If the two IDs matches then the plan is finished, instead if not, a replanning is needed to continue the
* investigation. 
*/

#include "exprob_ass2/oracle.h"
#include <unistd.h>
#include <exprob_ass2/Oracle.h>
#include <exprob_ass2/HypoFound.h>
#include <exprob_ass2/WinHypo.h>

namespace KCL_rosplan {

    OracleInterface::OracleInterface(ros::NodeHandle &nh) {
        // here the initialization
    }


/**
* \brief Callback of the oracle action
* \param msg: message from the plan_dispatcher
* \return true
*
* This function implements the beahvior of the robot when the action oracle is executed. This action allows
* the robot its own complete and consistent hypothesis just collected is the correct one or the wrong one.
* To do that we have a client to the oracle solution service that contains the ID of the winning hypothesis,
* the client to the hypo_ID service that contains the ID of the current hypothesis proposed by the robot, and
* the client to the winhypo service which contains the who, the what and the where associated to the current ID.
* If the two IDs matches, then the plan is completed since the game is finished. If not a replanning is needed. 
*/    
    bool OracleInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        std::cout << "Checking if the hypothesis is the correct one" << std::endl;
        
        ros::NodeHandle n;
        ros::ServiceClient oracle_client = n.serviceClient<exprob_ass2::Oracle>("/oracle_solution");
        exprob_ass2::Oracle oracle_srv;
        
        ros::NodeHandle hn;
        ros::ServiceClient hypo_found_client = hn.serviceClient<exprob_ass2::HypoFound>("hypo_ID");
        exprob_ass2::HypoFound hypo_srv;
        
        ros::NodeHandle nhh;
        ros::ServiceClient win_hypo_client = nhh.serviceClient<exprob_ass2::WinHypo>("winhypo");
        exprob_ass2::WinHypo winhypo_srv;

        oracle_client.call(oracle_srv);
        hypo_found_client.call(hypo_srv);
        win_hypo_client.call(winhypo_srv);
        
        sleep(1.0);
        std::cout << "Name your guess" << std::endl;
        sleep(1.0);
        std::cout << winhypo_srv.response.who << " with the " << winhypo_srv.response.what << " in the " << winhypo_srv.response.where << std::endl;
        sleep(1.0);
        
        if (oracle_srv.response.ID == hypo_srv.response.IDs) {
            ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
            std::cout << "Yes, you guessed right!" << std::endl;
            return true; 
        }
        
        else {
            ROS_INFO("Action (%s) not performed!", msg->name.c_str());
            std::cout << "No you are wrong, maybe next time you will have better luck" << std::endl;
            return false;
        }
        
    }
}


/**
* \brief Main function of the oracle action. 
* \param None
* \return 0
*
* This is the main function of the oracle action, where the node is initialized. Moreover there is the 
* OracleInterface to execute the real action as an action of the rosplan.
*/  
    int main(int argc, char **argv) {
        ros::init(argc, argv, "oracle_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::OracleInterface oracle(nh);
        oracle.runActionInterface();
        return 0;
    }
