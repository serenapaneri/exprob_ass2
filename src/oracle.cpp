#include "exprob_ass2/oracle.h"
#include <unistd.h>
#include <exprob_ass2/Oracle.h>
#include <exprob_ass2/HypoFound.h>
#include <exprob_ass2/WinHypo.h>

namespace KCL_rosplan {

    OracleInterface::OracleInterface(ros::NodeHandle &nh) {
        // here the initialization
    }
    
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

    int main(int argc, char **argv) {
        ros::init(argc, argv, "oracle_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::OracleInterface oracle(nh);
        oracle.runActionInterface();
        return 0;
    }
