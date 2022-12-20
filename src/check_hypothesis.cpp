#include "exprob_ass2/check_hypothesis.h"
#include <unistd.h>
#include <exprob_ass2/Command.h>

namespace KCL_rosplan {

    CheckHypoInterface::CheckHypoInterface(ros::NodeHandle &nh) {
        // here the initialization
    }
    
    bool CheckHypoInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        std::cout << "Checking the completeness and the consistency " << std::endl;
        
        ros::NodeHandle n;
        ros::ServiceClient comm_client = n.serviceClient<exprob_ass2::Command>("comm");
        exprob_ass2::Command srv;
        srv.request.command == "start";
        comm_client.call(srv);
        
        if (srv.response.answer == true) {
        
            ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
            srv.request.command == "stop";
            return true; 
        }
        
        else {
            ROS_INFO("Action (%s) not performed!", msg->name.c_str());
            srv.request.command == "stop";
            return false;
        }
        
    }
}

    int main(int argc, char **argv) {
        ros::init(argc, argv, "check_hypothesis_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::CheckHypoInterface go_oracle(nh);
        go_oracle.runActionInterface();
        return 0;
    }
