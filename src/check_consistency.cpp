#include "exprob_ass2/check_consistency.h"
#include <unistd.h>
#include <exprob_ass2/Command.h>

ros::ServiceClient comm_client;
exprob_ass2::Command srv;

namespace KCL_rosplan {

    CheckConsistencyInterface::CheckConsistencyInterface(ros::NodeHandle &nh) {
        // here the initialization
    }
    
    bool CheckConsistencyInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        std::cout << "Checking the consistency " << std::endl;
        
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
        comm_client = nh.serviceClient<exprob_ass2::Command>("comm");
        KCL_rosplan::CheckConsistencyInterface check_cons(nh);
        check_cons.runActionInterface();
        return 0;
    }
