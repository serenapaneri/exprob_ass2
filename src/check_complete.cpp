#include "exprob_ass2/check_complete.h"
#include <unistd.h>
#include <exprob_ass2/Complete.h>

namespace KCL_rosplan {

    CheckCompleteInterface::CheckCompleteInterface(ros::NodeHandle &nh) {
        // here the initialization
    }
    
    bool CheckCompleteInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        std::cout << "Checking the completeness " << std::endl;
        
        ros::NodeHandle n;
        ros::ServiceClient complete_client = n.serviceClient<exprob_ass2::Complete>("complete");
        
        exprob_ass2::Complete srv;
        
        complete_client.call(srv);
        
        std::cout << srv.response.complete << std::endl;
        
        if (srv.response.complete == true) {
        
            ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
            std::cout << "credo nei miracoli" << std::endl;
            return true; 
        }
        
        else {
            ROS_INFO("Action (%s) not performed!", msg->name.c_str());
            std::cout << "Ti sarebbe piaciuto funzionasse" << std::endl;
            return false;
        }
        
    }
}

    int main(int argc, char **argv) {
        ros::init(argc, argv, "check_hypothesis_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::CheckCompleteInterface check_complete(nh);
        check_complete.runActionInterface();
        return 0;
    }
