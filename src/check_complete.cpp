#include "exprob_ass2/check_complete.h"
#include <unistd.h>
#include <exprob_ass2/Complete.h>

ros::ServiceClient complete_client;
exprob_ass2::Complete srv;

namespace KCL_rosplan {

    CheckCompleteInterface::CheckCompleteInterface(ros::NodeHandle &nh) {
        // here the initialization
    }
    
    bool CheckCompleteInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        std::cout << "Checking the consistency " << std::endl;
        sleep(3.0);
        
        
        complete_client.call(srv);
        
        if (srv.response.complete == true) {
        
            ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
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
        complete_client = nh.serviceClient<exprob_ass2::Complete>("complete");
        KCL_rosplan::CheckCompleteInterface check_complete(nh);
        check_complete.runActionInterface();
        return 0;
    }
