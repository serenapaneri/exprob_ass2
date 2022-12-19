#include "exprob_ass2/check_hypothesis.h"
#include <unistd.h>

namespace KCL_rosplan {

    CheckHypoInterface::CheckHypoInterface(ros::NodeHandle &nh) {
        // here the initialization
    }
    
    bool CheckHypoInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        std::cout << "The robot is going to the " << msg->parameters[2].value << " room from " << msg->parameters[1].value << std::endl;
        
        
        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
        return true;
    }
}

    int main(int argc, char **argv) {
        ros::init(argc, argv, "check_hypothesis_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::CheckHypoInterface go_oracle(nh);
        go_oracle.runActionInterface();
        return 0;
    }
