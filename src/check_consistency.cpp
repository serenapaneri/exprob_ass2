#include "exprob_ass2/check_consistency.h"
#include <unistd.h>
#include <exprob_ass2/Command.h>
#include <exprob_ass2/Consistent.h>

namespace KCL_rosplan {

    CheckConsistencyInterface::CheckConsistencyInterface(ros::NodeHandle &nh) {
        // here the initialization
    }
    
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

    int main(int argc, char **argv) {
        ros::init(argc, argv, "check_hypothesis_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::CheckConsistencyInterface check_cons(nh);
        check_cons.runActionInterface();
        return 0;
    }
