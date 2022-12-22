#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"

namespace KCL_rosplan {

    class GoHomeInterface: public RPActionInterface
    {
    
    private:
    
    public:
    
        /* constructor */
        GoHomeInterface(ros::NodeHandle &nh);
        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };
}