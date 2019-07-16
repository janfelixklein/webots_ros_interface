#include <ros/ros.h>
#include <webots_ros/set_bool.h>
#include <std_msgs/Bool.h>

class Connector
{
    public:
        Connector(ros::NodeHandle* nh_pointer, std::string controllerName, std::string connectorName, std::string cmdTopic);
        bool lockStatus_;      
    private:
        void init();
        bool setLock(bool locktype); 
        void createSub();
        void cmdCallback(const std_msgs::Bool::ConstPtr& msg);
        ros::NodeHandle* nh_;
        ros::ServiceClient lockClient_;
        ros::Subscriber cmdSub_;
        std::string controllerName_;
        std::string connectorName_;
        std::string cmdTopic_;
        std::string lockPath_;
};