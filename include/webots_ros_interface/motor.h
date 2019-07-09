#include <ros/ros.h>
#include <webots_ros/set_float.h>
#include <std_msgs/Float32.h>

class Motor
{
    public:
        Motor(ros::NodeHandle* nh_pointer, std::string controllerName, std::string motorName, std::string controlType, std::string cmdTopic);
        void sendPosition(double position);
        void sendVelocity(double velocity);
        std::string controllerName_;
        std::string motorName_;
        std::string motorPositionServicePath_;
        std::string motorVelocityServicePath_;
        
        std::string cmdTopic_;
        std::string controlType_;       
    
    private:
        void init();
        bool initPositionClient();
        bool initVelocityClient();
        bool initVelocityControl();
        void createSub();
        void cmdCallback(const std_msgs::Float32::ConstPtr& msg);
        ros::NodeHandle* nh_;
        ros::ServiceClient MotorPositionClient_;
        ros::ServiceClient MotorVelocityClient_;
        ros::Subscriber cmdSub_;
};