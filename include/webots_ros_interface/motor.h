#include <ros/ros.h>
#include <webots_ros/set_float.h>
#include <webots_ros/Float64Stamped.h>
#include <std_msgs/Float64.h>
#include <actionlib/server/simple_action_server.h>
#include <webots_ros_interface/position_controlAction.h>

class Motor
{
    public:
        Motor(ros::NodeHandle* nh_pointer, std::string controllerName, std::string motorName, std::string controlType, std::string cmdTopic);
        Motor(ros::NodeHandle* nh_pointer, std::string controllerName, std::string motorName, std::string controlType, std::string sensorTopic, std::string actionName);
        void sendPosition(double position);
        void sendVelocity(double velocity);
        std::string controllerName_;
        std::string motorName_;
        std::string motorPositionServicePath_;
        std::string motorVelocityServicePath_;
        
        double processValue_;

        std::string cmdTopic_;
        std::string sensorTopic_;
        std::string controlType_;
        bool actionInterface_; 
        std::string actionName_;      
    
    private:
        void init();
        bool initPositionClient();
        bool initVelocityClient();
        bool initVelocityControl();
        void createCmdSub();
        void createSensorSub();
        void cmdCallback(const std_msgs::Float64::ConstPtr& msg);
        void sensorCallback(const webots_ros::Float64Stamped::ConstPtr& msg);
        void executeCB(const webots_ros_interface::position_controlGoalConstPtr &goal);
        ros::NodeHandle* nh_;
        ros::ServiceClient MotorPositionClient_;
        ros::ServiceClient MotorVelocityClient_;
        ros::Subscriber cmdSub_;
        ros::Subscriber sensorSub_;
        actionlib::SimpleActionServer<webots_ros_interface::position_controlAction> *as_;
        // create messages that are used to published feedback/result
        webots_ros_interface::position_controlFeedback feedback_;
        webots_ros_interface::position_controlResult result_;
};