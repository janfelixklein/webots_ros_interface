/*
This class includes several functions which can be used to interface ROS with the supervisor controller of webots
*/

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_publisher.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


class WebotsSupervisorInterface {
    public:
        WebotsSupervisorInterface(ros::NodeHandle* nh, std::string controllerName, std::string robotDEF);
        WebotsSupervisorInterface(ros::NodeHandle* nh, std::string controllerName);
        
        unsigned int getNodeFromDef(std::string defName);
        unsigned int getNodeFromSelf();
        unsigned int getField(unsigned int node, std::string fieldName);
        int getCount(unsigned int field);
        geometry_msgs::Point getPositionFromNode(ros::ServiceClient* Client, unsigned int node);
        geometry_msgs::Vector3 getVec3f(unsigned int field);
        double getFloat(unsigned int field);
        int setFloat(unsigned int field, double value);
        geometry_msgs::Quaternion getRotation(unsigned int field);
        void setRotation(ros::ServiceClient* setRotationClient, unsigned int field, geometry_msgs::Quaternion requestedRotation);
        void calcOrientationDifference(std::string parent_frame, std::string child_frame);



        void setRobotPoseClients();
        geometry_msgs::Vector3 getRobotTranslation();
        geometry_msgs::Quaternion getRobotRotation();
        void setRobotTranslation(geometry_msgs::Vector3 requestedTranslation);
        void setRobotRotation(geometry_msgs::Quaternion requestedRotation);
        void odometryUpdate(boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub, unsigned int translation_field, unsigned int rotation_field);

        geometry_msgs::Vector3 robotTranslation_;
        geometry_msgs::Quaternion robotRotation_;

        void broadcastTF(std::string parent_frame, std::string child_frame);
    
    
    private:
        std::string robotDEF_;
        std::string controllerName_;
        ros::NodeHandle* nh_;
        unsigned long int robotTranslationField_;
        unsigned long int robotRotationField_;

        

        ros::ServiceClient getVec3fClient_;
        ros::ServiceClient getFloatClient_;
        ros::ServiceClient setFloatClient_;
        ros::ServiceClient setVec3fClient_;
        ros::ServiceClient getRotationClient_;
        ros::ServiceClient setRotationClient_;
        ros::Publisher odomPub_;

};
