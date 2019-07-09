/*
This class includes several functions which can be used to easily interface your webots simulation with your ROS-network
*/

#include <ros/ros.h>
#include <webots_ros_interface/motor.h>
#include <std_msgs/Bool.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_bool.h>

class WebotsRobotInterface 
{
public:
    /**
    * \brief Constructor
    * \param nh_pointer   pointer to your node handle created in your main (ros::NodeHandle*)
    * \param controllerName name of your webots controller
    */
    WebotsRobotInterface(ros::NodeHandle* nh, std::string controllerName);

/**********************************************/
/*          MOTOR RELATED FUNCTIONS           */
/**********************************************/  
    /*
    * \brief Enables motors for velocity or position control
    * \param motorName    name of the motor that should be controlled
    * \param controlType  controlType, either "velocity" or "position"
    */

    bool setUpMotorControl(std::string motorName, std::string controlType, std::string cmdTopic);

    /*
    * \brief send a single position command to a motor
    * \param motorName    name of the motor that should be controlled
    * \param position     command in [rad] for revolute and [m] for linear joints
    */
    void sendMotorPosition(std::string motorName, double position);

    /*
    * \brief send a single velocity command to a motor
    * \param motorName    name of the motor that should be controlled
    * \param position     command in [rad] for revolute and [m] for linear joints
    */
    void sendMotorVelocity(std::string motorName, double velocity);


/**********************************************/
/*          SENSOR RELATED FUNCTIONS          */
/**********************************************/

    /**
    * \brief Enable any webots device specified by its name
    * \param device_name  device name as specified in webots (name="")
    */
    void enableDevice(std::string device_name);   

/**********************************************/
/*          OTHER FUNCTIONS                   */
/**********************************************/

    /**
    * \brief Allows to lock two connector devices 
    * \param connector_name  name of the connector that should be locked
    * \param lock_type       1 to lock, 0 to unlock
    */
    void lockConnector(std::string connector_name, bool lock_type);

    void broadcastTF(std::string parent_frame, std::string child_frame);



private: 
    // name of the topic on which webots is publishing the active controller names
    std::string modelTopicName_;

    // NodeHandle pointer;
    ros::NodeHandle* nh_;

    // Subscriber for the topic on which webots is publishing the active controller names
    ros::Subscriber nameSub_; 

    
    std::string controllerName_;  // string which holds a single controller name (the first active controller)       
    int controllerCount_;         // number of active controller 
    std::vector<std::string> controllerList;


    //Motor structure
    std::map<std::string, boost::shared_ptr< Motor > > motors;

};