/*
This class includes several functions which can be used to easily interface your webots simulation with your ROS-network
*/

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>

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
    bool setUpMotorControl(std::string motorName, std::string controlType);

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


    /**
    * \brief Creates Subscriber for the left and the right wheel of a diff drive robot
    * \param left_wheel_cmd_topic   topic name on which commands for the left wheel are published on
    * \param right_wheel_cmd_topic  topic name on which commands for the right wheel are published on
    */
    void setupWheelCmdSubscriber(std::string left_wheel_cmd_topic, std::string right_wheel_cmd_topic);

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

    void broadcastTF(std::string parent_frame, std::string child_frame);

    /**
    * \brief Sets up the rqt_multiplot configuration file to display multiple touch sensors
    * \param configuration_path  path of the configuration file, which needs to be updated
    */
    //void setUpTouchMultiplot(const char *configuration_path);



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
    std::map<std::string, ros::ServiceClient> MotorVelocityClients;
    std::map<std::string, ros::ServiceClient> MotorPositionClients;


    /**
    * \brief Callbacks for Left and right wheel subscriber
    * \param &msg   pointer for the incoming message
    */
    void leftWheelCmdCallback(const std_msgs::Float32::ConstPtr& msg);
    void rightWheelCmdCallback(const std_msgs::Float32::ConstPtr& msg);


    // subscriber for velocity messages for left and right wheel
    ros::Subscriber left_wheel_sub;
    ros::Subscriber right_wheel_sub;

};