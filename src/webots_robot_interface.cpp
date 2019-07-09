#include <webots_ros_interface/webots_robot_interface.h>

//#include <tinyxml2.h>


WebotsRobotInterface::WebotsRobotInterface(ros::NodeHandle* nh, std::string controllerName)
    : controllerName_(controllerName)
    , nh_(nh)
{
}


/**********************************************/
/*          MOTOR RELATED FUNCTIONS           */
/**********************************************/

bool WebotsRobotInterface::setUpMotorControl(std::string motorName, std::string controlType="position", std::string cmdTopic="")   {
    boost::shared_ptr< Motor > motor( new Motor(nh_, controllerName_, motorName, controlType, cmdTopic) );
    motors[motorName] = motor;
}

void WebotsRobotInterface::sendMotorPosition(std::string motorName, double position)   {
    motors[motorName]->sendPosition(position);
}

void WebotsRobotInterface::sendMotorVelocity(std::string motorName, double velocity)   {
    motors[motorName]->sendVelocity(velocity);
}

/**********************************************/
/*          SENSOR RELATED FUNCTIONS          */
/**********************************************/

// Function for sensor devices
void WebotsRobotInterface::enableDevice(std::string device_name)    {
    std::string device_path = "/" + controllerName_ + "/" + device_name + "/enable";
    ros::ServiceClient enableClient = nh_->serviceClient<webots_ros::set_int>(device_path);
    webots_ros::set_int enableRequest;
    enableRequest.request.value = 1;
    if (!enableClient.call(enableRequest)) {
        ROS_WARN("Couldnt't enable device named '%s' ,please check if the service path '%s' is viable.", device_name.c_str(), device_path.c_str());
    }
    else    {
        ROS_INFO("Successfully enabled device '%s' on robot '%s'", device_name.c_str(), controllerName_.c_str());
    }
}


/**********************************************/
/*          OTHER FUNCTIONS                   */
/**********************************************/

// connector related

void WebotsRobotInterface::lockConnector(std::string connector_name, bool locktype)    {
    std::string service_path = "/" + controllerName_ + "/" + connector_name + "/lock";
    ros::ServiceClient lockClient = nh_->serviceClient<webots_ros::set_bool>(service_path);
    webots_ros::set_bool lockRequest;
    lockRequest.request.value = locktype;
    if (!lockClient.call(lockRequest)) {
        ROS_WARN("Couldnt't lock the connector '%s', please check if the service path %s is viable.", connector_name.c_str(), service_path.c_str());
    }
    else    {
        if (locktype == true) {
            ROS_INFO("Changed the lock state of connector '%s' to 'locked'", connector_name.c_str());
        }
        else    {
            ROS_INFO("Changed the lock state of connector '%s' to 'unlocked'", connector_name.c_str());
        }
    }
}

// void WebotsRobotInterface::connectorLockCmdCallback(const std_msgs::Bool::ConstPtr& msg) {
//     lockConnector()
// }

// void WebotsRobotInterface::setupLockCmdSubscriber(std::string connector_name, std::string cmd_topic)  {
//     connector_lock_sub_ = nh_->subscribe(cmd_topic, 1, &WebotsRobotInterface::connectorLockCmdCallback, this);
// }
