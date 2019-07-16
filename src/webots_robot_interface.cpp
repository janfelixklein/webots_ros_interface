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

void WebotsRobotInterface::setUpConnector(std::string connectorName, std::string cmdTopic="") {
    // enable presence sensor
    enableDevice(connectorName + "/presence_sensor");
    // create connector object
    boost::shared_ptr< Connector > connector( new Connector(nh_, controllerName_, connectorName, cmdTopic) );
    connectors[connectorName] = connector;
}
