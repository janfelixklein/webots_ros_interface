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
bool WebotsRobotInterface::setUpMotorControl(std::string motorName, std::string controlType)   {

    if (controlType == "velocity")   {
        std::string MotorPositionServerPath = "/" + controllerName_ + "/" + motorName + "/set_position";
        std::string MotorVelocityServerPath = "/" + controllerName_ + "/" + motorName + "/set_velocity";
        ros::ServiceClient MotorPositionClient = nh_->serviceClient<webots_ros::set_float>(MotorPositionServerPath);
        ros::ServiceClient MotorVelocityClient = nh_->serviceClient<webots_ros::set_float>(MotorVelocityServerPath);

        if (!MotorPositionClient.waitForExistence(ros::Duration(0.5)))  {
            ROS_ERROR("Webots service not found, please check if your webots motor is named \"%s\"", motorName.c_str());
            return false;
        }
        ROS_INFO("Position and Velocity clients successfully set-up for motor \"%s\" on controller: \"%s\"", motorName.c_str(), controllerName_.c_str());

        MotorVelocityClients.insert(std::make_pair(motorName, MotorVelocityClient));

        //initialize velocity control
        webots_ros::set_float set_position_srv;
        set_position_srv.request.value = INFINITY;

        webots_ros::set_float set_velocity_srv;
        set_velocity_srv.request.value = 0.0;

        if (MotorPositionClient.call(set_position_srv) && set_position_srv.response.success)    {
            ROS_INFO("Position set to INFINITY for motor %s.", motorName.c_str());
        }
        else    {
            ROS_ERROR("Failed to call service set_position on motor \"%s\".", motorName.c_str());
            return false;
        }
        if (MotorVelocityClient.call(set_velocity_srv) && set_velocity_srv.response.success)    {
            ROS_INFO("Velocity set to 0.0 for motor %s.", motorName.c_str());
        }
        else    {
            ROS_ERROR("Failed to call service set_velocity on motor \"%s\".", motorName.c_str());
            return false;
        }
        return true;

    }
    else if (controlType == "position")  {
        std::string MotorPositionServerPath = "/" + controllerName_ + "/" + motorName + "/set_position";
        ros::ServiceClient MotorPositionClient = nh_->serviceClient<webots_ros::set_float>(MotorPositionServerPath);
        if (!MotorPositionClient.waitForExistence(ros::Duration(0.5)))  {
            ROS_ERROR("Webots service not found, please check if your webots motor is named \"%s\"", motorName.c_str());
            return false;
        }
        ROS_INFO("Position client successfully set-up for motor \"%s\" on controller: \"%s\"", motorName.c_str(), controllerName_.c_str());
        MotorPositionClients.insert(std::make_pair(motorName, MotorPositionClient));
        return true;
    }
    else {
        ROS_ERROR("ControlType wrongly specified, please specify either \"velocity\" for velocity-controlled motors or \"position\" for position-controlled motors");
        return false;
    }
}

void WebotsRobotInterface::sendMotorPosition(std::string motorName, double position)   {
    webots_ros::set_float set_position_srv;
    set_position_srv.request.value = position;
    MotorPositionClients[motorName].call(set_position_srv);
}

void WebotsRobotInterface::sendMotorVelocity(std::string motorName, double velocity)   {
    webots_ros::set_float set_velocity_srv;
    set_velocity_srv.request.value = velocity;
    MotorVelocityClients[motorName].call(set_velocity_srv);
}

void WebotsRobotInterface::setupWheelCmdSubscriber(std::string left_wheel_cmd_topic, std::string right_wheel_cmd_topic)  {
    left_wheel_sub = nh_->subscribe(left_wheel_cmd_topic, 1, &WebotsRobotInterface::leftWheelCmdCallback, this);
    right_wheel_sub = nh_->subscribe(right_wheel_cmd_topic, 1, &WebotsRobotInterface::rightWheelCmdCallback, this);
}

void WebotsRobotInterface::leftWheelCmdCallback(const std_msgs::Float32::ConstPtr& msg) {
    webots_ros::set_float WheelVelocitySrv;
    double data = msg->data;
    WheelVelocitySrv.request.value = 2*M_PI*data;
    MotorVelocityClients["left_wheel"].call(WheelVelocitySrv);
}

void WebotsRobotInterface::rightWheelCmdCallback(const std_msgs::Float32::ConstPtr& msg) {

    webots_ros::set_float WheelVelocitySrv;
    double data = msg->data;
    WheelVelocitySrv.request.value = 2*M_PI*data;
    MotorVelocityClients["right_wheel"].call(WheelVelocitySrv);
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
        ROS_WARN("Couldnt't enable device named %s ,please check if the service path %s is viable.", device_name.c_str(), device_path.c_str());
    }
    else    {
        ROS_INFO("Successfully enabled device named: %s", device_name.c_str());
    }
}


/**********************************************/
/*          OTHER FUNCTIONS                   */
/**********************************************/

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




void WebotsRobotInterface::broadcastTF(std::string parent_frame, std::string child_frame)  {
}
