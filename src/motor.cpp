#include <webots_ros_interface/motor.h>

Motor::Motor(ros::NodeHandle* nh_pointer, std::string controllerName, std::string motorName, std::string controlType,  std::string cmdTopic)
    :nh_(nh_pointer)
    ,controllerName_(controllerName)
    ,motorName_(motorName)
    ,controlType_(controlType)
    ,cmdTopic_(cmdTopic)    
{
    init();
    if (cmdTopic_!="") {
        createSub();
    }
}

void Motor::init()  {
    // create Service paths from given parameters
    motorPositionServicePath_ = "/" + controllerName_ + "/" + motorName_ + "/set_position";
    motorVelocityServicePath_ = "/" + controllerName_ + "/" + motorName_ + "/set_velocity";
    
    // initialize clients
    if (controlType_=="velocity")    {
        if (!initPositionClient())    {
            throw std::invalid_argument("Position client couldn't be set up correctly");
        };
        if (!initVelocityControl()) {
            throw std::invalid_argument("Velocity control couldn't be initialized");
        };
    }
    else if (controlType_=="position")  {
        if (!initPositionClient())    {
            throw std::invalid_argument("Position client couldn't be set up correctly");
        };
    }
    else {
        ROS_ERROR("Control type of motor %s is wrongly specified, it's need to be either 'velocity' or 'position'.", motorName_.c_str());
    }
}

bool Motor::initPositionClient()   {
    MotorPositionClient_ = nh_->serviceClient<webots_ros::set_float>(motorPositionServicePath_);
    if (!MotorPositionClient_.waitForExistence(ros::Duration(0.5)))  {
        ROS_ERROR("Webots service not found, please check if your webots motor is named \"%s\"", motorName_.c_str());
        return false;
    }
    else    {
        return true;
    }
}

bool Motor::initVelocityClient()   {
    MotorVelocityClient_ = nh_->serviceClient<webots_ros::set_float>(motorVelocityServicePath_);
    if (!MotorVelocityClient_.waitForExistence(ros::Duration(0.5)))  {
        ROS_ERROR("Webots service not found, please check if your webots motor is named \"%s\"", motorName_.c_str());
        return false;
    }
    else    {
        return true;
    }
}

bool Motor::initVelocityControl()   {
    if (!initVelocityClient())  {
        return false;
    };
    sendPosition(INFINITY);
    sendVelocity(0.0);
    ROS_INFO("Successfully set up '%s'-control of motor '%s' on robot '%s'", controlType_.c_str(), motorName_.c_str(), controllerName_.c_str());
    return true;        
}

void Motor::sendPosition(double position)   {
    webots_ros::set_float set_position_srv;
    set_position_srv.request.value = position;
    if (!MotorPositionClient_.call(set_position_srv))   {
        ROS_WARN("Failed to send position command to motor '%s' on robot '%s'", motorName_.c_str(), controllerName_.c_str());
    }
}

void Motor::sendVelocity(double velocity)   {
    webots_ros::set_float set_velocity_srv;
    set_velocity_srv.request.value = velocity;
    MotorVelocityClient_.call(set_velocity_srv);
    //ROS_INFO("Got velocity %f on motor %s",velocity, motorName_.c_str());
    if (!MotorVelocityClient_.call(set_velocity_srv))   {
         ROS_WARN("Failed to send velocity command to motor '%s' on robot '%s'", motorName_.c_str(), controllerName_.c_str());    
    }
}

void Motor::createSub() {
    cmdSub_ = nh_->subscribe(cmdTopic_, 1, &Motor::cmdCallback, this);
    ROS_INFO("Motor '%s' of robot '%s' is subscribing to cmd-topic '%s'.", motorName_.c_str(), controllerName_.c_str(), cmdTopic_.c_str());
}

void Motor::cmdCallback(const std_msgs::Float32::ConstPtr& msg) {
    if (controlType_ == "velocity") {
         sendVelocity(msg->data);
    }
    else if (controlType_ == "position")    {
         sendPosition(msg->data);
    }
}
