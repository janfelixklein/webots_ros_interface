#include <webots_ros_interface/connector.h>

Connector::Connector(ros::NodeHandle* nh_pointer, std::string controllerName, std::string connectorName, std::string cmdTopic)
    :nh_(nh_pointer)
    ,controllerName_(controllerName)
    ,connectorName_(connectorName)
    ,cmdTopic_(cmdTopic)
    ,lockStatus_(false)    
{
    init();
    if (cmdTopic_!="") {
        createSub();
    }
}

void Connector::init()  {
    lockPath_ = "/" + controllerName_ + "/" + connectorName_ + "/lock";
    lockClient_ = nh_->serviceClient<webots_ros::set_bool>(lockPath_);
}

bool Connector::setLock(bool locktype)  {
    webots_ros::set_bool lockRequest;
    lockRequest.request.value = locktype;
    if (!lockClient_.call(lockRequest)) {
        ROS_WARN("Couldnt't lock connector '%s' on robot '%s', please check if the service path %s is viable.", connectorName_.c_str(), controllerName_.c_str(), lockPath_.c_str());
        return false;
    }
    else    {
        if (locktype == true) {
            ROS_INFO("Changed the lock state of connector '%s' on robot '%s' to 'locked'", connectorName_.c_str(), controllerName_.c_str());
            lockStatus_ = true;
        }
        else    {
            ROS_INFO("Changed the lock state of connector '%s' on robto '%s' to 'unlocked'", connectorName_.c_str(), controllerName_.c_str());
            lockStatus_ = false;
        }
        return true;
    }
}

void Connector::createSub() {  
    cmdSub_ = nh_->subscribe(cmdTopic_, 1, &Connector::cmdCallback, this);
    ROS_INFO("Connector '%s' of robot '%s' is subscribing to cmd-topic '%s'.", connectorName_.c_str(), controllerName_.c_str(), cmdTopic_.c_str());  
}

void Connector::cmdCallback(const std_msgs::Bool::ConstPtr& msg) {
    bool lockRequest = msg->data;
    if (lockRequest != lockStatus_)    {
        setLock(lockRequest);
    }
    else {
        if (lockRequest == true)    {
            ROS_WARN("Connector '%s' of robot '%s' is already locked.", connectorName_.c_str(), controllerName_.c_str());
        }
        else if (lockRequest == false)  {
            ROS_WARN("Connector '%s' of robot '%s' is already unlocked.", connectorName_.c_str(), controllerName_.c_str());
        }            
    }
}



