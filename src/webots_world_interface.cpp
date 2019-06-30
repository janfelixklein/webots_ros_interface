#include <webots_ros_interface/webots_world_interface.h>

WebotsWorldInterface::WebotsWorldInterface(ros::NodeHandle* nh_pointer, std::string modelTopicName)
: nh_(nh_pointer)
, modelTopicName_(modelTopicName)
, controllerCount_(0)
{
    findController();
}

void WebotsWorldInterface::findController()    {
    ROS_INFO("Subscribing to topic \"/%s\" to receive available controller names", modelTopicName_.c_str());  
    nameSub_ = nh_->subscribe(modelTopicName_, 1000, &WebotsWorldInterface::controllerNameCb, this);

    int num_controller = 1;         //number of controller 
    bool num_controller_specified = false;  
    if (!nh_->getParam("num_controller", num_controller))
	{
		ROS_WARN("Parameter 'number_of_webots_controller' is not specified, this could result in an incorrect number of found controller");
	}
    else
    {
        ROS_INFO("Expecting to find a total number of #%i running webots controller", num_controller);
        num_controller_specified = true;
    }
    
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(10.0); // Timeout after 10 seconds if no controller were registered
    //ROS_INFO("Number of Publishers %ui",nameSub_.getNumPublishers());
    
    if (num_controller_specified==true)   {
        while(controllerCount_ != num_controller) {     
            //ROS_INFO("Number of Publishers %f",nameSub_.getNumPublishers())
            ros::spinOnce();
        }
        ros::spinOnce();
    }
    else {
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
    }    
    if (controllerCount_ == 0) {      
        ROS_ERROR("Didn't find a controller after waiting for %i seconds --> terminating", static_cast<int>(timeout.toSec()));
        exit(1);
    }  
    else if (controllerCount_ == 1) {
        controllerName_ = controllerList_[0];
        nh_->setParam("/webots_robot_1/name", controllerName_.c_str());
    }  
    else {
        int size = static_cast<int>(controllerList_.size());
        ROS_INFO("Found a total number of %i active controllers in your webots world", size);
        for (int i=0; i<size; i++) {
            nh_->setParam("/webots_robot_" + std::to_string(i+1) + "/name" , controllerList_[i].c_str());
        } 
    } 

    // leave topic once it's not necessary anymore
    nameSub_.shutdown();
}


void WebotsWorldInterface::controllerNameCb(const std_msgs::String::ConstPtr &name)   {
    controllerCount_++;
    controllerList_.push_back(name->data);
    ROS_INFO("Controller #%d: %s.", controllerCount_, controllerList_.back().c_str());
}


std::string WebotsWorldInterface::getControllerName()    {
    return controllerName_;
}


std::vector<std::string>  WebotsWorldInterface::getControllerList()  {
    return controllerList_;
}
