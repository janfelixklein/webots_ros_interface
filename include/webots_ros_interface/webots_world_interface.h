#include <ros/ros.h>
#include <std_msgs/String.h>

class WebotsWorldInterface
{
public:
    /**
    * \brief Constructor
    * \param nh_pointer   pointer to your node handle created in your main (ros::NodeHandle*)
    * \param modelTopicName name to subscribe to in order to get the controller names (std::string)
    */
    WebotsWorldInterface(ros::NodeHandle* nh_pointer, std::string modelTopicName);

	/**
    * \brief finds all active Webots controller and stores their names in the controller list
    */
	void findController();

	/**
    * \brief returns the controller name stored in the variable "controllerName_"
    */
	std::string getControllerName();

	/**
    * \brief returns a list of all active controller (controllerList)
    */
	std::vector<std::string> getControllerList();



private:

    /**
    * \brief Callback to receive controller names
    */
	void controllerNameCb(const std_msgs::String::ConstPtr &name);

	ros::NodeHandle* nh_;                       // Node Handle pointer
	std::string modelTopicName_;				// name of the topic on which webots is publishing the active controller names
	int controllerCount_;     		    		// number of active controller 
	std::string controllerName_;  				// string which holds a single controller name (the first active controller)
	ros::Subscriber nameSub_; 					// subscriber for the topic on which webots is publishing the active controller names
    std::vector<std::string> controllerList_;	// list of strings which holds all active webots controller

};