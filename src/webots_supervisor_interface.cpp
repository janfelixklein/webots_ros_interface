#include <webots_ros_interface/webots_supervisor_interface.h>
#include <webots_ros/supervisor_get_from_def.h>
#include <webots_ros/node_get_field.h>
#include <webots_ros/supervisor_get_from_def.h>
#include <webots_ros/field_get_vec3f.h>
#include <webots_ros/field_get_float.h>
#include <webots_ros/field_set_float.h>
#include <webots_ros/field_set_vec3f.h>
#include <webots_ros/field_set_rotation.h>
#include <webots_ros/field_get_count.h>
#include <webots_ros/get_uint64.h>
#include <webots_ros/field_get_rotation.h>
#include <webots_ros/node_get_position.h>



WebotsSupervisorInterface::WebotsSupervisorInterface (ros::NodeHandle* nh, std::string controllerName, std::string robotDEF)
    : nh_(nh)
    , robotDEF_(robotDEF) 
    , controllerName_(controllerName)
{
    ROS_INFO("Connected to the Supvisor Interface with controller: %s", controllerName_.c_str());
}
WebotsSupervisorInterface::WebotsSupervisorInterface (ros::NodeHandle* nh, std::string controllerName)
    : nh_(nh)
    , controllerName_(controllerName)
{
    ROS_INFO("Connected to the Supvisor Interface with controller: %s", controllerName_.c_str());
    getVec3fClient_ = nh_->serviceClient<webots_ros::field_get_vec3f>(controllerName_ + "/supervisor/field/get_vec3f");
    getFloatClient_ = nh->serviceClient<webots_ros::field_get_float>(controllerName_ + "/supervisor/field/get_float");
    setFloatClient_ = nh->serviceClient<webots_ros::field_set_float>(controllerName_ + "/supervisor/field/set_float");
    getRotationClient_ = nh_->serviceClient<webots_ros::field_get_rotation>(controllerName_ + "/supervisor/field/get_rotation");
}

void WebotsSupervisorInterface::setRobotPoseClients() {

    ros::ServiceClient getRobotClient = nh_->serviceClient<webots_ros::get_uint64>(controllerName_ + "/supervisor/get_self");
    //ros::ServiceClient getRobotClient = nh_->serviceClient<webots_ros::supervisor_get_from_def>(controllerName_ + "/supervisor/get_from_def");
    webots_ros::get_uint64 getRobotSrv;
    getRobotSrv.request.ask = 1;    
    getRobotClient.call(getRobotSrv);

    ros::ServiceClient FieldClient = nh_->serviceClient<webots_ros::node_get_field>(controllerName_ + "/supervisor/node/get_field");
    webots_ros::node_get_field getFieldSrv;

    // Get Translation field
    getFieldSrv.request.node = getRobotSrv.response.value;
    getFieldSrv.request.fieldName = "translation";      // get translation Field of robot name
    if (!FieldClient.call(getFieldSrv)) {
        ROS_ERROR("Couldnt receive field value");
    }
    else    {
        robotTranslationField_ = getFieldSrv.response.field;
        ROS_INFO("Controller %s has translation field #%lu", controllerName_.c_str(), getFieldSrv.response.field);
    }   

    // Get Rotation field
    getFieldSrv.request.node = getRobotSrv.response.value;
    getFieldSrv.request.fieldName = "rotation";         // get rotation Field of robot name
    if (!FieldClient.call(getFieldSrv)) {
        ROS_ERROR("Couldnt receive field value");
    }
    else    {
        robotRotationField_ = getFieldSrv.response.field;
        ROS_INFO("Controller %s has rotation field #%lu", controllerName_.c_str(), getFieldSrv.response.field);
    }   

    getVec3fClient_ = nh_->serviceClient<webots_ros::field_get_vec3f>(controllerName_ + "/supervisor/field/get_vec3f");
    setVec3fClient_ = nh_->serviceClient<webots_ros::field_set_vec3f>(controllerName_ + "/supervisor/field/set_vec3f");
    getRotationClient_ = nh_->serviceClient<webots_ros::field_get_rotation>(controllerName_ + "/supervisor/field/get_rotation");
    setRotationClient_ = nh_->serviceClient<webots_ros::field_set_rotation>(controllerName_ + "/supervisor/field/set_rotation");
    odomPub_ = nh_->advertise<nav_msgs::Odometry>("/" + controllerName_ + "/" + "SVodom", 1000);

}

geometry_msgs::Vector3 WebotsSupervisorInterface::getRobotTranslation()  {
    webots_ros::field_get_vec3f getRobotTranslationSrv;
    getRobotTranslationSrv.request.field = robotTranslationField_;

    if(!getVec3fClient_.call(getRobotTranslationSrv)){
        ROS_ERROR("Not able to return the robot's translation, since no supervisor client exists. Make sure that you have called setRobotPoseClients before");
    }
    else {
        robotTranslation_ = getRobotTranslationSrv.response.value;
    }
}

geometry_msgs::Point WebotsSupervisorInterface::getPositionFromNode(ros::ServiceClient* Client, unsigned int node)  {
    webots_ros::node_get_position getPositionSrv;
    getPositionSrv.request.node = node;
    if (Client->call(getPositionSrv))  {
        ROS_ERROR("Not able to return position from of node %u", node);
    }
    else {
        return getPositionSrv.response.position;
    }
}

geometry_msgs::Quaternion WebotsSupervisorInterface::getRobotRotation() {
    webots_ros::field_get_rotation getRobotRotationSrv;
    getRobotRotationSrv.request.field = robotRotationField_;

    if(!getRotationClient_.call(getRobotRotationSrv)){
        ROS_ERROR("Not able to return the robot's rotation, since no supervisor client exists. Make sure that you have called setRobotPoseClients before");
    }
    else {
        robotRotation_ = getRobotRotationSrv.response.value;
    }
}

void WebotsSupervisorInterface::setRobotTranslation(geometry_msgs::Vector3 requestedTranslation)   {
    webots_ros::field_set_vec3f setRobotTranslationSrv;
    setRobotTranslationSrv.request.field = robotTranslationField_;
    setRobotTranslationSrv.request.value = requestedTranslation;
    if(!setVec3fClient_.call(setRobotTranslationSrv)){
        ROS_ERROR("Not able to return the robot's translation, since no supervisor client exists. Make sure that you have called setRobotPoseClients before");
    }
    else {
        ROS_INFO("Set tranlsation of controller %s to x: %f  y:%f  z:%f", controllerName_.c_str(),requestedTranslation.x, requestedTranslation.y, requestedTranslation.z);
    }
}

void WebotsSupervisorInterface::setRobotRotation(geometry_msgs::Quaternion requestedRotation)   {
    webots_ros::field_set_rotation setRobotRotationSrv;
    setRobotRotationSrv.request.field = robotRotationField_;
    setRobotRotationSrv.request.value = requestedRotation;
    if(!setRotationClient_.call(setRobotRotationSrv)){
        ROS_ERROR("Not able to return the robot's translation, since no supervisor client exists. Make sure that you have called setRobotPoseClients before");
    }
    else {
        ROS_INFO("Set rotation of controller %s to x: %f  y:%f  z:%f  w:%f", controllerName_.c_str(),requestedRotation.x, requestedRotation.y, requestedRotation.z, requestedRotation.w);
    }
}

void WebotsSupervisorInterface::odometryUpdate(boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub, unsigned int translation_field, unsigned int rotation_field)  {
    //nav_msgs::Odometry odomMsg;
    geometry_msgs::Vector3 translation;
    geometry_msgs::Quaternion rotation;
    
    translation = getVec3f(translation_field);
    rotation = getRotation(rotation_field);
    
    //getRobotTranslation();
    //getRobotRotation();
    // odomMsg.header.stamp = ros::Time::now();
    // odomMsg.pose.pose.position.x = translation.x;
    // odomMsg.pose.pose.position.y = translation.y;
    // odomMsg.pose.pose.position.z = translation.z;
    // odomMsg.pose.pose.orientation.x = rotation.x;
    // odomMsg.pose.pose.orientation.y = rotation.y;
    // odomMsg.pose.pose.orientation.z = rotation.z;
    // odomMsg.pose.pose.orientation.w = rotation.w;

    // odom_pub->publish(odomMsg);


    if (odom_pub->trylock())
    {
      odom_pub ->msg_.header.stamp = ros::Time::now();
      odom_pub ->msg_.pose.pose.position.x = translation.x;
      odom_pub ->msg_.pose.pose.position.y = translation.y;
      odom_pub ->msg_.pose.pose.position.z = translation.z;
      odom_pub ->msg_.pose.pose.orientation.x = rotation.x;
      odom_pub ->msg_.pose.pose.orientation.y = rotation.y;
      odom_pub ->msg_.pose.pose.orientation.z = rotation.z;
      odom_pub ->msg_.pose.pose.orientation.w = rotation.w;
      odom_pub ->unlockAndPublish();                                 
    }
}



unsigned int WebotsSupervisorInterface::getNodeFromDef(std::string defName)    {
    ros::ServiceClient getNodeClient = nh_->serviceClient<webots_ros::supervisor_get_from_def>(controllerName_ + "/supervisor/get_from_def");
    webots_ros::supervisor_get_from_def getNodeSrv;
    getNodeSrv.request.name = defName;
    getNodeClient.call(getNodeSrv);
    if (!getNodeClient.call(getNodeSrv)) {
        ROS_ERROR("Couldn't retrieve node value from def: %s", defName.c_str());
    }
    else    {
        return getNodeSrv.response.node;
    } 
}

unsigned int WebotsSupervisorInterface::getNodeFromSelf()   {
    ros::ServiceClient getRobotNodeClient = nh_->serviceClient<webots_ros::get_uint64>(controllerName_ + "/supervisor/get_self");
    webots_ros::get_uint64 getRobotSrv;
    getRobotSrv.request.ask = 1;  
    if (!getRobotNodeClient.call(getRobotSrv)) {
        ROS_ERROR("Couldnt retrieve self node value for controller: %s", controllerName_.c_str());
    }
    else    {
        return getRobotSrv.response.value;
    } 
}



unsigned int WebotsSupervisorInterface::getField(unsigned int node, std::string fieldName)  {
    ros::ServiceClient getFieldClient = nh_->serviceClient<webots_ros::node_get_field>(controllerName_ + "/supervisor/node/get_field");
    webots_ros::node_get_field getFieldSrv;
    getFieldSrv.request.node = node;
    getFieldSrv.request.fieldName = fieldName;
    if (!getFieldClient.call(getFieldSrv)) {
        ROS_ERROR("Couldnt retrieve field value from field name: %s", fieldName.c_str());
    }
    else    {
        return getFieldSrv.response.field;
    } 
}
int WebotsSupervisorInterface::getCount(unsigned int field) {
    ros::ServiceClient getCountClient = nh_->serviceClient<webots_ros::field_get_count>(controllerName_ + "supervisor/field/get_count");
    webots_ros::field_get_count getCountSrv;
    getCountSrv.request.field = field;
    if (!getCountClient.call(getCountSrv)) {
        ROS_ERROR("Couldnt retrieve node count from field: %u", field);
    }
    else    {
        return getCountSrv.response.count;
    }
}






geometry_msgs::Vector3 WebotsSupervisorInterface::getVec3f(unsigned int field)  {
    //ros::ServiceClient getVec3fClient = nh_->serviceClient<webots_ros::field_get_vec3f>(controllerName_ + "/supervisor/field/get_vec3f");
    webots_ros::field_get_vec3f getVec3fSrv;
    getVec3fSrv.request.field = field;
    if (!getVec3fClient_.call(getVec3fSrv))  {
        ROS_ERROR("Couldn't retrieve Vector3f from field %u", field);
    }
    else    {
        return getVec3fSrv.response.value;
    }
}

double WebotsSupervisorInterface::getFloat(unsigned int field)   {
    webots_ros::field_get_float getFloatSrv;
    getFloatSrv.request.field = field;
    if (!getFloatClient_.call(getFloatSrv))  {
        ROS_ERROR("Couldn't retrieve float from field %u", field);
    }
    else    {
        return getFloatSrv.response.value;
    }


}

int WebotsSupervisorInterface::setFloat(unsigned int field, double value) {
    webots_ros::field_set_float setFloatSrv;
    setFloatSrv.request.field = field;
    setFloatSrv.request.value = value;
    if (!setFloatClient_.call(setFloatSrv))  {
        ROS_ERROR("Couldn't set float %f to field %u",value, field);
    }
    else    {
        return setFloatSrv.response.success;
    }

}

geometry_msgs::Quaternion WebotsSupervisorInterface::getRotation(unsigned int field) {
    webots_ros::field_get_rotation getRotationSrv;
    getRotationSrv.request.field = field;
    getRotationSrv.request.index = 0;

    if(!getRotationClient_.call(getRotationSrv)){
        ROS_ERROR("Couldn't retrieve rotation from field %u", field);
    }
    else {
        return getRotationSrv.response.value;
    }

}

void WebotsSupervisorInterface::setRotation(ros::ServiceClient* setRotationClient, unsigned int field, geometry_msgs::Quaternion requestedRotation) {
    webots_ros::field_set_rotation setRobotRotationSrv;
    setRobotRotationSrv.request.field = field;
    setRobotRotationSrv.request.value = requestedRotation;
    if(!setRotationClient->call(setRobotRotationSrv))    {
        ROS_ERROR("Couldn't set requested rotation for field %u, make sure the Client is valid", field);
        return;
    }
}


void WebotsSupervisorInterface::calcOrientationDifference(std::string parent_frame, std::string child_frame)    {
    
    unsigned int NodeID = getNodeFromDef(parent_frame);
    ROS_INFO("Node found with ID %u", NodeID);
    unsigned int fieldID = getField(NodeID, "rotation");
    ROS_INFO("Field found with ID %u", fieldID);
    geometry_msgs::Quaternion robot_rotation;
    robot_rotation = getRotation(fieldID);
    ROS_INFO("Rotation of main frame %s: %f, %f, %f, %f",parent_frame.c_str(), robot_rotation.x, robot_rotation.y, robot_rotation.z, robot_rotation.w);

    unsigned int NodeIDChild = getNodeFromDef(child_frame);
    ROS_INFO("Node found with ID %u", NodeIDChild);
    unsigned int fieldIDChild = getField(NodeIDChild, "rotation");
    ROS_INFO("Field found with ID %u", fieldIDChild);
    geometry_msgs::Quaternion child_rotation;
    child_rotation = getRotation(fieldIDChild);
    ROS_INFO("Rotation of child frame %s: %f, %f, %f, %f",child_frame.c_str(), child_rotation.x, child_rotation.y, child_rotation.z, child_rotation.w);
     
     
    //n = wb_supervisor_field_get_count(children);    

}

void WebotsSupervisorInterface::broadcastTF(std::string parent_frame, std::string child_frame)  {

  geometry_msgs::Vector3 translation_gm;
  geometry_msgs::Quaternion rotation_gm;


  rotation_gm.y = robotRotation_.x;// * std::sin(M_PI/2);
  rotation_gm.z = robotRotation_.y;// * std::sin(M_PI/2);
  rotation_gm.x = robotRotation_.z;// * std::sin(M_PI/2);
  rotation_gm.w = robotRotation_.w;// * std::cos(M_PI/2);

  translation_gm.y = robotTranslation_.x;
  translation_gm.z = robotTranslation_.y;
  translation_gm.x = robotTranslation_.z;

//  ROS_INFO("%f, %f, %f, %f", rotation_gm.x, rotation_gm.y, rotation_gm.z, rotation_gm.w);

  tf::Vector3 translation;
  tf::Quaternion rotation;
  vector3MsgToTF(translation_gm, translation);
  quaternionMsgToTF(rotation_gm, rotation);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(translation);
  transform.setRotation(rotation);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame));
}
