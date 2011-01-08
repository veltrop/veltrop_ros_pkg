#include <veltrobot_movement/posemanager.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

namespace veltrobot_movement
{

PoseManager::PoseManager(std::string joint_state_topic_)
{
  motion_name_pub_ = n_.advertise <std_msgs::String> ("motion_name", 10);
  pose_name_pub_ = n_.advertise <std_msgs::String> ("pose_name", 10);
  //joint_state_pub_ = n_.advertise<sensor_msgs::JointState>(joint_state_topic_, 10);
  //servo_command_pub = n_.advertise<std_msgs::Int16>("servo_command", 10);
  reload();
  
}

std::string PoseManager::getPosePath(const std::string& pose_name)
{
  std::string pose_path = ros::package::getPath("veltrobot_data") + "/pose";
	return pose_path + "/" + pose_name + ".xml";
}

void PoseManager::reload()
{
  poses_.clear();
  motions_.clear();

  // Load Motion and Pose Libraries
  std::string pose_path, default_pose_path, motion_path, default_motion_path;
  std::string veltrobot_data_path = ros::package::getPath("veltrobot_data");
  default_pose_path = veltrobot_data_path + "/pose";
  default_motion_path = veltrobot_data_path + "/motion";
  n_.param<std::string>("pose_path", pose_path, default_pose_path);
  n_.param<std::string>("motion_path", motion_path, default_motion_path);
  poses_.loadDirectory(pose_path);
  motions_.loadDirectory(motion_path);
}

void executePose(const std::string& pose_name)
{

}

void executeMotion(const std::string& motion_name)
{

}


}
