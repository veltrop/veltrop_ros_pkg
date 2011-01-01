#include <veltrobot_movement/posemanager.h>
#include <ros/package.h>
#include <std_msgs/String.h>

namespace veltrobot_movement
{

PoseManager::PoseManager()
{
  motion_name_pub_ = n_.advertise <std_msgs::String> ("motion_name", 10);
  pose_name_pub_ = n_.advertise <std_msgs::String> ("pose_name", 10);
  //servo_command_pub = n_.advertise<std_msgs::Int16>("servo_command", 10);
  reload();
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

}
