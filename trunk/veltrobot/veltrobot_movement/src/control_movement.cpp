#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include "pose.h"
#include "motion.h"

namespace veltrobot_movement
{

class ControlMovement
{
public:
  ControlMovement()
  : current_motion_("")
  , current_phase_("")
  {
    // Load Motion and Pose Libraries
    std::string pose_path, default_pose_path, motion_path, default_motion_path;
    std::string veltrobot_data_path = ros::package::getPath("veltrobot_data");
    default_pose_path = veltrobot_data_path + "/pose";
    default_motion_path = veltrobot_data_path + "/motion";
    n_.param<std::string>("pose_path", pose_path, default_pose_path);
    n_.param<std::string>("motion_path", motion_path, default_motion_path);
    poses_.loadDirectory(pose_path);
    motions_.loadDirectory(motion_path);        
    
    // Prepare joint state publisher
    joint_states_pub_ = n_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    
    // Prepare our subscription callbacks
    motion_name_sub_ = n_.subscribe("motion_name", 10, 
                                    &ControlMovement::motionNameCB, this);
    pose_name_sub_   = n_.subscribe("pose_name", 10,
                                    &ControlMovement::poseNameCB, this);
  }

  void spin()
  {
    ros::spin();
  }

private:
  ros::NodeHandle n_;
  PoseLibrary     poses_;
  MotionLibrary   motions_;
  ros::Publisher  joint_states_pub_;
  ros::Subscriber motion_name_sub_;
  ros::Subscriber pose_name_sub_;    
  ros::Timer      phase_timer_;
  std::string     current_motion_;
  std::string     current_phase_;

  void motionNameCB(const std_msgs::StringConstPtr& msg)
  {
    ROS_INFO("Requested motion [%s]", msg->data.c_str());
    std::string requested_motion = msg->data;
    //if (motions_.find(requested_motion) == motions_.end())
    //  return;
    if (requested_motion == current_motion_)
      return;
    current_motion_ = requested_motion;
    current_phase_ = motions_[current_motion_].first_phase_;
    phaseCB(ros::TimerEvent());
  }
  
  void poseNameCB(const std_msgs::StringConstPtr& msg)
  {
    ROS_INFO("Requested pose [%s]", msg->data.c_str());
    std::string requested_pose = msg->data;
    //if (poses_.find(requested_pose) == poses_.end())
    //    return;
    phase_timer_.stop();
    applyPoseToJointState(requested_pose);
  }
  
  void phaseCB(const ros::TimerEvent& e)
  {
    //ROS_INFO("phaseCB triggered");
    //if (motions_[current_motion_].phases.find(current_phase) == 
    MotionPhase& phase = motions_[current_motion_].phases_[current_phase_];
    float duration = -1;
    if (phase.poses_.size())
      duration = applyPoseToJointState(phase.poses_[0], phase.duration_);
    current_phase_ = phase.next_phase_;
    if (current_phase_ != "" && duration != -1)
      phase_timer_ = n_.createTimer(ros::Duration(duration / 1000.0f),
                                    &ControlMovement::phaseCB, this, true);
    //else
      // apply the idle motion
  }
  
  int applyPoseToJointState(const std::string& posestr, unsigned int duration=0)
  {
    //if (poses_.find(requested_pose) == poses_.end())
    //  return; 
    Pose& pose = poses_[posestr]; // thats the most convoluted line of code I have ever written :(
    if (pose.unit_ != Pose::radians_offset)
      return -1; // forget about the other units
    
    if (duration == 0)
      duration = pose.duration_;
    
    sensor_msgs::JointState js; 
    
    std::map<std::string, float>::iterator i = pose.positions_.begin();
    for(; i != pose.positions_.end(); ++i)
    {
      const std::string& joint_name = i->first;
      float position = i->second;

      js.name.push_back(joint_name);
      js.position.push_back(position);
      js.velocity.push_back(duration);
    }
    joint_states_pub_.publish(js);
    
    return duration;
  }
};

} // namespace veltrobot_movement

int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_movement");
  
  veltrobot_movement::ControlMovement this_node;
  this_node.spin();
  
  return 0;
}

