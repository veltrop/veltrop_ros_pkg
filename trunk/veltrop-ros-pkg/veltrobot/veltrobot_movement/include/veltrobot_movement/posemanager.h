#ifndef POSEMANAGER_H
#define POSEMANAGER_H

#include <ros/ros.h>
#include "pose.h"
#include "motion.h"

namespace veltrobot_movement
{

class PoseManager
{
public:
    PoseManager(std::string joint_state_topic_="");

    PoseLibrary     poses_;
    MotionLibrary   motions_;
    void reload();
    
    std::string getPosePath(const std::string& pose_name);

		void executePose(const std::string& pose_name);
    void executeMotion(const std::string& motion_name);
    
private:
    ros::NodeHandle n_;
    ros::Publisher motion_name_pub_;
    ros::Publisher pose_name_pub_;
    ros::Publisher joint_state_pub_;
    //ros::Publisher receive_servo_command_pub_;
};

}

#endif // POSEMANAGER_H
