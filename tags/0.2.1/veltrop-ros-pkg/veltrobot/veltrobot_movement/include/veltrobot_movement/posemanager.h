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
    PoseManager();

    PoseLibrary     poses_;
    MotionLibrary   motions_;

private:
    void reload();
    ros::NodeHandle n_;
    ros::Publisher motion_name_pub_;
    ros::Publisher pose_name_pub_;
    //ros::Publisher receive_servo_command_pub_;
};

}

#endif // POSEMANAGER_H
