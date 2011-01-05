#ifndef TELEOP_KINECT_H_
#define TELEOP_KINECT_H_

#include "KinectController.h"
#include <string>

using std::string;

namespace veltrobot_teleop
{

class KinectTeleop
{
	public:
		KinectTeleop();
		
		void init();
		void publishTransform(KinectController& kinect_controller,
		                      XnUserID const& user, XnSkeletonJoint const& joint,
													string const& frame_id, string const& child_frame_id);
		void processKinect(KinectController& kinect_controller);

	private:
		ros::Publisher   motion_pub_;
		ros::Publisher   joint_states_pub_;
		ros::Publisher   cmd_vel_pub_;
    ros::Subscriber   cmd_vel_pub_;
		bool             publish_kinect_tf_;
};

} // namespace veltrobot_teleop

#endif
