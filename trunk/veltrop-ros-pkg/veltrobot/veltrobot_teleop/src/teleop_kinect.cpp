#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

namespace veltrobot_teleop
{

class TeleopKinect
{
public:
	TeleopKinect()
	{
  	ros::NodeHandle n;
	  motion_pub_ = n.advertise <std_msgs::String> ("motion_name", 1);
    joint_states_pub_ = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    
	}
	
	void spin()
	{
	  ros::spin();
	}
	
private:
	ros::Publisher  motion_pub_;
  ros::Publisher  joint_states_pub_;
};

} // namespace veltrobot_teleop

#define SAMPLE_XML_PATH "/home/space/ni/ni/openni/lib/SamplesConfig.xml"

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
		return nRetVal;\
	}

xn::Context        kinect_context_;
xn::DepthGenerator depth_generator_;
xn::UserGenerator  user_generator_;
XnBool             need_pose_ = FALSE;
XnChar 						 str_pose_[20] = "";
  
// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  printf("New User %d\n", nId);
  if (need_pose_)
    user_generator_.GetPoseDetectionCap().StartPoseDetection(str_pose_, nId);
  else
    user_generator_.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  printf("Lost user %d\n", nId);
}  

// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
  printf("Pose %s detected for user %d\n", strPose, nId);
  user_generator_.GetPoseDetectionCap().StopPoseDetection(nId);
  user_generator_.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
  printf("Calibration started for user %d\n", nId);
}

// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
  if (bSuccess)
  {
    // Calibration succeeded
    printf("Calibration complete, start tracking user %d\n", nId);
    user_generator_.GetSkeletonCap().StartTracking(nId);
  }
  else
  {
    // Calibration failed
    printf("Calibration failed for user %d\n", nId);
    if (need_pose_)
      user_generator_.GetPoseDetectionCap().StartPoseDetection(str_pose_, nId);
    else
      user_generator_.GetSkeletonCap().RequestCalibration(nId, TRUE);
  }
} 

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_joy");
  ros::NodeHandle n;
  
  XnStatus nRetVal = XN_STATUS_OK;
  nRetVal          = kinect_context_.InitFromXmlFile(SAMPLE_XML_PATH);
  CHECK_RC(nRetVal, "InitFromXml");
  
  nRetVal = kinect_context_.FindExistingNode(XN_NODE_TYPE_DEPTH, depth_generator_);
  CHECK_RC(nRetVal, "Find depth generator");
  nRetVal = kinect_context_.FindExistingNode(XN_NODE_TYPE_USER, user_generator_);
  if (nRetVal != XN_STATUS_OK)
  {
    nRetVal = user_generator_.Create(kinect_context_);
    CHECK_RC(nRetVal, "Find user generator");
  }

  XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks;
  if (!user_generator_.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
  {
    printf("Supplied user generator doesn't support skeleton\n");
    return 1;
  }
  user_generator_.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
	user_generator_.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

  if (user_generator_.GetSkeletonCap().NeedPoseForCalibration())
  {
    need_pose_ = TRUE;
    if (!user_generator_.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
    {
      printf("Pose required, but not supported\n");
      return 1;
    }
    user_generator_.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);
    user_generator_.GetSkeletonCap().GetCalibrationPose(str_pose_);
  }

  user_generator_.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

  nRetVal = kinect_context_.StartGeneratingAll();
  CHECK_RC(nRetVal, "StartGenerating");		  
  
  veltrobot_teleop::TeleopKinect this_node_;
	this_node_.spin();
  
	return 0;
}
