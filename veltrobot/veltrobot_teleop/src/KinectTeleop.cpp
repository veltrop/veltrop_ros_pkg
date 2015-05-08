#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <veltrobot_msgs/EnableJointGroup.h>
#include "KinectTeleop.h"
#include <string>

#ifndef PI
#define PI 3.14159265359
#endif
#ifndef HALFPI
#define HALFPI 1.57079632679
#endif 
#ifndef QUARTPI
#define QUARTPI 0.785398163397
#endif

using std::string;

namespace veltrobot_teleop
{

KinectTeleop::KinectTeleop()
: publish_kinect_tf_(false)
, right_arm_enabled_(false)
, left_arm_enabled_(false)
, legs_enabled_(false)
, motion_enabled_(false)
, arm_control_method_(DIRECT)
{    
}
      
void KinectTeleop::init()
{
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	//motion_pub_ = n.advertise <std_msgs::String> ("motion_name", 1);
	joint_states_pub_ = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  left_arm_destination_pub_ = n.advertise<geometry_msgs::Point>("left_arm_destination", 1);
  right_arm_destination_pub_ = n.advertise<geometry_msgs::Point>("right_arm_destination", 1);	
  //cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);	
  torso_destination_pub_ = n.advertise<geometry_msgs::Point>("torso_destination", 1);
  pointing_destination_pub_ = n.advertise<geometry_msgs::Point>("pointing_destination", 1);
  
  np.param<bool>("publish_kinect_tf", publish_kinect_tf_, false);  
  np.param<bool>("force_left_arm_enabled", left_arm_enabled_, false);
  np.param<bool>("force_right_arm_enabled", right_arm_enabled_, false);
  np.param<bool>("force_legs_enabled", legs_enabled_, false);

  enable_joint_group_sub_ = n.subscribe("enable_joint_group", 10,
                                        &KinectTeleop::enableJointGroupCB, this);
  arm_control_method_sub_ = n.subscribe("arm_control_method", 10,
                                        &KinectTeleop::armControlMethodCB, this);
}

void KinectTeleop::armControlMethodCB(const std_msgs::StringConstPtr& msg)
{
  if (msg->data == "IK")
    arm_control_method_ = IK;
  else
    arm_control_method_ = DIRECT;
}

void KinectTeleop::enableJointGroupCB(const veltrobot_msgs::EnableJointGroupConstPtr& msg)
{
	for (size_t i=0; i < msg->jointGroups.size(); i++)
  {
  	if (msg->jointGroups[i] == "legs")
		{
			legs_enabled_ = msg->enabledStates[i];
		}
		else if (msg->jointGroups[i] == "arms")
		{
			right_arm_enabled_ = left_arm_enabled_ = msg->enabledStates[i];
		}
		else if (msg->jointGroups[i] == "left_arm")
		{
		  left_arm_enabled_ = msg->enabledStates[i];
		}	
		else if (msg->jointGroups[i] == "right_arm")
		{
			right_arm_enabled_ = msg->enabledStates[i];
		}
		else if (msg->jointGroups[i] == "motions")
		{
			motion_enabled_ = msg->enabledStates[i];      
  	}
	}
}
      
void KinectTeleop::publishTransform(KinectController& kinect_controller,
                                    XnUserID const& user, XnSkeletonJoint const& joint,
																		string const& frame_id, string const& child_frame_id)
{
	xn::UserGenerator& UserGenerator = kinect_controller.getUserGenerator();
	static tf::TransformBroadcaster br;

	XnSkeletonJointPosition joint_position;
	UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
	double x = joint_position.position.X / 1000.0;
	double y = joint_position.position.Y / 1000.0;
	double z = joint_position.position.Z / 1000.0;

	XnSkeletonJointOrientation joint_orientation;
	UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

	XnFloat* m = joint_orientation.orientation.elements;
	KDL::Rotation rotation(m[0], m[1], m[2],
							           m[3], m[4], m[5],
							           m[6], m[7], m[8]);
	double qx, qy, qz, qw;
	rotation.GetQuaternion(qx, qy, qz, qw);

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, z));
	transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
}
        
void KinectTeleop::processKinect(KinectController& kinect_controller)
{
	static double last_vel_linear_x = 0.0;
	static double last_vel_angular_z = 0.0;
	
	XnUserID users[2];
	XnUInt16 users_count = 2;
	xn::UserGenerator& UserGenerator = kinect_controller.getUserGenerator();
	UserGenerator.GetUsers(users, users_count);

	for (int i = 0; i < users_count; ++i)
	{
		XnUserID user = users[i];
		if (!UserGenerator.GetSkeletonCap().IsTracking(user))
			continue;
		
		if (publish_kinect_tf_)
		{
			string frame_id("openni_depth_frame");

			publishTransform(kinect_controller, user, XN_SKEL_HEAD,           frame_id, "head");
			publishTransform(kinect_controller, user, XN_SKEL_NECK,           frame_id, "neck");
			publishTransform(kinect_controller, user, XN_SKEL_TORSO,          frame_id, "torso");

			publishTransform(kinect_controller, user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder");
			publishTransform(kinect_controller, user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow");
			publishTransform(kinect_controller, user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand");

			publishTransform(kinect_controller, user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
			publishTransform(kinect_controller, user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow");
			publishTransform(kinect_controller, user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand");

			publishTransform(kinect_controller, user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip");
			publishTransform(kinect_controller, user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee");
			publishTransform(kinect_controller, user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot");

			publishTransform(kinect_controller, user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip");
			publishTransform(kinect_controller, user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee");
			publishTransform(kinect_controller, user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot");  
		}
		
		/////
		// Input Joint Positions And Orientations from Kinect																														
		/////
		
		XnFloat* m;
		
		// Upper Joints positions
		//XnSkeletonJointPosition joint_position_head;
		//UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_HEAD, joint_position_head);
		//KDL::Vector head(joint_position_head.position.X, joint_position_head.position.Y, joint_position_head.position.Z);
		
		//XnSkeletonJointPosition joint_position_neck;
		//UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_NECK, joint_position_neck);
		//KDL::Vector neck(joint_position_neck.position.X, joint_position_neck.position.Y, joint_position_neck.position.Z);
		
		XnSkeletonJointPosition joint_position_torso;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_TORSO, joint_position_torso);
		KDL::Vector torso(joint_position_torso.position.X, joint_position_torso.position.Y, joint_position_torso.position.Z);
							
	  /*
    // Upper joints rotations
    XnSkeletonJointOrientation joint_orientation_head;
		UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_HEAD, joint_orientation_head);
		m = joint_orientation_head.orientation.elements;
		KDL::Rotation head_rotation(m[0], m[1], m[2],
																m[3], m[4], m[5],
																m[6], m[7], m[8]);
	  double head_roll, head_pitch, head_yaw;
	  head_rotation.GetRPY(head_roll, head_pitch, head_yaw);
		
	  XnSkeletonJointOrientation joint_orientation_neck;
		UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_NECK, joint_orientation_neck);
		m = joint_orientation_neck.orientation.elements;
	  KDL::Rotation neck_rotation(m[0], m[1], m[2],
																m[3], m[4], m[5],
																m[6], m[7], m[8]);
		double neck_roll, neck_pitch, neck_yaw;
	  neck_rotation.GetRPY(neck_roll, neck_pitch, neck_yaw);		
	  */	

	  XnSkeletonJointOrientation joint_orientation_torso;
		UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_TORSO, joint_orientation_torso);
	  m = joint_orientation_torso.orientation.elements;
	  KDL::Rotation torso_rotation(m[0], m[1], m[2],
																 m[3], m[4], m[5],
																 m[6], m[7], m[8]);
		double torso_roll, torso_pitch, torso_yaw;
	  torso_rotation.GetRPY(torso_roll, torso_pitch, torso_yaw);	
    
    // We need this rotation to correct the rest into the users own coordinate system
    KDL::Rotation torso_rotation_inverse = torso_rotation.Inverse();

    // get right shoulder orientation in kinects default cordinate system
    XnSkeletonJointOrientation joint_orientation_right_shoulder;
		UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_SHOULDER, joint_orientation_right_shoulder);
    m = joint_orientation_right_shoulder.orientation.elements;
	  KDL::Rotation right_shoulder_rotation(m[0], m[1], m[2],
																 m[3], m[4], m[5],
																 m[6], m[7], m[8]);
		
    // Remove torso rotation and extract angles
    right_shoulder_rotation = right_shoulder_rotation * torso_rotation_inverse;
    double right_shoulder_roll, right_shoulder_pitch, right_shoulder_yaw;
	  right_shoulder_rotation.GetRPY(right_shoulder_roll, right_shoulder_pitch, right_shoulder_yaw);				

    // get left shoulder orientation in kinect's coord sys
    XnSkeletonJointOrientation joint_orientation_left_shoulder;
		UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_SHOULDER, joint_orientation_left_shoulder);
    m = joint_orientation_left_shoulder.orientation.elements;
	  KDL::Rotation left_shoulder_rotation(m[0], m[1], m[2],
																 m[3], m[4], m[5],
																 m[6], m[7], m[8]);
    
    // Remove torso rotation and extract angles
    left_shoulder_rotation = left_shoulder_rotation * torso_rotation_inverse;
		double left_shoulder_roll, left_shoulder_pitch, left_shoulder_yaw;
    left_shoulder_rotation.GetRPY(left_shoulder_roll, left_shoulder_pitch, left_shoulder_yaw);				

		// Left Arm 
		XnSkeletonJointPosition joint_position_left_hand;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HAND, joint_position_left_hand);
		KDL::Vector left_hand(joint_position_left_hand.position.X, joint_position_left_hand.position.Y, joint_position_left_hand.position.Z);
    // torso relative coordinates
    left_hand = left_hand - torso;
    left_hand = torso_rotation_inverse * left_hand;

		XnSkeletonJointPosition joint_position_left_elbow;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_ELBOW, joint_position_left_elbow);
		KDL::Vector left_elbow(joint_position_left_elbow.position.X, joint_position_left_elbow.position.Y, joint_position_left_elbow.position.Z);        
    // torso relative coordinates
    left_elbow = left_elbow - torso;
    left_elbow = torso_rotation_inverse * left_elbow;

		XnSkeletonJointPosition joint_position_left_shoulder;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_SHOULDER, joint_position_left_shoulder);
		KDL::Vector left_shoulder(joint_position_left_shoulder.position.X, joint_position_left_shoulder.position.Y, joint_position_left_shoulder.position.Z);        	        	
    // torso relative coordinates
    left_shoulder = left_shoulder - torso;
    left_shoulder = torso_rotation_inverse * left_shoulder;
		
		// Right Arm
		XnSkeletonJointPosition joint_position_right_hand;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HAND, joint_position_right_hand);
		KDL::Vector right_hand(joint_position_right_hand.position.X, joint_position_right_hand.position.Y, joint_position_right_hand.position.Z);
    // torso relative coordinates
    right_hand = right_hand - torso;
    right_hand = torso_rotation_inverse * right_hand;
		
		XnSkeletonJointPosition joint_position_right_elbow;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_ELBOW, joint_position_right_elbow);
		KDL::Vector right_elbow(joint_position_right_elbow.position.X, joint_position_right_elbow.position.Y, joint_position_right_elbow.position.Z);        
    // torso relative coordinates
    right_elbow = right_elbow - torso;
    right_elbow = torso_rotation_inverse * right_elbow;
		
		XnSkeletonJointPosition joint_position_right_shoulder;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_SHOULDER, joint_position_right_shoulder);
		KDL::Vector right_shoulder(joint_position_right_shoulder.position.X, joint_position_right_shoulder.position.Y, joint_position_right_shoulder.position.Z);
    // torso relative coordinates
    right_shoulder = right_shoulder - torso;
    right_shoulder = torso_rotation_inverse * right_shoulder;
		
		// Right Leg
		XnSkeletonJointPosition joint_position_right_hip;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HIP, joint_position_right_hip);
		KDL::Vector right_hip(joint_position_right_hip.position.X, joint_position_right_hip.position.Y, joint_position_right_hip.position.Z);
    // torso relative coordinates
    right_hip = right_hip - torso;
    right_hip = torso_rotation_inverse * right_hip;

		XnSkeletonJointPosition joint_position_right_knee;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_KNEE, joint_position_right_knee);
		KDL::Vector right_knee(joint_position_right_knee.position.X, joint_position_right_knee.position.Y, joint_position_right_knee.position.Z);
    // torso relative coordinates
    right_knee = right_knee - torso;
    right_knee = torso_rotation_inverse * right_knee;

		XnSkeletonJointPosition joint_position_right_foot;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_FOOT, joint_position_right_foot);
		KDL::Vector right_foot(joint_position_right_foot.position.X, joint_position_right_foot.position.Y, joint_position_right_foot.position.Z);
    // torso relative coordinates
    right_foot = right_foot - torso;
    right_foot = torso_rotation_inverse * right_foot;

		// Left Leg
		XnSkeletonJointPosition joint_position_left_hip;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HIP, joint_position_left_hip);
		KDL::Vector left_hip(joint_position_left_hip.position.X, joint_position_left_hip.position.Y, joint_position_left_hip.position.Z);
    // torso relative coordinates
    left_hip = left_hip - torso;
    left_hip = torso_rotation_inverse * left_hip;

		XnSkeletonJointPosition joint_position_left_knee;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_KNEE, joint_position_left_knee);
		KDL::Vector left_knee(joint_position_left_knee.position.X, joint_position_left_knee.position.Y, joint_position_left_knee.position.Z);
    // torso relative coordinates
    left_knee = left_knee - torso;
    left_knee = torso_rotation_inverse * left_knee;

		XnSkeletonJointPosition joint_position_left_foot;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_FOOT, joint_position_left_foot);
		KDL::Vector left_foot(joint_position_left_foot.position.X, joint_position_left_foot.position.Y, joint_position_left_foot.position.Z);
    // torso relative coordinates
    left_foot = left_foot - torso;
    left_foot = torso_rotation_inverse * left_foot;
		
    // the kinect and robot are in different rotation spaces, but at least 
    // they are still axis aligned so its easy to correct... but I should
    // have made correct transforms in the launch file...

    /////
		// Process and output joint rotations to the robot.																														
		/////
    
    // ARMS for robot
  																																																																									
		// left elbow roll
		KDL::Vector left_elbow_hand(left_hand - left_elbow);
		KDL::Vector left_elbow_shoulder(left_shoulder - left_elbow);
		left_elbow_hand.Normalize();
		left_elbow_shoulder.Normalize();
    //left_elbow_hand = torso_rotation_inverse * left_elbow_hand;
    //left_elbow_shoulder = torso_rotation_inverse * left_elbow_shoulder;
		static double left_elbow_angle_roll = 0;
		if (joint_position_left_hand.fConfidence >= 0.5 && 
				joint_position_left_elbow.fConfidence >= 0.5 && 
				joint_position_left_shoulder.fConfidence >= 0.5)
		{
			left_elbow_angle_roll = acos(KDL::dot(left_elbow_hand, left_elbow_shoulder));
			left_elbow_angle_roll = left_elbow_angle_roll - PI;
		}
		
		// right elbow roll
		KDL::Vector right_elbow_hand(right_hand - right_elbow);
		KDL::Vector right_elbow_shoulder(right_shoulder - right_elbow);
		right_elbow_hand.Normalize();
		right_elbow_shoulder.Normalize();
		static double right_elbow_angle_roll = 0;
		if (joint_position_right_hand.fConfidence >= 0.5 && 
				joint_position_right_elbow.fConfidence >= 0.5 && 
				joint_position_right_shoulder.fConfidence >= 0.5)
		{          
			right_elbow_angle_roll = acos(KDL::dot(right_elbow_hand, right_elbow_shoulder));
			right_elbow_angle_roll = -(right_elbow_angle_roll - PI);
		}  
						
		// left shoulder roll
		KDL::Vector left_shoulder_elbow(left_elbow - left_shoulder);
		KDL::Vector left_shoulder_right_shoulder(right_shoulder - left_shoulder);
		left_shoulder_elbow.Normalize();
		left_shoulder_right_shoulder.Normalize();
    //left_shoulder_elbow = torso_rotation_inverse * left_shoulder_elbow;
    //left_shoulder_neck = torso_rotation_inverse * left_shoulder_neck;
		static double left_shoulder_angle_roll = 0;
		if (joint_position_right_shoulder.fConfidence >= 0.5 && 
				joint_position_left_elbow.fConfidence >= 0.5 && 
				joint_position_left_shoulder.fConfidence >= 0.5)
		{
			left_shoulder_angle_roll = acos(KDL::dot(left_shoulder_elbow, left_shoulder_right_shoulder));
			left_shoulder_angle_roll = left_shoulder_angle_roll - HALFPI;
		}
						 
		// right shoulder roll
		KDL::Vector right_shoulder_elbow(right_elbow - right_shoulder);
		KDL::Vector right_shoulder_left_shoulder(left_shoulder - right_shoulder);
		right_shoulder_elbow.Normalize();
		right_shoulder_left_shoulder.Normalize();
		static double right_shoulder_angle_roll = 0;
		if (joint_position_left_shoulder.fConfidence >= 0.5 && 
				joint_position_right_elbow.fConfidence >= 0.5 && 
				joint_position_right_shoulder.fConfidence >= 0.5)
		{     
			right_shoulder_angle_roll = acos(KDL::dot(right_shoulder_elbow, right_shoulder_left_shoulder));
			right_shoulder_angle_roll = -(right_shoulder_angle_roll - HALFPI);                                      
		} 
										
		// left shoulder pitch
		static double left_shoulder_angle_pitch = 0;
		if (joint_position_left_shoulder.fConfidence >= 0.5 &&
        joint_position_left_elbow.fConfidence >= 0.5)
		{ 
		  left_shoulder_angle_pitch = asin(left_shoulder_elbow.y());
			left_shoulder_angle_pitch = left_shoulder_angle_pitch + HALFPI;
		  //left_shoulder_angle_pitch = -acos(left_shoulder_elbow.y());
      //if (left_shoulder.z() < left_elbow.z())
      ////  if (left_shoulder.x() < left_elbow.x()) 
      ////    left_shoulder_angle_pitch = PI + PI - left_shoulder_angle_pitch;
      ////  else
      //    left_shoulder_angle_pitch = -left_shoulder_angle_pitch;
    }
	  //if (joint_orientation_left_shoulder.fConfidence >= 0.5)
    //{
    //  left_shoulder_angle_pitch = left_shoulder_yaw + HALFPI;
    //}
	
		// right shoulder pitch
		static double right_shoulder_angle_pitch = 0;
	  if (joint_position_right_shoulder.fConfidence >= 0.5)
		{
			right_shoulder_angle_pitch = asin(right_shoulder_elbow.y());
			right_shoulder_angle_pitch = -(right_shoulder_angle_pitch + HALFPI);
			//right_shoulder_angle_pitch = acos(right_shoulder_elbow.y());
      //if (right_shoulder_elbow.z() > 0.0)
      //  right_shoulder_angle_pitch = -right_shoulder_angle_pitch;
  	}
    //if (joint_orientation_right_shoulder.fConfidence >= 0.5)
    //{
    //  right_shoulder_angle_pitch = -right_shoulder_pitch;
    //}


																									
		// left shoulder yaw
		static double left_shoulder_angle_yaw = 0;
		//if (joint_position_left_shoulder.fConfidence >= 0.5)
		//{           
		//	left_shoulder_angle_yaw = asin(left_elbow_hand.y());  // left_shoulder_elbow.x()
		//}
		if (joint_orientation_left_shoulder.fConfidence >= 0.4)
    {
      left_shoulder_angle_yaw = left_shoulder_roll;
      //left_shoulder_angle_yaw = left_shoulder_pitch;
    }

		// right shoulder yaw
		static double right_shoulder_angle_yaw = 0;
		//if (joint_position_right_shoulder.fConfidence >= 0.5)
		//{  
		//	right_shoulder_angle_yaw = asin(right_elbow_hand.y());  // left_shoulder_elbow.x()
		//	right_shoulder_angle_yaw = -right_shoulder_angle_yaw;
		//}
    if (joint_orientation_right_shoulder.fConfidence >= 0.4)
    {
      right_shoulder_angle_yaw = -(right_shoulder_roll);
    }
					
		// LEGS for robot
    
    // Knee left pitch					
		KDL::Vector left_knee_foot(left_foot - left_knee);
		KDL::Vector left_knee_hip(left_hip - left_knee);
		left_knee_foot.Normalize();
		left_knee_hip.Normalize();
		static double knee_left_angle_pitch = 0;
		if (joint_position_left_foot.fConfidence >= 0.5 && 
				joint_position_left_knee.fConfidence >= 0.5 && 
				joint_position_left_hip.fConfidence >= 0.5)
		{
			knee_left_angle_pitch = acos(KDL::dot(left_knee_foot, left_knee_hip));
			knee_left_angle_pitch = knee_left_angle_pitch - PI;
      //knee_left_angle_pitch *= 10.0;
		}	
		
		// Knee right pitch					
		KDL::Vector right_knee_foot(right_foot - right_knee);
		KDL::Vector right_knee_hip(right_hip - right_knee);
		right_knee_foot.Normalize();
		right_knee_hip.Normalize();
		static double knee_right_angle_pitch = 0;
		if (joint_position_right_foot.fConfidence >= 0.5 && 
				joint_position_right_knee.fConfidence >= 0.5 && 
				joint_position_right_hip.fConfidence >= 0.5)
		{
			knee_right_angle_pitch = acos(KDL::dot(right_knee_foot, right_knee_hip));
			knee_right_angle_pitch = -(knee_right_angle_pitch - PI);
      //knee_right_angle_pitch *= 10.0;
		}

    knee_right_angle_pitch = (knee_right_angle_pitch - knee_left_angle_pitch) / 2.0;  
		knee_right_angle_pitch *= 1.1;
    knee_right_angle_pitch += 0.4;
        
    static double prev_safe = 0.4;
    if (knee_right_angle_pitch >= 1.2 || knee_right_angle_pitch <= -0.1)
    {
      knee_right_angle_pitch = prev_safe;
    }
    else
    {
      prev_safe = knee_right_angle_pitch;
    }
    knee_left_angle_pitch = -knee_right_angle_pitch;


		// Hip left roll					
		//KDL::Vector hip_left_knee(left_knee - left_hip);
		//KDL::Vector hip_left_right(right_hip - left_hip);
		//hip_left_knee.Normalize();
		//hip_left_right.Normalize();
		static double hip_left_angle_roll = 0;
		//if (joint_position_left_knee.fConfidence >= 0.5 && 
		//		joint_position_left_hip.fConfidence >= 0.5 && 
		//		joint_position_right_hip.fConfidence >= 0.5)
		//{
		//	hip_left_angle_roll = acos(KDL::dot(hip_left_knee, hip_left_right));
	  //  hip_left_angle_roll = hip_left_angle_roll - HALFPI;
		//}	
		
		// Hip right roll					
		//KDL::Vector hip_right_knee(right_knee - right_hip);
		//KDL::Vector hip_right_left(left_hip - right_hip);
		//hip_right_knee.Normalize();
		//hip_right_left.Normalize();
		static double hip_right_angle_roll = 0;
		//if (joint_position_right_knee.fConfidence >= 0.5 && 
		//		joint_position_right_hip.fConfidence >= 0.5 && 
		//		joint_position_left_hip.fConfidence >= 0.5)
		//{
		//	hip_right_angle_roll = acos(KDL::dot(hip_right_knee, hip_right_left));
		//	hip_right_angle_roll = -(hip_right_angle_roll - HALFPI);
		//}			
											
    //KDL::Vector lower_roll((right_hip + left_hip / 2.0) -
    //                       (right_knee + left_knee / 2.0));
    KDL::Vector lower_roll((right_hip + left_hip / 2.0) -
                           (right_foot + left_foot / 2.0));
    lower_roll.Normalize();

		if (joint_position_left_knee.fConfidence >= 0.5 && 
				joint_position_left_hip.fConfidence >= 0.5 && 
				joint_position_right_hip.fConfidence >= 0.5 &&
        joint_position_right_knee.fConfidence >= 0.5)
		{
       hip_right_angle_roll = hip_left_angle_roll = asin(lower_roll.x()) / 2.0;
    }
    //hip_right_angle_roll = hip_left_angle_roll = 
    // (hip_right_angle_roll + hip_left_angle_roll) / 2.0 / 2.0;  

		// left ankle pitch
		static double left_ankle_angle_pitch = 0;
		//if (joint_position_left_foot.fConfidence >= 0.5)
		//{ 
      left_ankle_angle_pitch = knee_left_angle_pitch / 2.0;
			//left_ankle_angle_pitch = asin(left_knee_foot.y());
			//left_ankle_angle_pitch = -(left_ankle_angle_pitch + HALFPI);
    //}
		
		// right ankle pitch
		static double right_ankle_angle_pitch = 0;
		//if (joint_position_right_foot.fConfidence >= 0.5)
		//{ 
      right_ankle_angle_pitch = knee_right_angle_pitch / 2.0;
			//right_ankle_angle_pitch = asin(right_knee_foot.y());
			//right_ankle_angle_pitch = (right_ankle_angle_pitch + HALFPI);
		//}		
		
		// left ankle roll
		static double left_ankle_angle_roll = 0;
		//if (joint_position_left_foot.fConfidence >= 0.5)
		//{ 
      left_ankle_angle_roll = hip_left_angle_roll;
			//left_ankle_angle_roll = asin(left_knee_foot.x());
			//left_ankle_angle_roll = -(left_ankle_angle_roll);
		//}
		
		// right ankle roll
		static double right_ankle_angle_roll = 0;
		//if (joint_position_right_foot.fConfidence >= 0.5)
		//{ 
      right_ankle_angle_roll = hip_right_angle_roll;
			//right_ankle_angle_roll = asin(right_knee_foot.x());
			//right_ankle_angle_roll = -(right_ankle_angle_roll);
		//}				
    
		// left hip pitch
		static double left_hip_angle_pitch = 0;
		//if (joint_position_left_hip.fConfidence >= 0.5)
		//{ 
			left_hip_angle_pitch = -knee_left_angle_pitch / 2.0;
      //left_hip_angle_pitch = asin(left_knee_hip.y());
			//left_hip_angle_pitch = -(left_hip_angle_pitch - HALFPI);
		//}
		
		// right hip pitch
		static double right_hip_angle_pitch = 0;
		//if (joint_position_right_hip.fConfidence >= 0.5)
		//{ 
			right_hip_angle_pitch = -knee_right_angle_pitch / 2.0;
			//right_hip_angle_pitch = asin(right_knee_hip.y());
			//right_hip_angle_pitch = (right_hip_angle_pitch - HALFPI);
		//}	    
		
		// Torso Yaw
	  //static double torso_angle_yaw = 0;
		//if (joint_orientation_torso.fConfidence >= 0.5)
		//{           
		//	torso_angle_yaw = t_pitch;
		//}

		/*
		// Head Yaw
    static double head_angle_yaw = 0;
		if (joint_orientation_head.fConfidence >= 0.5)
		{     
			head_angle_yaw = head_pitch - torso_pitch;
		}

		// Head Pitch
	  static double head_angle_pitch = 0;
		if (joint_orientation_head.fConfidence >= 0.5)
		{           
			head_angle_pitch = head_roll - torso_roll;
		}
    */
		
	  
		// head pitch
		//KDL::Vector head_torso(head - torso);
		//KDL::Vector head_neck(head - neck);
		//head_torso.Normalize();
		//head_neck.Normalize();
		static double head_angle_pitch = 0;
    // Use head for counter balancing
    static double hp_min = -0.1;
    static double hp_max = 0.65;
    // if this number is close to zero head should be hp_min
    // if it is close to or above HALFPI then head should be hp_max
		double arm_pitch_factor = (fabs(right_shoulder_angle_pitch) + 
                               fabs(left_shoulder_angle_pitch)) / 2.0;
    if (arm_pitch_factor >= HALFPI)
      head_angle_pitch = 0.65;
    else
      head_angle_pitch = hp_min + ((arm_pitch_factor / HALFPI) * (hp_max - hp_min));
   
    // head follows arm most sideways
    static double head_angle_yaw = 0;
    double arm_roll_factor = (right_shoulder_angle_roll + 
                             left_shoulder_angle_roll) / 2.0;
    head_angle_yaw = arm_roll_factor * -1.1;
    

    //if (joint_position_neck.fConfidence >= 0.5 && 
		//		joint_position_torso.fConfidence >= 0.5 && 
		//		joint_position_head.fConfidence >= 0.5)
		//{
		//	head_angle_pitch = asin(KDL::dot(head_torso, head_neck));
			//head_angle_pitch = head_angle_pitch - HALFPI;
		//}
	  		
    /////
    // Velocity foot mouse gestures
    /////
    /*
	  KDL::Vector right_left_foot(right_foot - left_foot);
	  static double foot_mouse_angle = 0;
	  static double foot_mouse_length = 0;
	  geometry_msgs::Twist cmd_vel;

	  if (joint_position_left_foot.fConfidence >= 0.5 && 
	      joint_position_right_foot.fConfidence >= 0.5)
    {
      foot_mouse_length = sqrt(right_left_foot.x()*right_left_foot.x() + 
             right_left_foot.z()*right_left_foot.z());
      foot_mouse_angle = atan2(right_left_foot.x(), right_left_foot.z());
      foot_mouse_angle = foot_mouse_angle - HALFPI;

      cmd_vel.linear.x = foot_mouse_length / 2400;

      if (cmd_vel.linear.x > 0.2)
      {
        cmd_vel.linear.x = 0.2;
      }

      if (foot_mouse_angle > 0.75 && foot_mouse_angle < 1.5)
      {
        cmd_vel.angular.z = 0;
      }
      else if (foot_mouse_angle > 0.0 && foot_mouse_angle < 0.75)
      {
        //cmd_vel.angular.z = cmd_vel.linear.x;
        if (cmd_vel.linear.x > 0.1)
        	cmd_vel.angular.z = 0.4;
        else
        	cmd_vel.angular.z = 0.0;
        cmd_vel.linear.x = 0.0;
      }
      else if (foot_mouse_angle < 0.0 && foot_mouse_angle > -0.75)
      {
        //cmd_vel.angular.z = -cmd_vel.linear.x;
        if (cmd_vel.linear.x > 0.1) cmd_vel.angular.z = -0.4;
        else cmd_vel.angular.z = 0.0;
        cmd_vel.linear.x = 0.0;
      }
      else 
      {
        cmd_vel.linear.x = -cmd_vel.linear.x;
        cmd_vel.angular.z = 0;
      }
      last_vel_linear_x = cmd_vel.linear.x;
      last_vel_angular_z = cmd_vel.angular.z;
    }
	  else
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
    }

		//ROS_INFO("LENGTH: %f", foot_mouse_length);
	  //ROS_INFO("ANGLE: %f", foot_mouse_angle);
	
    cmd_vel_pub_.publish(cmd_vel);
		*/
    
    // adapt velocity to 4 veltrobot motions
    /*std_msgs::String mot;
    mot.data = "stand_squat";
    if (cmd_vel.linear.x > 0.0)
      mot.data = "walk_forward";
    if (cmd_vel.angular.z	> 0.0)
      mot.data = "rotate_right";
    if (cmd_vel.angular.z	< 0.0)
      mot.data = "rotate_left";
      
    if (motion_enabled_)
    	motion_pub_.publish(mot);*/
  
		/////
		// Send joint state to robot
		/////
		
		sensor_msgs::JointState js; 
    
    if (left_arm_enabled_ && (arm_control_method_ == DIRECT))
    {
      js.name.push_back("elbow_left_roll");
      js.position.push_back(left_elbow_angle_roll);
      js.velocity.push_back(10);
      js.name.push_back("shoulder_left_roll");
      js.position.push_back(left_shoulder_angle_roll);
      js.velocity.push_back(10);
      js.name.push_back("shoulder_left_pitch");
      js.position.push_back(left_shoulder_angle_pitch);
      js.velocity.push_back(10);
      js.name.push_back("shoulder_left_yaw");
      js.position.push_back(left_shoulder_angle_yaw);
      js.velocity.push_back(10);
    }

		if (right_arm_enabled_ && (arm_control_method_ == DIRECT))
		{
      js.name.push_back("elbow_right_roll");
      js.position.push_back(right_elbow_angle_roll);
      js.velocity.push_back(10);
      js.name.push_back("shoulder_right_roll");
      js.position.push_back(right_shoulder_angle_roll);
      js.velocity.push_back(10);          
      js.name.push_back("shoulder_right_pitch");
      js.position.push_back(right_shoulder_angle_pitch);
      js.velocity.push_back(10);          
      js.name.push_back("shoulder_right_yaw");
      js.position.push_back(right_shoulder_angle_yaw);
      js.velocity.push_back(10);    
		}

    /*if (right_arm_enabled_ || left_arm_enabled_)
    {
      js.name.push_back("neck_yaw");
      js.position.push_back(head_angle_yaw);
      js.velocity.push_back(10);          
      js.name.push_back("neck_pitch");
      js.position.push_back(head_angle_pitch);
      js.velocity.push_back(10);    
    }*/
    
    if (legs_enabled_ && (left_arm_enabled_ || right_arm_enabled_)
        && (arm_control_method_ == DIRECT) )
    {
      js.name.push_back("knee_left_pitch");
      js.position.push_back(knee_left_angle_pitch);
      js.velocity.push_back(10);
      js.name.push_back("knee_right_pitch");
      js.position.push_back(knee_right_angle_pitch);
      js.velocity.push_back(10);
      js.name.push_back("hip_left_roll");
      js.position.push_back(hip_left_angle_roll);
      js.velocity.push_back(10);		
      js.name.push_back("hip_right_roll");
      js.position.push_back(hip_right_angle_roll);
      js.velocity.push_back(10);		
      js.name.push_back("ankle_left_pitch");
      js.position.push_back(left_ankle_angle_pitch);
      js.velocity.push_back(10);		
      js.name.push_back("ankle_right_pitch");
      js.position.push_back(right_ankle_angle_pitch);
      js.velocity.push_back(10);
      js.name.push_back("ankle_left_roll");
      js.position.push_back(left_ankle_angle_roll);
      js.velocity.push_back(10);		
      js.name.push_back("ankle_right_roll");
      js.position.push_back(right_ankle_angle_roll);
      js.velocity.push_back(10); 
      js.name.push_back("hip_left_pitch");
      js.position.push_back(left_hip_angle_pitch);
      js.velocity.push_back(10);
			js.name.push_back("hip_right_pitch");
      js.position.push_back(right_hip_angle_pitch);
      js.velocity.push_back(10);      
      
      js.name.push_back("hip_left_yaw");
      js.position.push_back(0);
      js.velocity.push_back(10);
      js.name.push_back("hip_right_yaw");
      js.position.push_back(0);
      js.velocity.push_back(10);
    }  
		
    if (js.name.size()) 		
      joint_states_pub_.publish(js);

    /////
    //  Following is IK method
    /////

    KDL::Vector left_hand_torso  = left_hand;  // - torso;
    KDL::Vector right_hand_torso = right_hand; // - torso;
    
    // need to scale from me to the nao.
    // I am 1.69 meters, nao is 0.58.
    // nao is 0.3432 of me.
    // openni units in mm, nao in m.
    // magic number:
    double scale = 0.0003432;

    left_hand_torso  = scale * left_hand_torso;
    right_hand_torso = scale * right_hand_torso;

    geometry_msgs::Point p;

    if (left_arm_enabled_ && (arm_control_method_ == IK))
    {
      p.x = -left_hand_torso.z();
      p.y = -left_hand_torso.x();
      p.z = left_hand_torso.y();
      left_arm_destination_pub_.publish(p);
    }

    if (right_arm_enabled_ && (arm_control_method_ == IK))
    {
      p.x = -right_hand_torso.z();
      p.y = -right_hand_torso.x();
      p.z = right_hand_torso.y();
      right_arm_destination_pub_.publish(p);
    }

    // The robot will receive and try to mirror my position
    geometry_msgs::Point torso_destination;
    torso_destination.x = -torso.z() / 1000.0;
    torso_destination.y = -torso.x() / 1000.0;
    //robot_destination.z = torso.y() / 1000.0;
    torso_destination.z = torso_pitch;
    torso_destination_pub_.publish(torso_destination);
    
    break;	// only read first user
	}
}

} // namespace veltrobot_teleop
