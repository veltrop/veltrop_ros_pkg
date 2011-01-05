#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
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

// TODO I wish: hand open/close or palm recognition to control grippers

namespace veltrobot_teleop
{

KinectTeleop::KinectTeleop()
: publish_kinect_tf_(false)
, arms_enabled_(false)
, legs_enabled_(false)
{    
}
      
void KinectTeleop::init()
{
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	motion_pub_ = n.advertise <std_msgs::String> ("motion_name", 1);
	joint_states_pub_ = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
	cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	np.param<bool>("publish_kinect_tf_", publish_kinect_tf_, false);  
  enable_joint_group_sub_ = n.subscribe("/enable_joint_group", 1,
                                        &KinectTeleop::enableJointGroupCB, this);
}

void KinectTeleop::enableJointGroupCB(const veltrobot_msgs::EnableJointGroupConstPtr& msg)
{
	for (size_t i=0; i < msg->jointGroups.size(); i++)
  {
  	if (msg->jointGroups[i] == "legs")
    	legs_enabled_ = msg->enabledStates[i];
    else if (msg->jointGroups[i] == "arms")
    	arms_enabled_ = msg->enabledStates[i];
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
			string frame_id("openni_depth");

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
		
		// Upper Joints
		XnSkeletonJointPosition joint_position_head;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_HEAD, joint_position_head);
		KDL::Vector head(joint_position_head.position.X, joint_position_head.position.Y, joint_position_head.position.Z);
		
		XnSkeletonJointPosition joint_position_neck;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_NECK, joint_position_neck);
		KDL::Vector neck(joint_position_neck.position.X, joint_position_neck.position.Y, joint_position_neck.position.Z);
		
		XnSkeletonJointPosition joint_position_torso;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_TORSO, joint_position_torso);
		KDL::Vector torso(joint_position_torso.position.X, joint_position_torso.position.Y, joint_position_torso.position.Z);
							
	  XnSkeletonJointOrientation joint_orientation_head;
		UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_HEAD, joint_orientation_head);
		m = joint_orientation_head.orientation.elements;
		KDL::Rotation head_rotation(m[0], m[1], m[2],
																m[3], m[4], m[5],
																m[6], m[7], m[8]);
	  double head_roll, head_pitch, head_yaw;
	  head_rotation.GetRPY(head_roll, head_pitch, head_yaw);
		/*
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
																							
		// Left Arm
		XnSkeletonJointPosition joint_position_left_hand;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HAND, joint_position_left_hand);
		KDL::Vector left_hand(joint_position_left_hand.position.X, joint_position_left_hand.position.Y, joint_position_left_hand.position.Z);
		
		XnSkeletonJointPosition joint_position_left_elbow;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_ELBOW, joint_position_left_elbow);
		KDL::Vector left_elbow(joint_position_left_elbow.position.X, joint_position_left_elbow.position.Y, joint_position_left_elbow.position.Z);        
		
		XnSkeletonJointPosition joint_position_left_shoulder;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_SHOULDER, joint_position_left_shoulder);
		KDL::Vector left_shoulder(joint_position_left_shoulder.position.X, joint_position_left_shoulder.position.Y, joint_position_left_shoulder.position.Z);        	        	
		
		// Right Arm
		XnSkeletonJointPosition joint_position_right_hand;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HAND, joint_position_right_hand);
		KDL::Vector right_hand(joint_position_right_hand.position.X, joint_position_right_hand.position.Y, joint_position_right_hand.position.Z);
		
		XnSkeletonJointPosition joint_position_right_elbow;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_ELBOW, joint_position_right_elbow);
		KDL::Vector right_elbow(joint_position_right_elbow.position.X, joint_position_right_elbow.position.Y, joint_position_right_elbow.position.Z);        
		
		XnSkeletonJointPosition joint_position_right_shoulder;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_SHOULDER, joint_position_right_shoulder);
		KDL::Vector right_shoulder(joint_position_right_shoulder.position.X, joint_position_right_shoulder.position.Y, joint_position_right_shoulder.position.Z);
		
		// Right Leg
		XnSkeletonJointPosition joint_position_right_hip;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HIP, joint_position_right_hip);
		KDL::Vector right_hip(joint_position_right_hip.position.X, joint_position_right_hip.position.Y, joint_position_right_hip.position.Z);

		XnSkeletonJointPosition joint_position_right_knee;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_KNEE, joint_position_right_knee);
		KDL::Vector right_knee(joint_position_right_knee.position.X, joint_position_right_knee.position.Y, joint_position_right_knee.position.Z);

		XnSkeletonJointPosition joint_position_right_foot;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_FOOT, joint_position_right_foot);
		KDL::Vector right_foot(joint_position_right_foot.position.X, joint_position_right_foot.position.Y, joint_position_right_foot.position.Z);

		// Left Leg
		XnSkeletonJointPosition joint_position_left_hip;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HIP, joint_position_left_hip);
		KDL::Vector left_hip(joint_position_left_hip.position.X, joint_position_left_hip.position.Y, joint_position_left_hip.position.Z);

		XnSkeletonJointPosition joint_position_left_knee;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_KNEE, joint_position_left_knee);
		KDL::Vector left_knee(joint_position_left_knee.position.X, joint_position_left_knee.position.Y, joint_position_left_knee.position.Z);

		XnSkeletonJointPosition joint_position_left_foot;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_FOOT, joint_position_left_foot);
		KDL::Vector left_foot(joint_position_left_foot.position.X, joint_position_left_foot.position.Y, joint_position_left_foot.position.Z);

		///////////////////////////////////////////////////////////////////////////
		// two ways to go about this direct geometry approach...
		//
		// 1 semi calculated approach, the joint rotations are from the world perspective
		//   so we need to get rotations of one joint relative to the next in a chain,
		//   like how the robot is:
		// 1.1 publish transform of two joints relative to openni_depth frame.
		// 1.2 request transform between two joints own frames .
		// 1.3 apply that inner joint transform to correct robot joint.  problem
		//     is that that joint of robot may have too few dof.
		//
		// 2 [nearly] direct aproach using joint positions, and geometry to relate them:
		// 2.1 get points of three joints.
		// 2.2 do many specific calculations relating that geometry to my own
		//     robots geometry.
		// 2.3 apply rotations to robot.				
		///////////////////////////////////////////////////////////////////////////															
		
		/////
		// Process and output joint rotations to the robot.																														
		/////
																																																																																			
		// left elbow roll
		KDL::Vector left_elbow_hand(left_hand - left_elbow);
		KDL::Vector left_elbow_shoulder(left_shoulder - left_elbow);
		left_elbow_hand.Normalize();
		left_elbow_shoulder.Normalize();
		static double left_elbow_angle_roll = 0;
		if (joint_position_left_hand.fConfidence >= 0.5 && 
				joint_position_left_elbow.fConfidence >= 0.5 && 
				joint_position_left_shoulder.fConfidence >= 0.5)
		{
//			left_elbow_angle_roll = acos(KDL::dot(left_elbow_hand, left_elbow_shoulder));
//			left_elbow_angle_roll = left_elbow_angle_roll - PI;
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
//			right_elbow_angle_roll = acos(KDL::dot(right_elbow_hand, right_elbow_shoulder));
//			right_elbow_angle_roll = -(right_elbow_angle_roll - PI);
		}  
						
		// left shoulder roll
		KDL::Vector left_shoulder_elbow(left_elbow - left_shoulder);
		KDL::Vector left_shoulder_neck(neck - left_shoulder);
		left_shoulder_elbow.Normalize();
		left_shoulder_neck.Normalize();
		static double left_shoulder_angle_roll = 0;
		if (joint_position_neck.fConfidence >= 0.5 && 
				joint_position_left_elbow.fConfidence >= 0.5 && 
				joint_position_left_shoulder.fConfidence >= 0.5)
		{
//			left_shoulder_angle_roll = acos(KDL::dot(left_shoulder_elbow, left_shoulder_neck));
//			left_shoulder_angle_roll = left_shoulder_angle_roll - HALFPI;
		}
						 
		// right shoulder roll
		KDL::Vector right_shoulder_elbow(right_elbow - right_shoulder);
		KDL::Vector right_shoulder_neck(neck - right_shoulder);
		right_shoulder_elbow.Normalize();
		right_shoulder_neck.Normalize();
		static double right_shoulder_angle_roll = 0;
		if (joint_position_neck.fConfidence >= 0.5 && 
				joint_position_right_elbow.fConfidence >= 0.5 && 
				joint_position_right_shoulder.fConfidence >= 0.5)
		{     
//			right_shoulder_angle_roll = acos(KDL::dot(right_shoulder_elbow, right_shoulder_neck));
//			right_shoulder_angle_roll = -(right_shoulder_angle_roll - HALFPI);                                      
		} 
										
		// left shoulder pitch
		static double left_shoulder_angle_pitch = 0;
		if (joint_position_left_shoulder.fConfidence >= 0.5)
		{ 
//			left_shoulder_angle_pitch = asin(left_shoulder_elbow.y());
//			left_shoulder_angle_pitch = left_shoulder_angle_pitch + HALFPI;
		}
		
		// right shoulder pitch
		static double right_shoulder_angle_pitch = 0;
		if (joint_position_right_shoulder.fConfidence >= 0.5)
		{ 
//			right_shoulder_angle_pitch = asin(right_shoulder_elbow.y());
//			right_shoulder_angle_pitch = -(right_shoulder_angle_pitch + HALFPI);
		}
																									
		// left shoulder yaw
		static double left_shoulder_angle_yaw = 0;
		if (joint_position_left_shoulder.fConfidence >= 0.5)
		{           
//			left_shoulder_angle_yaw = asin(left_elbow_hand.y());  // left_shoulder_elbow.x()
		}
		
		// right shoulder yaw
		static double right_shoulder_angle_yaw = 0;
		if (joint_position_right_shoulder.fConfidence >= 0.5)
		{  
//			right_shoulder_angle_yaw = asin(right_elbow_hand.y());  // left_shoulder_elbow.x()
//			right_shoulder_angle_yaw = -right_shoulder_angle_yaw;
		}
					
		///////////////////////////////////////////////////////////////////////////
		// PROBLEM:
		// doing each of the 3 degrees of freedom in the shoulder separately is having problems.
		// 1. the acos has no sign, it only reads less than one half pi rotation
		// 2. there is a paradox/redundancy with joint movement to be considered: ie, to lift the
		//    arm vertically up it could either rotate the pitch or roll joint by one pi
		// SOLUTION?:
		// if we take the raw tf rotation from the input shoulder, can we apply that to our
		// final dof of our shoulder chain, and then back solve for the two joints leading to it?
		// or do that from the hand?
		///////////////////////////////////////////////////////////////////////////
											
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
//			knee_left_angle_pitch = acos(KDL::dot(left_knee_foot, left_knee_hip));
//			knee_left_angle_pitch = knee_left_angle_pitch - PI;
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
//			knee_right_angle_pitch = acos(KDL::dot(right_knee_foot, right_knee_hip));
//			knee_right_angle_pitch = -(knee_right_angle_pitch - PI);
		}	
		
		// Hip left roll					
		KDL::Vector hip_left_knee(left_knee - left_hip);
		KDL::Vector hip_left_right(right_hip - left_hip);
		hip_left_knee.Normalize();
		hip_left_right.Normalize();
		static double hip_left_angle_roll = 0;
		if (joint_position_left_knee.fConfidence >= 0.5 && 
				joint_position_left_hip.fConfidence >= 0.5 && 
				joint_position_right_hip.fConfidence >= 0.5)
		{
			hip_left_angle_roll = acos(KDL::dot(hip_left_knee, hip_left_right));
			hip_left_angle_roll = hip_left_angle_roll - HALFPI;
		}	
		
		// Hip right roll					
		KDL::Vector hip_right_knee(right_knee - right_hip);
		KDL::Vector hip_right_left(left_hip - right_hip);
		hip_right_knee.Normalize();
		hip_right_left.Normalize();
		static double hip_right_angle_roll = 0;
		if (joint_position_right_knee.fConfidence >= 0.5 && 
				joint_position_right_hip.fConfidence >= 0.5 && 
				joint_position_left_hip.fConfidence >= 0.5)
		{
			hip_right_angle_roll = acos(KDL::dot(hip_right_knee, hip_right_left));
			hip_right_angle_roll = -(hip_right_angle_roll - HALFPI);
		}			
											
		/*
		// Right hip pitch
		KDL::Vector right_hip_knee(right_knee - right_hip);
		KDL::Vector right_hip_torso(torso - right_hip);
		right_hip_knee.Normalize();
		right_hip_torso.Normalize();
		static double right_hip_angle_pitch = 0;
		if (joint_position_right_hip.fConfidence >= 0.5 && 
				joint_position_right_knee.fConfidence >= 0.5 && 
				joint_position_torso.fConfidence >= 0.5)
		{     
			right_hip_angle_pitch = asin(KDL::dot(right_hip_knee, right_hip_torso));
			// right_hip_angle_roll = -(right_hip_angle_roll - HALFPI);
		} 

		// Left hip roll
		KDL::Vector left_hip_knee(left_knee - left_hip);
		KDL::Vector left_hip_torso(torso - left_hip);
		left_hip_knee.Normalize();
		left_hip_torso.Normalize();
		static double left_hip_angle_pitch = 0;
		if (joint_position_left_hip.fConfidence >= 0.5 && 
				joint_position_left_knee.fConfidence >= 0.5 && 
				joint_position_torso.fConfidence >= 0.5)
		{     
			left_hip_angle_pitch = asin(KDL::dot(left_hip_knee, left_hip_torso));
			//left_hip_angle_roll = -(left_hip_angle_roll - HALFPI);
		}
		*/
		
		/*
		// Right ankle roll
		XnSkeletonJointOrientation joint_orientation_right_knee;
		UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_KNEE, joint_orientation_right_knee);
	  m = joint_orientation_right_knee.orientation.elements;
	  KDL::Rotation right_knee_rotation(m[0], m[1], m[2],
																 m[3], m[4], m[5],
																 m[6], m[7], m[8]);
		double right_knee_roll, right_knee_pitch, right_knee_yaw;
	  right_knee_rotation.GetRPY(right_knee_roll, right_knee_pitch, right_knee_yaw);			
		
		static double right_ankle_roll = 0;
		if (joint_orientation_right_knee.fConfidence >= 0.5)
		{
			right_ankle_roll = right_knee_roll;
		}
		*/
		
		// left ankle pitch
		static double left_ankle_angle_pitch = 0;
		if (joint_position_left_foot.fConfidence >= 0.5)
		{ 
//			left_ankle_angle_pitch = asin(left_knee_foot.y());
//			left_ankle_angle_pitch = -(left_ankle_angle_pitch + HALFPI);
		}
		
		// right ankle pitch
		static double right_ankle_angle_pitch = 0;
		if (joint_position_right_foot.fConfidence >= 0.5)
		{ 
//			right_ankle_angle_pitch = asin(right_knee_foot.y());
//			right_ankle_angle_pitch = (right_ankle_angle_pitch + HALFPI);
		}		
		
		// left ankle roll
		static double left_ankle_angle_roll = 0;
		if (joint_position_left_foot.fConfidence >= 0.5)
		{ 
			left_ankle_angle_roll = asin(left_knee_foot.x());
			left_ankle_angle_roll = -(left_ankle_angle_roll);
		}
		
		// right ankle roll
		static double right_ankle_angle_roll = 0;
		if (joint_position_right_foot.fConfidence >= 0.5)
		{ 
			right_ankle_angle_roll = asin(right_knee_foot.x());
			right_ankle_angle_roll = -(right_ankle_angle_roll);
		}				
		
		// Torso Yaw
	  //static double torso_angle_yaw = 0;
		//if (joint_orientation_torso.fConfidence >= 0.5)
		//{           
		//	torso_angle_yaw = t_pitch;
		//}

		// Head Yaw
	  static double head_angle_yaw = 0;
		if (joint_orientation_head.fConfidence >= 0.5)
		{     
			//head_angle_yaw = head_pitch - torso_pitch;
		}

		// Head Pitch
	  static double head_angle_pitch = 0;
		if (joint_orientation_head.fConfidence >= 0.5)
		{           
			//head_angle_pitch = head_roll - torso_roll;
		}
		
	  /*
		// head pitch
		KDL::Vector head_torso(head - torso);
		KDL::Vector head_neck(head - neck);
		head_torso.Normalize();
		head_neck.Normalize();
		static double head_angle_pitch = 0;
		if (joint_position_neck.fConfidence >= 0.5 && 
				joint_position_torso.fConfidence >= 0.5 && 
				joint_position_head.fConfidence >= 0.5)
		{
			head_angle_pitch = asin(KDL::dot(head_torso, head_neck));
			//head_angle_pitch = head_angle_pitch - HALFPI;
		}
	  */		
	
		/////
		// Send to robot
		/////
		
		sensor_msgs::JointState js; 
    
    if (arms_enabled_)
    {
      js.name.push_back("elbow_left_roll");
      js.position.push_back(left_elbow_angle_roll);
      js.velocity.push_back(10);
      js.name.push_back("elbow_right_roll");
      js.position.push_back(right_elbow_angle_roll);
      js.velocity.push_back(10);
      js.name.push_back("shoulder_left_roll");
      js.position.push_back(left_shoulder_angle_roll);
      js.velocity.push_back(10);
      js.name.push_back("shoulder_right_roll");
      js.position.push_back(right_shoulder_angle_roll);
      js.velocity.push_back(10);          
      js.name.push_back("shoulder_left_pitch");
      js.position.push_back(left_shoulder_angle_pitch);
      js.velocity.push_back(10);
      js.name.push_back("shoulder_right_pitch");
      js.position.push_back(right_shoulder_angle_pitch);
      js.velocity.push_back(10);          
      js.name.push_back("shoulder_left_yaw");
      js.position.push_back(left_shoulder_angle_yaw);
      js.velocity.push_back(10);
      js.name.push_back("shoulder_right_yaw");
      js.position.push_back(right_shoulder_angle_yaw);
      js.velocity.push_back(10);    
    }        
    
    if (legs_enabled_)
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
      
      js.name.push_back("hip_left_yaw");
      js.position.push_back(0);
      js.velocity.push_back(10);
      js.name.push_back("hip_left_pitch");
      js.position.push_back(0);
      js.velocity.push_back(10);
      
      js.name.push_back("hip_right_yaw");
      js.position.push_back(0);
      js.velocity.push_back(10);
      js.name.push_back("hip_right_pitch");
      js.position.push_back(0);
      js.velocity.push_back(10);
    }  
		
    /*
		js.name.push_back("neck_pitch");
		js.position.push_back(head_angle_pitch);
		js.velocity.push_back(10);
		js.name.push_back("neck_yaw");
		js.position.push_back(head_angle_yaw);
		js.velocity.push_back(10);
		*/

		//cmd_vel_pub_.publish(cmd_vel);
		joint_states_pub_.publish(js);

		break;	// only read first user
	}
}

} // namespace veltrobot_teleop
