#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <GL/glut.h>
#include <string>
#include <pthread.h>
#include "KinectController.h"
#include "KinectDisplay.h"

using std::string;

#ifndef PI
#define PI 3.14159265359
#endif
#ifndef HALFPI
#define HALFPI 1.57079632679
#endif 
#ifndef QUARTPI
#define QUARTPI 0.785398163397
#endif

namespace veltrobot_teleop
{

	class TeleopKinect
  {
    public:
      TeleopKinect()
			: publish_kinect_tf_(false)
      {    
      }
      
      void init()
      {
        ros::NodeHandle n;
				ros::NodeHandle np("~");
        motion_pub_ = n.advertise <std_msgs::String> ("motion_name", 1);
        joint_states_pub_ = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
				np.param<bool>("publish_kinect_tf_", publish_kinect_tf_, false);  
      }
      
      void publishTransform(KinectController& kinect_controller, XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id)
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
        
      void processKinect(KinectController& kinect_controller)
      {
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
      		
          // two ways to go about this direct geometry approach...
          // 1 calculated approach
          // 1.1? publish transform of two joints relative to openni_depth frame
          // 1.2? request transform between two joints own frames 
      		// 2 [nearly] direct aproach
          // 2.1 get points of three joints
          // 2.2 do many specific calculations relating that geometry to my own
          //     robots geometry

					// get joint positions
					XnSkeletonJointPosition joint_position_neck;
        	UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_NECK, joint_position_neck);
          KDL::Vector neck(joint_position_neck.position.X, joint_position_neck.position.Y, joint_position_neck.position.Z);
        	XnSkeletonJointPosition joint_position_left_hand;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HAND, joint_position_left_hand);
        	KDL::Vector left_hand(joint_position_left_hand.position.X, joint_position_left_hand.position.Y, joint_position_left_hand.position.Z);
        	XnSkeletonJointPosition joint_position_left_elbow;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_ELBOW, joint_position_left_elbow);
        	KDL::Vector left_elbow(joint_position_left_elbow.position.X, joint_position_left_elbow.position.Y, joint_position_left_elbow.position.Z);        
         	XnSkeletonJointPosition joint_position_left_shoulder;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_SHOULDER, joint_position_left_shoulder);
        	KDL::Vector left_shoulder(joint_position_left_shoulder.position.X, joint_position_left_shoulder.position.Y, joint_position_left_shoulder.position.Z);        	        	
          XnSkeletonJointPosition joint_position_right_hand;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HAND, joint_position_right_hand);
        	KDL::Vector right_hand(joint_position_right_hand.position.X, joint_position_right_hand.position.Y, joint_position_right_hand.position.Z);
          XnSkeletonJointPosition joint_position_right_elbow;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_ELBOW, joint_position_right_elbow);
        	KDL::Vector right_elbow(joint_position_right_elbow.position.X, joint_position_right_elbow.position.Y, joint_position_right_elbow.position.Z);        
         	XnSkeletonJointPosition joint_position_right_shoulder;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_SHOULDER, joint_position_right_shoulder);
        	KDL::Vector right_shoulder(joint_position_right_shoulder.position.X, joint_position_right_shoulder.position.Y, joint_position_right_shoulder.position.Z);
        	          
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
        	KDL::Vector left_shoulder_neck(neck - left_shoulder);
          left_shoulder_elbow.Normalize();
          left_shoulder_neck.Normalize();
          static double left_shoulder_angle_roll = 0;
          if (joint_position_neck.fConfidence >= 0.5 && 
          		joint_position_left_elbow.fConfidence >= 0.5 && 
              joint_position_left_shoulder.fConfidence >= 0.5)
          {
          	left_shoulder_angle_roll = acos(KDL::dot(left_shoulder_elbow, left_shoulder_neck));
          	left_shoulder_angle_roll = left_shoulder_angle_roll - HALFPI;
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
          	right_shoulder_angle_roll = acos(KDL::dot(right_shoulder_elbow, right_shoulder_neck));
          	right_shoulder_angle_roll = -(right_shoulder_angle_roll - HALFPI);                                      
          } 
                          
          // left shoulder pitch
          static double left_shoulder_angle_pitch = 0;
          if (joint_position_left_shoulder.fConfidence >= 0.5)
          { 
          	left_shoulder_angle_pitch = asin(left_shoulder_elbow.y());
          	left_shoulder_angle_pitch = left_shoulder_angle_pitch + HALFPI;
          }
          
  				// right shoulder pitch
          static double right_shoulder_angle_pitch = 0;
        	if (joint_position_right_shoulder.fConfidence >= 0.5)
          { 
          	right_shoulder_angle_pitch = asin(right_shoulder_elbow.y());
          	right_shoulder_angle_pitch = -(right_shoulder_angle_pitch + HALFPI);
          }
                                                        
          // left shoulder yaw
        	static double left_shoulder_angle_yaw = 0;
          if (joint_position_left_shoulder.fConfidence >= 0.5)
          {           
          	left_shoulder_angle_yaw = asin(left_elbow_hand.y());  // left_shoulder_elbow.x()
          }
          
          // right shoulder yaw
        	static double right_shoulder_angle_yaw = 0;
          if (joint_position_right_shoulder.fConfidence >= 0.5)
          {  
          	right_shoulder_angle_yaw = asin(right_elbow_hand.y());  // left_shoulder_elbow.x()
						right_shoulder_angle_yaw = -right_shoulder_angle_yaw;
          }
          
          // hand open/close or palm recognition to control grippers
          
          // PROBLEM:
          // doing each of the 3 degrees of freedom in the shoulder separately is having problems.
          // 1. the acos has no sign, it only reads less than one half pi rotation
          // 2. there is a paradox/redundancy with joint movement to be considered: ie, to lift the
          //    arm vertically up it could either rotate the pitch or roll joint by one pi
          // SOLUTION?:
          // if we take the raw tf rotation from the input shoulder, can we apply that to our
          // final dof of our shoulder chain, and then back solve for the two joints leading to it?
					// or do that from the hand?
					
					// knee left pitch
					static double knee_left_angle_pitch = 0;
					
					
        
        	// send to robot
          sensor_msgs::JointState js; 
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
   
          js.name.push_back("knee_left_pitch");
          js.position.push_back(knee_left_angle_pitch);
          js.velocity.push_back(10);
          js.name.push_back("knee_right_pitch");
          js.position.push_back(knee_right_angle_pitch);
          js.velocity.push_back(10);
					          
          /*
          js.name.push_back("neck_pitch");
          js.position.push_back(0);
          js.velocity.push_back(10);
          js.name.push_back("neck_yaw");
          js.position.push_back(0);
          js.velocity.push_back(10);

          js.name.push_back("hip_left_yaw");
          js.position.push_back(0);
          js.velocity.push_back(10);
          js.name.push_back("hip_left_roll");
          js.position.push_back(0);
          js.velocity.push_back(10);
          js.name.push_back("hip_left_pitch");
          js.position.push_back(0);
          js.velocity.push_back(10);
          js.name.push_back("ankle_left_pitch");
          js.position.push_back(0);
          js.velocity.push_back(10);
          js.name.push_back("ankle_left_roll");
          js.position.push_back(0);
          js.velocity.push_back(10);
          js.name.push_back("hip_right_yaw");
          js.position.push_back(0);
          js.velocity.push_back(10);
          js.name.push_back("hip_right_roll");
          js.position.push_back(0);
          js.velocity.push_back(10);
          js.name.push_back("hip_right_pitch");
          js.position.push_back(0);
          js.velocity.push_back(10);
          js.name.push_back("ankle_right_pitch");
          js.position.push_back(0);
          js.velocity.push_back(10);
          js.name.push_back("ankle_right_roll");
          js.position.push_back(0);
          js.velocity.push_back(10);                
          */          

					/*
					<joint name="base_yaw"  position="0.0" />
					<joint name="base_pitch"  position="0.0" />
					<joint name="base_roll"  position="0.0" />
					<joint name="base"  position="0.0" />
					<joint name="stereo_camera_fixed"  position="0.0" />
					*/                                 

          joint_states_pub_.publish(js);
			
        	break;	// only read first user
        }
      }
         
    private:
      ros::Publisher   motion_pub_;
      ros::Publisher   joint_states_pub_;
			bool             publish_kinect_tf_;
  };

} // namespace veltrobot_teleop


#define GL_WIN_SIZE_X 320
#define GL_WIN_SIZE_Y 240
KinectController g_kinect_controller;
veltrobot_teleop::TeleopKinect g_teleop_kinect;
bool g_running = false;
pthread_mutex_t g_kinect_data_mutex;

xn::UserGenerator g_user_genearator;
xn::DepthGenerator g_depth_genearator;

void glutIdle (void)
{
	// using loop_rate is less CPU intensive than an ROS timer.
	static ros::Rate loop_rate(15); // limit 15 fps display
	glutPostRedisplay();
	loop_rate.sleep();
}

void glutDisplay (void)
{
	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
	
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	
	pthread_mutex_lock(&g_kinect_data_mutex);	
	{
		g_depth_genearator.GetMetaData(depthMD);
		g_user_genearator.GetUserPixels(0, sceneMD); 	
		
		glOrtho(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);
		kinect_display_drawDepthMapGL(depthMD, sceneMD);
		kinect_display_drawSkeletonGL(g_user_genearator,
																	g_depth_genearator);
	}															
	pthread_mutex_unlock(&g_kinect_data_mutex);
  
	glutSwapBuffers();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
    exit(1);
    break;
	}
}

void* updateKinectThread(void *ptr)
{
	while (g_running)
	{
		g_kinect_controller.getContext().WaitAndUpdateAll();
		g_teleop_kinect.processKinect(g_kinect_controller);
		
		pthread_mutex_lock(&g_kinect_data_mutex);	
		{		
			g_depth_genearator = g_kinect_controller.getDepthGenerator();
			g_user_genearator = g_kinect_controller.getUserGenerator();
		}
		pthread_mutex_unlock(&g_kinect_data_mutex);
	}
	
	return NULL;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_kinect");
	ros::NodeHandle np("~");
  
	string filepath;
	bool is_recording;
	np.getParam("load_filepath", filepath); 
	np.param<bool>("load_recording", is_recording, false);      
		
	g_teleop_kinect.init();
	g_kinect_controller.init(filepath.c_str(), is_recording);
  
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow ("Veltrobot Kinect Controller");
	//glutFullScreen();
	//glutSetCursor(GLUT_CURSOR_NONE);
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);  

  g_running = true;
	static pthread_t kinect_thread;
	static pthread_attr_t kinect_thread_attr;
	
	pthread_mutex_init (&g_kinect_data_mutex, NULL);

	int rv;
	if ((rv = pthread_create(&kinect_thread, &kinect_thread_attr, updateKinectThread, NULL)) != 0)
		ROS_FATAL("Unable to create kinect thread: rv = %d", rv);
  glutMainLoop();
  g_running = false;
	pthread_join(kinect_thread, NULL);//(void **)&rv);
		
  g_kinect_controller.shutdown();
  
	return 0;
}
