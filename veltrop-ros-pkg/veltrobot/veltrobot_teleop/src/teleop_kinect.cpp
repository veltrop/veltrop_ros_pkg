#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <GL/glut.h>
#include <string>
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
      {    
      }
      
      void init()
      {
        ros::NodeHandle n;
        motion_pub_ = n.advertise <std_msgs::String> ("motion_name", 1);
        joint_states_pub_ = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
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
        XnUserID users[15];
        XnUInt16 users_count = 15;
        xn::UserGenerator& UserGenerator = kinect_controller.getUserGenerator();
        UserGenerator.GetUsers(users, users_count);

        for (int i = 0; i < users_count; ++i)
        {
          XnUserID user = users[i];
          if (!UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;
            
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
      		// 1? publish transform of two joints relative to openni_depth frame
          // 2? request transform between two joints own frames 
      		
          // 1 get points of three joints
          // 2 do many specific calculations relating that geometry to my own
          //   robots geometry

					// get joint positions
					XnSkeletonJointPosition joint_position;
        	UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_NECK, joint_position);
          KDL::Vector neck(joint_position.position.X, joint_position.position.Y, joint_position.position.Z);
        	UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HAND, joint_position);
        	KDL::Vector left_hand(joint_position.position.X, joint_position.position.Y, joint_position.position.Z);
        	UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_ELBOW, joint_position);
        	KDL::Vector left_elbow(joint_position.position.X, joint_position.position.Y, joint_position.position.Z);        
         	UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_SHOULDER, joint_position);
        	KDL::Vector left_shoulder(joint_position.position.X, joint_position.position.Y, joint_position.position.Z);        	        	
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HAND, joint_position);
        	KDL::Vector right_hand(joint_position.position.X, joint_position.position.Y, joint_position.position.Z);
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_ELBOW, joint_position);
        	KDL::Vector right_elbow(joint_position.position.X, joint_position.position.Y, joint_position.position.Z);        
         	UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_SHOULDER, joint_position);
        	KDL::Vector right_shoulder(joint_position.position.X, joint_position.position.Y, joint_position.position.Z);
        	
            
          
          // left elbow roll
          KDL::Vector left_elbow_hand(left_hand - left_elbow);
        	KDL::Vector left_elbow_shoulder(left_shoulder - left_elbow);
          left_elbow_hand.Normalize();
          left_elbow_shoulder.Normalize();
          double left_elbow_angle_roll = acos(KDL::dot(left_elbow_hand, left_elbow_shoulder));
          left_elbow_angle_roll = left_elbow_angle_roll - PI;
          
          // right elbow roll
        	KDL::Vector right_elbow_hand(right_hand - right_elbow);
        	KDL::Vector right_elbow_shoulder(right_shoulder - right_elbow);
          right_elbow_hand.Normalize();
          right_elbow_shoulder.Normalize();
          double right_elbow_angle_roll = acos(KDL::dot(right_elbow_hand, right_elbow_shoulder));
        	right_elbow_angle_roll = -(right_elbow_angle_roll - PI);  
                  
          // left shoulder roll
          KDL::Vector left_shoulder_elbow(left_elbow - left_shoulder);
        	KDL::Vector left_shoulder_neck(neck - left_shoulder);
          left_shoulder_elbow.Normalize();
          left_shoulder_neck.Normalize();
          double left_shoulder_angle_roll = acos(KDL::dot(left_shoulder_elbow, left_shoulder_neck));
          left_shoulder_angle_roll = left_shoulder_angle_roll - HALFPI;
                   
          // right shoulder roll
          KDL::Vector right_shoulder_elbow(right_elbow - right_shoulder);
        	KDL::Vector right_shoulder_neck(neck - right_shoulder);
          right_shoulder_elbow.Normalize();
          right_shoulder_neck.Normalize();
          double right_shoulder_angle_roll = acos(KDL::dot(right_shoulder_elbow, right_shoulder_neck));
          right_shoulder_angle_roll = -(right_shoulder_angle_roll - HALFPI);                                      
                   
          // left shoulder pitch
          double left_shoulder_angle_pitch = asin(left_shoulder_elbow.y());
          left_shoulder_angle_pitch = left_shoulder_angle_pitch + HALFPI;
          
  				// right shoulder pitch
          double right_shoulder_angle_pitch = asin(right_shoulder_elbow.y());
          right_shoulder_angle_pitch = -(right_shoulder_angle_pitch + HALFPI);
                                                        
          // left shoulder yaw
        	double left_shoulder_angle_yaw = asin(left_elbow_hand.y());  // left_shoulder_elbow.x()
          
          // right shoulder yaw
        	double right_shoulder_angle_yaw = asin(right_elbow_hand.y());  // left_shoulder_elbow.x()
					right_shoulder_angle_yaw = -right_shoulder_angle_yaw;
          
          // hand open/close or palm recognition to control grippers
          
          
          // PROBLEM:
          // doing each of the 3 degrees of freedom in the shoulder separately is having problems.
          // 1. the acos has no sign, it only reads less than one quater pi rotation
          // 2. there is a paradox/redundancy with joint movement to be considered: ie, to lift the
          //    arm vertically up it could either rotate the pitch or roll joint by one pi
          // SOLUTION?:
          // if we take the raw tf rotation from the input shoulder, can we apply that to our
          // final dof of our shoulder chain, and then back solve for the two joints leading to it?
        
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
          js.name.push_back("knee_left_pitch");
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
          js.name.push_back("knee_right_pitch");
          js.position.push_back(0);
          js.velocity.push_back(10);
          js.name.push_back("ankle_right_pitch");
          js.position.push_back(0);
          js.velocity.push_back(10);
          js.name.push_back("ankle_right_roll");
          js.position.push_back(0);
          js.velocity.push_back(10);                
          */          

//    <joint name="base_yaw"  position="0.0" />
//    <joint name="base_pitch"  position="0.0" />
//    <joint name="base_roll"  position="0.0" />
//    <joint name="base"  position="0.0" />
//    <joint name="stereo_camera_fixed"  position="0.0" />                                    

          joint_states_pub_.publish(js);
        
        
        	break;	// only read first user
        }
      }
         
    private:
      ros::Publisher   motion_pub_;
      ros::Publisher   joint_states_pub_;
  };

} // namespace veltrobot_teleop


#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480
KinectController g_kinect_controller;
veltrobot_teleop::TeleopKinect g_teleop_kinect;

void glutIdle (void)
{
	glutPostRedisplay();
}

void glutDisplay (void)
{
	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
  
  g_kinect_controller.getContext().WaitAndUpdateAll();
  g_teleop_kinect.processKinect(g_kinect_controller);
  g_kinect_controller.getDepthGenerator().GetMetaData(depthMD);
  g_kinect_controller.getUserGenerator().GetUserPixels(0, sceneMD);  
	
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Setup the OpenGL viewpoint
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glOrtho(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);
  
  glDisable(GL_TEXTURE_2D);
  
  kinect_display_drawDepthMapGL(depthMD, sceneMD);
	kinect_display_drawSkeletonGL(g_kinect_controller.getUserGenerator(),
                                g_kinect_controller.getDepthGenerator());  
  
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_kinect");
  ros::NodeHandle n;
  g_teleop_kinect.init();
  char* onifile = NULL;
  if (argc > 1)
  	onifile = argv[1];
  g_kinect_controller.init(onifile);
  
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
  
  glutMainLoop();
  
  g_kinect_controller.shutdown();
  
	return 0;
}
