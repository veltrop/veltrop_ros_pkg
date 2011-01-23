#include <ros/ros.h>
#include <GL/glut.h>
#include <pthread.h>
#include <string>
#include "KinectController.h"
#include "KinectDisplay.h"
#include "KinectTeleop.h"

using std::string;

#define GL_WIN_SIZE_X 352
#define GL_WIN_SIZE_Y 288

KinectController g_kinect_controller;
veltrobot_teleop::KinectTeleop g_teleop_kinect;
bool g_running = false;
pthread_mutex_t g_kinect_data_mutex;

xn::UserGenerator g_user_genearator;
xn::DepthGenerator g_depth_genearator;

void glutIdle (void)
{
	// using loop_rate is less CPU intensive than an ROS timer.
	static ros::Rate loop_rate(15); // limit to 15 fps display
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

void* spinThread(void *ptr)
{
  ros::spin();
  return NULL;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_kinect");
	ros::NodeHandle np("~");
  
	string filepath;
	bool is_a_recording;
	np.getParam("load_filepath", filepath); 
	np.param<bool>("load_recording", is_a_recording, false);      
		
	g_teleop_kinect.init();
	g_kinect_controller.init(filepath.c_str(), is_a_recording);
	
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

	static pthread_t kinect_thread;
	static pthread_attr_t kinect_thread_attr;
	static pthread_t spin_thread;
	static pthread_attr_t spin_thread_attr;
  	
	g_running = true;
	pthread_mutex_init (&g_kinect_data_mutex, NULL);

	int rv;
	if ((rv = pthread_create(&kinect_thread, &kinect_thread_attr, updateKinectThread, NULL)) != 0)
		ROS_FATAL("Unable to create kinect thread: rv = %d", rv);
	int rv2;
	if ((rv2 = pthread_create(&spin_thread, &spin_thread_attr, spinThread, NULL)) != 0)
		ROS_FATAL("Unable to create spin thread: rv = %d", rv);    
  
	usleep(1000000); // lame way to make sure data is ready before starting the display... (prevents a segfault)
	
	glutMainLoop();

	g_running = false;
  pthread_join(kinect_thread, NULL); //(void **)&rv);
  g_kinect_controller.shutdown();
  ros::shutdown();
	pthread_join(spin_thread, NULL);
  
	return 0;
}
