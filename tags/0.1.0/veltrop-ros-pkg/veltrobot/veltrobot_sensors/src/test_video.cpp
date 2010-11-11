// with code from stereo_sender.cpp of the sail-ros-pkg repository.

#include <cstdio>
//#include <opencv/highgui.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
//#include <veltrobot_sensors/PairYUYVImages.h>
#include <image_transport/image_transport.h>
//#include <uvc_cam/uvc_cam.h>
#include "uvc_cam.h"

// YUYV to RGB conversion
// There's many methods to this, and much ado about the color range...
// The below method nice because its only integer.
#define SAT(c) if (c & (~255)) { if (c < 0) c = 0; else c = 255; }
static void
//yuyv16_to_bgr24(int width, int height, const unsigned char *src, unsigned char *dst)
yuyv16_to_rgb24(int width, int height, const unsigned char *src, unsigned char *dst)
{
  unsigned char *s;
  unsigned char *d;
  int l, c;
  int r, g, b, cr, cg, cb, y1, y2;
  
  l = height;
  s = (unsigned char*)src;
  d = dst;
  while (l--)
  {
    c = width >> 1;
    while (c--)
    {
      y1 = *s++;
      cb = ((*s - 128) * 454) >> 8;
      cg = (*s++ - 128) * 88;
      y2 = *s++;
      cr = ((*s - 128) * 359) >> 8;
      cg = (cg + (*s++ - 128) * 183) >> 8;
      
      r = y1 + cr;
      b = y1 + cb;
      g = y1 - cg;
      SAT(r);
      SAT(g);
      SAT(b);
      
      *d++ = b;
      *d++ = g;
      *d++ = r;
      //*d++ = r;
      //*d++ = g;
      //*d++ = b;
      
      r = y2 + cr;
      b = y2 + cb;
      g = y2 - cg;
      SAT(r);
      SAT(g);
      SAT(b);
      
      *d++ = b;
      *d++ = g;
      *d++ = r;
      //*d++ = r;
      //*d++ = g;
      //*d++ = b;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_video");
  //cvInitSystem(argc, argv);
  //cvStartWindowThread();
  ros::NodeHandle n_private("~"); 
  //cvNamedWindow("video");

  std::string device, standard_str, topic;
  int width, height, fps, buffers;
  n_private.param<std::string>("device", device, "/dev/video0");
  n_private.param<std::string>("standard_str", standard_str, "DEFAULT");
  n_private.param<std::string>("topic", topic, "/image_raw");
  n_private.param("width", width, 640);
  n_private.param("height", height, 480);
  n_private.param("fps", fps, 1);
  n_private.param("buffers", buffers, 1);

  image_transport::ImageTransport it(n_private);
  image_transport::Publisher image_pub = it.advertise(topic.c_str(), 1);
  
  uvc_cam::Cam::std_t standard;
  standard = (standard_str == "NTSC") ? uvc_cam::Cam::STD_NTSC : uvc_cam::Cam::STD_DEFAULT;
  
  ROS_INFO("opening uvc_cam %s at %dx%d, %d fps", device.c_str(), width, height, fps);
  uvc_cam::Cam cam(device.c_str(), uvc_cam::Cam::MODE_YUYV, width, height,
                    buffers, fps, standard);
  
  unsigned char *frame=NULL;
  uint32_t bytes_used;
  int buf_idx=-1;
  
  IplImage *ipl = cvCreateImage(cvSize(width,height), 8, 3);
  
  ros::Rate loop_rate(fps);
  while (n_private.ok())
  {
    if (frame != NULL)
      cam.release(buf_idx);
  
    buf_idx = cam.grab(&frame, bytes_used);

    if (frame != NULL)
    {           
      //memcpy(DEST, frame, image_bytes);
      yuyv16_to_rgb24(width, height, frame, (unsigned char*)ipl->imageData);      
      //cvShowImage("video", ipl);
      sensor_msgs::Image::Ptr image = sensor_msgs::CvBridge::cvToImgMsg(ipl);
      image_pub.publish(image);
      
      loop_rate.sleep();
    }
  }
  
  if (frame != NULL)
    cam.release(buf_idx);
  
  return 0;
}

