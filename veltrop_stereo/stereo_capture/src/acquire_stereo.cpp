// purpose of this node is to losslessly use as little bandwidth and cpu as possible.
// therefore, images are not converted from yuyv(16bpp) to rgb(24bpp).
// even with the smaller yuyv images, 802.11g networks are still quickly saturated.
// image_transport compression methods require costly rgb conversion, and employ
// lossy compression, so it is not used. also, small cpus cant handle the compression.

// with code from stereo_sender.cpp of the sail-ros-pkg repository.
// also inspired with code from stereo.cpp from uvc_camera of camera_umd repo.

// TODO: separate parameter for transmit_fps and hardware_fps

#include <cstdio>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_capture/PairYUYVImages.h>
//#include <uvc_cam/uvc_cam.h>
#include "uvc_cam.h"  // use our modified version

int main(int argc, char **argv)
{
  ros::init(argc, argv, "acquire_stereo");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~"); 

  std::string left_device, right_device, stereo_name, standard_str;
  int width, height, fps, buffers;
  n_private.param<std::string>("stereo_name", stereo_name, "yuyv_pair");
  n_private.param<std::string>("left_device", left_device, "/dev/video0");
  n_private.param<std::string>("right_device", right_device, "/dev/video1");
  n_private.param<std::string>("standard_str", standard_str, "DEFAULT");
  n_private.param("width", width, 640);
  n_private.param("height", height, 480);
  n_private.param("fps", fps, 1);
  n_private.param("buffers", buffers, 2);

  ros::Publisher pub = n.advertise<stereo_capture::PairYUYVImages>(stereo_name.c_str(), 1);   

  uvc_cam::Cam::std_t standard;
  standard = (standard_str == "NTSC") ? uvc_cam::Cam::STD_NTSC : uvc_cam::Cam::STD_DEFAULT;
  
  ROS_INFO("opening uvc_cam's %s %s at %dx%d, %d fps",
           left_device.c_str(), right_device.c_str(), width, height, fps);
  uvc_cam::Cam camL(left_device.c_str(), uvc_cam::Cam::MODE_YUYV, width, height,
                    buffers, fps, standard);
  uvc_cam::Cam camR(right_device.c_str(), uvc_cam::Cam::MODE_YUYV, width, height, 
                    buffers, fps, standard);

  stereo_capture::PairYUYVImages images;
  images.left_image.encoding = images.right_image.encoding = sensor_msgs::image_encodings::TYPE_8UC2;
  images.left_image.height = images.right_image.height = height;
  images.left_image.width = images.right_image.width = width;
  images.left_image.step = images.right_image.step = 2 * width;
  images.left_image.data.resize(images.left_image.step * images.left_image.height);
  images.right_image.data.resize(images.right_image.step * images.right_image.height);
  
  unsigned char *frameL=NULL, *frameR=NULL;
  uint32_t bytes_usedL, bytes_usedR;
  int buf_idxL=-1, buf_idxR=-1;
  unsigned int image_bytes = width * height * 2;
  images.header.seq = 0;
  
  ros::Rate loop_rate(fps);
  while (n.ok())
  {
    if (frameL != NULL)
      camL.release(buf_idxL);
    if (frameR != NULL)  
      camR.release(buf_idxR);
  
    buf_idxL = camL.grab(&frameL, bytes_usedL);
    buf_idxR = camR.grab(&frameR, bytes_usedR);

    if (frameL != NULL && frameR != NULL)
    { 
      images.header.seq++;
      images.header.stamp = ros::Time::now();
          
      memcpy(&images.left_image.data[0], frameL, image_bytes);
      memcpy(&images.right_image.data[0], frameR, image_bytes);
      
      pub.publish(images);
      
      loop_rate.sleep();
    }
  }
  
  if (frameL != NULL)
    camL.release(buf_idxL);
  if (frameR != NULL)  
    camR.release(buf_idxR);
  
  return 0;
}

