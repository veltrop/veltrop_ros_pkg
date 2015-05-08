// http://bobobobo.wordpress.com/2008/02/23/how-to-use-zlib/

// purpose of this node is to losslessly use as little bandwidth and cpu as possible.
// therefore, images are not converted from yuyv(16bpp) to rgb(24bpp).
// even with the smaller yuyv images, 802.11g networks are still quickly saturated.
// image_transport compression methods require costly rgb conversion, and employ
// lossy compression, so it is not used.  zlib is used.

//#include <zlib.h>
#include <cstdio>
#include <ros/ros.h>
#include <ros/time.h>
//#include <std_msgs/UInt8MultiArray.h>
//#include <std_msgs/MultiArrayDimension.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
//#include <uvc_cam/uvc_cam.h>
#include "uvc_cam.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uvc_stereo_acquire_compact");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~"); 

  std::string deviceL, deviceR, stereoName, imageName;
  int width, height, fps, compression, buffers;
  n_private.param<std::string>("deviceL", deviceL, "/dev/video0");
  n_private.param<std::string>("deviceR", deviceR, "/dev/video1");
  n_private.param<std::string>("stereoName", stereoName, "/stereo");
  n_private.param<std::string>("imageName", imageName, "image_yuyv_z");
  n_private.param("width", width, 480);
  n_private.param("height", height, 320);
  n_private.param("fps", fps, 1);
  n_private.param("buffers", buffers, 1);
  n_private.param("compression", compression, 6);
  //n_private.param("std", std, "NTSC");

  std::string left = stereoName + "/left/" + imageName;
  std::string right = stereoName + "/right/" + imageName;

  image_transport::ImageTransport itL(n);
  image_transport::Publisher pubL = itL.advertise(left.c_str(), 1);
  image_transport::ImageTransport itR(n);
  image_transport::Publisher pubR = itR.advertise(right.c_str(), 1);

  //ros::Publisher pubL = n.advertise<std_msgs::UInt8MultiArray>(left.c_str(), 1);   
  //ros::Publisher pubR = n.advertise<std_msgs::UInt8MultiArray>(right.c_str(), 1);  

  ROS_INFO("opening uvc_cam's %s %s at %dx%d, %d fps",
           deviceL.c_str(), deviceR.c_str(), width, height, fps);
  uvc_cam::Cam camL(deviceL.c_str(), uvc_cam::Cam::MODE_YUYV, width, height,
                    buffers, fps, uvc_cam::Cam::STD_DEFAULT);//uvc_cam::Cam::STD_NTSC);//
  uvc_cam::Cam camR(deviceR.c_str(), uvc_cam::Cam::MODE_YUYV, width, height, 
                    buffers, fps, uvc_cam::Cam::STD_DEFAULT);//uvc_cam::Cam::STD_NTSC);//STD_DEFAULT);


  sensor_msgs::Image imageL, imageR; 
  imageL.encoding = imageR.encoding = sensor_msgs::image_encodings::TYPE_8UC2;
  imageL.height = imageR.height = height;
  imageL.width = imageR.width = width;
  imageL.step = imageR.step = 2 * width;
  imageL.data.resize(imageL.step * imageL.height);
  imageR.data.resize(imageR.step * imageR.height);


  //unsigned long frame_size_original = height * width * 2;
  //unsigned long frameL_size_compressed, frameR_size_compressed;
  //frameL_size_compressed = frameR_size_compressed = (frame_size_original * 1.01) + 12; 
  //unsigned char* frameL_compressed = (unsigned char*)malloc(frameL_size_compressed);
  //unsigned char* frameR_compressed = (unsigned char*)malloc(frameR_size_compressed);
 
  
  //std_msgs::UInt8MultiArray msgL, msgR;
  
  unsigned char *frameL, *frameR;
  frameL = frameR = NULL;
  uint32_t bytes_usedL, bytes_usedR;
  
  int buf_idxL, buf_idxR;
  while (frameL == NULL || frameR == NULL)
  {
    if (frameL != NULL)
      camL.release(buf_idxL);
    if (frameR != NULL)  
      camR.release(buf_idxR);
    buf_idxL = camL.grab(&frameL, bytes_usedL);
    buf_idxR = camR.grab(&frameR, bytes_usedR);
  }
  
  ros::Rate loop_rate(fps);
  while (n.ok())
  {
    if (frameL != NULL)
      camL.release(buf_idxL);
    if (frameR != NULL)  
      camR.release(buf_idxR);
  
    //ros::Time t = ros::Time::now();
    buf_idxL = camL.grab(&frameL, bytes_usedL);
    //printf("L grab: %f\n", (ros::Time::now() - t).toSec());
    //t = ros::Time::now();
    buf_idxR = camR.grab(&frameR, bytes_usedR);
    //printf("R grab: %f\n", (ros::Time::now() - t).toSec());

    if (frameL != NULL && frameR != NULL)
    { 
      imageL.header.stamp = imageR.header.stamp = ros::Time::now();
    
      //camL.release(buf_idxL);
      //camR.release(buf_idxR);
      
      memcpy(&imageL.data[0], frameL, width * height * 2);
      memcpy(&imageR.data[0], frameR, width * height * 2);
      
      //camL.release(buf_idxL);
      //camR.release(buf_idxR);
      
      // TODO: call compressBound(sourceLen)!!!!! to get this...
//      frameL_size_compressed = frameR_size_compressed = (frame_size_original * 1.01) + 12; 
//      msgL.data.resize(frameL_size_compressed);
//      msgR.data.resize(frameR_size_compressed);
      
//      int l_result = compress2(&msgL.data[0]/*frameL_compressed*/, &frameL_size_compressed,
//                              frameL, frame_size_original, compression);
//      int r_result = compress2(&msgR.data[0]/*frameR_compressed*/, &frameR_size_compressed,
//                              frameR, frame_size_original, compression);
                                                            
      // TODO: try compress2(same, same, same, same, int level)
      // default level = 6, use 0~9, 0 = no compression, 1 = fast, 9 = slow.
      
//      ROS_INFO("original: %d, compressed: %d\n", frame_size_original, frameR_size_compressed);
      
/*      switch (l_result)
      {
        case Z_MEM_ERROR:
          ROS_ERROR("L out of memory\n");
          exit(1);    // quit.
          break;
        case Z_BUF_ERROR:
          ROS_ERROR("L output buffer wasn't large enough\n");
          exit(1);    // quit.
          break;
      }
      switch (r_result)
      {
        case Z_MEM_ERROR:
          ROS_ERROR("R out of memory\n");
          exit(1);    // quit.
          break;
        case Z_BUF_ERROR:
          ROS_ERROR("R output buffer wasn't large enough\n");
          exit(1);    // quit.
          break;
      }
      
      msgL.data.resize(frameL_size_compressed);
      msgR.data.resize(frameR_size_compressed);
      */
      //memcpy(&msgL.data[0], frameL_compressed, frameL_size_compressed);
      //memcpy(&msgR.data[0], frameR_compressed, frameR_size_compressed);
      
      /*std_msgs::MultiArrayDimension dimL, dimR;
      dimL.label="X";
      dimL.stride=dimL.size=msgL.data.size();
      msgL.layout.dim.push_back(dimL);   
      dimR.label="X";
      dimR.stride=dimR.size=msgR.data.size();
      msgR.layout.dim.push_back(dimR);*/
      
      //pubL.publish(msgL);
      //pubR.publish(msgR);
      //imageL.header.stamp = imageR.header.stamp = ros::Time::now();
      pubL.publish(imageL);
      pubR.publish(imageR);
      
      //camL.release(buf_idxL);
      //camR.release(buf_idxR);
      
      loop_rate.sleep();
    }
  }
  
  //free(frameL_compressed);
  //free(frameR_compressed);
  
  return 0;
}

