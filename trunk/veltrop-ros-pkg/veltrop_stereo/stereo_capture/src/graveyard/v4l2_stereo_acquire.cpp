/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Taylor Veltrop
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Taylor Veltrop nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
 
// Based on usb_cam_node.cpp By Robert Bosch in the bosch-ros-pkg repository.
 
#include <stdio.h>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <v4l2_cam/v4l2_cam.h>
#include <image_transport/image_transport.h>

class StereoUsbCamNode
{
public:
  ros::NodeHandle node_;
  
  sensor_msgs::Image left_img_, right_img_;
  sensor_msgs::CameraInfo left_info_, right_info_;
  image_transport::CameraPublisher left_image_pub_, right_image_pub_;

  int fps_;
  std::string left_video_device_name_, right_video_device_name_;
  std::string left_io_method_name_, right_io_method_name_;
  int left_image_width_,left_image_height_,right_image_width_,right_image_height_;
  std::string left_pixel_format_name_, right_pixel_format_name_;

  //ros::Time next_time_;
  //int count_;

  v4l2Cam::usb_cam_camera_image_t* left_camera_image_;
  v4l2Cam::usb_cam_camera_image_t* right_camera_image_;
  v4l2Cam left_camera_, right_camera_;

  StereoUsbCamNode() :
      node_("~")
  {
    ros::NodeHandle left_node_("left");
    ros::NodeHandle right_node_("right");
  
    image_transport::ImageTransport left_it(left_node_);
    image_transport::ImageTransport right_it(right_node_);
    left_image_pub_ = left_it.advertiseCamera("image_raw", 1);
    right_image_pub_ = right_it.advertiseCamera("image_raw", 1);

    node_.param("left_video_device", left_video_device_name_, std::string("/dev/video0"));
    node_.param("left_io_method", left_io_method_name_, std::string("mmap")); // possible values: mmap, read, userptr
    node_.param("left_image_width", left_image_width_, 640);
    node_.param("left_image_height", left_image_height_, 480);
    node_.param("left_pixel_format", left_pixel_format_name_, std::string("mjpeg")); // possible values: yuyv, uyvy, mjpeg
    node_.param("right_video_device", right_video_device_name_, std::string("/dev/video1"));
    node_.param("right_io_method", right_io_method_name_, std::string("mmap")); // possible values: mmap, read, userptr
    node_.param("right_image_width", right_image_width_, 640);
    node_.param("right_image_height", right_image_height_, 480);
    node_.param("right_pixel_format", right_pixel_format_name_, std::string("mjpeg")); // possible values: yuyv, uyvy, mjpeg
    node_.param("fps", fps_, 2);

    {
      XmlRpc::XmlRpcValue double_list;
      left_info_.height = left_image_height_;
      left_info_.width = left_image_width_;

      node_.param("left_camera_frame_id", left_img_.header.frame_id, std::string("left_camera"));
      left_info_.header.frame_id = left_img_.header.frame_id;

      node_.getParam("left_K", double_list);
      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 9)) {
        for (int i=0; i<9; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          left_info_.K[i] = double_list[i];
        }
      }

      node_.getParam("left_D", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 5)) {
        for (int i=0; i<5; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          left_info_.D[i] = double_list[i];
        }
      }

      node_.getParam("left_R", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 9)) {
        for (int i=0; i<9; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          left_info_.R[i] = double_list[i];
        }
      }

      node_.getParam("left_P", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 12)) {
        for (int i=0; i<12; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          left_info_.P[i] = double_list[i];
        }
      }
    }
    
    {
      XmlRpc::XmlRpcValue double_list;
      right_info_.height = right_image_height_;
      right_info_.width = right_image_width_;

      node_.param("right_camera_frame_id", right_img_.header.frame_id, std::string("right_camera"));
      right_info_.header.frame_id = right_img_.header.frame_id;

      node_.getParam("right_K", double_list);
      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 9)) {
        for (int i=0; i<9; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          right_info_.K[i] = double_list[i];
        }
      }

      node_.getParam("right_D", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 5)) {
        for (int i=0; i<5; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          right_info_.D[i] = double_list[i];
        }
      }

      node_.getParam("right_R", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 9)) {
        for (int i=0; i<9; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          right_info_.R[i] = double_list[i];
        }
      }

      node_.getParam("right_P", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 12)) {
        for (int i=0; i<12; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          right_info_.P[i] = double_list[i];
        }
      }
    }

    //printf("usb_cam video_device set to [%s]\n", video_device_name_.c_str());
    //printf("usb_cam io_method set to [%s]\n", io_method_name_.c_str());
    //printf("usb_cam image_width set to [%d]\n", image_width_);
    //printf("usb_cam image_height set to [%d]\n", image_height_);
    //printf("usb_cam pixel_format set to [%s]\n", pixel_format_name_.c_str());

    v4l2Cam::usb_cam_io_method left_io_method;
    if(left_io_method_name_ == "mmap")
      left_io_method = v4l2Cam::IO_METHOD_MMAP;
    else if(left_io_method_name_ == "read")
      left_io_method = v4l2Cam::IO_METHOD_READ;
    else if(left_io_method_name_ == "userptr")
      left_io_method = v4l2Cam::IO_METHOD_USERPTR;
    else {
      ROS_FATAL("Unknown left io method.");
      node_.shutdown();
      return;
    }
    
    v4l2Cam cam;
    
   
    
    v4l2Cam::usb_cam_io_method right_io_method;
    if(right_io_method_name_ == "mmap")
      right_io_method = v4l2Cam::IO_METHOD_MMAP;
    else if(right_io_method_name_ == "read")
      right_io_method = v4l2Cam::IO_METHOD_READ;
    else if(right_io_method_name_ == "userptr")
      right_io_method = v4l2Cam::IO_METHOD_USERPTR;
    else {
      ROS_FATAL("Unknown right io method.");
      node_.shutdown();
      return;
    }

    v4l2Cam::usb_cam_pixel_format left_pixel_format;
    if(left_pixel_format_name_ == "yuyv")
      left_pixel_format = v4l2Cam::PIXEL_FORMAT_YUYV;
    else if(left_pixel_format_name_ == "uyvy")
      left_pixel_format = v4l2Cam::PIXEL_FORMAT_UYVY;
    else if(left_pixel_format_name_ == "mjpeg") {
      left_pixel_format = v4l2Cam::PIXEL_FORMAT_MJPEG;
    }
    else {
      ROS_FATAL("Unknown pixel format.");
      node_.shutdown();
      return;
    }
    
    v4l2Cam::usb_cam_pixel_format right_pixel_format;
    if(right_pixel_format_name_ == "yuyv")
      right_pixel_format = v4l2Cam::PIXEL_FORMAT_YUYV;
    else if(right_pixel_format_name_ == "uyvy")
      right_pixel_format = v4l2Cam::PIXEL_FORMAT_UYVY;
    else if(right_pixel_format_name_ == "mjpeg") {
      right_pixel_format = v4l2Cam::PIXEL_FORMAT_MJPEG;
    }
    else {
      ROS_FATAL("Unknown pixel format.");
      node_.shutdown();
      return;
    }

    left_camera_image_ = left_camera_.usb_cam_camera_start(left_video_device_name_.c_str(),
        left_io_method,
        left_pixel_format,
        left_image_width_,
        left_image_height_);
        
    right_camera_image_ = right_camera_.usb_cam_camera_start(right_video_device_name_.c_str(),
        right_io_method,
        right_pixel_format,
        right_image_width_,
        right_image_height_);        

    //next_time_ = ros::Time::now();
    //count_ = 0;
  }
  
  virtual ~StereoUsbCamNode()
  {
    left_camera_.usb_cam_camera_shutdown();
    right_camera_.usb_cam_camera_shutdown();
  }

  bool take_and_send_images()
  {
    left_camera_.usb_cam_camera_grab_image(left_camera_image_);
    right_camera_.usb_cam_camera_grab_image(right_camera_image_);    

    left_img_.header.stamp = ros::Time::now();
    left_info_.header.stamp = left_img_.header.stamp;
    right_img_.header.stamp = left_img_.header.stamp;
    right_info_.header.stamp = left_img_.header.stamp;

    fillImage(left_img_, "rgb8", left_camera_image_->height,
              left_camera_image_->width, 3 * left_camera_image_->width,
              left_camera_image_->image);
    fillImage(right_img_, "rgb8", right_camera_image_->height,
              right_camera_image_->width, 3 * right_camera_image_->width,
              right_camera_image_->image);

    left_image_pub_.publish(left_img_, left_info_);
    right_image_pub_.publish(right_img_, right_info_);    
    
    return true;
  }


  bool spin()
  {
    
    while (node_.ok())
    {
      ros::Rate loop_rate(fps_);
      if (take_and_send_images())
      {
        
        //count_++;
        //ros::Time now_time = ros::Time::now();
        //if (now_time > next_time_)
        //{
        //  std::cout << count_ << " frames/sec at " << now_time << std::endl;
        //  count_ = 0;
        //  next_time_ = next_time_ + ros::Duration(1,0);
        //}
        loop_rate.sleep();
      } else {
        ROS_ERROR("couldn't take image.");
        usleep(1000000);
      }
    }
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  StereoUsbCamNode a;
  a.spin();
  return 0;
}
