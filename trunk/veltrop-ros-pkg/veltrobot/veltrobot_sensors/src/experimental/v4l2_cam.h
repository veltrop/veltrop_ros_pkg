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

// Based on usb_cam.h By Robert Bosch in the bosch-ros-pkg repository.
// Changes by Taylor Veltrop:
// - C++ version of library using class to allow multiple capture devices.
// - Attepted to preserve original coding style where possible.

#ifndef USB_CAM_USB_CAM_H
#define USB_CAM_USB_CAM_H


extern "C" {
//#include <linux/videodev2.h>
//#include <libavcodec/avcodec.h>
//#include <libswscale/swscale.h>
#include <ffmpeg/avcodec.h>
#include <ffmpeg/swscale.h>
}

#include <string>
#include <sstream>

class v4l2Cam
{
public:
  v4l2Cam();
  ~v4l2Cam();
      
  typedef struct {
    int width;
    int height;
    int bytes_per_pixel;
    int image_size;
    char *image;
    int is_new;
  } usb_cam_camera_image_t;

  typedef enum {
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
  } usb_cam_io_method;

  typedef enum {
    PIXEL_FORMAT_YUYV,
    PIXEL_FORMAT_UYVY,
    PIXEL_FORMAT_MJPEG,
  } usb_cam_pixel_format;

  // start camera
  usb_cam_camera_image_t *usb_cam_camera_start(const char* dev, usb_cam_io_method io, usb_cam_pixel_format pf, int image_width, int image_height);
  // shutdown camera
  void usb_cam_camera_shutdown(void);
  // grabs a new image from the camera
  void usb_cam_camera_grab_image(usb_cam_camera_image_t *image);  
  
private:
  struct buffer {
    void * start;
    size_t length;
  };

  void errno_exit(const char * s);
  int xioctl(int fd, int request, void * arg);
  void YUV2RGB(const unsigned char y,
        const unsigned char u,
        const unsigned char v,
        unsigned char* r,
        unsigned char* g,
        unsigned char* b);
  void uyvy2rgb (char *YUV, char *RGB, int NumPixels);
  void yuyv2rgb(char *YUV, char *RGB, int NumPixels);
  int init_mjpeg_decoder(int image_width, int image_height);
  void mjpeg2rgb(char *MJPEG, int len, char *RGB, int NumPixels);
  void process_image(const void * src, int len, usb_cam_camera_image_t *dest);
  int read_frame(usb_cam_camera_image_t *image);
  void stop_capturing(void);
  void start_capturing(void);
  void uninit_device(void);
  void init_read(unsigned int buffer_size);
  void init_mmap(void);
  void init_userp(unsigned int buffer_size);
  void init_device(int image_width, int image_height);
  void close_device(void);
  void open_device(void);
  
  
  char *camera_dev;
  unsigned int pixelformat;
  usb_cam_io_method io;
  int fd;
  buffer * buffers;
  unsigned int n_buffers;
  AVFrame *avframe_camera;
  AVFrame *avframe_rgb;
  AVCodec *avcodec;
  AVCodecContext *avcodec_context;
  int avframe_camera_size;
  int avframe_rgb_size;
  SwsContext *video_sws;
};

#endif

