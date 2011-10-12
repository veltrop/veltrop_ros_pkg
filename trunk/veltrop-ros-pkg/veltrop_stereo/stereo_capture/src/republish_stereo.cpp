// also inspired with code from stereo.cpp from uvc_camera of camera_umd repo.

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <stereo_capture/PairYUYVImages.h>

namespace stereo_capture
{

struct pixel_t {
  unsigned char r, g, b;
};

/* Rotate an 8-bit, 3-channel image by 90 degrees CCW. */
static inline void rotate90CCW(unsigned char *dst_chr, const unsigned char *src_chr, int dstWidth, int dstHeight) {

  struct pixel_t *src = (pixel_t *) src_chr;

  for (int i = 0; i < dstWidth; ++i) {
    for (int j = dstHeight-1; j >= 0; --j) {
      struct pixel_t *dst = &(((pixel_t *) dst_chr)[j * dstWidth + i]);
      *dst = *src;
      src++;
    }
  } 
}

/* Rotate an 8-bit, 3-channel image by 180 degrees. */
/*static inline void rotate180(unsigned char *dst_chr, unsigned char *src_chr, int pixels) {

  struct pixel_t *src = (pixel_t *) src_chr;
  struct pixel_t *dst = &(((pixel_t *) dst_chr)[pixels - 1]);

  for (int i = pixels; i != 0; --i) {
    *dst = *src;
    src++;
    dst--;
  }
}*/

// TODO: save CPU: integrate rotation in to yuyv -> rgb conversion to only step thru the image one time
// YUYV to RGB conversion
// There's many methods to this, and much ado about the color range...
// The below method nice because its only integer.
#define SAT(c) if (c & (~255)) { if (c < 0) c = 0; else c = 255; }
static void
yuyv16_to_rgb24(unsigned char *dst, const unsigned char *src, int width, int height)
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

         *d++ = r;
         *d++ = g;
         *d++ = b;
        
         r = y2 + cr;
         b = y2 + cb;
         g = y2 - cg;
         SAT(r);
         SAT(g);
         SAT(b);

         *d++ = r;
         *d++ = g;
         *d++ = b;
      }
   }
}

class RepublishStereo
{
public:
  RepublishStereo()
  : n_()
  , it_(n_)
  , left_info_mgr_(ros::NodeHandle(n_, "left"), "left_camera")
  , right_info_mgr_(ros::NodeHandle(n_, "right"), "right_camera")
  {
    ros::NodeHandle n_private("~"); 
    std::string input_stereo_name;
    
    n_private.param<std::string>("stereo_name", input_stereo_name, "yuyv_pair");
    n_private.param<std::string>("left_frame_id", left_frame_id_, "stereo_camera");
    n_private.param<std::string>("right_frame_id", right_frame_id_, "stereo_camera");
    n_private.param<bool>("rotate90ccw", rotate90ccw_, false);
  
    std::string left_url, right_url;
    n_private.getParam("left_camera_info_url", left_url);
    n_private.getParam("right_camera_info_url", right_url);
    left_info_mgr_.loadCameraInfo(left_url);
    right_info_mgr_.loadCameraInfo(right_url);
        
    left_image_pub_ = it_.advertise("left/image_raw", 1);
    right_image_pub_ = it_.advertise("right/image_raw", 1);
    
    left_info_pub_ = n_.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
    right_info_pub_ = n_.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);
    
    stereo_sub_ = n_.subscribe(input_stereo_name, 1, &RepublishStereo::getStereoCB, this);
  }
  
  ~RepublishStereo()
  {
  }
  
  void spin()
  {
    t_prev_ = ros::Time::now();
    frames_ = 0;
    ros::spin();
  }

private:
  ros::NodeHandle n_; 
  ros::Subscriber stereo_sub_;
  ros::Time t_prev_;
  int frames_;
  image_transport::ImageTransport it_;
  image_transport::Publisher left_image_pub_, right_image_pub_;
  ros::Publisher left_info_pub_, right_info_pub_;
  CameraInfoManager left_info_mgr_, right_info_mgr_;
  std::string left_frame_id_, right_frame_id_;
  bool rotate90ccw_;
    
  void doFps()
  {
    if (++frames_ == 10)
    {
      frames_ = 0;
      ros::Duration d = ros::Time::now() - t_prev_;
      t_prev_ = ros::Time::now();
      float fps = 10.0f / d.toSec();
      ROS_INFO("Recent 10 image pairs received at %f fps\n", fps);
    }  
  }
  
  void getStereoCB(const stereo_capture::PairYUYVImagesConstPtr& msg)
  {
    ros::Duration d = ros::Time::now() - msg->header.stamp;
    ROS_INFO("Image pair receive delay: %f\n", d.toSec());
    doFps();

    int width, height;
    if (rotate90ccw_)
    {
      width  = msg->left_image.height;
      height = msg->left_image.width;
    }
    else
    {
      height = msg->left_image.height;
      width  = msg->left_image.width;    
    }
                        
    // prepare image messages
    sensor_msgs::ImagePtr left_image(new sensor_msgs::Image);
    sensor_msgs::ImagePtr right_image(new sensor_msgs::Image);    
    left_image->header.frame_id  = left_frame_id_;
    right_image->header.frame_id = right_frame_id_;
    left_image->header.stamp     = right_image->header.stamp = msg->header.stamp;
    left_image->header.seq       = right_image->header.seq   = msg->header.seq;
    left_image->encoding         = right_image->encoding     = sensor_msgs::image_encodings::RGB8;
    left_image->height           = right_image->height       = height;
    left_image->width            = right_image->width        = width;
    left_image->step             = right_image->step         = 3 * width;    

    // process image messages
    left_image->data.resize(left_image->step * left_image->height);
	  right_image->data.resize(right_image->step * right_image->height);                  

    if (rotate90ccw_)
    {
      unsigned char* tmp = new unsigned char(left_image->step * left_image->height);
      
	    yuyv16_to_rgb24(tmp, &msg->left_image.data[0],
                      msg->left_image.width, msg->left_image.height);                                
      rotate90CCW(&left_image->data[0], tmp,
                  left_image->width, left_image->height);

      yuyv16_to_rgb24(tmp, &msg->right_image.data[0],
                      msg->right_image.width, msg->right_image.height);     
      rotate90CCW(&right_image->data[0], tmp,
                  right_image->width, right_image->height);
      
      delete tmp;
    }
    else
    {
	    yuyv16_to_rgb24(&left_image->data[0], &msg->left_image.data[0],
                      msg->left_image.width, msg->left_image.height);        
      yuyv16_to_rgb24(&right_image->data[0], &msg->right_image.data[0],
                      msg->right_image.width, msg->right_image.height);     
    }

    // publish image messages
    left_image_pub_.publish(left_image);    
    right_image_pub_.publish(right_image);
    
    // prepare info messages
    sensor_msgs::CameraInfoPtr info_left(new sensor_msgs::CameraInfo(left_info_mgr_.getCameraInfo()));
    sensor_msgs::CameraInfoPtr info_right(new sensor_msgs::CameraInfo(right_info_mgr_.getCameraInfo()));
    info_left->header.stamp = info_right->header.stamp = msg->header.stamp;
    info_left->header.frame_id = left_frame_id_;
    info_right->header.frame_id = right_frame_id_;

    // publish info messages
    left_info_pub_.publish(info_left);
    right_info_pub_.publish(info_right);
  }
};

} // namespace stereo_capture

int main(int argc, char** argv)
{
  ros::init(argc, argv, "republish_stereo");

  stereo_capture::RepublishStereo this_node;
  this_node.spin();
  
  return 0;
}
