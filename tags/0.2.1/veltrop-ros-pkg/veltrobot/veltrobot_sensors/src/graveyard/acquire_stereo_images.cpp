#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "libcam.h"

namespace veltrobot_sensors
{
    
class AcquireStereoImages
{
public:
  AcquireStereoImages()
  : np_("~")
  {
    np_.param<int>("width", width_, 320); 
    np_.param<int>("height", height_, 240); 
    np_.param<int>("fps", fps_, 30); 
    np_.param<std::string>("left_dev", left_dev_, "/dev/video0"); 
    np_.param<std::string>("right_dev", right_dev_, "/dev/video1"); 
    
    // TODO: image header
    left_image_.height       = right_image_.height       = height_;
    left_image_.width        = right_image_.width        = width_;
    left_image_.step         = right_image_.step         = width_*3;
    left_image_.encoding     = right_image_.encoding     = "8UC3";
    left_image_.is_bigendian = right_image_.is_bigendian = false;
		left_image_.data.resize(width_*height_*3); // (unsigned char *)malloc(w*h*4);
		right_image_.data.resize(width_*height_*3);    
    
    left_cam_ = new Camera(left_dev_.c_str(), width_, height_, fps_);
    right_cam_ = new Camera(right_dev_.c_str(), width_, height_, fps_);
    
    left_pub_ = n_.advertise<sensor_msgs::Image>("/left/image_raw", 1);
    right_pub_ = n_.advertise<sensor_msgs::Image>("/right/image_raw", 1);
    
    double freq;
    np_.param<double>("freqency", freq, 4); 
    poll_timer_ = n_.createTimer(ros::Duration(1.0 / freq), &AcquireStereoImages::pollCB, this); 
  }
  
  ~AcquireStereoImages()
  {
    delete left_cam_;
    delete right_cam_;
  }
  
  void spin()
  {
    ros::spin();
  }
  
private:
  ros::NodeHandle n_; 
  ros::NodeHandle np_;
  ros::Timer      poll_timer_;
  ros::Publisher  left_pub_;
  ros::Publisher  right_pub_;
  int             width_;
  int             height_;
  int             fps_;
  std::string     left_dev_;
  std::string     right_dev_;
  Camera*         left_cam_;
  Camera*         right_cam_;
  sensor_msgs::Image left_image_;
  sensor_msgs::Image right_image_;
  
  void pollCB(const ros::TimerEvent& e)
  {
    //std::copy(buffer, buffer+(width*height*3), msg.data.begin());
    
    while(!left_cam_->Get(&left_image_.data[0]) ||
          !right_cam_->Get(&right_image_.data[0])) 
      usleep(100);
    
    // TODO: image header timestamp
    
    left_pub_.publish(left_image_);
    right_pub_.publish(right_image_);
  }
};
  
} // namespace veltrobot_sensors

int main(int argc, char** argv)
{
  ros::init(argc, argv, "acquire_stereo_images");
  
  veltrobot_sensors::AcquireStereoImages this_node;
  this_node.spin();
  
  return 0;
}
