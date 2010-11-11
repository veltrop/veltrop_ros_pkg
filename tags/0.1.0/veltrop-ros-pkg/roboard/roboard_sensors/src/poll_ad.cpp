#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <roboard.h>

namespace roboard_sensors
{

class PollAD
{
public:
  PollAD()
  : np_("~")
  , sensor_publishers_(8)
  , calib_(8)
  , used_channels_(0)
  { 
    setupChan(0, "ch0");
    setupChan(1, "ch1");
    setupChan(2, "ch2");
    setupChan(3, "ch3");
    setupChan(4, "ch4");
    setupChan(5, "ch5");
    setupChan(6, "ch6");
    setupChan(7, "ch7");      
    
    double freq;
    np_.param<double>("frequency", freq, 1000); 
    poll_timer_ = n_.createTimer(ros::Duration(1.0 / freq), &PollAD::pollCB, this);  
  }
  
  void spin()
  {
    if (!spi_Initialize(SPICLK_21400KHZ))
    {
      ROS_ERROR("Cant Initialize SPI");
      ros::shutdown();
      exit(-1);
    } 
    
    // This is according to the documentation, the VREF is flipped...
    // AD7918MODE_RANGE_2VREF // for 0~5v
    // AD7918MODE_RANGE_VREF  // for 0~2.5v
    // AD7918MODE_CODING_1023 // for 0~1023
    // AD7918MODE_CODING_511  // for -512 ~ 511
    if(!ad79x8_InitializeMCH(used_channels_,
                             AD7918MODE_RANGE_VREF,
                             AD7918MODE_CODING_511))
    { 
      ROS_ERROR("Cant Initialize MCH");
      ros::shutdown();
      exit(-1);
    }
    
    ros::spin();
    ad7918_CloseMCH(); 
    spi_Close();    
  }
  
private:
  ros::NodeHandle             n_;
  ros::NodeHandle             np_;
  ros::Timer                  poll_timer_;
  std::vector<ros::Publisher> sensor_publishers_;
  std::vector<short>          calib_;
  unsigned char               used_channels_;
  
  void setupChan(unsigned int chNum, std::string chString)
  {
    if (chNum > 7)
      return;
            
    std::string tmpS;
    np_.param<std::string>(chString + "name", tmpS, ""); 
    
    if (tmpS != "")
    {
      sensor_publishers_[chNum] = n_.advertise<std_msgs::Int16>(tmpS, 1);
      used_channels_ |= 1<<(7 - chNum); 
    }
    
    int tmpI;
    np_.param<int>(chString + "calib", tmpI, 0); 
    calib_[chNum] = tmpI;
  }
  
  void pollCB(const ros::TimerEvent& e)
  {
    int* ad_data = ad7918_ReadMCH();
    for (size_t i=0; i < 8; i++)
      if (used_channels_ & 1<<(7 - i))
      {
        std_msgs::Int16 msg;
        msg.data = ad_data[i] - calib_[i];
        sensor_publishers_[i].publish(msg);
      }
  }
};

} // namespace roboard_sensors

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poll_ad");
  
  roboard_sensors::PollAD this_node;
  this_node.spin();
  
  return 0;
}

