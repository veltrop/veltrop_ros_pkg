#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <roboard.h>
#include "poll_ad.h"
#include "ad_device.h"
#include "ad_device_2y0a21.h"
#include "ad_device_2d120x.h"
#include "ad_device_krg3.h"

namespace roboard_sensors
{

PollAD::PollAD() : devices_(8), used_channels_(0)
{   
	ros::NodeHandle np("~");
    
  if (!spi_Initialize(SPICLK_21400KHZ))
  {
    ROS_ERROR("Cant Initialize SPI");
    ros::shutdown();
    exit(-1);
  }     
  
  loadDevices();
  
  // AD7918MODE_RANGE_2VREF // for 0~2.5v
  // AD7918MODE_RANGE_VREF  // for 0~5v
  // AD7918MODE_CODING_1023 // for 0~1023
  // AD7918MODE_CODING_511  // for -512 ~ 511
  if (!ad79x8_InitializeMCH(used_channels_, AD7918MODE_RANGE_2VREF,
                            AD7918MODE_CODING_1023))
  { 
    ROS_ERROR("Cant Initialize MCH");
    ros::shutdown();
    exit(-1);
  }
  
  np.param<bool>("publish_raw", publish_raw_, false); 
  
  if (publish_raw_)
  {
    raw_msg_.data.resize(8);
    raw_msg_dim_.size=raw_msg_.data.size();
    raw_msg_dim_.stride=raw_msg_.data.size();
    for (size_t i=0; i < 8; i++) 
    {
      if (devices_[i])
        raw_msg_dim_.label += devices_[i]->getName();
      if (i != 7)
        raw_msg_dim_.label += std::string(", ");
    }    
    raw_msg_.layout.dim.push_back(raw_msg_dim_);
    raw_pub_ = np.advertise<std_msgs::Int16MultiArray>("/ad_raw", 1);
  }
    
  double freq;
  np.param<double>("fps", freq, 10.0); 
  freq_ = freq;
}
  
PollAD::~PollAD()
{
  ad7918_CloseMCH(); 
  spi_Close(); 
}

void PollAD::loadDevices()
{
	ros::NodeHandle np("~");
  std::string param_name = "devices";
  XmlRpc::XmlRpcValue all_devices;
  
  if (!np.hasParam(param_name))
  {
    ROS_WARN_STREAM("No ad devices in " << param_name);
    return;
  } 
  
  np.getParam(param_name, all_devices);
  
  if (all_devices.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN("Devices is not an array");
    return;
  }

  for (int i=0; i < all_devices.size(); i++)
  {
    if (!all_devices[i].hasMember("address"))
    {
      ROS_WARN("All devices must have an address");
      continue;
    }
        
    ADDevice* new_device = NULL;
    if (all_devices[i]["driver"] == "2Y0A21")
      new_device = new ADDevice2Y0A21(all_devices[i]);
    else if (all_devices[i]["driver"] == "2D120X")
      new_device = new ADDevice2D120X(all_devices[i]);
    else if (all_devices[i]["driver"] == "KRG3")
      new_device = new ADDeviceKRG3(all_devices[i]);           
    else
      new_device = new ADDevice(all_devices[i]);
      
    devices_[new_device->getAddress()] = new_device;
    used_channels_ |= 1<<(7 - new_device->getAddress());  
  }
}

void PollAD::spin()
{
	ros::Rate loop_rate(freq_); 
	while (ros::ok())
  {
    int* ad_data = ad7918_ReadMCH();
    for (size_t i=0; i < 8; i++) 
    {
      if (devices_[i])
      {
      	if (publish_raw_)
        	raw_msg_.data[i] = ad_data[i] + devices_[i]->getRawCalibration();  
        
      	// TODO? Launch this in a new thread?:
      	devices_[i]->processData(ad_data[i]);
      }
    }  
  
  	if (publish_raw_)
    	raw_pub_.publish(raw_msg_);
    
    loop_rate.sleep();
  }
}

} // namespace roboard_sensors

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poll_ad");
  
  ros::NodeHandle n;
  roboard_sensors::PollAD poller;
  poller.spin();
  
  return 0;
}

