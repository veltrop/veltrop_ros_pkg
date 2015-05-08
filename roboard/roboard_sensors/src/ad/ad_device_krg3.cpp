#include <roboard.h>
#include <std_msgs/Float32.h>
#include "ad_device_krg3.h"

namespace roboard_sensors
{

ADDeviceKRG3::ADDeviceKRG3(XmlRpc::XmlRpcValue& device_info)
 :ADDevice(device_info)
{
	ros::NodeHandle n;
	pub_ = n.advertise<std_msgs::Float32>(device_name_, 1);
}

// degrees per sec rotation
void ADDeviceKRG3::processData(int data)
{
 	if (!data)
  	return;
  
	float volts = float(data + raw_calibration_)/1023.0f*5.0f;
 	std_msgs::Float32 msg;
  msg.data = (volts - 1.25f) / 0.00067f; 
  pub_.publish(msg);   
}

}
