#include <roboard.h>
#include <std_msgs/Float32.h>
#include "ad_device_2d120x.h"

namespace roboard_sensors
{

ADDevice2D120X::ADDevice2D120X(XmlRpc::XmlRpcValue& device_info)
 :ADDevice(device_info)
{
	ros::NodeHandle n;
	pub_ = n.advertise<std_msgs::Float32>(device_name_, 1);
}

// dist in cm
void ADDevice2D120X::processData(int data)
{
 	std_msgs::Float32 msg;
  
  float volts = float(data + raw_calibration_)/1023.0f*5.0f;
	msg.data = -(3.078f*pow(volts,5.0f)) + (29.645f*pow(volts,4.0f)) - 
             (110.68f*pow(volts,3.0f)) + (201.94f*pow(volts,2.0f)) -
             (186.84f*volts)           + 81.524f;
  pub_.publish(msg); 
}

}
