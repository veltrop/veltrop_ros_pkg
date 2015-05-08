#include <math.h>
#include <roboard.h>
#include <std_msgs/Float32.h>
#include "ad_device_2y0a21.h"

namespace roboard_sensors
{

ADDevice2Y0A21::ADDevice2Y0A21(XmlRpc::XmlRpcValue& device_info)
 :ADDevice(device_info)
{
	ros::NodeHandle n;
	pub_ = n.advertise<std_msgs::Float32>(device_name_, 1);
}

// dist in cm
void ADDevice2Y0A21::processData(int data)
{
	if (!data)
  	return;
  
 	std_msgs::Float32 msg;
  msg.data = pow(3027.4f / float(data + raw_calibration_), 1.2134);
  pub_.publish(msg);   
}

}
