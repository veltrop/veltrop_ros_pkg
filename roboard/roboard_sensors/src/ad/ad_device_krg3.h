#ifndef _AD_DEVICE_KRG3_H_
#define _AD_DEVICE_KRG3_H_

#include <ros/ros.h>
#include "ad_device.h"

namespace roboard_sensors
{

class ADDeviceKRG3 : public ADDevice
{
	public:
		ADDeviceKRG3(XmlRpc::XmlRpcValue& device_info);
    
  	virtual void processData(int data);
    
  protected:  
    ros::Publisher   pub_;
};

}

#endif
