#ifndef _AD_DEVICE_2Y0A21_H_
#define _AD_DEVICE_2Y0A21_H_

#include <ros/ros.h>
#include "ad_device.h"

namespace roboard_sensors
{

class ADDevice2Y0A21 : public ADDevice
{
	public:
		ADDevice2Y0A21(XmlRpc::XmlRpcValue& device_info);
    
  	virtual void processData(int data);
    
  protected:  
    ros::Publisher   pub_;
};

}

#endif
