#ifndef _AD_DEVICE_2D120X_H_
#define _AD_DEVICE_2D120X_H_

#include <ros/ros.h>
#include "ad_device.h"

namespace roboard_sensors
{

class ADDevice2D120X : public ADDevice
{
	public:
		ADDevice2D120X(XmlRpc::XmlRpcValue& device_info);
    
  	virtual void processData(int data);
    
  protected:  
    ros::Publisher   pub_;
};

}

#endif
