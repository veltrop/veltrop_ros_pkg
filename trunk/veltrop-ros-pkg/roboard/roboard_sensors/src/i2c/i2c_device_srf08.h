#ifndef _I2C_DEVICE_SRF08_H_
#define _I2C_DEVICE_SRF08_H_

#include <ros/ros.h>
#include "i2c_device.h"

namespace roboard_sensors
{

class I2CDeviceSRF08 : public I2CDevice
{
	public:
		I2CDeviceSRF08(XmlRpc::XmlRpcValue& device_info);
    
  protected:
  	virtual void pollCB(const ros::TimerEvent& e);
    
    ros::Publisher   pub_;
};

}

#endif
