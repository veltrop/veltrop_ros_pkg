#ifndef _I2C_DEVICE_HMC6343_H_
#define _I2C_DEVICE_HMC6343_H_

#include <ros/ros.h>
#include "i2c_device.h"

namespace roboard_sensors
{

class I2CDeviceHMC6343 : public I2CDevice
{
	public:
		I2CDeviceHMC6343(XmlRpc::XmlRpcValue& device_info);
    
  protected:
    virtual void pollCB(const ros::TimerEvent& e);

		ros::Publisher   pub_;
};

}

#endif
