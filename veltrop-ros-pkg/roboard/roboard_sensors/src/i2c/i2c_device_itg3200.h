#ifndef _I2C_DEVICE_ITG3200_H_
#define _I2C_DEVICE_ITG3200_H_

#include <ros/ros.h>
#include "i2c_device.h"

namespace roboard_sensors
{

class I2CDeviceITG3200 : public I2CDevice
{
	public:
		I2CDeviceITG3200(XmlRpc::XmlRpcValue& device_info);
    
  protected:
    virtual void pollCB(const ros::TimerEvent& e);

	private:
    void init();
};

}

#endif
