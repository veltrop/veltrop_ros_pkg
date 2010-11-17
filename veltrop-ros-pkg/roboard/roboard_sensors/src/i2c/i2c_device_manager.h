#ifndef _I2C_DEVICE_MANAGER_H_
#define _I2C_DEVICE_MANAGER_H_

#include <vector>
#include <ros/ros.h>
#include "i2c_device.h"

namespace roboard_sensors
{

class I2CDeviceManager
{
	public:
  	I2CDeviceManager();
    ~I2CDeviceManager();
    
    void spin();
    
  private:    
  	void loadDevices();
    
    std::vector<I2CDevice*> devices_;
    float												freq_;
};


}

#endif