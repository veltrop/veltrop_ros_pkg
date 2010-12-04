#ifndef _I2C_DEVICE_ITG3200_H_
#define _I2C_DEVICE_ITG3200_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
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
    
    ros::Publisher x_pub_;
    ros::Publisher y_pub_;
    
    
    std_msgs::Float32 x_msg;
    std_msgs::Float32 y_msg;
    char msb1, lsb1, msb2, lsb2; // msb3, lsb3; 
    short x, y;
};

}

#endif
