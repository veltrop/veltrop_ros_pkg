#include <roboard.h>
#include <std_msgs/UInt16.h>
#include "i2c_device_srf08.h"

namespace roboard_sensors
{

I2CDeviceSRF08::I2CDeviceSRF08(XmlRpc::XmlRpcValue& device_info)
 : I2CDevice(device_info)
{
	ros::NodeHandle n;
  if (publish_raw_)
		raw_pub_ = n.advertise<std_msgs::UInt16>(device_name_ + "/raw", 1);
}

void I2CDeviceSRF08::pollCB(const ros::TimerEvent& e)
{
	char b1, b2;
  
  lockI2C();
  {
    i2c0master_StartN(0xe0>>1, I2C_WRITE, 2);        
    
    i2c0master_WriteN(0);       //set SRF command register 
    i2c0master_WriteN(81);      //set ranging Mode - result in cm 
  }
  unlockI2C();
  
  usleep(75000);
  
  lockI2C();
  {
    i2c0master_StartN(0xe0>>1, I2C_WRITE, 1);         
    i2c0master_SetRestartN(I2C_READ, 2); 
    
    i2c0master_WriteN(2);       //set 1st SRF range register 
    b1 = i2c0master_ReadN();    //read 1st echo high byte 
    b2 = i2c0master_ReadN();    //read 1st echo low byte 
  }
  unlockI2C();
  
  //ROS_INFO_STREAM ( b1*256 + b2 );
 	std_msgs::UInt16 msg;
  //msg.data = b1*256 + b2;
  msg.data = b1<<8 | b2;
  raw_pub_.publish(msg); 
}

}
