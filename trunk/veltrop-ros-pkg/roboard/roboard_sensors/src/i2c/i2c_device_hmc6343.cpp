#include <roboard.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include "i2c_device_hmc6343.h"

namespace roboard_sensors
{

I2CDeviceHMC6343::I2CDeviceHMC6343(XmlRpc::XmlRpcValue& device_info)
 : I2CDevice(device_info)
{
	ros::NodeHandle n;
	pub_ = n.advertise<std_msgs::Int16MultiArray>(device_name_ + "/raw", 1);
}

void I2CDeviceHMC6343::pollCB(const ros::TimerEvent& e)
{
  char msb1, lsb1, msb2, lsb2, msb3, lsb3;
  
  lockI2C();
  {
    i2c0master_StartN(0x32>>1, I2C_WRITE, 1); 
    i2c0master_WriteN(0x50);  // request data
  }
  unlockI2C();
  
  usleep(1000); // 1 ms
  
  lockI2C(); 
  {
    i2c0master_StartN( 0x33>>1, I2C_WRITE, 1 ); // (+ 1)
    i2c0master_SetRestartN( I2C_READ, 6 ); 
    i2c0master_WriteN(6); 
    
    msb1 = i2c0master_ReadN(); 
    lsb1 = i2c0master_ReadN(); 
    msb2 = i2c0master_ReadN(); 
    lsb2 = i2c0master_ReadN(); 
    msb3 = i2c0master_ReadN(); 
    lsb3 = i2c0master_ReadN(); 
  }
  unlockI2C();
  
  int head = 0;
  if (msb1 & 0x0080) // 128 (2's compliment negative)
    head |= (msb1<<8 & 0xFFFFFF00) | (lsb1 & 0xFFFF00FF);
  else
    head |= (msb1<<8 & 0x0000FF00) | (lsb1 & 0x000000FF);
  
  int pitch = 0;
  if (msb2 & 0x0080) // 128 (2's compliment negative)
    pitch |= (msb2<<8 & 0xFFFFFF00) | (lsb2 & 0xFFFF00FF);
  else
    pitch |= (msb2<<8 & 0x0000FF00) | (lsb2 & 0x000000FF);
  
  int roll = 0;
  if (msb3 & 0x0080) // 128 (2's compliment negative)
    roll |= (msb3<<8 & 0xFFFFFF00) | (lsb3 & 0xFFFF00FF);
  else
    roll |= (msb3<<8 & 0x0000FF00) | (lsb3 & 0x000000FF);
  
  //ROS_INFO_STREAM( (float)head/10.0f << " " << (float)pitch/10.0f << " " << (float)roll/10.0f );
	std_msgs::Int16MultiArray msg;
  msg.data.resize(3);
  msg.data[0] = head;
  msg.data[1] = pitch;
  msg.data[2] = roll; 
  std_msgs::MultiArrayDimension dim;
  dim.label="Head,Pitch,Roll";
  dim.size=msg.data.size();
  dim.stride=msg.data.size();
  msg.layout.dim.push_back(dim);
  pub_.publish(msg);
}

}
