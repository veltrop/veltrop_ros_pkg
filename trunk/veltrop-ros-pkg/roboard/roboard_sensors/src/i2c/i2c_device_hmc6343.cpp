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
  if (publish_raw_)
		raw_pub_ = n.advertise<std_msgs::Int16MultiArray>(device_name_ + "/raw", 1);
}

void I2CDeviceHMC6343::pollCB(const ros::TimerEvent& e)
{
  char msb1, lsb1, msb2, lsb2, msb3, lsb3;
  
  lockI2C();
  {
  	i2c0_SetSpeed(I2CMODE_AUTO, speed_);
    
    i2c0master_StartN(0x32>>1, I2C_WRITE, 1); 
    i2c0master_WriteN(0x50);  // request data
  }
  unlockI2C();
  
  usleep(1000); // 1 ms
  
  lockI2C(); 
  {
  	i2c0_SetSpeed(I2CMODE_AUTO, speed_);
    
		i2c0master_StartN( 0x33>>1, I2C_READ, 6 );
    msb1 = i2c0master_ReadN(); 
    lsb1 = i2c0master_ReadN(); 
    msb2 = i2c0master_ReadN(); 
    lsb2 = i2c0master_ReadN(); 
    msb3 = i2c0master_ReadN(); 
    lsb3 = i2c0master_ReadN(); 
  }
  unlockI2C();
  
  short head = msb1<<8 | lsb1;
  short pitch = msb2<<8 | lsb2;
	short roll = msb3<<8 | lsb3;
  
  //ROS_INFO_STREAM( (float)head/10.0f << " " << (float)pitch/10.0f << " " << (float)roll/10.0f );
  if (publish_raw_)
  {
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
    raw_pub_.publish(msg);
  }
}

}
