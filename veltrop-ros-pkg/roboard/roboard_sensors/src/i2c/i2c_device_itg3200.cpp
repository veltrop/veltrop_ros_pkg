#include <roboard.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include "i2c_device_itg3200.h"

namespace roboard_sensors
{

I2CDeviceITG3200::I2CDeviceITG3200(XmlRpc::XmlRpcValue& device_info)
 : I2CDevice(device_info)
{  
	ros::NodeHandle n;
  if (publish_raw_)
		raw_pub_ = n.advertise<std_msgs::Float32MultiArray>(device_name_ + "/raw", 1);

  init();
}

void I2CDeviceITG3200::init()
{
	lockI2C();
  {
    i2c0master_StartN(0x69,I2C_WRITE,2);
    i2c0master_WriteN(0x3E); 
    i2c0master_WriteN(0x80);  // Reset to defaults
    
    usleep(1000);
    
    i2c0master_StartN(0x69,I2C_WRITE,2);
    i2c0master_WriteN(0x15); 
    i2c0master_WriteN(0x09);  // sample divider to 10			(100 fps)
    //i2c0master_WriteN(0xC8);  // sample divider to 200
    //i2c0master_WriteN(0x64);  // sample divider to 100
    
    usleep(1000);  
    
    i2c0master_StartN(0x69,I2C_WRITE,2);
    i2c0master_WriteN(0x16);  
    //i2c0master_WriteN(0x18); // DLPF_CFG = 0, FS_SEL = 3 
    //i2c0master_WriteN(0x1E); // DLPF_CFG = 6, FS_SEL = 3 
    //i2c0master_WriteN(0x1D); // DLPF_CFG = 5, FS_SEL = 3  
    //i2c0master_WriteN(0x1A); // DLPF_CFG = 2, FS_SEL = 3 
    i2c0master_WriteN(0x1B); // DLPF_CFG = 3, FS_SEL = 3 
    
    usleep(1000);

    i2c0master_StartN(0x69,I2C_WRITE,2);
    i2c0master_WriteN(0x3E); 
    //i2c0master_WriteN(0x03); // use z gyro as clock reference
    //i2c0master_WriteN(0x0B); // use z gyro as clock reference, sleep z
    i2c0master_WriteN(0x09); // use x gyro as clock reference, sleep z
  }
  unlockI2C();
}

void I2CDeviceITG3200::pollCB(const ros::TimerEvent& e)
{
  char msb1, lsb1, msb2, lsb2; // msb3, lsb3; 
  
  lockI2C();
  {  
    i2c0master_StartN(0x69, I2C_WRITE, 1);
    i2c0master_SetRestartN(I2C_READ, 4);
    i2c0master_WriteN(0x1D); //Read from X register 
    
    msb1 = i2c0master_ReadN(); 
    lsb1 = i2c0master_ReadN(); 
    msb2 = i2c0master_ReadN(); 
    lsb2 = i2c0master_ReadN(); 
    //msb3 = i2c0master_ReadN(); 
    //lsb3 = i2c0master_ReadN(); 
  }
  unlockI2C();
  
  short x = msb1<<8 | lsb1;  
  short y = msb2<<8 | lsb2;
  //short z = msb3<<8 | lsb3;
      
  //ROS_INFO_STREAM( (float)x/10.0f << " " << (float)y/10.0f << " " << (z)roll/10.0f );
	std_msgs::Float32MultiArray msg;
  msg.data.resize(2);
  //msg.data.resize(3);
  msg.data[0] = (float)x/14.375f;
  msg.data[1] = (float)y/14.375f;
  //msg.data[2] = (float)z/14.375f;
  std_msgs::MultiArrayDimension dim;
  dim.label="X,Y";
  //dim.label="X,Y,Z";
  dim.size=msg.data.size();
  dim.stride=msg.data.size();
  msg.layout.dim.push_back(dim);
  raw_pub_.publish(msg);
}

}
