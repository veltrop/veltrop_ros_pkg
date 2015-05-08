#include <string>
#include <math.h>
#include <roboard.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include "i2c_device_itg3200.h"

namespace roboard_sensors
{

I2CDeviceITG3200::I2CDeviceITG3200(XmlRpc::XmlRpcValue& device_info)
 : I2CDevice(device_info)
 , x_calib_(0)
 , y_calib_(0)
 , x_deadzone_(0)
 , y_deadzone_(0)
{  
	ros::NodeHandle n;
  if (publish_raw_)
  {
		raw_pub_ = n.advertise<std_msgs::Int16MultiArray>(device_name_ + "/raw", 1);
    raw_calib_pub_ = n.advertise<std_msgs::Int16MultiArray>(device_name_ + "/raw_calib", 1);
  }

	std::string x_pub_string("x");	
  if (device_info.hasMember("x_topic"))
  	x_pub_string = (std::string)device_info["x_topic"];
  x_pub_ = n.advertise<std_msgs::Float32>(device_name_ + "/" + x_pub_string, 1);
  
  std::string y_pub_string("y");	
  if (device_info.hasMember("y_topic"))
  	y_pub_string = (std::string)device_info["y_topic"];  
  y_pub_ = n.advertise<std_msgs::Float32>(device_name_ + "/" + y_pub_string, 1);

	if (device_info.hasMember("x_calib"))
  	x_calib_ = (int)device_info["x_calib"];
  if (device_info.hasMember("y_calib"))
  	y_calib_ = (int)device_info["y_calib"];
  if (device_info.hasMember("x_deadzone"))
  	x_deadzone_ = (int)device_info["x_deadzone"];
  if (device_info.hasMember("y_deadzone"))
  	y_deadzone_ = (int)device_info["y_deadzone"];

  init();
}

void I2CDeviceITG3200::init()
{
	lockI2C();
  {
  	i2c0_SetSpeed(I2CMODE_AUTO, speed_);
    
    i2c0master_StartN(0x69,I2C_WRITE,2);
    i2c0master_WriteN(0x3E); 
    i2c0master_WriteN(0x80);  // Reset to defaults
    
    usleep(1000);
    
    i2c0master_StartN(0x69,I2C_WRITE,2);
    i2c0master_WriteN(0x15); 
    short divider = 1000 / short(poll_frequency_);
    i2c0master_WriteN(divider);
    //i2c0master_WriteN(0x17);  // sample divider to 23			(42 fps)
		//i2c0master_WriteN(0x09);  // sample divider to 10			(100 fps)
    //i2c0master_WriteN(0xC8);  // sample divider to 200
    //i2c0master_WriteN(0x64);  // sample divider to 100
    
    usleep(1000);  
    
    i2c0master_StartN(0x69,I2C_WRITE,2);
    i2c0master_WriteN(0x16);  
    //i2c0master_WriteN(0x1F); // DLPF_CFG = 7 (?,?), FS_SEL = 3 
    //i2c0master_WriteN(0x1E); // DLPF_CFG = 6 (5hz,1khz), FS_SEL = 3 
    //i2c0master_WriteN(0x1D); // DLPF_CFG = 5 (10hz,1khz), FS_SEL = 3  
    //i2c0master_WriteN(0x1C); // DLPF_CFG = 4 (20hz,1khz), FS_SEL = 3
//    i2c0master_WriteN(0x1B); // DLPF_CFG = 3 (42hz,1khz), FS_SEL = 3
    i2c0master_WriteN(0x1A);   // DLPF_CFG = 2 (98hz,1khz), FS_SEL = 3 
//    i2c0master_WriteN(0x19); // DLPF_CFG = 1 (188hz,1khz), FS_SEL = 3 
    //i2c0master_WriteN(0x18); // DLPF_CFG = 0 (256hz,8khz), FS_SEL = 3     
    
    usleep(1000);

    i2c0master_StartN(0x69,I2C_WRITE,2);
    i2c0master_WriteN(0x3E); 
    //i2c0master_WriteN(0x03); // use z gyro as clock reference
    i2c0master_WriteN(0x0B); // use z gyro as clock reference, sleep z
    //i2c0master_WriteN(0x09); // use x gyro as clock reference, sleep z
  }
  unlockI2C();
}

void I2CDeviceITG3200::pollCB(const ros::TimerEvent& e)
{
  lockI2C();
  { 
  	i2c0_SetSpeed(I2CMODE_AUTO, speed_);
     
    if (!i2c0master_StartN(0x69, I2C_WRITE, 1))
    {
    	ROS_ERROR_STREAM("ITG3200 i2c0master_StartN " << roboio_GetErrMsg());
      reset();
      unlockI2C();
      return;
    }
    if (!i2c0master_SetRestartN(I2C_READ, 4))
    {
    	ROS_ERROR_STREAM("ITG3200 i2c0master_SetRestartN " << roboio_GetErrMsg());
      reset();
      unlockI2C();
      return;
    }      
    if (!i2c0master_WriteN(0x1D)) //Read from X register
    {
    	ROS_ERROR_STREAM("ITG3200 i2c0master_WriteN " << roboio_GetErrMsg()); 
      reset();
      unlockI2C();
      return;
    }      
    
    msb1 = i2c0master_ReadN(); 
    lsb1 = i2c0master_ReadN(); 
    msb2 = i2c0master_ReadN(); 
    lsb2 = i2c0master_ReadN(); 
    //msb3 = i2c0master_ReadN(); 
    //lsb3 = i2c0master_ReadN(); 
  }
  unlockI2C();
  
  x = msb1<<8 | lsb1;  
  y = msb2<<8 | lsb2;
  //short z = msb3<<8 | lsb3;
      
  //ROS_INFO_STREAM( (float)x/10.0f << " " << (float)y/10.0f << " " << (z)roll/10.0f );
	if (publish_raw_)
  {
    std_msgs::Int16MultiArray msg;
    msg.data.resize(2);
    //msg.data.resize(3);
    msg.data[0] = x;
    msg.data[1] = y;
    //msg.data[2] = z;
    std_msgs::MultiArrayDimension dim;
    dim.label="X,Y";
    //dim.label="X,Y,Z";
    dim.size=msg.data.size();
    dim.stride=msg.data.size();
    msg.layout.dim.push_back(dim);
    raw_pub_.publish(msg);
    
    msg.data[0] = x + x_calib_;
    msg.data[1] = y + y_calib_;    
    raw_calib_pub_.publish(msg);
  }
  
  (abs(x + x_calib_) > x_deadzone_) ? x_msg.data = (float)(x + x_calib_)/14.375f : x_msg.data = 0;	    
	(abs(y + y_calib_) > y_deadzone_) ? y_msg.data = (float)(y + y_calib_)/14.375f : y_msg.data = 0;
  x_pub_.publish(x_msg);
  y_pub_.publish(y_msg);
}

}
