#include "i2c_device.h"

namespace roboard_sensors
{

bool I2CDevice::initialized_ = false;
pthread_mutex_t I2CDevice::i2c_mutex_;

I2CDevice::I2CDevice(XmlRpc::XmlRpcValue& device_info)
: publish_raw_(true)
, speed_(100000L)
{
  ros::NodeHandle n;

	if (!initialized_)
  {
  	initialized_ = true;
		pthread_mutex_init(&i2c_mutex_, NULL);
  }
    
  device_name_ = std::string(device_info["name"]);
  base_address_ = int(device_info["base_address"]);
  poll_frequency_ = 0;
  if (device_info.hasMember("pole_frequency"))
  {
  	if (device_info["pole_frequency"].getType() == XmlRpc::XmlRpcValue::TypeInt)
    	poll_frequency_ = double(int(device_info["pole_frequency"]));
    else if (device_info["pole_frequency"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    	poll_frequency_ = double(device_info["pole_frequency"]);
    
    ros::Duration freq(1.0 / poll_frequency_);
    poll_timer_ = n.createTimer(freq, &I2CDevice::pollCB, this);
  }
  
  if (device_info.hasMember("publish_raw"))
  	publish_raw_ = device_info["publish_raw"];
    
	if (device_info.hasMember("speed"))
  	speed_ = (int)device_info["speed"];    
}

I2CDevice::~I2CDevice()
{

}

/*void I2CDevice::lockI2C()
{
	pthread_mutex_lock(&i2c_mutex_);
}

void I2CDevice::unlockI2C()
{
	pthread_mutex_unlock(&i2c_mutex_);
}*/

void I2CDevice::pollCB(const ros::TimerEvent& e)
{

}

}
