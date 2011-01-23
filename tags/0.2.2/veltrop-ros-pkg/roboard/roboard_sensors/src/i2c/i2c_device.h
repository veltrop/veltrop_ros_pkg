#ifndef _I2C_DEVICE_H_
#define _I2C_DEVICE_H_

#include <ros/ros.h>
#include <string>
#include <pthread.h>

#define lockI2C() pthread_mutex_lock(&roboard_sensors::I2CDevice::i2c_mutex_)
#define unlockI2C() pthread_mutex_unlock(&roboard_sensors::I2CDevice::i2c_mutex_)

namespace roboard_sensors
{
  
class I2CDevice
{
	public:
  	I2CDevice(XmlRpc::XmlRpcValue& device_info);
    virtual ~I2CDevice();
    
    //static void lockI2C();
    //static void unlockI2C();
  
  protected:
    virtual void pollCB(const ros::TimerEvent& e);
    
    virtual void reset();
    
    std::string device_name_;
    char base_address_;
    double poll_frequency_;
    ros::Timer poll_timer_;
    bool publish_raw_;
    long speed_;
    
  	ros::Publisher   raw_pub_;
    
    static pthread_mutex_t i2c_mutex_;
    static bool initialized_;
};
  
}

#endif 
