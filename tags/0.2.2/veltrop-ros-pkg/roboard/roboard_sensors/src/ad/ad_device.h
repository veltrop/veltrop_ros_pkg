#ifndef _AD_DEVICE_H_
#define _AD_DEVICE_H_

#include <ros/ros.h>
#include <string>

namespace roboard_sensors
{
  
class ADDevice
{
	public:
  	ADDevice(XmlRpc::XmlRpcValue& device_info);
    virtual ~ADDevice();
    
    short getRawCalibration();
    std::string getName();
    int getAddress();
    
    virtual void processData(int data);
      
  protected:
    std::string device_name_;
    int address_;
    short raw_calibration_;
};
  
}

#endif 
