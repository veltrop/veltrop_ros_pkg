//#include <ros/ros.h>
#include <roboard.h>
#include "i2c_device_manager.h"
#include "i2c_device_hmc6343.h"
#include "i2c_device_srf08.h"
#include "i2c_device_itg3200.h"

namespace roboard_sensors
{

I2CDeviceManager::I2CDeviceManager()
{
	ros::NodeHandle np("~");
  double freq;
  np.param<double>("spin_rate", freq, 10000.0); 
  freq_ = freq;
  
  if (!i2c_Initialize(I2CIRQ_DISABLE))
  { 
    ROS_ERROR("Failed to initialize i2c");
    ros::shutdown();
    exit(-1);
  } 

  i2c0_SetSpeed(I2CMODE_AUTO, 100000L); 
  
  loadDevices();
}

I2CDeviceManager::~I2CDeviceManager()
{
	for (size_t i=0; i < devices_.size(); i++)
  	delete devices_[i];

	i2c_Close();
}

void I2CDeviceManager::spin()
{
	ros::Rate loop_rate(freq_);
	while (ros::ok())
  {
  	loop_rate.sleep();	// initial delay before callbacks happen.
  	ros::spinOnce();
  }
}

void I2CDeviceManager::loadDevices()
{
	ros::NodeHandle np("~");
  std::string param_name = "devices";
  XmlRpc::XmlRpcValue all_devices;
  
  if (!np.hasParam(param_name))
  {
    ROS_WARN_STREAM("No i2c devices in " << param_name);
    return;
  } 
  
  np.getParam(param_name, all_devices);
  
  if (all_devices.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN("Devices is not an array");
    return;
  }

  for (int i=0; i < all_devices.size(); i++)
  {
    if (!all_devices[i].hasMember("driver") ||
        !all_devices[i].hasMember("base_address"))
    {
      ROS_WARN("All devices must have a driver and base_address");
      continue;
    }
        
    I2CDevice* new_device = NULL;
    if (all_devices[i]["driver"] == "HMC6343")
      new_device = new I2CDeviceHMC6343(all_devices[i]);
    else if (all_devices[i]["driver"] == "SRF08")
      new_device = new I2CDeviceSRF08(all_devices[i]);
    else if (all_devices[i]["driver"] == "ITG3200")
      new_device = new I2CDeviceITG3200(all_devices[i]);
    else
    {
      ROS_WARN("Unknown driver");
      continue;
    }  
      
    devices_.push_back(new_device);
	}
}

}

