#include "ad_device.h"

namespace roboard_sensors
{

ADDevice::ADDevice(XmlRpc::XmlRpcValue& device_info) : raw_calibration_(0)
{    
  address_ = int(device_info["address"]);
  
  if (device_info.hasMember("raw_calibration"))
  	raw_calibration_ = int(device_info["raw_calibration"]);
  
  if (device_info.hasMember("name"))
		device_name_ = std::string(device_info["name"]);
}

ADDevice::~ADDevice()
{

}

short ADDevice::getRawCalibration()
{
	return raw_calibration_;
}

std::string ADDevice::getName()
{
	return device_name_;
}

int ADDevice::getAddress()
{
	return address_;
}

void ADDevice::processData(int data)
{

}

}
