#include <ros/ros.h>
#include "i2c_device_manager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "i2c_device_manager");
  ros::NodeHandle n;
  roboard_sensors::I2CDeviceManager man;
  man.spin();
  
  return 0;
}
