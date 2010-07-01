#ifndef __GYRO_COMPENSATION_H__
#define __GYRO_COMPENSATION_H__

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>

namespace roboard_servos
{

class GyroCompensaton
{
public:
  GyroCompensaton(const YAML::Node& gyro_info)
  : joint_name_("")
  , modifier10_(10)
  {
    for (YAML::Iterator it=gyro_info.begin(); it!=gyro_info.end(); ++it) {
      std::string paramater;
      it.first() >> paramater;
      if (paramater == "name")
        it.second() >> joint_name_;
      else if (paramater == "modifier10")
        it.second() >> modifier10_;                                   
    }      
  }
  
  GyroCompensaton(const std::string &joint_name, const int &modifier10)
  : joint_name_(joint_name)
  , modifier10_(modifier10)
  {
  }

  std::string joint_name_;
  int         modifier10_;  
};

class GyroCompensatonList : public std::vector<GyroCompensaton>
{
public:
  bool loadFromParamServer(std::string gyro_conf_name)
  {
    std::string gyro_conf;
      
    if (!ros::param::getCached(gyro_conf_name, gyro_conf))
    {
      ROS_ERROR("Gyro Compensation Configuration not on paramater server"); 
      return false; 
    }
    
    std::istringstream gyro_conf_stream(gyro_conf); 
    YAML::Parser gyro_parser(gyro_conf_stream);
    YAML::Node gyro_info;
    do
    {
      gyro_parser.GetNextDocument(gyro_info);
      if (gyro_info.size()) 
        push_back(GyroCompensaton(gyro_info));
    } while(gyro_info.size());
    
    return true;
  }
  
  void setGyroCompensation(const std::string &joint_name, const int &modifier10)
  {
    size_t i;
    for (i=0; i < size(); i++)
      if (this->at(i).joint_name_ == joint_name)
        this->at(i).modifier10_ = modifier10;
    if (i == size())
      push_back(GyroCompensaton(joint_name, modifier10));
  }
 
};

}

#endif // __GYRO_COMPENSATION_H__

