#include <vector>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <roboard.h>

namespace roboard_sensors
{

class GenericSensorI2C
{
public:
  GenericSensorI2C(ros::NodeHandle& n, YAML::Node& sensor_info)
  : topic_name_("unknown_i2c_sensor_raw")
  , freq_(0)
  , address_(0)
  , command_(0)
  , delay_(0)
  , data_size_(0)
  {
    for (YAML::Iterator it=sensor_info.begin(); it!=sensor_info.end(); ++it) {
      std::string paramater;
      it.first() >> paramater;
      if (paramater == "name")
        it.second() >> topic_name_;
      else if (paramater == "sleep")
      {
        float ms;
        it.second() >> ms;
        freq_ = ros::Duration(ms / 1000.0f);
      }
      else if (paramater == "address")
        it.second() >> address_;
      else if (paramater == "command")
        it.second() >> command_;
      else if (paramater == "delay")
        it.second() >> delay_;
      else if (paramater == "data_size")
        it.second() >> data_size_;                                      
    }  
    std::string poo = "this" + address_;
    pub_ = n.advertise<std_msgs::UInt8MultiArray>(topic_name_, 1);   
  
    poll_timer_ = n.createTimer(freq_, &GenericSensorI2C::pollCB, this);
  }

private:
  ros::Publisher   pub_;
  ros::Timer       poll_timer_;
  std::string      topic_name_;
  ros::Duration    freq_;
  unsigned char    address_;
  unsigned char    command_;
  unsigned int     delay_;
  unsigned int     data_size_;

  void pollCB(const ros::TimerEvent& e)
  {  
    std_msgs::UInt8MultiArray msg;
    
    i2c0master_StartN(address_>>1, I2C_WRITE, 1); 
    i2c0master_WriteN(command_);  
    usleep(delay_*1000);

    i2c0master_StartN((address_+1)>>1, I2C_WRITE, 1); 
    i2c0master_SetRestartN(I2C_READ, data_size_); 
    i2c0master_WriteN(data_size_); 
    msg.data.resize(data_size_);
    for (unsigned int i=0; i < data_size_; i++)
      msg.data[i] = i2c0master_ReadN(); 
    std_msgs::MultiArrayDimension dim;
    dim.label="X";
    dim.size=msg.data.size();
    dim.stride=msg.data.size();
    msg.layout.dim.push_back(dim);
       
    pub_.publish(msg);
  }
};

class PollI2C
{
public:
  PollI2C()
  : np_("~")
  { 
    std::string i2c_sensors_conf;
    np_.param<std::string>("i2c_sensors_conf", i2c_sensors_conf, ""); 
    std::istringstream i2c_sensors_conf_stream(i2c_sensors_conf); 
    YAML::Parser yaml_parser(i2c_sensors_conf_stream);
    YAML::Node sensor_info;
    do
    {
      yaml_parser.GetNextDocument(sensor_info);
      if (sensor_info.size()) 
      {
        sensors_.push_back(GenericSensorI2C(n_, sensor_info));
      }
    } while(sensor_info.size());
  }
  
  void spin()
  {
    if (!i2c_Initialize(I2CIRQ_DISABLE))
    { 
      ROS_ERROR("Failed to initialize i2c");
      ros::shutdown();
      exit(-1);
    } 

    i2c0_SetSpeed(I2CMODE_STANDARD, 100000); 
    ros::spin();
    i2c_Close();
  }
  
private:
  ros::NodeHandle               n_;
  ros::NodeHandle               np_;
  std::vector<GenericSensorI2C> sensors_;
};

} // namespace roboard_sensors

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poll_i2c");
  
  roboard_sensors::PollI2C this_node;
  this_node.spin();
  
  return 0;
}

