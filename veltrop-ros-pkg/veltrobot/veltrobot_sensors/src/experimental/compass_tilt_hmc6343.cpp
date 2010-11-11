#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

namespace veltrobot_sensors
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
      
      //std::string out(first + ": " + second);
      //std::cout << string<< <<std::endl;
    }  
    
    pub_ = n.advertise<std_msgs::UInt8MultiArray>(topic_name_, 10);   
  
    poll_timer_ = n.createTimer(freq_, &GenericSensorI2C::pollCB, this);
  }

private:
  ros::NodeHandle& n_;

  void pollCB(const ros::TimerEvent& e)
  {
    std_msgs::UInt8MultiArray msg;
    
    i2c0master_StartN(address_>>1, I2C_WRITE, 1); 
    i2c0master_WriteN(command_);  
    usleep(delay_*1000);

    i2c0master_StartN((address_+1)>>1, I2C_WRITE, 1); 
    i2c0master_SetRestartN(I2C_READ, data_size_); 
    i2c0master_WriteN(data_size_); 
    for (unsigned int i=0; i < data_size_; i++)
      msg.data.push_back(i2c0master_ReadN()); 
      
    pub_.publish(msg);
    
    // TODO: do this conversion in the HMC6343 sensor node
    /*
    char msb1 = i2c0master_ReadN(); 
    char lsb1 = i2c0master_ReadN(); 
    char msb2 = i2c0master_ReadN(); 
    char lsb2 = i2c0master_ReadN(); 
    char msb3 = i2c0master_ReadN(); 
    char lsb3 = i2c0master_ReadN();
    
    int head = 0;
    if (msb1 & 0x0080) // 128 (2's compliment negative)
        head |= (msb1<<8 & 0xFFFFFF00) | (lsb1 & 0xFFFF00FF);
    else
        head |= (msb1<<8 & 0x0000FF00) | (lsb1 & 0x000000FF);
    
    int pitch = 0;
    if (msb2 & 0x0080) // 128 (2's compliment negative)
        pitch |= (msb2<<8 & 0xFFFFFF00) | (lsb2 & 0xFFFF00FF);
    else
        pitch |= (msb2<<8 & 0x0000FF00) | (lsb2 & 0x000000FF);
    
    int roll = 0;
    if (msb3 & 0x0080) // 128 (2's compliment negative)
        roll |= (msb3<<8 & 0xFFFFFF00) | (lsb3 & 0xFFFF00FF);
    else
        roll |= (msb3<<8 & 0x0000FF00) | (lsb3 & 0x000000FF);
    */    
    //cout << setw(6) << (float)head/10.0f << " " << setw(6) << (float)pitch/10.0f << " " << setw(6) << (float)roll/10.0f <<endl;
  }
};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poll_i2c");
  
  roboard_sensors::PollI2C this_node;
  this_node.spin();
  
  return 0;
}

