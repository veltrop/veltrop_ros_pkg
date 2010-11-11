#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/JointState.h>

namespace veltrobot_sensors
{
  
class TiltHeading
{
public:
  TiltHeading()
  {
    input_data_sub_ = n_.subscribe("/hmc6343_raw", 1, 
                                    &TiltHeading::receiveDataCB, this); 
    joint_states_pub_ = n_.advertise<sensor_msgs::JointState>("/joint_states", 1);
  }
  
  void spin()
  {
    sensor_msgs::JointState js; 
    js.name.push_back("base_yaw");
    js.position.push_back(0);
    js.velocity.push_back(10);
    js.name.push_back("base_pitch");
    js.position.push_back(0);
    js.velocity.push_back(10);
    js.name.push_back("base_roll");
    js.position.push_back(0);
    js.velocity.push_back(10);
    
    joint_states_pub_.publish(js);
    
    ros::spin();
  }
  
private:
  ros::NodeHandle n_;
  ros::Subscriber input_data_sub_;
  ros::Publisher  joint_states_pub_;
  
  void receiveDataCB(const std_msgs::UInt8MultiArrayConstPtr& msg)
  {
    char msb1 = msg->data[0]; 
    char lsb1 = msg->data[1]; 
    char msb2 = msg->data[2]; 
    char lsb2 = msg->data[3];  
    char msb3 = msg->data[4];  
    char lsb3 = msg->data[5]; 
    
    int heading_i = 0;
    if (msb1 & 0x0080) // 128 (2's compliment negative)
      heading_i |= (msb1<<8 & 0xFFFFFF00) | (lsb1 & 0xFFFF00FF);
    else
      heading_i |= (msb1<<8 & 0x0000FF00) | (lsb1 & 0x000000FF);    
    
    int pitch_i = 0;
    if (msb2 & 0x0080) // 128 (2's compliment negative)
      pitch_i |= (msb2<<8 & 0xFFFFFF00) | (lsb2 & 0xFFFF00FF);
    else
      pitch_i |= (msb2<<8 & 0x0000FF00) | (lsb2 & 0x000000FF);
    
    int roll_i = 0;
    if (msb3 & 0x0080) // 128 (2's compliment negative)
      roll_i |= (msb3<<8 & 0xFFFFFF00) | (lsb3 & 0xFFFF00FF);
    else
      roll_i |= (msb3<<8 & 0x0000FF00) | (lsb3 & 0x000000FF);
    
    float yaw_f   = (float)heading_i / 10.0f * 3.14159f / 180.0f;
    float pitch_f = (float)pitch_i   / 10.0f * 3.14159f / 180.0f;
    float roll_f  = (float)roll_i    / 10.0f * 3.14159f / 180.0f;
    
    sensor_msgs::JointState js; 
    js.name.push_back("base_yaw");
    js.position.push_back(yaw_f);
    js.velocity.push_back(10);
    js.name.push_back("base_pitch");
    js.position.push_back(pitch_f);
    js.velocity.push_back(10);
    js.name.push_back("base_roll");
    js.position.push_back(roll_f);
    js.velocity.push_back(10);
    
    joint_states_pub_.publish(js);
  }
  
};
} // namespace veltrobot_sensors

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tilt_heading");
  
  veltrobot_sensors::TiltHeading this_node;
  this_node.spin();
  
  return 0;
}

