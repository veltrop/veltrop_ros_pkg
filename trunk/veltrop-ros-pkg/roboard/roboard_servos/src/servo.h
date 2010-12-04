#ifndef __SERVO_H__
#define __SERVO_H__

#include <string>
#include <map>

#ifndef PI
#define PI 3.14159265359f
#endif
#ifndef HALFPI
#define HALFPI 1.57079632679f
#endif

namespace roboard_servos
{

///////////////////////////////////////////////////////////////////////////////
// Servo
///////////////////////////////////////////////////////////////////////////////

class Servo
{
public:
	enum ServoBus { PWM, COM4 };

  Servo();
  Servo(XmlRpc::XmlRpcValue& servo_info);

  int          channel_;
  unsigned int type_;
  std::string  joint_name_;
  int          trim_pwm_;
  unsigned int max_pwm_;
  unsigned int min_pwm_;
  float        max_rot_; // in radians
  float        min_rot_; // in radians
  ServoBus     bus_;

private:

};

///////////////////////////////////////////////////////////////////////////////
// ServoLibrary
///////////////////////////////////////////////////////////////////////////////

class ServoLibrary : public std::map <std::string, Servo>
{
public:
  ServoLibrary();
  void loadServos();
  //bool openURDFfile(std::string robot_description_filepath); 
  //bool saveURDFfile(std::string robot_description_filepath); 
  //bool openURDFparam(std::string robot_description_name="robot_description");
  //bool saveURDFparam(std::string robot_description_name="robot_description");
  
  unsigned long getUsedPWMChannels() { return used_pwm_channels_; };
  //std::string   getURDF()         { return URDF_; };
  
private:
  //std::string   URDF_;
  unsigned long used_pwm_channels_;
  
  //bool parseURDF();
  //bool updateURDF();
};

} // namespace roboard_servos

#endif // __SERVO_H__

