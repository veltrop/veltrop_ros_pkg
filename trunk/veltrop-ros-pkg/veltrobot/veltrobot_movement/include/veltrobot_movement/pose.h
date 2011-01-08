#ifndef _POSE_H
#define _POSE_H

#include <string>
#include <map>
#include <stdint.h>
#include <tinyxml/tinyxml.h>


namespace veltrobot_movement
{

///////////////////////////////////////////////////////////////////////////////
// Pose
///////////////////////////////////////////////////////////////////////////////

class Pose
{
public:
  enum unit { pwm, pwm_offset, degrees_offset, radians_offset };
  // pwm            | direct pulse width
  // pwm_offset     | pulse width offset from a home position
  // degrees_offset | angle in degrees offset from a home position
  // radians_offset | angle in radians offset from a home position

public:
  Pose();  
  void clear();
  void loadXML(std::string filename);
  void saveXML(std::string filename="");
    
public:
  std::string                  name_;
  int32_t                      duration_;    // in ms
  unit                         unit_;
  std::map<std::string, float> positions_;   // in veltrobot_movement::unit
  std::string                  filename_;
  //TiXmlDocument                doc_;
};

///////////////////////////////////////////////////////////////////////////////
// PoseLibrary
///////////////////////////////////////////////////////////////////////////////

class PoseLibrary : public std::map <std::string, Pose>
{
public:
  void loadDirectory(std::string directory);    
};

}

#endif // _POSE_H

