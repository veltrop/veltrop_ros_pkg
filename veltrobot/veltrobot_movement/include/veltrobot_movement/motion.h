#ifndef _MOTION_H
#define _MOTION_H

#include <string>
#include <vector>
#include <map>
#include <stdint.h>
#include <tinyxml/tinyxml.h>

namespace veltrobot_movement
{

///////////////////////////////////////////////////////////////////////////////
// MotionPhase
///////////////////////////////////////////////////////////////////////////////

class MotionPhase
{
public:
  MotionPhase();
  void clear();
  void loadXML(TiXmlElement* pElem);

public:    
  std::string               name_;
  int32_t                   duration_;
  std::vector <std::string> poses_;
  std::string               next_phase_;
  bool                      abort_safe_;
  bool											balancing_enabled_;
};

///////////////////////////////////////////////////////////////////////////////
// Motion
///////////////////////////////////////////////////////////////////////////////

class Motion
{
public:
  Motion();  
  void clear();
  void loadXML(std::string filename);

public:
  std::string                          name_;
  std::string                          first_phase_;
  std::map <std::string, MotionPhase>  phases_;
};

///////////////////////////////////////////////////////////////////////////////
// MotionLibrary
///////////////////////////////////////////////////////////////////////////////

class MotionLibrary : public std::map <std::string, Motion>
{
public:
  void loadDirectory(std::string directory);    
};

}

#endif // MOTION_H

