#include <sys/types.h>
#include <dirent.h>
#include <ros/ros.h>
//#include <string>
//#include <tinyxml/tinyxml.h>
#include <veltrobot_movement/motion.h>

namespace veltrobot_movement
{

///////////////////////////////////////////////////////////////////////////////
// MotionPhase
///////////////////////////////////////////////////////////////////////////////

MotionPhase::MotionPhase()
{
  clear();
}

void MotionPhase::clear()
{
  name_ = "";
  next_phase_ = "";
  abort_safe_ = true;
  duration_ = 0;
  poses_.clear();
}

void MotionPhase::loadXML(TiXmlElement* pElem)
{
  if (!pElem)
    return;

  const char *pStr=pElem->Value();		
  if (strcmp("phase", pStr) != 0)
    return;      
      
  clear();
  
  name_ = pElem->Attribute("name");
  next_phase_ = pElem->Attribute("next_phase"); 
  pElem->QueryIntAttribute("duration", &duration_);
  pStr = pElem->Attribute("abort_safe");
  if (strcmp("true", pStr) == 0)
    abort_safe_ = true;
  else
    abort_safe_ = false;  
        
  //
	// Load child node "pose" and its siblings
	//
	TiXmlHandle hRoot(0);
	hRoot=TiXmlHandle(pElem);
	pElem = hRoot.FirstChild("pose").Element();  
	for (; pElem; pElem=pElem->NextSiblingElement()) {
    const char *pStr=pElem->Value();
		
		if (strcmp("pose", pStr) == 0) {
		  std::string new_pose = pElem->Attribute("name");	
		  poses_.push_back(new_pose);	
		}
	}	 
}

///////////////////////////////////////////////////////////////////////////////
// Motion
///////////////////////////////////////////////////////////////////////////////

Motion::Motion()
{
  clear();
}

void Motion::clear()
{
  name_ = "";
  first_phase_ = "";
  phases_.clear();
}

void Motion::loadXML(std::string filename)
{
	TiXmlDocument doc(filename.c_str());
	if (!doc.LoadFile())
    return;
	
	clear();

	TiXmlHandle hDoc(&doc);
	TiXmlElement* pElem;
	TiXmlHandle hRoot(0);
	
  //
	// Load root node "motion"
	//
	//pElem=hDoc.FirstChildElement().Element(); // load first child
	pElem=hDoc.FirstChild("motion").Element();    // load first child named motion
	if (!pElem)
	  return;
	name_ = pElem->Attribute("name");
	first_phase_ = pElem->Attribute("first_phase");
	hRoot=TiXmlHandle(pElem);
	
  //
	// Load child nodes "phase" below "phases"
	//
	pElem=hRoot.FirstChild("phases").FirstChild().Element();
	for (; pElem; pElem=pElem->NextSiblingElement()) {
    const char *pStr=pElem->Value();
		
		if (strcmp("phase", pStr) == 0) {
	    MotionPhase new_phase;
	    new_phase.loadXML(pElem);
	    phases_[new_phase.name_] = new_phase; 	    
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// MotionLibrary
///////////////////////////////////////////////////////////////////////////////

void MotionLibrary::loadDirectory(std::string directory)
{
  DIR *dp = opendir(directory.c_str());
  struct dirent *ep; 

  if (dp != NULL)
  {
    while ((ep = readdir(dp)))
    {
      std::string fname = ep->d_name;
      if (fname[0] == '.' || fname.substr(fname.length()-4 ,4) != ".xml")
        continue;
      
      std::string pathname = directory + '/' + fname;
      Motion new_motion;
      new_motion.loadXML(pathname);
      
      (*this)[new_motion.name_] = new_motion;
    }

    closedir(dp);
  }
  else
  {
    ROS_ERROR("Can't open motion directory");  
  }
}

}

