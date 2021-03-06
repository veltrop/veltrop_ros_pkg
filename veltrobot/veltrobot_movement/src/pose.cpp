#include <sys/types.h>
#include <dirent.h>
#include <ros/ros.h>
#include <string>
#include <veltrobot_movement/pose.h>
#include <tinyxml/tinyxml.h>

namespace veltrobot_movement
{

///////////////////////////////////////////////////////////////////////////////
// Pose
///////////////////////////////////////////////////////////////////////////////

Pose::Pose()
{
  clear();
}

void Pose::clear()
{
  name_ = "";
  unit_ = pwm_offset;
  duration_ = 0;
  positions_.clear();
}

void Pose::saveXML(std::string filename)
{
	if (filename != "")
  	filename_ = filename;
    
  TiXmlDocument doc;
	TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "", "");
  doc.LinkEndChild(decl);
  
  TiXmlElement* pose_element = new TiXmlElement("pose");
  doc.LinkEndChild(pose_element);
  pose_element->SetAttribute("name", name_.c_str());
  
  TiXmlElement* joint_positions_element = new TiXmlElement("joint_positions");
  pose_element->LinkEndChild(joint_positions_element);
  joint_positions_element->SetAttribute("duration", duration_);
  joint_positions_element->SetAttribute("units", "radians_offset");
	    
  std::map<std::string, float>::iterator i = positions_.begin();
  for(; i != positions_.end(); ++i)
  {
    const std::string& joint_name = i->first;
    float position = i->second;

		TiXmlElement* joint_element = new TiXmlElement("joint");
    joint_positions_element->LinkEndChild(joint_element);
    joint_element->SetAttribute("name", joint_name.c_str());
    joint_element->SetDoubleAttribute("position", position);
  }
  
	doc.SaveFile(filename.c_str());
}

void Pose::loadXML(std::string filename)
{
	TiXmlDocument doc(filename.c_str());
	if (!doc.LoadFile())
    return;
    
  filename_ = filename;
	
	clear();

	TiXmlHandle hDoc(&doc);
	TiXmlElement* pElem;
	TiXmlHandle hRoot(0);
	
	//
	// Load root node "pose"
	//
	//pElem=hDoc.FirstChildElement().Element();   // load first child
	pElem = hDoc.FirstChild("pose").Element();    // load first child named pose
	if (!pElem)
	  return;
	name_ = pElem->Attribute("name");
	hRoot=TiXmlHandle(pElem);

	//
	// Load child node "joint_positions"
	//
	pElem=hRoot.FirstChild("joint_positions").Element();
  pElem->QueryIntAttribute("duration", &duration_);
  const char *pUnitsStr=pElem->Attribute("units");
  if (strcmp("radians_offset", pUnitsStr) == 0)
    unit_ = radians_offset;    
  else if (strcmp("degrees_offset", pUnitsStr) == 0)
    unit_ = degrees_offset;
  else if (strcmp("pwm_offset", pUnitsStr) == 0)
    unit_ = pwm_offset;       
  else 
    unit_ = pwm;
   
	//
	// Load child nodes "joint"
	//    
	pElem=hRoot.FirstChild("joint_positions").FirstChild().Element();
	for (; pElem; pElem=pElem->NextSiblingElement()) {
    const char *pStr=pElem->Value();
		
		if (strcmp("joint", pStr) == 0) {
      std::string joint_name;
      float joint_position=0;
      joint_name = pElem->Attribute("name");
      pElem->QueryFloatAttribute("position", &joint_position);

      positions_[joint_name] = joint_position;
		}
	}  
}

///////////////////////////////////////////////////////////////////////////////
// PoseLibrary
///////////////////////////////////////////////////////////////////////////////

void PoseLibrary::loadDirectory(std::string directory)
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
      Pose new_pose;
      new_pose.loadXML(pathname);
      
      (*this)[new_pose.name_] = new_pose;
    }

    closedir(dp);
  }
  else
  {
    ROS_ERROR("Can't open pose directory");  
  }
}

}

