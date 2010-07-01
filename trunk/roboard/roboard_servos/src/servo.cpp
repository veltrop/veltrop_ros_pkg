#include <string>
#include <iostream>
#include <ros/ros.h>
#include <tinyxml/tinyxml.h>
#include <roboard.h>
#include "servo.h"

namespace roboard_servos
{

Servo::Servo()
      : channel_(-1)
      , type_(RCSERVO_SERVO_DEFAULT)
      , joint_name_("")
      , trim_pwm_(0)
      , max_pwm_(2300)
      , min_pwm_(700)
      , max_rot_(HALFPI)
      , min_rot_(-HALFPI)
{  
}

ServoLibrary::ServoLibrary()
             : std::map <std::string, Servo> ()
             , URDF_("")
             , used_channels_(0)
{
}
            
bool ServoLibrary::openURDFfile(std::string robot_description_filepath)
{
  // TODO: load file
 
  return false;
}
 
bool ServoLibrary::saveURDFfile(std::string robot_description_filepath)
{
  updateURDF();
  
  TiXmlDocument doc;
  doc.Parse(URDF_.c_str());

  return doc.SaveFile(robot_description_filepath.c_str());
}
 
bool ServoLibrary::openURDFparam(std::string robot_description_name)
{
  //ros::param::param<std::string>(robot_description_name, URDF_, "");
  std::string new_URDF;
  if (!ros::param::getCached(robot_description_name, new_URDF))
  {
    ROS_ERROR("Robot Description not on paramater server");
    return false;
  }
  
  if (new_URDF == URDF_)
    return true;
  else
    URDF_ = new_URDF;
  
  bool ret = parseURDF();
  
  return ret;
}

bool ServoLibrary::saveURDFparam(std::string robot_description_name)
{ 
  updateURDF();
  
  ros::param::set(robot_description_name, URDF_);
  
  return true;
}

bool ServoLibrary::updateURDF()
{
  TiXmlDocument doc;
  doc.Parse(URDF_.c_str());
  TiXmlHandle hDoc(&doc);         
  TiXmlElement* pElem = hDoc.FirstChild("robot").Element();
  
  if (pElem)
  {    
    TiXmlHandle hRoot=TiXmlHandle(pElem);        
    pElem=hRoot.FirstChild("joint").Element();
    for (; pElem; pElem=pElem->NextSiblingElement("joint"))
    {
      std::string joint_name = pElem->Attribute("name"); 
      TiXmlHandle hJointRoot = TiXmlHandle(pElem);
      TiXmlElement* pServoElem = hJointRoot.FirstChild("servo").Element();          
      
      Servo the_servo = (*this)[joint_name];
      if (the_servo.channel_ == -1)
        continue;
      
      if (pServoElem)
      {
        pServoElem->SetAttribute("channel", the_servo.channel_);
        // TODO: reverse this:
        //pServoElem->QueryIntAttribute("lower", &new_servo.min_pwm_);
        //pServoElem->QueryIntAttribute("upper", &new_servo.max_pwm_);
        pServoElem->SetAttribute("trim", the_servo.trim_pwm_);
        
        // TODO: reverse this:
        //std::string type_string = pServoElem->Attribute("type");
        //if (type_string == "RCSERVO_KONDO_KRS78X")
        //    new_servo.type_ = RCSERVO_KONDO_KRS78X;
        //else if (type_string == "RCSERVO_HITEC_HSR8498")
        //    new_servo.type_ = RCSERVO_HITEC_HSR8498;
        //else
        //    new_servo.type_ = RCSERVO_SERVO_DEFAULT;   
      }
         
      // TODO: reverse this:           
      //TiXmlElement* pLimitElem = hJointRoot.FirstChild("limit").Element();
      //if (pLimitElem)
      //{     
      //  pLimitElem->QueryFloatAttribute("lower", &new_servo.min_rot_);
      //  pLimitElem->QueryFloatAttribute("upper", &new_servo.max_rot_);  
      //}
    }   
  }
  
  TiXmlPrinter printer;
	//printer.SetIndent( "\t" );
	doc.Accept( &printer );

  URDF_ = printer.CStr();
  
  return true;
}

bool ServoLibrary::parseURDF()
{
  // Get servo-id <-> joint-name mappings from model
  TiXmlDocument doc;
  doc.Parse(URDF_.c_str());
  TiXmlHandle hDoc(&doc);         
  TiXmlElement* pElem = hDoc.FirstChild("robot").Element();
  
  if (pElem)
  {    
    //std::string robot_name = pElem->Attribute("name");
    TiXmlHandle hRoot=TiXmlHandle(pElem);        
    pElem=hRoot.FirstChild("joint").Element();
    for (; pElem; pElem=pElem->NextSiblingElement("joint"))
    {
      std::string joint_name = pElem->Attribute("name"); 
      TiXmlHandle hJointRoot = TiXmlHandle(pElem);
      TiXmlElement* pServoElem = hJointRoot.FirstChild("servo").Element();          
      
      Servo new_servo;
      new_servo.joint_name_ = joint_name;
      
      if (pServoElem)
      {
        pServoElem->QueryIntAttribute("channel", &new_servo.channel_);
        // TODO: if these attributes arent present the we crash!!!!
        //pServoElem->QueryIntAttribute("lower", &new_servo.min_pwm_);
        //pServoElem->QueryIntAttribute("upper", &new_servo.max_pwm_);
        pServoElem->QueryIntAttribute("trim", &new_servo.trim_pwm_);
        std::string type_string = pServoElem->Attribute("type");
        
        if (type_string == "RCSERVO_KONDO_KRS78X")
            new_servo.type_ = RCSERVO_KONDO_KRS78X;
        else if (type_string == "RCSERVO_HITEC_HSR8498")
            new_servo.type_ = RCSERVO_HITEC_HSR8498;
        else
            new_servo.type_ = RCSERVO_SERVO_DEFAULT;   
      }
                    
      TiXmlElement* pLimitElem = hJointRoot.FirstChild("limit").Element();
      if (pLimitElem)
      {     
        pLimitElem->QueryFloatAttribute("lower", &new_servo.min_rot_);
        pLimitElem->QueryFloatAttribute("upper", &new_servo.max_rot_);  
      }
      
      if (new_servo.channel_ >= 1 && new_servo.channel_ <= 32)  
      { 
        (*this)[joint_name] = new_servo;
        used_channels_ |= 1L << ((unsigned int)new_servo.channel_ - 1);
      }
    }   
  }
  else
  {
    ROS_ERROR("No \"robot\" node in Robot Description."); 
    return false;
  }
  
  return true;
}

} // namespace roboard_servos

