#include <string>
//#include <iostream>
#include <ros/ros.h>
//#include <tinyxml/tinyxml.h>
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
      , bus_(PWM)
{  
}

Servo::Servo(XmlRpc::XmlRpcValue& servo_info)
      : channel_(-1)
      , type_(RCSERVO_SERVO_DEFAULT)
      , joint_name_("")
      , trim_pwm_(0)
      , max_pwm_(2300)
      , min_pwm_(700)
      , max_rot_(HALFPI)
      , min_rot_(-HALFPI)
      , bus_(PWM)
{
  if (servo_info.hasMember("joint_name"))
    joint_name_ = (std::string)servo_info["joint_name"];
  
	if (servo_info.hasMember("channel"))
  	channel_ = servo_info["channel"];

  if (servo_info.hasMember("type"))
  {
    std::string type_string = servo_info["type"];
    if (type_string == "RCSERVO_KONDO_KRS78X")
      type_ = RCSERVO_KONDO_KRS78X;
    else if (type_string == "RCSERVO_HITEC_HSR8498")
      type_ = RCSERVO_HITEC_HSR8498;
    else
      type_ = RCSERVO_SERVO_DEFAULT; 
  }   

  if (servo_info.hasMember("bus")) 
  {
    std::string bus_string = servo_info["bus"];
    if (bus_string == "COM4")
      bus_ = COM4;
  }

  if (servo_info.hasMember("min_rotation"))
    min_rot_ = (double)servo_info["min_rotation"];
  if (servo_info.hasMember("max_rotation"))
    max_rot_ = (double)servo_info["max_rotation"];  
    
  if (servo_info.hasMember("min_pwm"))
    min_pwm_ = (int)servo_info["min_pwm"];
  if (servo_info.hasMember("max_pwm"))
    max_pwm_ = (int)servo_info["max_pwm"]; 
    
  if (servo_info.hasMember("trim"))
    trim_pwm_ = servo_info["trim"];  
}

ServoLibrary::ServoLibrary()
             : std::map <std::string, Servo> ()
             //, URDF_("")
             , used_pwm_channels_(0)
{
	loadServos();
}
            
            
void ServoLibrary::loadServos()
{
	used_pwm_channels_ = 0;
  clear();

	ros::NodeHandle np("~");
  std::string param_name = "servos";
  XmlRpc::XmlRpcValue all_servos;
  
  if (!np.hasParam(param_name))
  {
    ROS_WARN_STREAM("No servos in " << param_name);
    return;
  }
  
  np.getParam(param_name, all_servos);
  
  if (all_servos.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN("Servos is not an array");
    return;
  }

  for (int i=0; i < all_servos.size(); i++) 
  {
  	Servo new_servo(all_servos[i]);
    (*this)[new_servo.joint_name_] = new_servo;
    
    if (new_servo.bus_ == Servo::PWM && new_servo.channel_ >= 1 && new_servo.channel_ <= 32)
    	used_pwm_channels_ |= 1L << ((unsigned int)new_servo.channel_ - 1);
  }
}
            
            
//bool ServoLibrary::openURDFfile(std::string robot_description_filepath)
//{ 
//  return false;
//}
 
//bool ServoLibrary::saveURDFfile(std::string robot_description_filepath)
//{
//  updateURDF();
//  
//  TiXmlDocument doc;
//  doc.Parse(URDF_.c_str());
//
//  return doc.SaveFile(robot_description_filepath.c_str());
//}
 
//bool ServoLibrary::openURDFparam(std::string robot_description_name)
//{
//  //ros::param::param<std::string>(robot_description_name, URDF_, "");
//  std::string new_URDF;
//  if (!ros::param::getCached(robot_description_name, new_URDF))
//  {
//    ROS_ERROR("Robot Description not on paramater server");
//    return false;
//  }
//  
//  if (new_URDF == URDF_)
//    return true;
//  else
//    URDF_ = new_URDF;
//  
//  bool ret = parseURDF();
//  
//  return ret;
//}

//bool ServoLibrary::saveURDFparam(std::string robot_description_name)
//{ 
//  updateURDF();
//  
//  ros::param::set(robot_description_name, URDF_);
//  
//  return true;
//}

//bool ServoLibrary::updateURDF()
//{
//  TiXmlDocument doc;
//  doc.Parse(URDF_.c_str());
//  TiXmlHandle hDoc(&doc);         
//  TiXmlElement* pElem = hDoc.FirstChild("robot").Element();
//  
//  if (pElem)
//  {    
//    TiXmlHandle hRoot=TiXmlHandle(pElem);        
//    pElem=hRoot.FirstChild("joint").Element();
//    for (; pElem; pElem=pElem->NextSiblingElement("joint"))
//    {
//      std::string joint_name = pElem->Attribute("name"); 
//      TiXmlHandle hJointRoot = TiXmlHandle(pElem);
//      TiXmlElement* pServoElem = hJointRoot.FirstChild("servo").Element();          
//      
//      Servo the_servo = (*this)[joint_name];
//      if (the_servo.channel_ == -1)
//        continue;
//      
//      if (pServoElem)
//      {
//        pServoElem->SetAttribute("channel", the_servo.channel_);
//        // TODO: reverse this:
//        //pServoElem->QueryIntAttribute("lower", &new_servo.min_pwm_);
//        //pServoElem->QueryIntAttribute("upper", &new_servo.max_pwm_);
//        pServoElem->SetAttribute("trim", the_servo.trim_pwm_);
//        
//        // TODO: reverse this:
//        //std::string type_string = pServoElem->Attribute("type");
//        //if (type_string == "RCSERVO_KONDO_KRS78X")
//        //    new_servo.type_ = RCSERVO_KONDO_KRS78X;
//        //else if (type_string == "RCSERVO_HITEC_HSR8498")
//        //    new_servo.type_ = RCSERVO_HITEC_HSR8498;
//        //else
//        //    new_servo.type_ = RCSERVO_SERVO_DEFAULT;   
//      }
//         
//      // TODO: reverse this:           
//      //TiXmlElement* pLimitElem = hJointRoot.FirstChild("limit").Element();
//      //if (pLimitElem)
//      //{     
//      //  pLimitElem->QueryFloatAttribute("lower", &new_servo.min_rot_);
//      //  pLimitElem->QueryFloatAttribute("upper", &new_servo.max_rot_);  
//      //}
//    }   
//  }
//  
//  TiXmlPrinter printer;
//	//printer.SetIndent( "\t" );
//	doc.Accept( &printer );
//
//  URDF_ = printer.CStr();
//  
//  return true;
//}

//bool ServoLibrary::parseURDF()
//{
//  // Get servo-id <-> joint-name mappings from model
//  TiXmlDocument doc;
//  doc.Parse(URDF_.c_str());
//  TiXmlHandle hDoc(&doc);         
//  TiXmlElement* pElem = hDoc.FirstChild("robot").Element();
//  
//  if (pElem)
//  {    
//    //std::string robot_name = pElem->Attribute("name");
//    TiXmlHandle hRoot=TiXmlHandle(pElem);        
//    pElem=hRoot.FirstChild("joint").Element();
//    for (; pElem; pElem=pElem->NextSiblingElement("joint"))
//    {
//      std::string joint_name = pElem->Attribute("name"); 
//      TiXmlHandle hJointRoot = TiXmlHandle(pElem);
//      TiXmlElement* pServoElem = hJointRoot.FirstChild("servo").Element();          
//      
//      Servo new_servo;
//      new_servo.joint_name_ = joint_name;
//      
//      if (pServoElem)
//      {
//        pServoElem->QueryIntAttribute("channel", &new_servo.channel_);
//        // TODO: if these attributes arent present the we crash!!!!
//        //pServoElem->QueryIntAttribute("lower", &new_servo.min_pwm_);
//        //pServoElem->QueryIntAttribute("upper", &new_servo.max_pwm_);
//        pServoElem->QueryIntAttribute("trim", &new_servo.trim_pwm_);
//        std::string type_string = pServoElem->Attribute("type");
//        
//        if (type_string == "RCSERVO_KONDO_KRS78X")
//            new_servo.type_ = RCSERVO_KONDO_KRS78X;
//        else if (type_string == "RCSERVO_HITEC_HSR8498")
//            new_servo.type_ = RCSERVO_HITEC_HSR8498;
//        else
//            new_servo.type_ = RCSERVO_SERVO_DEFAULT;   
//      }
//                    
//      TiXmlElement* pLimitElem = hJointRoot.FirstChild("limit").Element();
//      if (pLimitElem)
//      {     
//        pLimitElem->QueryFloatAttribute("lower", &new_servo.min_rot_);
//        pLimitElem->QueryFloatAttribute("upper", &new_servo.max_rot_);  
//      }
//      
//      if (new_servo.channel_ >= 1 && new_servo.channel_ <= 32)  
//      { 
//        (*this)[joint_name] = new_servo;
//        used_channels_ |= 1L << ((unsigned int)new_servo.channel_ - 1);
//      }
//    }   
//  }
//  else
//  {
//    ROS_ERROR("No \"robot\" node in Robot Description."); 
//    return false;
//  }
//  
//  return true;
//}

} // namespace roboard_servos

