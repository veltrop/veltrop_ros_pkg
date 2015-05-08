#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <veltrobot_msgs/EnableJointGroup.h>
#include <joy/Joy.h>

#ifndef PI
#define PI 3.14159265359
#endif
#ifndef HALFPI
#define HALFPI 1.57079632679
#endif 
#ifndef QUARTPI
#define QUARTPI 0.785398163397
#endif

namespace veltrobot_teleop
{

class VeltrobotTeleopDualWii
{
public:
	VeltrobotTeleopDualWii()
	{
	  motion_pub_ = n_.advertise <std_msgs::String> ("motion_name", 10);
    joint_states_pub_ = n_.advertise<sensor_msgs::JointState>("joint_states", 10);
    enable_joint_group_pub_ = n_.advertise<veltrobot_msgs::EnableJointGroup>("enable_joint_group", 10);
    virtual_joy_pub_ = n_.advertise <joy::Joy> ("joy", 10);
    joy1_sub_ = n_.subscribe <joy::Joy> ("wiimote1/wiijoy", 10, &VeltrobotTeleopDualWii::joy1CB, this);
    joy2_sub_ = n_.subscribe <joy::Joy> ("wiimote2/wiijoy", 10, &VeltrobotTeleopDualWii::joy2CB, this);
    arm_control_method_pub_ = n_.advertise <std_msgs::String> ("arm_control_method", 10);
    text_to_speech_pub_ = n_.advertise <std_msgs::String> ("speech", 1);
    calib_torso_pub_ = n_.advertise <std_msgs::Empty> ("calibrate_torso", 1);
    goto_torso_dest_pub_ = n_.advertise <std_msgs::Empty> ("goto_torso_destination", 1);
    
    virtual_joy_state_.axes.resize(4);
    virtual_joy_state_.buttons.resize(22); 
  }
	
	void spin()
	{
	  ros::spin();
	}
	
private:
	ros::NodeHandle n_;
  ros::Publisher  goto_torso_dest_pub_;
  ros::Publisher  calib_torso_pub_;
  ros::Publisher  motion_pub_;
  ros::Publisher  joint_states_pub_;
  ros::Publisher  enable_joint_group_pub_;
  ros::Subscriber joy1_sub_;
	ros::Subscriber	joy2_sub_;
  ros::Publisher  virtual_joy_pub_;
  joy::Joy        virtual_joy_state_;
  ros::Publisher  arm_control_method_pub_;
  ros::Publisher  text_to_speech_pub_;

  void processVirtual(const joy::Joy::ConstPtr& joy, int num)
  {
    int axis_offset = 0;
    int button_offset = 0;
    if (num == 2)
    {
      axis_offset = 2;
      button_offset = 11;
    }

    float dpad_x = 0.0;
    float dpad_y = 0.0;
    if (joy->buttons[7])
      dpad_y = -0.5;
    if (joy->buttons[6])
      dpad_y = 0.5;
    if (joy->buttons[9])
      dpad_x = -0.5;
    if (joy->buttons[8])
      dpad_x = 0.5;

    virtual_joy_state_.axes[axis_offset  ] = dpad_x;
    virtual_joy_state_.axes[axis_offset+1] = dpad_y;
    for (size_t i=0; i < 11 && i < joy->buttons.size(); i++)
      virtual_joy_state_.buttons[button_offset + i] = joy->buttons[i];

    virtual_joy_pub_.publish(virtual_joy_state_);
  }

	void joy1CB(const joy::Joy::ConstPtr& joy)
  {
    veltrobot_msgs::EnableJointGroup group;
    std_msgs::Empty em;
    sensor_msgs::JointState js; 
    //std_msgs::String mot;
    //mot.data = "";
   
    static bool prev_minus = false;
    if (joy->buttons[5])
    {
      // activate on button depress
      if (prev_minus == false)
      {
        calib_torso_pub_.publish(em);
      }
      // activate continuously during depress
      else
      {
        //goto_torso_dest_pub_.publish(em);
      }
    } 
    else
    {
      // activate on button raise
      if (prev_minus == true)
      {
        goto_torso_dest_pub_.publish(em);
      }
    }
    prev_minus = joy->buttons[5];

    static bool prev_home = false;
    if (joy->buttons[10])
    {
      // activate on button depress
      if (prev_home == false)
      {
        calib_torso_pub_.publish(em);
      }
      // activate continuously during depress
      else
      {
      }
    } 
    else
    {
      // activate on button raise
      if (prev_home == true)
      {
      }
    }
    prev_home = joy->buttons[10];

    static bool prev_1 = false;
    if (joy->buttons[0] && prev_1 == false)
    {
      std_msgs::String method;
      method.data = "DIRECT";
      arm_control_method_pub_.publish(method);
      std_msgs::String speech;
      speech.data = "Direct mode.";
      text_to_speech_pub_.publish(speech);
    }
    prev_1 = joy->buttons[0];
    
    static bool prev_2 = false;
    if (joy->buttons[1] && prev_2 == false)
    {
      std_msgs::String method;
      method.data = "IK";
      arm_control_method_pub_.publish(method);
      std_msgs::String speech;
      speech.data = "Ai kay mode.";
      text_to_speech_pub_.publish(speech);
    }
    prev_2 = joy->buttons[1];

    static bool prev_A = false;
    if (joy->buttons[2] != prev_A)
    {
      prev_A = joy->buttons[2];
      group.jointGroups.push_back("left_arm");
      group.enabledStates.push_back(prev_A);
    }
		
    static bool prev_B = false;
		//if (joy->buttons[3] != prev_B)
    //{    
			prev_B = joy->buttons[3];
		  if (prev_A)
			{	
				// 1 = open, 0 = closed
				//float pos = prev_B ? 0.0f : 0.95f;
				float pos = prev_B ? 0.0f : 0.5f;
      	js.name.push_back("hand_left");
      	js.position.push_back(pos);
      	js.velocity.push_back(10);
    	}
		//}

		if (prev_A)
		{
		  static float prev_pos = 0;
			// input range is -10~+10 over -halfpi~+halfpi
			// but the robot has 1.15pi range available
		  float pos = -joy->axes[0] * HALFPI / 10.0f;
      //pos *= 1.15;
	   	js.name.push_back("wrist_left_yaw");
     	js.position.push_back((pos+prev_pos)*0.5f);
     	js.velocity.push_back(10);
     	
     	prev_pos = pos;
		}

    //if (mot.data != "")
    //  motion_pub_.publish(mot); 
      
    if (js.name.size())
      joint_states_pub_.publish(js);

		if (group.jointGroups.size())
      enable_joint_group_pub_.publish(group);

    processVirtual(joy, 1);
  }
	
	void joy2CB(const joy::Joy::ConstPtr& joy)
	{ 
    veltrobot_msgs::EnableJointGroup group;
    sensor_msgs::JointState js; 
    std_msgs::String mot;
    mot.data = "";
  
   
    // these are triggered when depressed
    static bool prev_up = false;
    if (joy->buttons[8] && prev_up == false)
      mot.data = "top_camera";
    prev_up = joy->buttons[0];
    
    static bool prev_down = false;
    if (joy->buttons[9] && prev_down == false)
      mot.data = "bottom_camera";
    prev_down = joy->buttons[0];          
    
    static bool prev_1 = false;
    if (joy->buttons[0] && prev_1 == false)
      mot.data = "sit";
    prev_1 = joy->buttons[0];
    
    static bool prev_2 = false;
    if (joy->buttons[1] && prev_2 == false)
      mot.data = "stand";
    prev_2 = joy->buttons[1];
    
    static bool prev_minus = false;
    if (joy->buttons[5] && prev_minus == false)
      mot.data = "relax";
    prev_minus = joy->buttons[5];
    
    static bool prev_plus = false;
    if (joy->buttons[4] && prev_plus == false)
      mot.data = "stiffen";
    prev_plus = joy->buttons[4];

    // trigger different signals when depressed or lifted
    static bool prev_A = false;
    if (joy->buttons[2] != prev_A)
    {
      prev_A = joy->buttons[2];
      group.jointGroups.push_back("right_arm");
      group.enabledStates.push_back(prev_A);
    }

    static bool prev_B = false;
		//if (joy->buttons[3] != prev_B)
    //{    
			prev_B = joy->buttons[3];
		  if (prev_A)
			{	
				// 1 = open, 0 = closed
				float pos = prev_B ? 0.0f : -0.50f;
				//float pos = prev_B ? 0.0f : 0.8f;
      	js.name.push_back("hand_right");
      	js.position.push_back(pos);
      	js.velocity.push_back(10);
			}
		//}

		if (prev_A)
		{
		  static float prev_pos = 0;
			// input range is -10~+10 over -halfpi~+halfpi
			// but the robot has 1.15pi range available
		  float pos = -joy->axes[0] * HALFPI / 10.0f;
      //pos *= 1.15;
	   	js.name.push_back("wrist_right_yaw");
     	js.position.push_back((pos+prev_pos)*0.5f);
     	js.velocity.push_back(10);
     	prev_pos = pos;
		}

	  if (mot.data != "")
      motion_pub_.publish(mot); 
      
    if (js.name.size())
      joint_states_pub_.publish(js);

		if (group.jointGroups.size())
      enable_joint_group_pub_.publish(group);
    
    processVirtual(joy, 2);
	}

};

} // namespace veltrobot_teleop

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_dual_wii");
	veltrobot_teleop::VeltrobotTeleopDualWii this_node;

	this_node.spin();
  
	return(0);
}

