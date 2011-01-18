#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
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
    joint_states_pub_ = n_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    enable_joint_group_pub_ = n_.advertise<veltrobot_msgs::EnableJointGroup>("/enable_joint_group", 10);
    joy1_sub_ = n_.subscribe <joy::Joy> ("/wiimote1/wiijoy", 10, &VeltrobotTeleopDualWii::joy1CB, this);
    joy2_sub_ = n_.subscribe <joy::Joy> ("/wiimote2/wiijoy", 10, &VeltrobotTeleopDualWii::joy2CB, this);
	}
	
	void spin()
	{
	  ros::spin();
	}
	
private:
	ros::NodeHandle n_;
	ros::Publisher  motion_pub_;
  ros::Publisher  joint_states_pub_;
  ros::Publisher  enable_joint_group_pub_;
  ros::Subscriber joy1_sub_;
	ros::Subscriber	joy2_sub_;

	void joy1CB(const joy::Joy::ConstPtr& joy)
  {
    veltrobot_msgs::EnableJointGroup group;
    sensor_msgs::JointState js; 
    std_msgs::String mot;
    mot.data = "";
    
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
				float pos = prev_B ? 0.0f : 0.95f;
      	js.name.push_back("hand_left");
      	js.position.push_back(pos);
      	js.velocity.push_back(10);
    	}
		//}

		if (prev_A)
		{
			// input range is -10~+10 over -halfpi~+halfpi
			// but the robot has 1.15pi range available
		  float pos = joy->axes[0] * HALFPI / 10.0f;
	   	js.name.push_back("wrist_left_yaw");
     	js.position.push_back(pos);
     	js.velocity.push_back(10);
		}

    if (mot.data != "")
      motion_pub_.publish(mot); 
      
    if (js.name.size())
      joint_states_pub_.publish(js);

		if (group.jointGroups.size())
      enable_joint_group_pub_.publish(group);
  }
	
	void joy2CB(const joy::Joy::ConstPtr& joy)
	{ 
    veltrobot_msgs::EnableJointGroup group;
    sensor_msgs::JointState js; 
    std_msgs::String mot;
    mot.data = "";
    
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
				float pos = prev_B ? 0.0f : 0.95f;
      	js.name.push_back("hand_right");
      	js.position.push_back(pos);
      	js.velocity.push_back(10);
			}
		//}

		if (prev_A)
		{
			// input range is -10~+10 over -halfpi~+halfpi
			// but the robot has 1.15pi range available
		  float pos = joy->axes[0] * HALFPI / 10.0f;
	   	js.name.push_back("wrist_right_yaw");
     	js.position.push_back(pos);
     	js.velocity.push_back(10);
		}

	  if (mot.data != "")
      motion_pub_.publish(mot); 
      
    if (js.name.size())
      joint_states_pub_.publish(js);

		if (group.jointGroups.size())
      enable_joint_group_pub_.publish(group);
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

