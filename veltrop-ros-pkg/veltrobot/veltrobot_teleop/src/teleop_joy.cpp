#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <veltrobot_msgs/EnableJointGroup.h>
#include <joy/Joy.h>

namespace veltrobot_teleop
{

class VeltrobotTeleopJoy
{
public:
	VeltrobotTeleopJoy()
	{
  
	  motion_pub_ = n_.advertise <std_msgs::String> ("motion_name", 1);
    joint_states_pub_ = n_.advertise<sensor_msgs::JointState>("/joint_states", 1);
    joy_sub_ = n_.subscribe <joy::Joy> ("joy", 1, &VeltrobotTeleopJoy::joyCB, this);
    enable_joint_group_pub_ = n_.advertise<veltrobot_msgs::EnableJointGroup>("/enable_joint_group", 1);
	}
	
	void spin()
	{
	  ros::spin();
	}
	
private:
	ros::NodeHandle n_;
	ros::Publisher motion_pub_;
  ros::Publisher  joint_states_pub_;
  ros::Publisher  enable_joint_group_pub_;
  ros::Subscriber joy_sub_;
	
	void joyCB(const joy::Joy::ConstPtr& joy)
  {
    veltrobot_msgs::EnableJointGroup group;
  
    std_msgs::String mot;
    mot.data = "";
    static bool prev_move = false;
    
    if (!joy->buttons[4] && !joy->buttons[5] && 
        !joy->buttons[6] && !joy->buttons[7] && prev_move)
    {
      mot.data = "stand_squat";
      prev_move = false;
    }
    else if (joy->buttons[7])
    {
      mot.data = "rotate_left";
      prev_move = true;
    }
    else if (joy->buttons[5])
		{
      mot.data = "rotate_right";
      prev_move = true;
    }
    else if (joy->buttons[4])
		{
      mot.data = "walk_forward";
      prev_move = true;
    }
    else if (joy->buttons[6])
    {
    	mot.data = "walk_backward";
      prev_move = true;
    }
    
    else if (joy->buttons[3])
		{
      mot.data = "init";
      prev_move = false;
    }
    else if (joy->buttons[0])
    {
    	mot.data = "stand_squat";
      prev_move = false;
    }
    
    static bool prev_L1 = false;
    if (joy->buttons[10] != prev_L1) // L1
    {
      prev_L1 = joy->buttons[10];
      group.jointGroups.push_back("arms");
      group.enabledStates.push_back(prev_L1);
    }
    
    static bool prev_L2 = false;
    if (joy->buttons[8] != prev_L2) // L2
    {
      prev_L2 = joy->buttons[8];
      group.jointGroups.push_back("legs");
      group.enabledStates.push_back(prev_L2);
    }
                         
    if (joy->buttons[1])
    {    
      float neck_yaw = joy->axes[0] * -1.57;
      float neck_pitch = joy->axes[1] * -1.57;

      sensor_msgs::JointState js; 
      js.name.push_back("neck_yaw");
      js.position.push_back(neck_yaw);
      js.velocity.push_back(10);
      js.name.push_back("neck_pitch");
      js.position.push_back(neck_pitch);
      js.velocity.push_back(10);

      joint_states_pub_.publish(js);
    }

    if (mot.data != "")
      motion_pub_.publish(mot); 
      
    if (group.jointGroups.size())
      enable_joint_group_pub_.publish(group);
      
  }
};

} // namespace veltrobot_teleop

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_joy");
	veltrobot_teleop::VeltrobotTeleopJoy this_node;

	this_node.spin();
  
	return(0);
}

