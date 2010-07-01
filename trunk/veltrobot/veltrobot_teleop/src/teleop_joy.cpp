#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
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
	}
	
	void spin()
	{
	  ros::spin();
	}
	
private:
	ros::NodeHandle n_;
	ros::Publisher motion_pub_;
  ros::Publisher  joint_states_pub_;
  ros::Subscriber joy_sub_;
	
	void joyCB(const joy::Joy::ConstPtr& joy)
  {
    std_msgs::String mot;
    mot.data = "";
    
    if (!joy->buttons[4] && !joy->buttons[5] && 
        !joy->buttons[6] && !joy->buttons[7])
      mot.data = "stand_squat";
    else if (joy->buttons[7])
      mot.data = "rotate_left";
    else if (joy->buttons[5])
      mot.data = "rotate_right";
    else if (joy->buttons[4])
      mot.data = "walk_forward";
    else if (joy->buttons[6])
      mot.data = "walk_backward";
                      
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

