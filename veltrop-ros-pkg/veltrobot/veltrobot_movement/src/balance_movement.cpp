#include <vector>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>

namespace veltrobot_movement
{

struct JointMultiplier
{
	std::string joint_name;
	float multiplier;
  size_t joint_state_index;
};

class BalanceMovement
{
public:
  BalanceMovement()
  {      
  	ros::NodeHandle n;
    ros::NodeHandle np("~");
    
		XmlRpc::XmlRpcValue all_joints;
  
  	if (!np.hasParam("pitch"))
    	ROS_WARN("No pitch devices");
    if (!np.hasParam("roll"))
    	ROS_WARN("No roll devices");  
  
  	np.getParam("pitch", all_joints);
  	if (all_joints.getType() != XmlRpc::XmlRpcValue::TypeArray)
    	ROS_WARN("pitch is not an array");
		for (int i=0; i < all_joints.size(); i++)
  	{
    	if (!all_joints[i].hasMember("joint_name") ||
          !all_joints[i].hasMember("multiplier"))
    	{
      	ROS_WARN("All entries must have a joint_name and multiplier");
      	continue;
    	}
      JointMultiplier new_joint;
			new_joint.joint_name = std::string(all_joints[i]["joint_name"]);
      new_joint.multiplier = float(double(all_joints[i]["multiplier"]));
			js_.name.push_back(new_joint.joint_name);
			js_.position.push_back(0);
			js_.velocity.push_back(0);
      new_joint.joint_state_index = js_.name.size() - 1;
    	pitch_multipliers_.push_back(new_joint);
		}
    
   	np.getParam("roll", all_joints);
  	if (all_joints.getType() != XmlRpc::XmlRpcValue::TypeArray)
    	ROS_WARN("roll is not an array");
		for (int i=0; i < all_joints.size(); i++)
  	{
    	if (!all_joints[i].hasMember("joint_name") ||
          !all_joints[i].hasMember("multiplier"))
    	{
      	ROS_WARN("All entries must have a joint_name and multiplier");
      	continue;
    	}
      JointMultiplier new_joint;
			new_joint.joint_name = std::string(all_joints[i]["joint_name"]);
      new_joint.multiplier = float(double(all_joints[i]["multiplier"]));
			js_.name.push_back(new_joint.joint_name);
			js_.position.push_back(0);
			js_.velocity.push_back(0);
      new_joint.joint_state_index = js_.name.size() - 1;      
    	roll_multipliers_.push_back(new_joint);
		}
    
    double freq;
  	np.param<double>("processing_fps", freq, 10.0); 
    processing_freq_ = freq;
    np.param<double>("movement_fps", freq, 10.0); 
    movement_freq_ = freq;
    
  	//transmit_timer_ = np.createTimer(ros::Duration(1.0 / processing_freq_), &BalanceMovement::transmitCB, this);
       
    // Prepare joint state publisher
    joint_states_pub_ = n.advertise<sensor_msgs::JointState>("/balancing_joint_states", 1);
  
    gyro_pitch_sub_ = n.subscribe("/gyro/pitch", 1, 
                                    &BalanceMovement::receiveGyroPitchCB, this);  
    gyro_roll_sub_ = n.subscribe("/gyro/roll", 1, 
                                    &BalanceMovement::receiveGyroRollCB, this);                                
  }
  
  void spin()
  {
  	ros::Rate loop_rate(processing_freq_); 
    
    while (ros::ok())
    {
    	ros::spinOnce();
      // TODO: better logic of to publish or not.
      //       services to enable/disable balancing, set balancing style
    	joint_states_pub_.publish(js_);
    	loop_rate.sleep();
    }
  }

private:
  ros::Publisher  joint_states_pub_;
  ros::Subscriber gyro_pitch_sub_;
  ros::Subscriber gyro_roll_sub_;
  std::vector <JointMultiplier> pitch_multipliers_;
	std::vector <JointMultiplier> roll_multipliers_;
  sensor_msgs::JointState js_; 
  //ros::Timer transmit_timer_;
  float processing_freq_;
  float movement_freq_;
  
  // NOTE: disabled becuase using ros::timer's is too CPU intensive on Roboard...
  /*void transmitCB(const ros::TimerEvent& e)
  {
  	// TODO: post process: prune joints below a threshold
    //       define threshold in config file.
    
  	joint_states_pub_.publish(js_);
  }*/
  
  void processGyro(const std_msgs::Float32ConstPtr& msg,
                   const std::vector <JointMultiplier>& multipliers)
  {
  	
    // TODO: add averaging over time. freq of average determined by config.
    
  	for (size_t i=0; i < multipliers.size(); i++)
    {
    	size_t j = multipliers[i].joint_state_index;
  		float pi_sec = msg->data / 180.0f * 3.14159f; 
      float duration = 1.0f / movement_freq_;
      float offset = pi_sec * duration;
      js_.position[j] = multipliers[i].multiplier * offset;
      js_.velocity[j] = duration * 1000.0f;
    }
  }

  void receiveGyroPitchCB(const std_msgs::Float32ConstPtr& msg)
  {
		processGyro(msg, pitch_multipliers_);
  }
  
  void receiveGyroRollCB(const std_msgs::Float32ConstPtr& msg)
  {
    processGyro(msg, roll_multipliers_);
  }  
};

} // namespace veltrobot_movement

int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_movement");
  ros::NodeHandle n;
  veltrobot_movement::BalanceMovement balance;
  //ros::spin();
  balance.spin();
  
  return 0;
}

