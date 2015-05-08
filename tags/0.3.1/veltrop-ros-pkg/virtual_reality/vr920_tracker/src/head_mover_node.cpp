#include <vector>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

#define RING_SIZE 8
int ring_i = 0;
float processRing(std::vector<float>& v, float f)
{
  if (v.size() != RING_SIZE)
    v.resize(RING_SIZE, 0.0f);
  v[ring_i] = f;

  float total = 0.0f;
  for (size_t i=0; i < v.size(); i++)
    total += v[i];

  return total / float(v.size());
}
void incrementRing()
{
  ring_i = ++ring_i % RING_SIZE;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "head_mover_node");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	tf::TransformListener lr;
	ros::Publisher joint_states_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	
	static std::vector <float> pitch_ring(RING_SIZE, 0.0f);
	static std::vector <float> yaw_ring(RING_SIZE, 0.0f);
	
	
	ros::Rate r(40);
  while (ros::ok())
	{
	  sensor_msgs::JointState js; 
	  
    tf::StampedTransform vr_tf;
    try
    {
      lr.lookupTransform("vr920_base", "vr920", ros::Time(0), vr_tf);
    }
    catch (tf::TransformException ex)
    {
      //ROS_ERROR("%s", ex.what());
      r.sleep();
      continue;
    }
    
    double vr_roll, vr_pitch, vr_yaw;
    btMatrix3x3(vr_tf.getRotation()).getRPY(vr_roll, vr_pitch, vr_yaw);
    
    float out_yaw = processRing(yaw_ring, -vr_yaw);
    float out_pitch = processRing(pitch_ring, vr_pitch);
    
    js.name.push_back("neck_yaw");
    js.position.push_back(out_yaw);
    //js.position.push_back(-vr_yaw);
    js.velocity.push_back(10);          
    js.name.push_back("neck_pitch");
    js.position.push_back(out_pitch);
    //js.position.push_back(-vr_pitch);
    js.velocity.push_back(10);  
	
	  joint_states_pub.publish(js);
	  incrementRing();
		r.sleep();
	}
	
	return 0;
}
	
