#ifndef _POLL_AD_H_
#define _POLL_AD_H_

#include <vector>
#include <ros/ros.h>
#include "ad_device.h"

namespace roboard_sensors
{

class PollAD
{
	public:
  	PollAD();
    ~PollAD();
    
    void spin();
  
	private:
    std_msgs::Int16MultiArray 	raw_msg_;
		std_msgs::MultiArrayDimension raw_msg_dim_;
  	std::vector<ADDevice*> 			devices_;
  	unsigned char               used_channels_;
    ros::Publisher   						raw_pub_;
    bool												publish_raw_;
    float												freq_;
  
  	void loadDevices();
};

} // namespace roboard_sensors

#endif
