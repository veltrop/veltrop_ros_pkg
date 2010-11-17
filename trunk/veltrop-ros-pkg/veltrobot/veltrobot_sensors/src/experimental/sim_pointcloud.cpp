#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_pointcloud");
  ros::NodeHandle n_;
  //ros::NodeHandle n_private("~"); 
  
  ros::Publisher points_pub_ = n_.advertise<sensor_msgs::PointCloud>("/sim_points", 1);

  sensor_msgs::PointCloud points_msg_;
  
  points_msg_.header.frame_id = "/stereo_camera";
  
  float xorig = -2.0f;
  float yorig = -2.0f;
  //float zorig = -10.0f;
  //float zplane = 1.0f;
  float width = 4.0f;
  float height = 4.0f;
  //float depth = 20.0f;
  float stepping = 0.10f;
  
  //size_t num_points=(width / stepping) * (height / stepping);
  size_t num_points=(width / stepping) * (height / stepping) * 2;
  //size_t num_points=(width / stepping) * (height / stepping) * (depth / stepping);
  points_msg_.points.resize(num_points);
  size_t i=0;
  for (float x=xorig; x < width + xorig; x += stepping)
  {
    for (float y=yorig; y < height + yorig; y += stepping)
    {
      //for (float z=zorig; z < depth + zorig; z += stepping) 
      float z=-x;
      {
        if (i >= num_points)
          break;
        points_msg_.points[i].x = x;
        points_msg_.points[i].y = y;
        points_msg_.points[i].z = z;
        i++;
      }
      z=x;
      {
        if (i >= num_points)
          break;
        points_msg_.points[i].x = x;
        points_msg_.points[i].y = y;
        points_msg_.points[i].z = z;
        i++;
      }
      
      //std::cout << x <<" "<< y <<" " << z << std::endl;
    }
  }
  
  ros::Rate loop_rate_(1.0f);
  int seq_ = 0;
  while (n_.ok())
  {  
    points_msg_.header.seq = seq_++;
    points_msg_.header.stamp = ros::Time::now();
    
    points_pub_.publish(points_msg_);    
    loop_rate_.sleep();
  }
  
  return 0;
}
