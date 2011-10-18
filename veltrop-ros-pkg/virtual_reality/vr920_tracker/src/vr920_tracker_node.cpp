// This is based on multicastdemoclient.cpp from the vrtrack package
// Modified to include ROS & TF support

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <kdl/frames.hpp>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

struct TrackingData {
	double viewmatrix[16];
	double yaw;
	double pitch;
	double roll;
};

TrackingData trackingdata;

struct ip_mreq command;
int socket_descriptor;
socklen_t sin_len;
struct sockaddr_in sock_in;

char multicastAddress[] = "224.0.0.42";
int port = 4242;

bool connected = false;

int createMulticastSocket()
{
	int loop = 1;

	memset(&sock_in, 0, sizeof(sock_in));
	sock_in.sin_family = AF_INET;
	sock_in.sin_addr.s_addr = htonl(INADDR_ANY);
	sock_in.sin_port = htons(port);
	if ((socket_descriptor = socket(PF_INET, SOCK_DGRAM, 0)) == -1)
	{
		perror("socket()");
		exit(EXIT_FAILURE);
	}

	/* Allow multiple processes to use the same port */
	loop = 1;
	if (setsockopt(socket_descriptor, SOL_SOCKET, SO_REUSEADDR, &loop,
			sizeof(loop)) < 0)
	{
		perror("setsockopt:SO_REUSEADDR");
		exit(EXIT_FAILURE);
	}

	if (bind(socket_descriptor, (struct sockaddr *) &sock_in, sizeof(sock_in)) < 0)
	{
		perror("bind");
		exit(EXIT_FAILURE);
	}
	/* allow multicast */
	loop = 1;
	if (setsockopt(socket_descriptor, IPPROTO_IP, IP_MULTICAST_LOOP, &loop,
			sizeof(loop)) < 0)
	{
		printf("setsockopt:IP_MULTICAST_LOOP");
		exit(EXIT_FAILURE);
	}
	/* join the multicast group: */
	command.imr_multiaddr.s_addr = inet_addr(multicastAddress);
	command.imr_interface.s_addr = htonl(INADDR_ANY);
	if (command.imr_multiaddr.s_addr == -1)
	{
		printf("%s is no multicast address\n", multicastAddress);
		exit(EXIT_FAILURE);
	}
	if (setsockopt(socket_descriptor, IPPROTO_IP, IP_ADD_MEMBERSHIP, &command,
			sizeof(command)) < 0)
	{
		printf("setsockopt:IP_ADD_MEMBERSHIP");
	}

	fcntl(socket_descriptor, F_SETFL, O_NONBLOCK);

	return socket_descriptor;
}

static float toRad(float deg)
{
  return deg * 0.0174532925;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "vr920_tracker_node");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	tf::TransformListener    lr;
	tf::TransformBroadcaster br;
	KDL::Rotation calib_rotation;
	bool got_calib_rotation = false;

  // TODO: take these arguments from the parameter server
	//multicastAddress
	//port

	while (ros::ok())
	{
		if (!connected)
		{
			if (createMulticastSocket() == -1)
			{
				std::cerr << "Could not connect to daemon" << std::endl;
				break;
			}
			connected = true;
		}

		if (recvfrom(socket_descriptor, &trackingdata, sizeof(TrackingData), 0,
				(struct sockaddr *) &sock_in, &sin_len) != -1)
		{

//			KDL::Rotation rotation(trackingdata.viewmatrix[0], trackingdata.viewmatrix[1], trackingdata.viewmatrix[2],
//							               trackingdata.viewmatrix[4], trackingdata.viewmatrix[5], trackingdata.viewmatrix[6],
//							               trackingdata.viewmatrix[8], trackingdata.viewmatrix[9], trackingdata.viewmatrix[10]);

      KDL::Rotation rotation = KDL::Rotation::RPY(toRad(trackingdata.roll), toRad(trackingdata.pitch), toRad(trackingdata.yaw));	    
	    
	    if (!got_calib_rotation)
	    {
	      calib_rotation = rotation.Inverse();
  			got_calib_rotation = true;
	    }
	    else
	    {
	      rotation = rotation * calib_rotation;
	      
        tf::StampedTransform head_tf;
        try
        {
          lr.lookupTransform("world", "vr920_base", ros::Time(0), head_tf);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("%s", ex.what());
          // continue;
        }
	      
	      double qx, qy, qz, qw;
	      rotation.GetQuaternion(qx, qy, qz, qw);
	
			  tf::Transform transform;
	      transform.setOrigin(head_tf.getOrigin());
	      transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
			
			  // send a transform from the users "head" to the vr920
			  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "vr920"));
			}
		}

		usleep(10000); // 100 hz
	}

	//cleanup socket
	if (setsockopt(socket_descriptor, IPPROTO_IP, IP_DROP_MEMBERSHIP, &command,
			sizeof(command)) < 0) {
		perror("setsockopt:IP_DROP_MEMBERSHIP");
	}
	close(socket_descriptor);
}

