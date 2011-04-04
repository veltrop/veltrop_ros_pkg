#include <ros/ros.h>
#include <std_msgs/String.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <sstream>

// TODO: these globals are bad, interfere with the class design cleanliness
int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

namespace veltrobot_teleop
{

enum
{
  KEYCODE_RT =0x43, 
  KEYCODE_LT =0x44,
  KEYCODE_UP =0x41,
  KEYCODE_DN =0x42,
  KEYCODE_Q  =0x71,
  KEYCODE_I  =0x69,
  KEYCODE_SP =0x20
};

class VeltrobotTeleopKB
{
public:
	VeltrobotTeleopKB()
	{
	  motion_pub_ = n_.advertise <std_msgs::String> ("motion_name", 10);
	}
	
	void keyLoop()
	{
		char c;

	  // get the console in raw mode                                                              
	  tcgetattr(kfd, &cooked);
	  memcpy(&raw, &cooked, sizeof(struct termios));
	  raw.c_lflag &=~ (ICANON | ECHO);
	  // Setting a new line, then end of file                         
	  raw.c_cc[VEOL] = 1;
	  raw.c_cc[VEOF] = 2;
	  tcsetattr(kfd, TCSANOW, &raw);

	  puts("Keyboard Commands:");
    puts(" I     - Init");
	  puts(" Q     - Quit");
	  puts(" Up    - Forward");
	  puts(" Down  - Backward");
	  puts(" Left  - Rotate Left");
	  puts(" Right - Rotate Right");
    puts(" Space - Stand Squat");

	  for(;;)
	  {
		  usleep(100000); // 10hz

      // get the next event from the keyboard  
		  if(read(kfd, &c, 1) < 0)
		  {
			  perror("read():");
			  exit(-1);
		  }

		  std_msgs::String mot;
	    mot.data != "";

		  printf("%X\n", c);

		  switch (c)
		  {
			  case KEYCODE_Q:
				  quit(0);
				  break;		
			  case KEYCODE_LT:
				  mot.data = "rotate_left";
				  break;
			  case KEYCODE_RT:
				  mot.data = "rotate_right";
				  break;
			  case KEYCODE_UP:
				  mot.data = "walk_forward";
				  break;
			  case KEYCODE_DN:
				  mot.data = "walk_backward";
				  break;
			  case KEYCODE_I:
				  mot.data = "init";
				  break;			
        case KEYCODE_SP:
				  mot.data = "stand_squat";
				  break;	
		  }

		  if (mot.data != "") 	
			  motion_pub_.publish(mot);    
	  }
	}
	
private:
	ros::NodeHandle n_;
	ros::Publisher motion_pub_;
};

} // namespace veltrobot_teleop

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_kb");
	veltrobot_teleop::VeltrobotTeleopKB this_node;

	signal(SIGINT,quit);

	this_node.keyLoop();
  
	return(0);
}

