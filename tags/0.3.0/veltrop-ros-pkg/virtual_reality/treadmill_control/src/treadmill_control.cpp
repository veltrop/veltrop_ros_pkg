#include <ros/ros.h>
#include <joy/Joy.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <string.h>

namespace treadmill
{

#define KINECT_DIST 2.7    // distance from kinect camera to center of treadmil
//#define DEAD_ZONE   0.25    // forward distance * 2 from center of treadmill to start motion.
#define DEAD_ZONE   0.27                           // sideways discance from center of treadmil to kill it.

class TreadmillController
{
public:
  typedef enum { FORWARD, BACKWARD } TreadmillDirection;

	TreadmillController()
	{
    init_tty();
    joy_sub_ = n_.subscribe <joy::Joy> ("joy", 1, &TreadmillController::joyCB, this);
    robot_move_pub_ = n_.advertise <geometry_msgs::Twist> ("cmd_vel", 1);
    behavior_pub_ = n_.advertise <std_msgs::String> ("motion_name", 1);
  }

  ~TreadmillController()
  {
    stopMotor();
    setDirection(FORWARD);
  }
 
  // returns true on successful init
  bool init_tty() 
  {
    // O_RDWR   : open for read/write
    // O_NOCTTY : do not cause terminal device to be controlling terminal
    // O_NDELAY : program ignores DCD line (else program sleeps until DCD)
    tty_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY); // 
    if (tty_ == -1 )
    {
      ROS_ERROR_STREAM("Cannot open /dev/USB0" << strerror(errno));
      return false;  
    }
    
    fcntl(tty_, F_SETFL,0);	// set file status flag
    
    struct termios tio;
    int result = 0;
    tcgetattr(tty_, &tio);
    
    // set baud 115200
    result = cfsetispeed(&tio, B115200);
    if (result < 0)
    {
      ROS_ERROR_STREAM("cfsetispeed() failed" << strerror(errno));
      return false;
    }
    result = cfsetospeed(&tio, B115200);
    if (result < 0)
    {
      ROS_ERROR_STREAM("cfsetospeed() failed" << strerror(errno));
      return false;
    }  
    
    // control options
    // |= CLOCAL : Local connection, no modem control
    // |= CREAD  : Enable the receiver
    // |= PARENB : Enable parity bit, even parity
    // |= CS8    : 8 data bits  
    tio.c_cflag |= CS8/* | CSTOPB*/ | PARENB | CLOCAL | CREAD;
    // ~PARODD   : Disable odd parity (even parity)
    // ~CSTOPB   : Use 1 stop bit
    // ~CSIZE    : Disable bit mask
    tio.c_cflag &= ~PARODD;
    tio.c_cflag &= ~CSTOPB;
    //tio.c_cflag &= ~CSIZE;		// this prevents it from working?

    // raw output, disable processing
    tio.c_oflag = 0;
    
    // line options
    // non-canonical input, disable all line options
    // tio.c_lflag = 0;
    // ~ICANON : disable canonical mode (special chars EOF, EOL, etc)
    // ~ECHO   : echo off
    // ~ISIG   : disable signal handling (e.g. on INTR, QUIT, etc)  
    tio.c_lflag &= ~(ICANON | ECHO | ISIG);
    
    // inter-character timer
    //tio.c_cc[VTIME]    = 0;  // 1 = 0.1 sec  
    // blocking read until x chars received
    //tio.c_cc[VMIN]     = 0;   
    
    // input options
    // IGNPAR : ignore framing / parity errors
    // IGNBRK : ignore BREAK condition on input
    tio.c_iflag = 0;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_iflag |= (IGNPAR | IGNBRK);   
    
    tcflush(tty_, TCIOFLUSH);
    tcsetattr(tty_, TCSANOW, &tio);  
    tcflush(tty_, TCIOFLUSH);
  
    return true;
  }

  // returns true on successful transmission 
  bool stopMotor()
  {
    char buf[3];
    buf[0] = 1;
    buf[1] = 0;
    buf[2] = 0;
    int w = write(tty_, buf, 3);
    if (w != 3)
    {
      ROS_ERROR("stopMotor(): Fail to write serial port data");
      return false;
    }
    return true;
  }
 
  // returns true on successful transmission 
/*  bool setVelocity(double meter_per_sec)
  {
    if (!setSpeed(meter_per_sec))
      return false;
    
    // seems to work more smoothly if we set direction after speed
    return setDirection(meter_per_sec >= 0.0 ? FORWARD : BACKWARD);
  }*/
  
  bool setVelocity(double meter_per_sec)
  {
    if (fabs(meter_per_sec) > 0.01)
      if (!setDirection(meter_per_sec >= 0.0 ? FORWARD : BACKWARD))
        return false;
    
    return setSpeed(meter_per_sec);
  }  
  
  // returns true on successful transmission 
  bool setSpeed(double meter_per_sec)
  {
    // speed in meter per second
    // pwm in miliseconds
    
    // speed pwm
    // 0.5   0.50
    // 1.0   0.75
    // 1.5   1.00
    //  .     .
    // 5.0   2.75 (factory maximum)
    //  .     .
    // 12.6  6.55 (treadmill_control maximum)
    
    // speed = pwm * 2.0 - 0.5
    // pwm = speed * 0.5 + 0.25

    // serial port data protocol is pwm * 10000 (for precision)
  
    double pwm_scaled_double = meter_per_sec * 5000.0 + 2500.0;
    unsigned short pwm_scaled = floor(fabs(pwm_scaled_double));

    //ROS_INFO_STREAM(meter_per_sec << " " << pwm_scaled_double << " " << pwm_scaled);
    
    // threshold seems like a bad workaround for the treadmills odd behavior. 
    // if we send a zero velocity signal the treadmil will suddenly stop.
    // but if we send a very low velocity, the treadmill will ramp down to this speed.
    // at this very lwo velocity, the treadmil doesnt have enough power to
    //  move me, so in effect it stops the treadmil.
    // however, sometimes the treadmil stops responding to commands and enters
    // some sort of safety mode.  Sending a true 0 speed can get out of this saftey mode
    // and so does restarting the treadmill.
    //int THRESHHOLD = 3000;
    int THRESHHOLD = 4000;
    //int THRESHHOLD = 5000;
    if (pwm_scaled < THRESHHOLD)
      pwm_scaled = THRESHHOLD;

    char buf[3];
    buf[0] = 1;
    buf[1] = pwm_scaled >> 8;
    buf[2] = pwm_scaled & 0x00FF;
    int w = write(tty_, buf, 3);
    if (w != 3)
    {
      ROS_ERROR("setSpeed(): Fail to write serial port data");
      return false;
    }
    return true;
  }

  // returns true on successful transmission 
  bool setDirection(TreadmillDirection dir)
  {
    char buf[2];
    buf[0] = 2;
    if (dir == BACKWARD)
      buf[1] = 1;
    else // go forward as fallback
      buf[1] = 2;
      
    int w = write(tty_, buf, 2);
    if (w != 2)
    {
      ROS_ERROR("setDirection(): Fail to write serial port data");
      return false;
    }
    return true;
  }

  void spin()
  {
    //ros::Rate rate(30);
    //ros::Rate rate(20);
    ros::Rate rate(10);
    
    ros::Time time_prev = ros::Time::now();
    //ros::Time prev_stand_sit = ros::Time::now();
    bool robotStanding = false;
    double prev_treadmill_velocity = 0.0;
    while (n_.ok())
    {
      ros::spinOnce();

      tf::StampedTransform torso_tf;
      try
      {
        tf_listener_.lookupTransform("/openni_depth_frame", "/torso", 
                                     ros::Time(0), torso_tf);
      }
      catch (tf::TransformException ex)
      {
        //ROS_ERROR("%s", ex.what());
        rate.sleep();
        continue;
      }

      //ROS_INFO_STREAM(torso_tf.getOrigin().z());
      
      double torso_dist = torso_tf.getOrigin().z();
 /*     
      ///////////////////////////////////////////////////////////////////////
      //  The following is a first derivative position based method
      double torso_dist_prev = 0;
      double torso_delta = torso_dist - torso_dist_prev;
      
      ros::Time time_now = ros::Time::now(); 
      ros::Duration time_delta = time_now - time_prev;
      
      double meters_per_sec = -torso_delta / time_delta.toSec();
      torso_dist_prev = torso_dist;
      time_prev = time_now;
      
      setVelocity(meters_per_sec * 10.0);
  */  

      ///////////////////////////////////////////////////////////////////////
      //  The following is a purely position based method

      double treadmill_velocity = -torso_dist + KINECT_DIST; // offset from goal
      
      // linear speed factor until 0.3 cm displacement, then curved speed factor.
      treadmill_velocity /= 0.4;
      if (treadmill_velocity > 1.0)
        treadmill_velocity *= treadmill_velocity;
      else if (treadmill_velocity < -1.0)
        treadmill_velocity *= -treadmill_velocity;
      treadmill_velocity *= 7.5; // baseline meters per second
      
      if (prev_treadmill_velocity < 0.0 && treadmill_velocity > 0.0)
        treadmill_velocity = 0.01;
      else if (prev_treadmill_velocity > 0.0 && treadmill_velocity < 0.0)
        treadmill_velocity = -0.01;
      //prev_treadmill_velocity = treadmill_velocity;

      double torso_alignment = fabs(torso_tf.getOrigin().x());
      double torso_height = torso_tf.getOrigin().y();
      double torso_home_dist = fabs(torso_dist - KINECT_DIST);
            
      // handle controlling the treadmill   
      bool robotShouldntWalk = true;  
      bool humanStanding = torso_height >= 0.0;
      bool withinSafeZone = torso_alignment <= DEAD_ZONE && torso_home_dist <= 0.65;
      if (!withinSafeZone || !humanStanding)
      {
        stopMotor(); 
        robotShouldntWalk = true;
      }
      else if (torso_home_dist >= DEAD_ZONE * 0.5)
      {
        setVelocity(treadmill_velocity);
        robotShouldntWalk = false;
      }
      else if (prev_treadmill_velocity < 0.0)
      {
        setVelocity(-0.01);
        robotShouldntWalk = true;  
      }
      else
      {
        setVelocity(0.01); 
        robotShouldntWalk = true;
      }    
      
      prev_treadmill_velocity = treadmill_velocity;
        
      // next extract rotation of human for robot
      double roll, pitch, yaw;
      btMatrix3x3(torso_tf.getRotation()).getRPY(roll, pitch, yaw);
 
      // prepare robot movement data type
      geometry_msgs::Twist motion;
      motion.linear.x = motion.linear.y = motion.angular.z = 0.0;
       
      // handle controlling the robot       
      // go straight   
      if (robotStanding && !robotShouldntWalk && withinSafeZone)
      {  
        motion.linear.x = treadmill_velocity * 0.1;  // treat my going 10m/s as NAO's 1.0
        motion.linear.x = std::max(std::min(motion.linear.x, 1.0), -1.0);
        robot_move_pub_.publish(motion); 
      }
      // side step
      else if (robotStanding && robotShouldntWalk && withinSafeZone && torso_alignment >= 0.11)
      {
        motion.linear.y = -torso_tf.getOrigin().x() * 5.0;
        motion.linear.y = std::max(std::min(motion.linear.y, 1.0), -1.0);
        robot_move_pub_.publish(motion); 
      }
      // rotate
      else if (robotStanding && robotShouldntWalk && withinSafeZone && fabs(pitch) >= 0.42)
      {
        //motion.angular.z = pitch / 1.57;
        motion.angular.z = pitch / 1.85;
        motion.angular.z = std::max(std::min(motion.angular.z, 1.0), -1.0);
        robot_move_pub_.publish(motion); 
      }
      // stop
      else if (robotShouldntWalk || !withinSafeZone)
      {
        motion.linear.x = motion.linear.y = motion.angular.z = 0.0;
        robot_move_pub_.publish(motion); 
      }
      
      //ros::Duration stand_sit_delta = ros::Time::now() - prev_stand_sit;
      if (withinSafeZone && humanStanding && !robotStanding /*&& stand_sit_delta.toSec() > 5.0*/)
      {
        (treadmill_velocity < 0.0) ? setVelocity(-0.01) : setVelocity(0.01); 
      
        motion.linear.x = motion.linear.y = motion.angular.z = 0.0;
        robot_move_pub_.publish(motion); 
        usleep(200000);
        
        std_msgs::String behavior;
        behavior.data = "get_up_and_ready";
        behavior_pub_.publish(behavior);
        usleep(18000000);
        
        robotStanding = true;
        //prev_stand_sit = ros::Time::now();
      }
      else if (withinSafeZone && !humanStanding && robotStanding /*&& stand_sit_delta.toSec() > 5.0*/)
      {
        (treadmill_velocity < 0.0) ? setVelocity(-0.01) : setVelocity(0.01); 
        
        motion.linear.x = motion.linear.y = motion.angular.z = 0.0;
        robot_move_pub_.publish(motion); 
        usleep(200000);
        
        std_msgs::String behavior;
        behavior.data = "sit_and_relax";
        behavior_pub_.publish(behavior);
        usleep(15000000);
        
        robotStanding = false;
        //prev_stand_sit = ros::Time::now();
      }
           
      rate.sleep();
    }
  }

private:
	ros::NodeHandle n_;
  ros::Subscriber joy_sub_;
  tf::TransformListener tf_listener_;
  int tty_;
  ros::Publisher robot_move_pub_, behavior_pub_;

  void joyCB(const joy::Joy::ConstPtr& joy)
  {
    //joy->buttons[]    
    // range of axes is -10~10
    // want output range -5~5m/s
    double mps = joy->axes[0] * 0.5;
    setVelocity(mps);
  }
};

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "treadmill_control");
	treadmill::TreadmillController this_node;

	this_node.spin();
  
  /////////////////////////////////////////////////////////////////
  // a most basic test, send a speed of 500ms width pwm.
  //char buf[3];
  //buf[0] = 1;
  //unsigned short pwm = 5000;
  //buf[1] = pwm >> 8;
  //buf[2] = pwm & 0xFF;
	//int w = write(tty, buf, 3);

  return 0;
}

