#include <ros/ros.h>
#include <joy/Joy.h>
#include <tf/transform_listener.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <string.h>

namespace treadmill
{

class TreadmillController
{
public:
  typedef enum { FORWARD, BACKWARD } TreadmillDirection;

	TreadmillController()
	{
    init_tty();
    joy_sub_ = n_.subscribe <joy::Joy> ("joy", 1, &TreadmillController::joyCB, this);
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
  bool setVelocity(double meter_per_sec)
  {
    if (!setSpeed(meter_per_sec))
      return false;
    
    return setDirection(meter_per_sec > 0 ? FORWARD : BACKWARD);
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

    // serial port data is pwm * 10000
  
    double pwm_scaled_double = meter_per_sec * 5000.0 + 2500.0;
    unsigned short pwm_scaled = floor(fabs(pwm_scaled_double));

    //ROS_INFO_STREAM(meter_per_sec << " " << pwm_scaled_double << " " <<
    //                pwm_scaled);
    
    // threshold seems like a bad workaround for the treadmills odd behavior 
    if (pwm_scaled < 3000)
      pwm_scaled = 3000;

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
    //ros::spin();

    // TODO: fork into new node
    ros::Rate rate(30);
    double torso_dist_prev = 0;
    ros::Time time_prev = ros::Time::now();
    while (n_.ok())
    {
      ros::spinOnce();

      tf::StampedTransform torso_tf;
      try
      {
        tf_listener_.lookupTransform("/openni_depth", "/torso", 
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
      double torso_delta = torso_dist - torso_dist_prev;
      
      ros::Time time_now = ros::Time::now(); 
      ros::Duration time_delta = time_now - time_prev;
      
      double meters_per_sec = -torso_delta / time_delta.toSec();
      torso_dist_prev = torso_dist;
      time_prev = time_now;
      
      //setVelocity(meters_per_sec * 10.0);
    
      // goal position will be 2.71 for destination based.

      double treadmill_velocity = -torso_dist + 2.7; // offset from goal
      
      // make it non linear
      // This attempt at non-linear smooth for stopping transitions
      // but walking at desired speed is not quite intuitive, need to alter
      // the curve
      if (treadmill_velocity < 0.0)
      {
        treadmill_velocity *= treadmill_velocity;
        treadmill_velocity = -treadmill_velocity;
      }
      else
      {
        treadmill_velocity *= treadmill_velocity;
      }
      // 0.6, 0.5, 0.4 good for linear, 0.01, 0.005 good for non-linear
      treadmill_velocity *= (1.0 / 0.005);

      // TODO: complete the active zone
      double torso_alignment = fabs(torso_tf.getOrigin().x());
      double torso_height = fabs(torso_tf.getOrigin().y());
      if (torso_alignment < 0.5/* && torso_height > 0.85*/)
        setVelocity(treadmill_velocity);
      else
        //setVelocity(0.0);
        stopMotor();

      rate.sleep();
    }
  }

private:
	ros::NodeHandle n_;
  // TODO: fork into new node
  ros::Subscriber joy_sub_;
  // TODO: fork into new node
  tf::TransformListener tf_listener_;
  TreadmillDirection dir_;
  int tty_;

	// TODO: fork into new node
  void joyCB(const joy::Joy::ConstPtr& joy)
  {
    //joy->buttons[] 
    
    // range of axes is -10~10
    // want output range -5~5mps
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
  
  // a most basic test, send a speed of 500ms width pwm.
  //char buf[3];
  //buf[0] = 1;
  //unsigned short pwm = 5000;
  //buf[1] = pwm >> 8;
  //buf[2] = pwm & 0xFF;
	//int w = write(tty, buf, 3);

  return 0;
}

