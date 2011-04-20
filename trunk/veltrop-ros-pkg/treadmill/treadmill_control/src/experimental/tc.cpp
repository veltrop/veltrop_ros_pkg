// g++ -o com com.cpp -I.

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <string.h>

int tty;

void init_tty() 
{
  /*
  // settings that i figured out with trial and error
  com4 = open("/dev/ttyS3", O_RDWR | O_NOCTTY); // O_NDELAY
  
  if (com4 == -1 )
    perror("Unable to open port");
  else
    fcntl(com4, F_SETFL,0);
  
  struct termios tio;
  tcgetattr(com4,&tio);
  // set 115200 baud, 8 data bits, 1 stop bit, even parity, no modem control, enable receiving
  tio.c_cflag = B115200 | CS8 | CSTOPB | PARENB | CLOCAL | CREAD;
  /// ignore bytes with parity errors
  tio.c_iflag = IGNPAR;
  // raw output
  tio.c_oflag = 0;
  // non-canonical input
  tio.c_lflag = 0;
  // inter-character timer
  tio.c_cc[VTIME]    = 0;  // 1 = 0.1 sec  
  // blocking read until x chars received
  tio.c_cc[VMIN]     = 0;   
  
  tcflush(com4, TCIFLUSH);
  tcsetattr(com4,TCSANOW,&tio);
  */
  
  
  // combined efforts settings
	// O_RDWR   : open for read/write
	// O_NOCTTY : do not cause terminal device to be controlling terminal
	// O_NDELAY : program ignores DCD line (else program sleeps until DCD)
  tty = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY); // 
  if (tty == -1 )
  {
    fprintf(stderr, "Cannot open /dev/USB0 - %s\n", strerror(errno));
  	return;  
  }
  
  fcntl(tty, F_SETFL,0);	// set file status flag
  
  struct termios tio;
  int result = 0;
  tcgetattr(tty, &tio);
  
	// set baud 115200
	result = cfsetispeed(&tio, B115200);
	if (result < 0) {
		printf("cfsetispeed() failed - %s\n", strerror(errno));
		return;
	}
	result = cfsetospeed(&tio, B115200);
	if (result < 0) {
		printf("cfsetospeed() failed - %s\n", strerror(errno));
		return;
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
  //tio.c_cflag &= ~CSIZE;		// this prevents shit from working

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
  
  //tcflush(com4, TCIFLUSH);
  tcflush(tty, TCIOFLUSH);
  tcsetattr(tty, TCSANOW, &tio);  
  tcflush(tty, TCIOFLUSH);
}

int main(int argc, char** argv)
{
  init_tty();

  char buf[3];
  buf[0] = 1;
  unsigned short pwm = 5000;
  buf[1] = pwm >> 8;
  buf[2] = pwm & 0xFF;
	int w = write(tty, buf, 3);

  return 0;
}
