#ifndef _UART_COMPAT
#define _UART_COMPAT

// Code here contains contributions from Kondo's KCB library and Chris Vo's libondo4

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <string.h>

int com4;


void com4_ics_init()
{
  /*
  // settings that i figured out a while back with trial and error
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
  com4 = open("/dev/ttyS3", O_RDWR | O_NOCTTY | O_NDELAY); // 
  if (com4 == -1 )
  {
    fprintf(stderr, "uart.h: Cannot open /dev/ttyS3 - %s\n", strerror(errno));
  	return;  
  }
  
  fcntl(com4, F_SETFL,0);	// set file status flag
  
  struct termios tio;
  int result = 0;
  tcgetattr(com4, &tio);
  
	// set baud 115200
	result = cfsetispeed(&tio, B115200);
	if (result < 0) {
		printf("uart.h: cfsetispeed() failed - %s\n", strerror(errno));
		return;
	}
	result = cfsetospeed(&tio, B115200);
	if (result < 0) {
		printf("uart.h: cfsetospeed() failed - %s\n", strerror(errno));
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
  tcflush(com4, TCIOFLUSH);
  tcsetattr(com4, TCSANOW, &tio);  
  tcflush(com4, TCIOFLUSH);
  
}

void com4_ics_close()
{
	close(com4);
}

// data transmission / reception
/*
void com4_trx(char *tx, size_t txsize, char *rx, size_t rxsize)
{
  write(com4, tx, txsize);
  
  
  if (rxsize)
  {
		// set up fd
		fd_set set;
		FD_ZERO(&set);
		FD_SET(com4,&set);

		// set up timeout
		struct timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = 500;

    // block for at most usecs for a byte to come in
    int r = select(FD_SETSIZE, &set, NULL, NULL, &timeout);
    if (r <= 0)
      return;

    // now, finally, do the read.
    read(com4, rx, rxsize);  
  }
  
  //read(com4, rx, rxsize);
  //usleep(500000);
  //if (rxsize)
  //{
  //  read(com4, rx, 1);
  //  printf("%X \n", rx[0]);
  //}
  
  //int i;
  //for (i=0; i < rxsize; i++)
  //  printf("%X ", rx[i]);
  //printf("\n");
}

void uart0_trx (BYTE *tx, size_t txsize, BYTE *rx, size_t rxsize)
{
  com4_trx(tx, txsize, rx, rxsize);
}

void uart2_trx (BYTE *tx, size_t txsize, BYTE *rx, size_t rxsize)
{
  com4_trx(tx, txsize, rx, rxsize);
}
*/

int kondo_serial_write(unsigned char * buf, int n)
{
	return (int) write(com4, buf, n);
}

int kondo_serial_read_timeout(unsigned char * buf, unsigned int n,
		unsigned long secs, unsigned long usecs)
{
	// set up fd
	fd_set set;
	FD_ZERO(&set);
	FD_SET(com4,&set);

	// set up timeout
	struct timeval timeout;
	timeout.tv_sec = secs;
	timeout.tv_usec = usecs;

	// block for at most usecs for a byte to come in
	int r = select(FD_SETSIZE, &set, NULL, NULL, &timeout);
	if (r <= 0)
		return r;

	// now, finally, do the read.
	return (int) read(com4, buf, n);
}

int ics_trx_timeout(unsigned char * buf_out, unsigned int bytes_out, unsigned char * buf_in, unsigned int bytes_in,
										long timeout)
{
	int i;
  
  //tcflush(com4, TCIOFLUSH);
  
	if ((i = kondo_serial_write(buf_out, bytes_out)) < 0)
		return i;
    
  //tcflush(com4, TCIOFLUSH);

	//i = kondo_serial_read_timeout(buf_in, bytes_in, 0, timeout);

	//tcflush(com4, TCIOFLUSH);

	return i;
}

#define ICS_CMD_ID     0xE0
#define ICS_ID_TIMEOUT 900
#define ICS_SC_READ 0

int com4_ics_get_id()
{
	int i;

	unsigned char in_buf[1] = {0};
	unsigned char out_buf[4];
	// build command
	out_buf[0] = ICS_CMD_ID; // 0xFF; // command (0xFF for read)
	out_buf[1] = ICS_SC_READ; // subcommand (read)
	out_buf[2] = ICS_SC_READ; // subcommand (read)
	out_buf[3] = ICS_SC_READ; // subcommand (read)

	if ((i = ics_trx_timeout(out_buf, 4, in_buf, 1, ICS_ID_TIMEOUT)) < 0)
		return i;

	// return the ID
	return in_buf[0] & 0x1F;
}


#define ICS_CMD_POS 0x80
#define ICS_POS_TIMEOUT 300

int com4_ics_pos(unsigned int id, unsigned int pos)
{
	int i;

	unsigned char out_buf[3];
  unsigned char in_buf[3] = {0, 0, 0};
	// build command
	out_buf[0] = id | ICS_CMD_POS;  // id and command
	out_buf[1] = (pos >> 7) & 0x7F; // high 7 bits of pos
	out_buf[2] = pos & 0x7F;        // low 7 bits of pos

	if ((i = ics_trx_timeout(out_buf, 3, in_buf, 3, ICS_POS_TIMEOUT)) < 0)
		return i;

	// return the position
	return ((in_buf[1] & 0x7F) << 7) | (in_buf[2] & 0x7F);
}

#endif
