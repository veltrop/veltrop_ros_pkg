//#define UART1_ENABLE_INT
//#include <uart.h>
#include <led.h>
#include <pwm.h>
#include <pio.h>

void init_treadmill()
{
	pwm16_init(PIO0, PWM_F2);  // 10MHz
	pwm_start(PIO0);
	pwm16_out(PIO0, 0);
	pio7_out(LOW);	
}

//typedef enum { FWD, REV } motor_direction_t;
//motor_direction_t dir = FWD;

//void run_test_procedure()
//{
//	unsigned int i=0;
//	
//	while (1)
//	{		
//		// 0 = 0ms width
//		// 65000 = 6.5ms width
//		// want 3ms limit for now
//		// need 0.2ms minimum to get motor going, 0.1 kills it.  What's the cutoff?
//		unsigned int top = 20000;
//		//unsigned int bottom = 2000; 
//		unsigned int bottom = 5000;
//		for (i = bottom; i <= top; /*i++*/ i += 2)
//		{
//			pwm16_out(PIO0, i);
//			wait(500); 
//		}
//
//		for (i = top; i >= bottom; i-=100) // be careful that bottom > increment (unsigned)
//		{		
//			pwm16_out(PIO0, i);
//		}
//				
//		wait(250000);	// 1/4 sec
//				
//		pwm16_out(PIO0, 0);			// can do zero if soon reactivate it below with 5000
//		
//		wait(5000);
//		
//		// flip motor direction
//		if (dir == REV)
//		{
//			dir = FWD;
//			pio7_out(LOW); // (0V)
//		}
//		else
//		{
//			dir = REV;
//			pio7_out(HIGH); // (5V)	
//		}
//		
//		wait(5000);
//		
//		//pwm16_out(PIO0, 5000);
//		
//		ledred_switch();
//	}	
//}

void init_com()
{
	//uart1_asyncmode(BR115200, 8, 1, PARITY_EVEN, FALSE);
	//uart1_send_start ();
	//uart1_recv_start ();
	//uart1_enable_echoback = TRUE;	
	
	ckdir_u1mr = 0;
	smd0_u1mr = 1;
	smd1_u1mr = 0;
	smd2_u1mr = 1;
	stps_u1mr = 0;
	prye_u1mr = 1;
	pry_u1mr = 1;
	u1irs = 1;
	clk0_u1c0 = 0;
	clk1_u1c0 = 0;
 	u1brg = 10;
	ckpol_u1c0 = 0;
	uform_u1c0 = 0;
	nch_u1c0 = 1; 
	u1rrm = 0;
	te_u1c1 = 1; 
	re_u1c1 = 1;
	s1ric	= (s1ric & 0xF8) | (0x01 & 0x07);
}

#define UART1_INT_FIFO_MAX_SIZE 128
char u1buf;
char u1fifo[UART1_INT_FIFO_MAX_SIZE];	// circular buffer
// with char types here, remember not to set the max size above 256...
char u1fifo_write_pos = 0;
char u1fifo_read_pos = 0;
char u1fifo_size = 0;	// maintain size to monitor fullness				
// (alternative to u1fifo_size is compare if write pos == read pos, but that logic may lead to bugs)

_Bool u1fifo_get(char* get)
{
	if (!u1fifo_size)
		return FALSE;

	*get = u1fifo[u1fifo_read_pos];

	u1fifo_size--;
	// increment buffer current position with circular wrap around
	u1fifo_read_pos = ++u1fifo_read_pos % UART1_INT_FIFO_MAX_SIZE;	

	return TRUE;
}

void u1fifo_put(char c)
{
	u1fifo[u1fifo_write_pos] = c;
	// if the size overflows, we go back to zero size and loose whole buffer...
	u1fifo_size = ++u1fifo_size % UART1_INT_FIFO_MAX_SIZE;				
	u1fifo_write_pos = ++u1fifo_write_pos % UART1_INT_FIFO_MAX_SIZE;

	// TODO: if size > max, set OVERRUN flag?
}

void u1_putchar(char data)
{
	u1tb = data;

	while (ti_u1c1 == 0) {
//		if (++j == 300) {    
//			break; 
//		}
	}
}

#pragma INTERRUPT u1_receive_interrupt(vect=20)
void u1_receive_interrupt() 
{
	ledgrn_off();
	
	u1buf = u1rbl;
	if (sum_u1rb)
	{ 
		re_u1c1 = 0;
		re_u1c1 = 1; 
	}
	
	u1fifo_put(u1buf);
		
	ledgrn_on();
}

typedef enum { COM_IDLE=0, COM_PWM=1, COM_DIR=2 } com_mode_t;
com_mode_t com_mode = COM_IDLE;
unsigned int byte_number = 0;
unsigned int motor_speed = 0;
void run_com_controll()
{
	while(1)
	{
		char com_input;
		if (u1fifo_get(&com_input))
		{
			u1_putchar('0');
			//u1_putchar(com_input);
			
			if (com_mode == COM_IDLE)
			{
				if (com_input == COM_PWM || com_input == COM_DIR)
				{
					com_mode = com_input; 
					byte_number++;
					u1_putchar('a');
				}
			}
			else if (com_mode == COM_PWM)
			{
				if (byte_number == 1)
				{
					motor_speed = 0;
					motor_speed = com_input;
					motor_speed <<= 8;
					motor_speed &= 0xFF00;
					byte_number++;
					u1_putchar('b');
				}
				else if (byte_number == 2)
				{
					motor_speed |= (com_input & 0x00FF);
					pwm16_out(PIO0, motor_speed);
					com_mode = COM_IDLE;
					byte_number = 0;
					u1_putchar('c');
				}
				else
				{
					com_mode = COM_IDLE;
					byte_number = 0;
					u1_putchar('d');	
				}
			}
			else if (com_mode == COM_DIR)
			{
				if (com_input == 1)
				{
					pio7_out(HIGH);
					ledred_on();
					u1_putchar('f');
				}
				else
				{
					pio7_out(LOW);
					ledred_off();
					u1_putchar('e');
				}
				com_mode = COM_IDLE;
				byte_number = 0;
			}
			else
			{
				com_mode = COM_IDLE;
				byte_number = 0;
				u1_putchar('g');	
			}
		}
	}
}

void main(void)
{		
	cpu_init(); 
	cpu_int_set();
	init_com();
	init_treadmill();

	wait(500000);
	
	ledgrn_on();
	
	//run_test_procedure();
	run_com_controll();
	
	ledgrn_off();
}
