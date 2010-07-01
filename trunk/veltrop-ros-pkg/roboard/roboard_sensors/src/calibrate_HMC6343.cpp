#include <iostream>
#include <roboard.h>

int main(int argc, char** argv)
{  
  if (!i2c_Initialize(I2CIRQ_DISABLE))
  { 
    std::cout << "Failed to Initialize i2c." << std::endl;
    return -1;
  } 

  i2c0_SetSpeed(I2CMODE_STANDARD, 100000); 
  
  std::cout << "Calibrate? (y/n)" << std::endl;
  char a;
  std::cin >> a;
      
  if (a != 'y')
    return 0;
  
  usleep(500000); // 500ms
  
  i2c0master_StartN(0x32>>1, I2C_WRITE, 1); 
  i2c0master_WriteN(0x71);  // enter calib mode
  usleep(1000); // 1 ms

  std::cout << "Type y and enter when finished to save calibration" << std::endl;
  std::cin >> a;
  
  if (a == 'y')
  {
    i2c0master_StartN(0x32>>1, I2C_WRITE, 1); 
    i2c0master_WriteN(0x7E);  // save calib
    std::cout <<"Calibration Saved" << std::endl;
    usleep(50000); // 50 ms
  } 
  
  i2c_Close();

  return 0;
}

