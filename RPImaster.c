/* 

RPImaster.c, compile on Raspberry PI 3

Copyright (c) 2017 Warren E Smith  smiwarsky@gmail.com

Please see the MIT License in the project root directory for specifics.  

*/

#include<stdio.h>
#include<fcntl.h>
#include<sys/ioctl.h>
#include<linux/i2c.h>
#include<linux/i2c-dev.h>

# define BUFFER_SIZE 12
const byte slaveAdd = 0x44; 

int main() {

  int file;
  int N = 1000; // number of IMU reads
  char data[BUFFER_SIZE], send;

  if((file=open("/dev/i2c-1", O_RDWR)) < 0) {
    perror("Failed to open the bus\n");
    return 1;
  }
  if(ioctl(file, I2C_SLAVE, slaveAdd) < 0) {
    perror("Failed to connect to the arduino\n");
    return 1;
  }

  for (int k = 0; k < N; k++) {
    
    for (int i = 0; i < BUFFER_SIZE; i++) {
      send = (char) i;
      if(write(file, &send, 1) != 1) {
	perror("Failed to request a register\n");
	return 1;
      }
      if(read(file, &data[i], 1) != 1) {
	perror("Failed to read the data\n");
	return 1;
      }
    }
    printf("%4d  ROLL:%+4d PITCH:%+4d HEADING:%+4d\n",
	   k, data[0]<<8|data[1], data[2]<<8|data[3], data[4]<<8|data[5]);
  }
  close(file);
  return 0;
}
