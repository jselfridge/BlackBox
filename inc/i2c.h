
//============================================================
//  i2c.h
//  Justin M Selfridge
//============================================================
#ifndef _I2C_H_
#define _I2C_H_
#include <main.h>


//#define MIN_I2C_BUS 0
//#define MAX_I2C_BUS 7


//#define mpu_write   i2c_write
//#define mpu_read    i2c_read
//#define log_i       printf
//#define log_e       printf
//#define min(a, b)   ((a < b) ? a : b)

void linux_set_i2c_bus ( int bus );  // Debugging function for generating global variable


// I2C functions
int  i2c_open   ( void );
void i2c_close  ( void );
int  i2c_slave  ( unsigned char slave_addr );
int  i2c_tx     ( unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data );
int  i2c_rx     ( unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data ); 
int  i2c_ct     ( unsigned long *count );


#endif



