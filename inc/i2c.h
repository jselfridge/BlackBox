
//============================================================
//  i2c.h
//  Justin M Selfridge
//============================================================
#ifndef _I2C_H_
#define _I2C_H_
#include <main.h>




#define MIN_I2C_BUS 0
#define MAX_I2C_BUS 7

#define i2c_write	linux_i2c_write
#define i2c_read	linux_i2c_read
#define delay_ms	linux_delay_ms
#define get_ms		linux_get_ms
#define log_i		printf
#define log_e		printf
#define min(a, b) 	((a < b) ? a : b)



void linux_set_i2c_bus ( int bus );
int  linux_i2c_write   ( unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data );
int  linux_i2c_read    ( unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data ); 
int  linux_delay_ms    ( unsigned long num_ms );
int  linux_get_ms      ( unsigned long *count );

#endif

