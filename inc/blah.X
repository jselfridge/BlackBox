
//============================================================
//  i2c.h
//  Justin M Selfridge
//============================================================
#ifndef _I2C_H_
#define _I2C_H_
#include <main.h>


#define I2C_MAX_BUF 511

//#include "inv_mpu.h"


//static inline int reg_int_cb(struct int_param_s *int_param)  {
//	return 0;
//}


//#define i2c_write	linux_i2c_write
//#define i2c_read	linux_i2c_read
//#define delay_ms	linux_delay_ms
//#define get_ms		linux_get_ms
//#define log_i		printf
//#define log_e		printf
//#define min(a, b) 	((a < b) ? a : b)



//void linux_set_i2c_bus(int bus);

//int linux_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
//       unsigned char length, unsigned char const *data);

//int linux_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
//       unsigned char length, unsigned char *data);
 
//int linux_delay_ms(unsigned long num_ms);
//int linux_get_ms(unsigned long *count);




// I2C functions
int  i2c_open   ( uint i2c_bus, uint* i2c_fd );
void i2c_close  ( uint *i2c_fd );
int  i2c_write  ( uint i2c_fd, unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data );
int  i2c_read   ( uint i2c_fd, unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data );


#endif



