
//============================================================
//  i2c.h
//  Justin M Selfridge
//============================================================
#ifndef _I2C_H_
#define _I2C_H_
#include <main.h>


//void linux_set_i2c_bus ( int bus );  // Debugging function for generating global variable


// I2C functions
int  i2c_init   ( int *fd, ushort bus, ushort addr ); 
void i2c_exit   ( int *fd );
int  i2c_slave  ( unsigned char slave_addr );
int  i2c_tx     ( unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data );
int  i2c_rx     ( unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data ); 
int  i2c_get_ms ( unsigned long *ms );


#endif



