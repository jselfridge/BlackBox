
//============================================================
//  sys.h
//  Justin M Selfridge
//============================================================
#ifndef _SYS_H_
#define _SYS_H_
#include <main.h>


// Define statements
<<<<<<< HEAD
#define DEBUG        true
#define SYS_STACK    ( 100 * 1024 * 1024 )


// Function declarations
void sys_init   (  );
void sys_debug  ( tmr_struct* tmr_debug, sensor_struct* gyr_sensor );
void sys_flag   (  );
=======
#define DEBUG           true
#define SYS_STACK       ( 100 * 1024 * 1024 )


// Function declarations  { Add void? }
void sys_init   ( void );
void sys_debug  ( void );
void sys_imu    ( void );
>>>>>>> altlog
void sys_exit   (  );


#endif



