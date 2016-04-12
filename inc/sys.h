

#ifndef SYS_H
#define SYS_H


// Includes
#include <signal.h>
#include <stdbool.h>


// Define statements
#define DEBUG           true
#define SYS_STACK       ( 100 * 1024 * 1024 )


// Globals
bool running;
struct sigaction sys_signal;


// Function declarations
void sys_init   ( void );
void sys_exit   (  );

//void sys_update ( void );
//void sys_sio    ( void );
//void sys_imuA   ( void );
//void sys_imuB   ( void );
//void sys_ahrs   ( void );
//void sys_gps    ( void );
//void sys_ctrl   ( void );

//void sys_uart1  ( void );
//void sys_uart2  ( void );
//void sys_uart4  ( void );
//void sys_uart5  ( void );


#endif



