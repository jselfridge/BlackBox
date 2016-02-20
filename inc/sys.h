
//============================================================
//  sys.h
//  Justin M Selfridge
//============================================================
#ifndef _SYS_H_
#define _SYS_H_
#include <main.h>


// Define statements
#define DEBUG           true
#define SYS_STACK       ( 100 * 1024 * 1024 )


// Function declarations
void sys_init   ( void );
void sys_update ( void );
void sys_sio    ( void );
void sys_imuA   ( void );
void sys_imuB   ( void );
void sys_ahr    ( void );
void sys_ctrl   ( void );
void sys_exit   (  );


#endif



