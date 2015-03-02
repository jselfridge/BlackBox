
//============================================================
//  sys.h
//  Justin M Selfridge
//============================================================
#ifndef _SYS_H_
#define _SYS_H_
#include <main.h>


// Define statements
#define SYS_LOOP        10000000u
#define SYS_FREQ        (double)( NSEC_PER_SEC / SYS_LOOP )
#define SYS_DT          (double)( 1/SYS_FREQ )


// UAV structure
//typedef struct {
//  short radio[8];
//  short servo[8];
//  bool  running;
//} uav_struct;
//uav_struct  uav;


// Global variables
struct sigaction sys_signal;
//uint* memoryPtr;


// Function declarations
void sys_err    ( bool cond, char* msg );
void sys_init   (  );
void sys_loop   (  );
void sys_debug  (  );
void sys_exit   (  );
void sys_memory ( void );


#endif



