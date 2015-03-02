
//============================================================
//  sys.h
//  Justin M Selfridge
//============================================================
#ifndef _SYS_H_
#define _SYS_H_
#include <main.h>


// Standard includes
//#include <fcntl.h>
//#include <sched.h>
//#include <signal.h>
//#include <stdbool.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <sys/mman.h>
//#include <time.h>
//#include <unistd.h>


// Custom libraries
//#include <matLib.h>
//#include <rotLib.h>


// Custom includes
//#include <ctrl.h>
//#include <gpio.h>
//#include <led.h>
//#include <log.h>
//#include <mpu.h>
//#include <pru.h>
//#include <timer.h>


// PRU includes
//#include <prussdrv.h>
//#include <pruss_intc_mapping.h>


// MPU includes
//#include <inv_mpu.h>
//#include <inv_mpu_dmp_motion_driver.h>
//#include <inv_glue.h>


// Define statements
#define SYS_LOOP        10000000u
#define SYS_FREQ        (double)( NSEC_PER_SEC / SYS_LOOP )
#define SYS_DT          (double)( 1/SYS_FREQ )


// UAV structure
//typedef struct {
//  short radio[8];
//  short servo[8];
//  FILE* logfile;
//  char* filename;
//  bool  fileopen;
//  bool  logdata;
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



