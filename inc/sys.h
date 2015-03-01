
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
//#define DEBUG           false
//#define NSEC_PER_SEC    1000000000u
//#define MAIN_LOOP_NS    10000000u
//#define FREQ            (double)( NSEC_PER_SEC / MAIN_LOOP_NS )
//#define DT              (double) 1/FREQ


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


// Time structure
//typedef struct {
//  ulong   start_sec;
//  ulong   start_nano;
//  ulong   end_sec;
//  ulong   end_nano;
//  ulong   count;
//  ulong   dur;
//  double  runtime;
//  double  percent;
//} time_struct;
//time_struct t;


// Global variables
struct sigaction sys_signal;
//uint* memoryPtr;
uint ret;
//struct timespec timeval;
//timer_t timerid;


// Function declarations
void sys_err    ( bool cond, char* msg );
void sys_init   (  );
//void uav_loop   (  );
//void uav_debug  (  );
void sys_exit   (  );
void sys_memory ( void );


#endif



