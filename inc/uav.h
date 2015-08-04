
//============================================================
//  uav.h
//  Justin M Selfridge
//============================================================
#ifndef _UAV_H_
#define _UAV_H_


// Standard includes
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>


// Custom libraries
#include <matLib.h>
#include <rotLib.h>


// Custom includes
#include <ctrl.h>
#include <gpio.h>
#include <led.h>
#include <log.h>
#include <mpu.h>
#include <pru.h>
#include <timer.h>


// PRU includes
#include <prussdrv.h>
#include <pruss_intc_mapping.h>


// MPU includes
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <inv_glue.h>


// Define statements
#define DEBUG           false
#define NSEC_PER_SEC    1000000000u
#define MAIN_LOOP_NS    10000000u
#define FREQ            (double)( NSEC_PER_SEC / MAIN_LOOP_NS )
#define DT              (double) 1/FREQ
#define LED_MOT  0
#define LED_LOG  1
#define LED_MPU  2
#define LED_PRU  3


// UAV structure
typedef struct {
  short radio[8];
  short servo[8];
  FILE* logfile;
  char* filename;
  bool  fileopen;
  bool  logdata;
  bool  running;
} uav_struct;
uav_struct  uav;


// Time structure
typedef struct {
  ulong   start_sec;
  ulong   start_nano;
  ulong   end_sec;
  ulong   end_nano;
  ulong   count;
  ulong   dur;
  double  runtime;
  double  percent;
} time_struct;
time_struct t;


// Global variables
struct sigaction exit_signal;
uint* memoryPtr;
uint ret;
struct timespec timeval;
timer_t timerid;


// Function declarations
void uav_err    ( bool cond, char* msg );
void uav_init   (  );
void uav_loop   (  );
void uav_debug  (  );
void uav_exit   (  );
void uav_memory ( void );


#endif



