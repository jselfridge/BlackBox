
//============================================================
//  main.h
//  Justin M Selfridge
//============================================================
#ifndef _MAIN_H_
#define _MAIN_H_


// Standard includes
#include <fcntl.h>
#include <limits.h>
#include <malloc.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>


// Global variables = EVIL... just use them for debugging
bool running;
struct sigaction sys_signal;
FILE* datalog;


// Timer structure
typedef struct tmr_struct {
  char*     name;
  pthread_t id;
  int       fd;
  ushort    priority;
  ulong     period;
  time_t    start_sec;
  ulong     start_usec;
  time_t    finish_sec;
  ulong     finish_usec;
  ulong     dur;
} tmr_struct;


// Sensor structure
typedef struct sensor_struct {
  short raw[3];
  float calib[3];
} sensor_struct;


// Gyroscope argument structure
typedef struct gyr_arg_struct {
  sensor_struct*  gyr_sensor;
  tmr_struct*     gyr_tmr;
} gyr_arg_struct;


// Debug argument structure
typedef struct debug_arg_struct {
  tmr_struct*     debug_tmr;
  sensor_struct*  gyr_sensor;
} debug_arg_struct;


// Custom includes
#include <gpio.h>
#include <led.h>
#include <sys.h>
#include <thread.h>


// Add these in later
//#include <ctrl.h>
//#include <imu.h>
//#include <log.h>
//#include <loop.h>
//#include <pru.h>
//#include <telem.h>


// PRU includes
//#include <prussdrv.h>
//#include <pruss_intc_mapping.h>


// MPU includes
//#include <inv_mpu.h>
//#include <inv_mpu_dmp_motion_driver.h>
//#include <inv_glue.h>


#endif



