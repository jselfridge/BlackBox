
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


// Thread structure
typedef struct thread_struct {
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
} thread_struct;


/*// Sensor structure
typedef struct sensor_struct {
  uint  hz;
  float dt;
  float lpf;
  float tc;
  float gain;
  short raw;
  float filter;
  float calib;
} sensor_struct;
*/

/*// IMU structure
typedef struct imu_struct {
  ushort addr;
  sensor_struct gyr;
  sensor_struct acc;
  sensor_struct mag;
} imu_struct;
*/

// Temp structure
typedef struct temp_struct {
  ushort raw;
} temp_struct;


// Custom includes
//#include <ctrl.h>
#include <gpio.h>
//#include <imu.h>
#include <led.h>
//#include <log.h>
//#include <loop.h>
//#include <pru.h>
#include <sys.h>
//#include <telem.h>
#include <thread.h>


// PRU includes
#include <prussdrv.h>
#include <pruss_intc_mapping.h>


// MPU includes
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <inv_glue.h>


#endif



