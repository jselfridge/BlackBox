
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


// Custom includes
#include <gpio.h>
#include <imu.h>
#include <led.h>
#include <log.h>
//#include <sio.h>
#include <sys.h>
#include <timer.h>


// PRU includes
//#include <prussdrv.h>
//#include <pruss_intc_mapping.h>


// MPU includes
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <inv_glue.h>


// Global variables (remove as quickly as possible)
bool running;
bool armed;
struct sigaction sys_signal;

#endif



