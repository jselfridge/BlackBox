
//============================================================
//  main.h
//  Justin M Selfridge
//============================================================
#ifndef _MAIN_H_
#define _MAIN_H_


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
//#include <matLib.h>
//#include <rotLib.h>


// Custom includes
//#include <ctrl.h>
#include <gpio.h>
#include <led.h>
#include <log.h>
//#include <mpu.h>
#include <pru.h>
#include <sys.h>
#include <timer.h>


// PRU includes
#include <prussdrv.h>
#include <pruss_intc_mapping.h>


// MPU includes
//#include <inv_mpu.h>
//#include <inv_mpu_dmp_motion_driver.h>
//#include <inv_glue.h>


// Define statements
#define DEBUG           true
#define NSEC_PER_SEC    1000000000u


// UAV structure
//typedef struct {
//  short radio[8];
//  short servo[8];
//  bool  running;
//} uav_struct;
//uav_struct  uav;


// Global variables
//struct sigaction exit_signal;
//uint* memoryPtr;
uint ret;


#endif



