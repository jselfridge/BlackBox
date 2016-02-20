
//============================================================
//  main.h
//  Justin M Selfridge
//============================================================
#ifndef _MAIN_H_
#define _MAIN_H_


// Standard includes
#include <fcntl.h>
#include <limits.h>
#include <linux/i2c-dev.h>
#include <malloc.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>


// Custom includes
#include <ahr.h>
#include <ctrl.h>
#include <flag.h>
#include <gpio.h>
#include <i2c.h>
#include <imu.h>
#include <led.h>
#include <log.h>
#include <mpu.h>
#include <sio.h>
#include <sys.h>
#include <timer.h>


// PRU includes
#include <prussdrv.h>
#include <pruss_intc_mapping.h>


// MPU function wrapper
#define get_ms         i2c_get_ms
#define log_i          printf
#define log_e          printf
#define delay_ms(t)    usleep(t*1000)
#define min(a,b)       ( (a < b) ? a : b )


// Global variables (remove as quickly as possible)
bool running;
bool armed;
unsigned char slave;
struct sigaction sys_signal;


#endif



