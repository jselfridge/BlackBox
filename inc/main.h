
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
#include <sys/timerfd.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>


// Might be needed... not certain...
#include <sys/time.h>       // Needed for 'gettimeofday'
//#include <errno.h>          // Wasn't needed to compile
//#include <stdint.h>         // Wasn't needed to compile
//#include <sys/ioctl.h>      // Needed for 'ioctl'
//#include <linux/i2c-dev.h>  // Needed for 'I2C_SLAVE'  


uint i2c_fd;
//uint i2c_addr = 0x68;

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


// Global variables (remove as quickly as possible)
bool running;
bool armed;
struct sigaction sys_signal;


#endif



