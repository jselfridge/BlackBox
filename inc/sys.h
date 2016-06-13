

#ifndef SYS_H
#define SYS_H


#include <signal.h>
#include <stdbool.h>


#define DEBUG       false
#define SYS_IO      false
#define SYS_LPF     false
#define SYS_IMU     false
#define SYS_AHRS    false
#define SYS_EKF     false
#define SYS_GPS     false
#define SYS_GCS     false
#define SYS_CTRL    false
#define SYS_STACK  ( 100 * 1024 * 1024 )


struct sigaction sys_signal;
bool running;
bool armed;


void sys_init   ( void );
void sys_exit   ( void );
void sys_quit   ( );
void sys_update ( void );


#endif



