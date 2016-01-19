
//============================================================
//  thread.h
//  Justin M Selfridge
//============================================================
#ifndef _THREAD_H_
#define _THREAD_H_
#include <main.h>


// Define frequencies
//#define HZ_IMU     500
//#define HZ_ATT     100
//#define HZ_STAB     50
//#define HZ_NAV      10
//#define HZ_TELEM    10
#define HZ_DEBUG    10
//#define HZ_FLAGS     2
//#define HZ_HEART     1


// Define priorities
//#define PRIO_IMU     98
//#define PRIO_ATT     96
//#define PRIO_STAB    94
//#define PRIO_NAV     92
//#define PRIO_TELEM   90
#define PRIO_DEBUG    88
//#define PRIO_FLAGS   86
//#define PRIO_HEART   84


// Mutex variables
//pthread_mutex_t mutex_imu;
//pthread_mutex_t mutex_fusion;
//pthread_mutex_t mutex_sysio;


// Thread functions
void  thr_attr      ( pthread_attr_t* attr );
//void  thr_init      ( thread_struct* thr, pthread_attr_t* attr, void* fcn );
void  thr_periodic  ( thread_struct* thr );
void  thr_pause     ( thread_struct* thr );
void  thr_start     ( thread_struct* thr );
void  thr_finish    ( thread_struct* thr );
void  thr_exit      ( thread_struct* thr );


// Function handlers
//void* fcn_imu    ( void* arg );
//void* fcn_fus    ( void* arg );
//void* fcn_sio    ( void* arg );
//void* fcn_ctrl   ( void* arg );
//void* fcn_telem  ( void* arg );
void* fcn_debug  ( void* arg );


#endif



