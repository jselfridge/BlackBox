
//============================================================
//  thread.h
//  Justin M Selfridge
//============================================================
#ifndef _THREAD_H_
#define _THREAD_H_
#include <main.h>


// Thread structure
typedef struct thread_struct {
  pthread_t id;
  int pin;
  uint period;
  ushort priority;
  ushort missed;
} thread_struct;
thread_struct thr_stab; 
thread_struct thr_nav; 
thread_struct thr_telem; 


// Global variables
//timer_t timerid;
//struct timespec timeval;

// Thread functions
void  thread_init    ( void );
//void  thread_attr    ( pthread_attr_t *attr );
//void  thread_create  ( pthread_attr_t *attr, thread_struct *thr, char *name );
void thread_exit  ( );

void *thread_stab ( );
void *thread_nav ( );
void *thread_telem ( );

#endif



