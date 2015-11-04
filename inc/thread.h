
//============================================================
//  thread.h
//  Justin M Selfridge
//============================================================
#ifndef _THREAD_H_
#define _THREAD_H_
#include <main.h>


// Thread structure
typedef struct thread_struct {
  pthread_t thread_stab;
  pthread_t thread_nav;
  pthread_t thread_telem;
} thread_struct;
thread_struct threads; 


// Global variables
//timer_t timerid;
//struct timespec timeval;

// Thread functions
void  thread_init   ( void );
//void  timer_begin  ( void );
//void  timer_exit   ( void );
//void  timer_start  ( void );
//void  timer_finish ( void );


#endif



