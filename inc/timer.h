
//============================================================
//  timer.h
//  Justin M Selfridge
//============================================================
#ifndef _TIMER_H_
#define _TIMER_H_
#include <main.h>


// Timer structure
typedef struct {
  ulong   start_sec;
  ulong   start_nano;
  ulong   end_sec;
  ulong   end_nano;
  ulong   count;
  ulong   dur;
  double  runtime;
  double  percent;
} timer_struct;
timer_struct t; 


// Global variables
timer_t timerid;
struct timespec timeval;

// Realtime loop functions (Make it more modular for other processes)
void  timer_init   ( void );
void  timer_begin  ( void );
void  timer_exit   ( void );
void  timer_start  ( void );
void  timer_finish ( void );


#endif



