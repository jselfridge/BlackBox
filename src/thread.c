
//============================================================
//  thread.c
//  Justin M Selfridge
//============================================================
#include "thread.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_init
//  Initializes the various threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thread_init ( void )  {
  if(DEBUG)  printf("Initializing threads \n");


  return;
}

/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  timer_begin
//  Begins the timing loop thread for the process.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void timer_begin ( void )  {
  if(DEBUG)  printf("Begin timing loop \n");
  struct itimerspec its;
  its.it_value.tv_sec      = 0;
  its.it_value.tv_nsec     = 500000000;
  its.it_interval.tv_sec   = 0;
  its.it_interval.tv_nsec  = SYS_LOOP;
  ret = timer_settime( timerid, 0, &its, NULL );
  sys_err( ret == -1, "Error (timer_begin): Could not assign timer values." );
  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  timer_exit
//  Closes the timing loop process.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void timer_exit ( void )  {
  if(DEBUG)  printf("  Closing timing loop \n");
  ret = timer_delete(timerid);
  sys_err( ret == -1, "Error (timer_exit): Could not exit the timer." );
  usleep(200000);
  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  timer_start
//  Start code for a timing loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void timer_start ( void )  {
  clock_gettime( CLOCK_MONOTONIC, &timeval );
  t.start_sec  = timeval.tv_sec;
  t.start_nano = timeval.tv_nsec;
  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  timer_finish
//  Finish code for a timing loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void timer_finish ( void )  {
  clock_gettime( CLOCK_MONOTONIC, &timeval );
  t.end_sec  = timeval.tv_sec;
  t.end_nano = timeval.tv_nsec;
  if ( t.end_sec == t.start_sec )  t.dur = 0;
  else                             t.dur = NSEC_PER_SEC;
  t.dur += t.end_nano - t.start_nano;
  t.percent = t.dur / (double)SYS_LOOP;
  t.runtime = t.count / SYS_FREQ;
  t.count++;
  return;
}
*/


