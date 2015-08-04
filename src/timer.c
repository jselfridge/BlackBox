
//============================================================
//  timer.c
//  Justin M Selfridge
//============================================================
#include "timer.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  timer_init
//  Initializes the timing loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void timer_init ( void )  {
  if(DEBUG)  printf("Initializing timing loop \n");

  // Define timer function
  struct sigaction uav_timer;
  if(DEBUG)  printf("  Defining timer... ");
  uav_timer.sa_sigaction = uav_loop;
  ret = sigaction( SIGRTMIN, &uav_timer, NULL );
  if ( ret == -1 )  printf("Error (uav_init): Could not assign sigaction. \n");
  if(DEBUG)  printf("complete \n");

  // Create timer
  struct sigevent sev;
  if(DEBUG)  printf("  Creating timer... ");
  sev.sigev_notify = SIGEV_SIGNAL;
  sev.sigev_signo = SIGRTMIN;
  sev.sigev_value.sival_ptr = &timerid;
  ret = timer_create( CLOCK_REALTIME, &sev, &timerid );
  if ( ret == -1 )  printf("Error (uav_init): Could not create timer. \n");
  if(DEBUG)  printf("complete \n");

  // Initialize timing counters
  if(DEBUG)  printf( "  Sample rate: %d \n", (int)FREQ );
  t.count = 0;
  t.dur   = 0;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  timer_begin
//  Begins the timing loop thread after hardware is loaded.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void timer_begin ( void )  {
  if(DEBUG)  printf("Begin timing loop \n");
  struct itimerspec its;
  its.it_value.tv_sec      = 0;
  its.it_value.tv_nsec     = 500000000;
  its.it_interval.tv_sec   = 0;
  its.it_interval.tv_nsec  = MAIN_LOOP_NS;
  ret = timer_settime( timerid, 0, &its, NULL );
  uav_err( ret == -1, "Error (uav_init): Could not assign timer values. \n");
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  timer_exit
//  Closes the timing structure.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void timer_exit ( void )  {
  if(DEBUG)  printf("  Closing timing loop \n");
  ret = timer_delete(timerid);
  if ( ret == -1 )  printf("Error (uav_init): Could not create timer. \n");
  usleep(200000);
  return;
}


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
  t.percent = t.dur / (double)MAIN_LOOP_NS;
  t.runtime = t.count / FREQ;
  t.count++;
  return;
}



