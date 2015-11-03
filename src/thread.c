
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

  // Local variables
  pthread_attr_t attr;
  struct sched_param param;

  // Stabilization
  thr_stab.priority  =     90;
  thr_stab.pin       =     60;
  thr_stab.period    =   2500;

  // Navigation
  thr_nav.priority   =     80;
  thr_nav.pin        =     50;
  thr_nav.period     =  10000;

  // Telemetry
  thr_telem.priority =     70;
  thr_telem.pin      =     51;
  thr_telem.period   =  50000;

  // Initialize attribute variable
  pthread_attr_init(&attr);

  // Make threads joinable
  sys.ret = pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_JOINABLE );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'joinable' attribute." );

  // Specify inherit schedule
  sys.ret = pthread_attr_setinheritsched( &attr, PTHREAD_EXPLICIT_SCHED );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'explicit' attribute." );

  // Set scheduler policy
  sys.ret = pthread_attr_setschedpolicy( &attr, SCHED_FIFO );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'FIFO' attribute." );

  // Initialize 'stabilization' thread
  param.sched_priority = thr_stab.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'stab' priority." );
  sys.ret = pthread_create ( &thr_stab.id, &attr, thread_stab, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'stab' thread." );

  // Initialize 'navigation' thread
  param.sched_priority = thr_nav.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'nav' priority." );
  sys.ret = pthread_create ( &thr_nav.id, &attr, thread_nav, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'nav' thread." );

  // Initialize 'telemetry' thread
  param.sched_priority = thr_telem.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'telem' priority." );
  sys.ret = pthread_create ( &thr_telem.id, &attr, thread_telem, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'telem' thread." );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_periodic
//  Establishes the periodic attributes for a thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thread_periodic ( thread_struct *thr )  {

  // Local variables
  unsigned int fd, sec, nsec;
  struct itimerspec itval;

  // Create the timer
  fd = timerfd_create ( CLOCK_MONOTONIC, 0 );
  sys_err( fd == -1, "Error (thread_periodic): Failed to create timer." );
  thr->fd = fd;

  // Determine time values
  sec = thr->period / 1000000;
  nsec = ( thr->period - ( sec * 1000000 ) ) * 1000;

  // Set interval duration
  itval.it_interval.tv_sec = sec;
  itval.it_interval.tv_nsec = nsec;

  // Set start value
  itval.it_value.tv_sec = 0;
  itval.it_value.tv_nsec = 200000000;

  // Enable the timer
  sys.ret = timerfd_settime ( fd, 0, &itval, NULL );
  sys_err( sys.ret, "Error (thread_periodic): Failed to enable the timer." );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_pause
//  Implements the pause before starting the next loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thread_pause ( thread_struct *thr )  {

  // Local variables
  unsigned long long missed;

  // Wait for timer event and obtain number of "missed" loops
  sys.ret = read( thr->fd, &missed, sizeof(missed) );
  sys_err( sys.ret == -1, "Error (thread_pause): Failed to read timer file." );

  // Play around with the "missed" feature some more...
  if ( missed > 0 )  {  thr->missed += (missed - 1);  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_exit
//  Cleanly exits the threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thread_exit ( void )  {
  printf("Closing threads  \n");
  void *status;

  // Exit 'stabilization' thread
  sys.ret = pthread_join ( thr_stab.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'stab' thread." );
  if(DEBUG)  printf( "  Status %ld for 'stabilization' thread \n", (long)status );

  // Exit 'navigation' thread
  sys.ret = pthread_join ( thr_nav.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'nav' thread." );
  if(DEBUG)  printf( "  Status %ld for 'navigation' thread \n", (long)status );

  // Exit 'telemetry' thread
  sys.ret = pthread_join ( thr_telem.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'telem' thread." );
  if(DEBUG)  printf( "  Status %ld for 'telemetry' thread \n", (long)status );

  return;
}


//----------------------------------------
void *thread_stab ( )  {
  printf("  Running 'stabilization' thread \n");
  gpio_export(thr_stab.pin);
  gpio_set_dir( thr_stab.pin, OUTPUT_PIN );
  thread_periodic (&thr_stab);
  while (sys.running) {
    gpio_set_val( thr_stab.pin, HIGH );
    usleep(2000);
    gpio_set_val( thr_stab.pin, LOW );
    thread_pause(&thr_stab);
  }
  pthread_exit(NULL);
  return NULL;
}


//----------------------------------------
void *thread_nav ( )  {
  printf("  Running 'navigation' thread \n");
  gpio_export(thr_nav.pin);
  gpio_set_dir( thr_nav.pin, OUTPUT_PIN );
  thread_periodic (&thr_nav);
  while (sys.running) {
    gpio_set_val( thr_nav.pin, HIGH );
    usleep(5000);
    gpio_set_val( thr_nav.pin, LOW );
    thread_pause(&thr_nav);
  }
  pthread_exit(NULL);
  return NULL;
}


//----------------------------------------
void *thread_telem ( )  {
  printf("  Running 'telemetry' thread \n");
  gpio_export(thr_telem.pin);
  gpio_set_dir( thr_telem.pin, OUTPUT_PIN );
  thread_periodic (&thr_telem);
  while (sys.running) {
    gpio_set_val( thr_telem.pin, HIGH );
    usleep(5000);
    gpio_set_val( thr_telem.pin, LOW );
    thread_pause(&thr_telem);
  }
  pthread_exit(NULL);
  return NULL;
}



