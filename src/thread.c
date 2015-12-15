
//============================================================
//  thread.c
//  Justin M Selfridge
//============================================================
#include "thread.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_init
//  Initializes the various threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_init ( void )  {
  if(DEBUG)  printf("Initializing threads \n");

  // Local variables
  pthread_attr_t attr;
  struct sched_param param;

  // MEMS devices
  thr_mems.priority  =       99;
  thr_mems.period    =  MEMS_HZ;

  // Compass readings
  //thr_comp.priority  =       98;
  //thr_comp.period    =  COMP_HZ;

  // Debugging
  thr_debug.priority =     94;
  thr_debug.period   = 100000;

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

  // Initialize 'mems' thread
  param.sched_priority = thr_mems.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'mems' priority." );
  sys.ret = pthread_create ( &thr_mems.id, &attr, thread_mems, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'mems' thread." );

  // Initialize 'compass' thread
  //param.sched_priority = thr_comp.priority;
  //sys.ret = pthread_attr_setschedparam( &attr, &param );
  //sys_err( sys.ret, "Error (thread_init): Failed to set 'comp' priority." );
  //sys.ret = pthread_create ( &thr_comp.id, &attr, thread_comp, (void *)NULL );
  //sys_err( sys.ret, "Error (thread_init): Failed to create 'comp' thread." );

  // Initialize 'debug' thread
  if(DEBUG) {
  param.sched_priority = thr_debug.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'debug' priority." );
  sys.ret = pthread_create ( &thr_debug.id, &attr, thread_debug, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'debug' thread." );
  printf("\n");
  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_periodic
//  Establishes the periodic attributes for a thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_periodic ( thread_struct *thr )  {

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
//  thr_pause
//  Implements the pause before starting the next loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_pause ( thread_struct *thr )  {

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
//  thr_start
//  Start code for a thread loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_start ( thread_struct *thr )  {

  // Get current time
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );

  // Assign start time to thread
  thr->start_sec  = timeval.tv_sec;
  thr->start_usec = timeval.tv_nsec / 1000;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_finish
//  Finish code for a thread loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_finish ( thread_struct *thr )  {

  // Get current time
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );

  // Assign finish time to thread
  thr->finish_sec  = timeval.tv_sec;
  thr->finish_usec = timeval.tv_nsec /1000;

  // Adjust for rollover
  if ( thr->finish_sec == thr->start_sec )  thr->dur = 0;
  else                                      thr->dur = 1000000;

  // Calculate timing metrics
  thr->dur += thr->finish_usec - thr->start_usec;
  thr->perc = thr->dur / (double)thr->period;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_exit
//  Cleanly exits the threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_exit ( void )  {
  printf("Closing threads  \n");
  void *status;

  // Exit 'mems' thread
  sys.ret = pthread_join ( thr_mems.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'mems' thread." );
  if(DEBUG)  printf( "  Status %ld for 'mems' thread \n", (long)status );

  // Exit 'compass' thread
  //sys.ret = pthread_join ( thr_comp.id, &status );
  //sys_err( sys.ret, "Error (thread_exit): Failed to exit 'comp' thread." );
  //if(DEBUG)  printf( "  Status %ld for 'compass' thread \n", (long)status );

  // Exit 'debug' thread
  if(DEBUG) {
  sys.ret = pthread_join ( thr_debug.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'debug' thread." );
  printf( "  Status %ld for 'debug' thread \n", (long)status );
  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_mems
//  Run the 'mems' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *thread_mems ( )  {
  printf("  Running 'mems' thread \n");
  thr_periodic (&thr_mems);
  while (sys.running) {
    thr_start(&thr_mems);
    imu_mems(&imu1);
    thr_finish(&thr_mems);
    log_write(LOG_GYRO);
    log_write(LOG_ACC);
    log_write(LOG_MAG);
    thr_pause(&thr_mems);
  }
  pthread_exit(NULL);
  return NULL;
}

/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_comp
//  Run the 'compass' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *thread_comp ( )  {
  printf("  Running 'compass' thread \n");
  thr_periodic (&thr_comp);
  while (sys.running) {
    thr_start(&thr_comp);
    imu_comp(&imu1);
    thr_finish(&thr_comp);
    //log_write(LOG_MAG);
    thr_pause(&thr_comp);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_debug
//  Run the 'debug' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *thread_debug ( )  {
  printf("  Running 'debug' thread \n");
  thr_periodic (&thr_debug);
  while (sys.running) {
    thr_start(&thr_debug);
    usleep(10000);
    sys_debug();
    thr_finish(&thr_debug);
    thr_pause(&thr_debug);
  }
  pthread_exit(NULL);
  return NULL;
}



