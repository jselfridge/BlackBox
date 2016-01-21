
//============================================================
//  timer.c
//  Justin M Selfridge
//============================================================
#include "timer.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  tmr_init
//  Initializes the various timing threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void tmr_init ( void )  {
  if(DEBUG)  printf("Initializing timing threads \n");

  // Populate the timer structures
  tmr_setup();

  // Specify thread attributes
  pthread_attr_t attr;
  tmr_attr(&attr);


  /*
  // Local variables
  struct sched_param param;

  // Mutex initialization
  pthread_mutex_init( &mutex_cal, NULL );
  pthread_mutex_init( &mutex_fusion, NULL );

  // Initialize 'imu' thread
  param.sched_priority = thr_imu.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'imu' priority." );
  sys.ret = pthread_create ( &thr_imu.id, &attr, thread_imu, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'imu' thread." );

  // Initialize 'fusion' thread
  param.sched_priority = thr_fusion.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'fusion' priority." );
  sys.ret = pthread_create ( &thr_fusion.id, &attr, thread_fusion, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'fusion' thread." );

  // Initialize 'debug' thread
  if(DEBUG) {
  param.sched_priority = thr_debug.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'debug' priority." );
  sys.ret = pthread_create ( &thr_debug.id, &attr, thread_debug, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'debug' thread." );
  printf("\n");
  }
  */
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  tmr_setup
//  Assign timing thread parameters to a data structure.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void tmr_setup ( void )  {
  if(DEBUG)  printf("  Assign parameters to data structure \n");

  // Rate gyro timer
  tmr_gyr.prio     =  PRIO_GYR;
  tmr_gyr.per      =  1000000 / HZ_GYR;

  // Accelerometer timer
  tmr_acc.prio     =  PRIO_ACC;
  tmr_acc.per      =  1000000 / HZ_ACC;

  // Magnetometer timer
  tmr_mag.prio     =  PRIO_MAG;
  tmr_mag.per      =  1000000 / HZ_MAG;

  // Debugging timer
  tmr_debug.prio   =  PRIO_DEBUG;
  tmr_debug.per    =  1000000 / HZ_DEBUG;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  tmr_attr
//  Sets the attributes of the timing threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void tmr_attr ( pthread_attr_t *attr )  {
  if(DEBUG)  printf("  Setting timer thread attributes \n");

  // Initialize attribute variable
  if( pthread_attr_init(attr) )
    printf( "Error (thr_attr): Failed to initialize thread attribute. \n" );

  // Set stack size of threads
  if ( pthread_attr_setstacksize( attr, PTHREAD_STACK_MIN + (100*1024) ) )
    printf( "Error (thr_attr): Failed to set 'stack size' attribute. \n" );

  // Make threads joinable
  if ( pthread_attr_setdetachstate( attr, PTHREAD_CREATE_JOINABLE ) )
    printf( "Error (thr_attr): Failed to set 'joinable' attribute. \n" );

  // Specify inherit schedule
  if ( pthread_attr_setinheritsched( attr, PTHREAD_EXPLICIT_SCHED ) )
    printf( "Error (thr_attr): Failed to set 'explicit' attribute. \n" );

  // Set scheduler policy
  if ( pthread_attr_setschedpolicy( attr, SCHED_FIFO ) )
    printf( "Error (thr_attr): Failed to set 'FIFO' attribute." );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_periodic
//  Establishes the periodic attributes for a thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void thr_periodic ( thread_struct *thr )  {

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
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_pause
//  Implements the pause before starting the next loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void thr_pause ( thread_struct *thr )  {

  // Local variables
  unsigned long long missed;

  // Wait for timer event and obtain number of "missed" loops
  sys.ret = read( thr->fd, &missed, sizeof(missed) );
  sys_err( sys.ret == -1, "Error (thread_pause): Failed to read timer file." );

  // Play around with the "missed" feature some more...
  //if ( missed > 0 )  {  thr->missed += (missed - 1);  }

  return;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_start
//  Start code for a thread loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void thr_start ( thread_struct *thr )  {

  // Get current time
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );

  // Assign start time to thread
  thr->start_sec  = timeval.tv_sec;
  thr->start_usec = timeval.tv_nsec / 1000;

  return;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_finish
//  Finish code for a thread loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void thr_finish ( thread_struct *thr )  {

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

  return;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_exit
//  Cleanly exits the threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void thr_exit ( void )  {
  printf("Closing threads  \n");
  void *status;

  // Exit 'imu' thread
  sys.ret = pthread_join ( thr_imu.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'imu' thread." );
  if(DEBUG)  printf( "  Status %ld for 'imu' thread \n", (long)status );

  // Exit 'fusion' thread
  sys.ret = pthread_join ( thr_fusion.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'fusion' thread." );
  if(DEBUG)  printf( "  Status %ld for 'fusion' thread \n", (long)status );

  // Exit 'debug' thread
  if(DEBUG) {
  sys.ret = pthread_join ( thr_debug.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'debug' thread." );
  printf( "  Status %ld for 'debug' thread \n", (long)status );
  }

  // Destroy mutex
  pthread_mutex_destroy(&mutex_cal);
  pthread_mutex_destroy(&mutex_fusion);

  return;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_imu
//  Run the 'imu' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void *thread_imu ( )  {
  printf("  Running 'imu' thread \n");
  thr_periodic (&thr_imu);
  while (sys.running) {
    thr_start(&thr_imu);
    imu_data(&imu1);
    thr_finish(&thr_imu);
    log_write(LOG_GYR);
    log_write(LOG_ACC);
    if (!imu1.count)  log_write(LOG_MAG);
    thr_pause(&thr_imu);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_fusion
//  Run the 'fusion' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void *thread_fusion ( )  {
  printf("  Running 'fusion' thread \n");
  usleep(500000);
  thr_periodic (&thr_fusion);
  while (sys.running) {
    thr_start(&thr_fusion);
    imu_fusion(&imu1);
    thr_finish(&thr_fusion);
    log_write(LOG_FUSION);
    thr_pause(&thr_fusion);
  }
  pthread_exit(NULL);

  return NULL;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_debug
//  Run the 'debug' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void *thread_debug ( )  {
  printf("  Running 'debug' thread \n");
  usleep(500000);
  thr_periodic (&thr_debug);
  while (sys.running) {
    thr_start(&thr_debug);
    sys_debug();
    thr_finish(&thr_debug);
    thr_pause(&thr_debug);
  }
  pthread_exit(NULL);
  return NULL;
}
*/


