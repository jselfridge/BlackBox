
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

  // IMU thread
  thr_imu.priority     =  99;
  thr_imu.period       =  1000000 / HZ_IMU;

  // Data fusion thread
  //thr_fusion.priority  =  96;
  //thr_fusion.period    =  1000000 / FUSION_HZ;

  // System input/output thread
  //thr_sysio.priority  =  94;
  //thr_sysio.period    =  1000000 / SYSIO_HZ;

  // Controller thread
  //thr_ctrl.priority =  92;
  //thr_ctrl.period   =  1000000 / CTRL_HZ;

  // Telemetry thread
  //thr_telem.priority =  90;
  //thr_telem.period   =  1000000 / TELEM_HZ;

  // Debugging thread
  //thr_debug.priority =  88;
  //thr_debug.period   =  1000000 / HZ_DEBUG;

  // Mutex initialization
  //pthread_mutex_init( &mutex_imu,    NULL );
  //pthread_mutex_init( &mutex_fusion, NULL );
  //pthread_mutex_init( &mutex_sysio,  NULL );

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

  // Test scheduler value
  if(DEBUG)  printf("  Priority range: %d to %d \n", \
    sched_get_priority_min(SCHED_RR), \
    sched_get_priority_max(SCHED_RR) );

  // Initialize 'imu' thread
  param.sched_priority = thr_imu.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'imu' priority." );
  sys.ret = pthread_create ( &thr_imu.id, &attr, thread_imu, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'imu' thread." );

  // Check for proper policy value....
  //int returnval;
  //returnval = pthread_getschedparam( thr_imu.id, &SCHED_FIFO, &param.sched_priority );


  /*  // Initialize 'fusion' thread
  param.sched_priority = thr_fusion.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'fusion' priority." );
  sys.ret = pthread_create ( &thr_fusion.id, &attr, thread_fusion, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'fusion' thread." ); */

  /*  // Initialize 'sysio' thread
  param.sched_priority = thr_sysio.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'sysio' priority." );
  sys.ret = pthread_create ( &thr_sysio.id, &attr, thread_sysio, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'sysio' thread." ); */

  /*  // Initialize 'ctrl' thread
  param.sched_priority = thr_ctrl.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'ctrl' priority." );
  sys.ret = pthread_create ( &thr_ctrl.id, &attr, thread_ctrl, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'ctrl' thread." ); */

  /*  // Initialize 'telem' thread
  param.sched_priority = thr_telem.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'telem' priority." );
  sys.ret = pthread_create ( &thr_telem.id, &attr, thread_telem, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'telem' thread." ); */

  /*  // Initialize 'debug' thread
  if(DEBUG) {
  param.sched_priority = thr_debug.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'debug' priority." );
  sys.ret = pthread_create ( &thr_debug.id, &attr, thread_debug, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'debug' thread." );
  printf("\n");
  } */

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
  //if ( missed > 0 )  {  thr->missed += (missed - 1);  }

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

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_exit
//  Cleanly exits the threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_exit ( void )  {
  if(DEBUG)  printf("Closing threads  \n");
  void *status;

  // Exit 'imu' thread
  sys.ret = pthread_join ( thr_imu.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'imu' thread." );
  if(DEBUG)  printf( "  Status %ld for 'imu' thread \n", (long)status );

  /*  // Exit 'fusion' thread
  sys.ret = pthread_join ( thr_fusion.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'fusion' thread." );
  if(DEBUG)  printf( "  Status %ld for 'fusion' thread \n", (long)status ); */

  /*  // Exit 'sysio' thread
  sys.ret = pthread_join ( thr_sysio.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'sysio' thread." );
  if(DEBUG)  printf( "  Status %ld for 'sysio' thread \n", (long)status ); */

  /*  // Exit 'ctrl' thread
  sys.ret = pthread_join ( thr_ctrl.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'ctrl' thread." );
  if(DEBUG)  printf( "  Status %ld for 'ctrl' thread \n", (long)status ); */

  /*  // Exit 'telem' thread
  sys.ret = pthread_join ( thr_telem.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'telem' thread." );
  if(DEBUG)  printf( "  Status %ld for 'telem' thread \n", (long)status ); */

  /*  // Exit 'debug' thread
  if(DEBUG) {
  sys.ret = pthread_join ( thr_debug.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'debug' thread." );
  printf( "  Status %ld for 'debug' thread \n", (long)status );
  } */

  // Destroy mutex
  //pthread_mutex_destroy(&mutex_imu);
  //pthread_mutex_destroy(&mutex_fusion);
  //pthread_mutex_destroy(&mutex_sysio);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_imu
//  Run the 'imu' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *thread_imu ( )  {
  if(DEBUG)  printf("  Running 'imu' thread \n");
  thr_periodic (&thr_imu);
  while (sys.running) {
    thr_start(&thr_imu);
    imu_data(&imu1);
    thr_finish(&thr_imu);
    log_write(LOG_GYR);
    log_write(LOG_ACC);
    //if (!imu1.count)  log_write(LOG_MAG);
    thr_pause(&thr_imu);
  }
  pthread_exit(NULL);
  return NULL;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_fusion
//  Run the 'fusion' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void *thread_fusion ( )  {
  if(DEBUG)  printf("  Running 'fusion' thread \n");
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
//  thread_sysio
//  Run the 'sysio' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void *thread_sysio ( )  {
  if(DEBUG)  printf("  Running 'sysio' thread \n");
  thr_periodic (&thr_sysio);
  ushort i;
  while (sys.running) 
  {
    thr_start(&thr_sysio);
    pthread_mutex_lock(&mutex_sysio);
    for ( i=0; i<10; i++ ) 
    {
      sys.input[i] = pru_read_pulse(i);
      pru_send_pulse( i, sys.output[i] );      
    }
    pthread_mutex_unlock(&mutex_sysio);
    thr_finish(&thr_sysio);
    log_write(LOG_SYSIO);
    thr_pause(&thr_sysio);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_ctrl
//  Run the 'ctrl' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void *thread_ctrl ( )  {
  if(DEBUG)  printf("  Running 'ctrl' thread \n");
  usleep(500000);
  thr_periodic (&thr_ctrl);
  while (sys.running) {
    thr_start(&thr_ctrl);
    ctrl_law();
    thr_finish(&thr_ctrl);
    log_write(LOG_CTRL);
    thr_pause(&thr_ctrl);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_telem
//  Run the 'telem' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void *thread_telem ( )  {
  if(DEBUG)  printf("  Running 'telem' thread \n");
  usleep(500000);
  thr_periodic (&thr_telem);
  while (sys.running) {
    thr_start(&thr_telem);
    telem_update();
    thr_finish(&thr_telem);
    //log_write(LOG_PARAM);
    thr_pause(&thr_telem);
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
  if(DEBUG)  printf("  Running 'debug' thread \n");
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
  } */



