
//============================================================
//  thread.c
//  Justin M Selfridge
//============================================================
#include "thread.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_attr
//  Sets the thread attributes.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_attr ( pthread_attr_t* attr )  {
  if(DEBUG)  printf("  Setting thread attributes \n");

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
//  thr_init
//  Initializes a new thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_init ( thread_struct* thr, pthread_attr_t* attr, void* fcn )  {
  if(DEBUG)  printf( "  Running '%s' \n", thr->name );
  struct sched_param param;
  param.sched_priority = thr->priority;
  if( pthread_attr_setschedparam( attr, &param ) )
    printf( "Error (thr_init): Failed to set '%s' priority. \n", thr->name );
  if( pthread_create ( &thr->id, attr, fcn, (void *)NULL ) )
    printf( "Error (thr_init): Failed to create '%s' thread. \n", thr->name );
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_blah
//  
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_blah ( void )  {
  if(DEBUG)  printf("           Blah threads \n");

  // Local variables
  //struct sched_param param;

  // Mutex initialization
  //pthread_mutex_init( &mutex_imu,    NULL );
  //pthread_mutex_init( &mutex_fusion, NULL );
  //pthread_mutex_init( &mutex_sysio,  NULL );

  /*
  // Determine priority range
  if(DEBUG)  printf("  Priority range: %d to %d \n", \
    sched_get_priority_min(SCHED_FIFO), \
    sched_get_priority_max(SCHED_FIFO) );
  */
  /*  // Create 'imu' thread
  param.sched_priority = thr_imu.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'imu' priority." );
  sys.ret = pthread_create ( &thr_imu.id, &attr, thread_imu, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'imu' thread." ); */

  /*  // Create 'fusion' thread
  param.sched_priority = thr_fusion.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'fusion' priority." );
  sys.ret = pthread_create ( &thr_fusion.id, &attr, thread_fusion, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'fusion' thread." ); */

  /*  // Create 'sysio' thread
  param.sched_priority = thr_sysio.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'sysio' priority." );
  sys.ret = pthread_create ( &thr_sysio.id, &attr, thread_sysio, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'sysio' thread." ); */

  /*  // Create 'ctrl' thread
  param.sched_priority = thr_ctrl.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'ctrl' priority." );
  sys.ret = pthread_create ( &thr_ctrl.id, &attr, thread_ctrl, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'ctrl' thread." ); */

  /*  // Create 'telem' thread
  param.sched_priority = thr_telem.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'telem' priority." );
  sys.ret = pthread_create ( &thr_telem.id, &attr, thread_telem, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'telem' thread." ); */
  /*
  // Create 'debug' thread
  if(DEBUG) {
  param.sched_priority = PRIO_DEBUG;
  if( pthread_attr_setschedparam( &attr, &param ) )
    printf( "Error (thread_init): Failed to set 'debug' priority. \n" );
  if( pthread_create ( &thr_debug.id, &attr, thread_debug, (void *)NULL ) )
    printf( "Error (thread_init): Failed to create 'debug' thread. \n" );
  }
  */
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_periodic
//  Establishes the periodic attributes for a thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_periodic ( thread_struct *thr )  {
  /*
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
  */
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_pause
//  Implements the pause before starting the next loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_pause ( thread_struct *thr )  {
  /*
  // Local variables
  unsigned long long missed;

  // Wait for timer event and obtain number of "missed" loops
  sys.ret = read( thr->fd, &missed, sizeof(missed) );
  sys_err( sys.ret == -1, "Error (thread_pause): Failed to read timer file." );

  // Play around with the "missed" feature some more...
  //if ( missed > 0 )  {  thr->missed += (missed - 1);  }
  */
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_start
//  Start code for a thread loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_start ( thread_struct *thr )  {
  /*
  // Get current time
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );

  // Assign start time to thread
  thr->start_sec  = timeval.tv_sec;
  thr->start_usec = timeval.tv_nsec / 1000;
  */
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_finish
//  Finish code for a thread loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_finish ( thread_struct *thr )  {
  /*
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
  */
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_exit
//  Cleanly exits the threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_exit ( thread_struct* thr )  {
  void *status;
  if( pthread_join ( thr->id, &status ) )
    printf( "Error (thread_exit): Failed to exit '%s' thread. \n", thr->name );
  if(DEBUG)  printf( "  Status %ld for '%s' \n", (long)status, thr->name );
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_exit
//  Cleanly exits the threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//void thr_exit ( void )  {
  //if(DEBUG)  printf("Closing threads  \n");
  //void *status;

  /*
  // Exit 'imu' thread
  sys.ret = pthread_join ( thr_imu.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'imu' thread." );
  if(DEBUG)  printf( "  Status %ld for 'imu' thread \n", (long)status );
  */
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
  /*
  // Exit 'debug' thread
  if(DEBUG) {
  sys.ret = pthread_join ( thr_debug.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'debug' thread." );
  printf( "  Status %ld for 'debug' thread \n", (long)status );
  }
  */
  //return;
//}






//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_imu
//  Function handler for the 'imu' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void* fcn_imu ( )  {
  if(DEBUG)  printf("  Is that an IMU... or are you happy to see me?... \n");
  /*
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
  */
  return NULL;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_fusion
//  Function handler for  the 'fusion' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void* fcn_fusion ( )  {
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
//  fcn_sysio
//  Function handler for the 'sysio' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void* fcn_sysio ( )  {
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
//  fcn_ctrl
//  Function handler for the 'ctrl' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void* fcn_ctrl ( )  {
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
//  fcn_telem
//  Function handler for the 'telem' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void* fcn_telem ( )  {
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
//  fcn_debug
//  Function handler for the 'debug' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void* fcn_debug ( )  {
  if(DEBUG)  printf("  hey there... good lookin'... \n");
  /*
  thr_periodic (&thr_debug);
  while (sys.running) {
    thr_start(&thr_debug);
    sys_debug();
    thr_finish(&thr_debug);
    thr_pause(&thr_debug);
  }
  pthread_exit(NULL);
  */
  return NULL;
}



