
//============================================================
//  timer.c
//  Justin M Selfridge
//============================================================
#include "timer.h"


  // Mutex initialization
  //pthread_mutex_init( &mutex_cal, NULL );
  //pthread_mutex_init( &mutex_fusion, NULL );


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  tmr_init
//  Initializes the various timing threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void tmr_init ( void )  {
  if(DEBUG)  printf("Initializing timing threads \n");

  // Local variables
  pthread_attr_t attr;
  //struct sched_param param;

  // Run through setup functions
  tmr_setup();
  tmr_attr(&attr);

  // Begin each thread
  if(DEBUG)  printf("  Create threads: ");
  tmr_thread( &tmr_gyr,   &attr, fcn_gyr   );
  tmr_thread( &tmr_acc,   &attr, fcn_acc   );
  tmr_thread( &tmr_mag,   &attr, fcn_mag   );
  tmr_thread( &tmr_debug, &attr, fcn_debug );
  if(DEBUG)  printf("\n");

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  tmr_setup
//  Assign timing thread parameters to a data structure.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void tmr_setup ( void )  {
  if(DEBUG)  printf("  Assign thread structure elements \n");

  // Rate gyro timer
  tmr_gyr.name     =  "gyr";
  tmr_gyr.prio     =  PRIO_GYR;
  tmr_gyr.per      =  1000000 / HZ_GYR;

  // Accelerometer timer
  tmr_acc.name     =  "acc";
  tmr_acc.prio     =  PRIO_ACC;
  tmr_acc.per      =  1000000 / HZ_ACC;

  // Magnetometer timer
  tmr_mag.name     =  "mag";
  tmr_mag.prio     =  PRIO_MAG;
  tmr_mag.per      =  1000000 / HZ_MAG;

  // Debugging timer
  tmr_debug.name   =  "debug";
  tmr_debug.prio   =  PRIO_DEBUG;
  tmr_debug.per    =  1000000 / HZ_DEBUG;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  tmr_attr
//  Sets the attributes of the timing threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void tmr_attr ( pthread_attr_t *attr )  {
  if(DEBUG)  printf("  Set timing thread attributes \n");

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
//  tmr_thread
//  Create a new pthread to run the timer. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void tmr_thread ( timer_struct *tmr, pthread_attr_t *attr, void *fcn )  {
  if(DEBUG)  printf( "%s ", tmr->name );

  // Declare thread priority
  struct sched_param param;
  param.sched_priority = tmr->prio;

  // Assign thread priority and attributes
  if( pthread_attr_setschedparam( attr, &param ) )
    printf( "Error (thr_create): Failed to set '%s' priority. \n", tmr->name );

  // Create thread
  if( pthread_create ( &tmr->id, attr, fcn, (void*)NULL ) )
    printf( "Error (thr_init): Failed to create '%s' thread. \n", tmr->name );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  tmr_exit
//  Cleanly exits the timing threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void tmr_exit ( void )  {
  printf("Close timing threads: ");

  // Exit rate gyro thread
  if( pthread_join ( tmr_gyr.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'gyr' thread. \n" );
  if(DEBUG)  printf( "gyr " );

  // Exit accelerometer thread
  if( pthread_join ( tmr_acc.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'acc' thread. \n" );
  if(DEBUG)  printf( "acc " );

  // Exit magnetometer thread
  if( pthread_join ( tmr_mag.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'mag' thread. \n" );
  if(DEBUG)  printf( "mag " );

  // Exit debugging thread
  if( pthread_join ( tmr_debug.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'debug' thread. \n" );
  if(DEBUG)  printf( "degub " );

  // Destroy mutex
  //pthread_mutex_destroy(&mutex_cal);
  //pthread_mutex_destroy(&mutex_fusion);
  if(DEBUG)  printf("\n");

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  tmr_create
//  Creates the timer file descriptor within a thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void tmr_create ( timer_struct *tmr )  {

  // Local variables
  uint sec;
  ulong nsec;
  struct itimerspec itval;

  // Create the timer
  tmr->fd = timerfd_create ( CLOCK_MONOTONIC, 0 );
  if( tmr->fd == -1 )  printf( "Error (tmr_create): Failed to create '%s' timer. \n", tmr->name );

  // Determine time values
  sec = tmr->per / 1000000;
  nsec = ( tmr->per % 1000000 ) * 1000;

  // Set interval duration
  itval.it_interval.tv_sec  = sec;
  itval.it_interval.tv_nsec = nsec;

  // Set start value
  itval.it_value.tv_sec  = 0;
  itval.it_value.tv_nsec = 200 * 1000 * 1000;

  // Enable the timer
  if( timerfd_settime ( tmr->fd, 0, &itval, NULL ) )
    printf( "Error (tmr_create): Failed to enable the '%s' timer. \n", tmr->name );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  tmr_pause
//  Implements the pause before starting the next loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void tmr_pause ( timer_struct *tmr )  {

  // Local variables
  unsigned long long missed;

  // Wait for timer event and obtain number of "missed" loops
  if( read( tmr->fd, &missed, sizeof(missed) ) == -1 )
    printf( "Error (tmr_pause): Failed to read '%s' timer file. \n", tmr->name );

  // Play around with the "missed" feature some more...
  //if ( missed > 0 )  {  tmr->missed += (missed - 1);  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  tmr_start
//  Start code for a timer loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void tmr_start ( timer_struct *tmr )  {

  // Get current time
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );

  // Assign start time to thread
  tmr->start_sec  = timeval.tv_sec;
  tmr->start_usec = timeval.tv_nsec / 1000;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  tmr_finish
//  Finish code for a timer loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void tmr_finish ( timer_struct *tmr )  {

  // Get current time
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );

  // Assign finish time to thread
  tmr->finish_sec  = timeval.tv_sec;
  tmr->finish_usec = timeval.tv_nsec /1000;

  // Adjust for rollover
  if ( tmr->finish_sec == tmr->start_sec )  tmr->dur = 0;
  else                                      tmr->dur = 1000000;

  // Calculate timing metrics
  tmr->dur += tmr->finish_usec - tmr->start_usec;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_gyr
//  Function handler for the rate gyro timing thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *fcn_gyr (  )  {
  tmr_create(&tmr_gyr);
  while (running) {
    tmr_start(&tmr_gyr);
    printf("g"); fflush(stdout);
    //imu_gyr(&imu);
    tmr_finish(&tmr_gyr);
    //log_write(LOG_GYR);
    tmr_pause(&tmr_gyr);
  }
  pthread_exit(NULL);
  return NULL;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_acc
//  Function handler for the accelerometer timing thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *fcn_acc (  )  {
  tmr_create(&tmr_acc);
  while (running) {
    tmr_start(&tmr_acc);
    printf("a"); fflush(stdout);
    //imu_acc(&imu);
    tmr_finish(&tmr_acc);
    //log_write(LOG_ACC);
    tmr_pause(&tmr_acc);
  }
  pthread_exit(NULL);
  return NULL;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_mag
//  Function handler for the magnetometer timing thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *fcn_mag (  )  {
  tmr_create(&tmr_mag);
  while (running) {
    tmr_start(&tmr_mag);
    printf("m"); fflush(stdout);
    //imu_mag(&imu);
    tmr_finish(&tmr_mag);
    //log_write(LOG_MAG);
    tmr_pause(&tmr_mag);
  }
  pthread_exit(NULL);
  return NULL;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_debug
//  Function handler for the debugging timing thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *fcn_debug (  )  {
  tmr_create(&tmr_debug);
  while (running) {
    tmr_start(&tmr_debug);
    printf("d"); fflush(stdout);
    //sys_debug();
    tmr_finish(&tmr_debug);
    tmr_pause(&tmr_debug);
  }
  pthread_exit(NULL);
  return NULL;
}



