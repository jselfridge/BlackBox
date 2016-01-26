
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

  // Local variables
  pthread_attr_t attr;

  // Run through setup functions
  tmr_setup();
  tmr_attr(&attr);

  // Begin each thread and mutex
  if(DEBUG)  printf("  Create threads and mutexes: ");

  // Create primary timing threads
  tmr_thread( &tmr_imu, &attr, fcn_imu );  pthread_mutex_init( &mutex_imu, NULL );
  tmr_thread( &tmr_sio, &attr, fcn_sio );  pthread_mutex_init( &mutex_sio, NULL );

  // Possibly create debugging thread
  if(DEBUG) {
    tmr_thread( &tmr_debug, &attr, fcn_debug );
    printf("\n");
  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  tmr_setup
//  Assign timing thread parameters to a data structure.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void tmr_setup ( void )  {
  if(DEBUG)  printf("  Assign thread structure elements \n");

  // IMU timer
  tmr_imu.name     =  "imu";
  tmr_imu.prio     =  PRIO_IMU;
  tmr_imu.per      =  1000000 / HZ_IMU_FAST;

  // SIO timer
  tmr_sio.name     =  "sio";
  tmr_sio.prio     =  PRIO_SIO;
  tmr_sio.per      =  1000000 / HZ_SIO;

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

  // Exit imu thread
  pthread_mutex_destroy(&mutex_imu);
  if( pthread_join ( tmr_imu.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'imu' thread. \n" );
  if(DEBUG)  printf( "imu " );

  // Exit system input/output thread
  pthread_mutex_destroy(&mutex_sio);
  if( pthread_join ( tmr_sio.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'sio' thread. \n" );
  if(DEBUG)  printf( "sio " );

  // Exit debugging thread
  if(DEBUG) {
  if( pthread_join ( tmr_debug.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'debug' thread. \n" );
  printf( "debug \n" );
  }

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
//  fcn_imu
//  Function handler for the imu timing thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *fcn_imu (  )  {
  tmr_create(&tmr_imu);
  while (running) {
    tmr_start(&tmr_imu);
    imu_data();
    tmr_finish(&tmr_imu);
    log_record(LOG_IMU);
    tmr_pause(&tmr_imu);
  }
  pthread_exit(NULL);
  return NULL;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_sio
//  Function handler for the system input/output timing thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *fcn_sio (  )  {
  tmr_create(&tmr_sio);
  while (running) {
    tmr_start(&tmr_sio);
    sio_update();
    tmr_finish(&tmr_sio);
    log_record(LOG_SIO);
    tmr_pause(&tmr_sio);
  }
  pthread_exit(NULL);
  return NULL;
}

void sio_update() {
  sio_input();
  ushort i;
  for ( i=0; i<10; i++ ) {
    output.reg[i] = input.reg[i] + 1000;
  }
  sio_output();
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_debug
//  Function handler for the debugging timing thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *fcn_debug (  )  {
  tmr_create(&tmr_debug);
  while (running) {
    tmr_start(&tmr_debug);
    sys_debug();
    tmr_finish(&tmr_debug);
    tmr_pause(&tmr_debug);
  }
  pthread_exit(NULL);
  return NULL;
}



