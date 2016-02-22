
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
  if(DEBUG)  printf("  Create threads and mutexes:  ");

  // Create mutex conditions
  pthread_mutex_init( &mutex_input,  NULL );
  pthread_mutex_init( &mutex_output, NULL );

  pthread_mutex_init( &mutex_gyrA,   NULL );
  pthread_mutex_init( &mutex_accA,   NULL );
  pthread_mutex_init( &mutex_magA,   NULL );

  pthread_mutex_init( &mutex_gyrB,   NULL );
  pthread_mutex_init( &mutex_accB,   NULL );
  pthread_mutex_init( &mutex_magB,   NULL );

  //pthread_mutex_init( &mutex_quat,   NULL );
  //pthread_mutex_init( &mutex_eul,    NULL );
  //pthread_mutex_init( &mutex_ctrl,   NULL );

  // Create primary timing threads
  tmr_thread( &tmr_sio,  &attr, fcn_sio  );  usleep(100000);
  tmr_thread( &tmr_flag, &attr, fcn_flag );  usleep(100000);
  if(USE_IMUA)  { tmr_thread( &tmr_imuA, &attr, fcn_imuA );  usleep(100000);  }
  if(USE_IMUB)  { tmr_thread( &tmr_imuB, &attr, fcn_imuB );  usleep(100000);  }
  //tmr_thread( &tmr_ahr,  &attr, fcn_ahr  );  usleep(100000);
  tmr_thread( &tmr_ctrl, &attr, fcn_ctrl );  usleep(100000);

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

  // System I/O timer
  tmr_sio.name    =  "sio";
  tmr_sio.prio    =  PRIO_SIO;
  tmr_sio.per     =  1000000 / HZ_SIO;

  // Flags timer
  tmr_flag.name   =  "flag";
  tmr_flag.prio   =  PRIO_FLAG;
  tmr_flag.per    =  1000000 / HZ_FLAG;

  // IMUA timer
  tmr_imuA.name   =  "imuA";
  tmr_imuA.prio   =  PRIO_IMU;
  tmr_imuA.per    =  1000000 / HZ_IMU_FAST;

  // IMUB timer
  tmr_imuB.name   =  "imuB";
  tmr_imuB.prio   =  PRIO_IMU;
  tmr_imuB.per    =  1000000 / HZ_IMU_FAST;

  /*// AHRS timer
  tmr_ahr.name    =  "ahr";
  tmr_ahr.prio    =  PRIO_AHR;
  tmr_ahr.per     =  1000000 / HZ_AHR;
  */
  // Control timer
  tmr_ctrl.name   =  "ctrl";
  tmr_ctrl.prio   =  PRIO_CTRL;
  tmr_ctrl.per    =  1000000 / HZ_CTRL;

  // Debugging timer
  tmr_debug.name  =  "debug";
  tmr_debug.prio  =  PRIO_DEBUG;
  tmr_debug.per   =  1000000 / HZ_DEBUG;

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
  printf("Close timing threads:  ");

  // Destroy mutex locks
  pthread_mutex_destroy(&mutex_input);
  pthread_mutex_destroy(&mutex_output);

  pthread_mutex_destroy(&mutex_gyrA);
  pthread_mutex_destroy(&mutex_accA);
  pthread_mutex_destroy(&mutex_magA);

  pthread_mutex_destroy(&mutex_gyrB);
  pthread_mutex_destroy(&mutex_accB);
  pthread_mutex_destroy(&mutex_magB);

  //pthread_mutex_destroy(&mutex_quat);
  //pthread_mutex_destroy(&mutex_eul);
  //pthread_mutex_destroy(&mutex_ctrl);

  // Exit control thread
  if( pthread_join ( tmr_ctrl.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'ctrl' thread. \n" );
  if(DEBUG)  printf( "ctrl " );

  /*// Exit AHR thread
  if( pthread_join ( tmr_ahr.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'ahr' thread. \n" );
  if(DEBUG)  printf( "ahr " );
  */
  // Exit IMUB thread
  if(USE_IMUB)  {
  if( pthread_join ( tmr_imuB.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'imuB' thread. \n" );
  if(DEBUG)  printf( "imuB " );  }

  // Exit IMUA thread
  if(USE_IMUA)  {
  if( pthread_join ( tmr_imuA.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'imuA' thread. \n" );
  if(DEBUG)  printf( "imuA " );  }

  // Exit program execution flags thread
  if( pthread_join ( tmr_flag.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'flag' thread. \n" );
  if(DEBUG)  printf( "flag " );

  // Exit system input/output thread
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
//  fcn_sio
//  Function handler for the system input/output timing thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *fcn_sio (  )  {
  tmr_create(&tmr_sio);
  while (running) {
    tmr_start(&tmr_sio);
    sio_update();
    tmr_finish(&tmr_sio);
    if (datalog.enabled)  log_record(LOG_SIO);
    tmr_pause(&tmr_sio);
  }
  pthread_exit(NULL);
  return NULL;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_flag
//  Function handler for the program execution flag timing thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *fcn_flag (  )  {
  tmr_create(&tmr_flag);
  while (running) {
    tmr_start(&tmr_flag);
    flg_update();
    tmr_finish(&tmr_flag);
    tmr_pause(&tmr_flag);
  }
  pthread_exit(NULL);
  return NULL;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_imuA
//  Function handler for the IMUA timing thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *fcn_imuA (  )  {
  tmr_create(&tmr_imuA);
  while (running) {
    tmr_start(&tmr_imuA);
    if (!datalog.saving)  imu_update(&imuA);
    tmr_finish(&tmr_imuA);
    if (datalog.enabled)  log_record(LOG_IMUA);
    tmr_pause(&tmr_imuA);
  }
  pthread_exit(NULL);
  return NULL;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_imuB
//  Function handler for the IMUB timing thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *fcn_imuB (  )  {
  tmr_create(&tmr_imuB);
  while (running) {
    tmr_start(&tmr_imuB);
    if (!datalog.saving)  imu_update(&imuB);
    tmr_finish(&tmr_imuB);
    if (datalog.enabled)  log_record(LOG_IMUB);
    tmr_pause(&tmr_imuB);
  }
  pthread_exit(NULL);
  return NULL;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_ahr
//  Function handler for the AHR timing thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void *fcn_ahr (  )  {
  tmr_create(&tmr_ahr);
  while (running) {
    tmr_start(&tmr_ahr);
    ahr_update();
    tmr_finish(&tmr_ahr);
    if (datalog.enabled)  log_record(LOG_AHR);
    tmr_pause(&tmr_ahr);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_ctrl
//  Function handler for the control law timing thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *fcn_ctrl (  )  {
  tmr_create(&tmr_ctrl);
  while (running) {
    tmr_start(&tmr_ctrl);
    ctl_update();
    tmr_finish(&tmr_ctrl);
    //if (datalog.enabled)  log_record(LOG_CTL);
    tmr_pause(&tmr_ctrl);
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
    sys_update();
    tmr_finish(&tmr_debug);
    tmr_pause(&tmr_debug);
  }
  pthread_exit(NULL);
  return NULL;
}



