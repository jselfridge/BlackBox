

//#include "timer.h"
//#include <limits.h>
//#include <stdio.h>
//#include <sys/timerfd.h>
//#include <unistd.h>
//#include "ekf.h"
//#include "flag.h"
//#include "gcs.h"
//#include "imu.h"
//#include "ins.h"
//#include "io.h"
//#include "log.h"
//#include "stab.h"
//#include "sys.h"


/**
 *  tmr_init
 *  Initializes the various timing thread functions.
 */
/*
void tmr_init ( void )  {
  if(DEBUG)  printf("Initializing timing threads \n");
  pthread_attr_t attr;
  tmr_setup();
  tmr_attr(&attr);
  tmr_begin(&attr);
  return;
}
*/

/**
 *  tmr_setup
 *  Assign timing thread parameters to a data structure.
 */
/*
void tmr_setup ( void )  {
  if(DEBUG)  printf("  Assign thread structure elements \n");

  // Input/Output timer
  tmr_io.name = "io";
  tmr_io.prio = PRIO_IO;
  tmr_io.per  = 1000000 / HZ_IO;

  // Flags timer
  tmr_flag.name = "flag";
  tmr_flag.prio = PRIO_FLAG;
  tmr_flag.per  = 1000000 / HZ_FLAG;

  // IMU timer
  tmr_imu.name = "imu";
  tmr_imu.prio = PRIO_IMU;
  tmr_imu.per  = 1000000 / HZ_IMU;

  // Stabilization timer
  tmr_stab.name = "stab";
  tmr_stab.prio = PRIO_STAB;
  tmr_stab.per  = 1000000 / HZ_STAB;

  // Inertial Navigation System timer
  tmr_ins.name = "ins";
  tmr_ins.prio = PRIO_INS;
  tmr_ins.per  = 1000000 / HZ_INS;

  // GCSTX timer
  tmr_gcstx.name = "gcstx";
  tmr_gcstx.prio = PRIO_GCSTX;
  tmr_gcstx.per  = 1000000 / HZ_GCSTX;

  // GCSRX timer
  tmr_gcsrx.name = "gcsrx";
  tmr_gcsrx.prio = PRIO_GCSRX;
  tmr_gcsrx.per  = 1000000 / HZ_GCSRX;

  // Debugging timer
  tmr_debug.name = "debug";
  tmr_debug.prio = PRIO_DEBUG;
  tmr_debug.per  = 1000000 / HZ_DEBUG;

  return;
}
*/

/**
 *  tmr_attr
 *  Sets the attributes of the timing threads.
 */
/*
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
*/

/**
 *  tmr_begin
 *  Function that begins each of the timing loop threads.
 */
/*
void tmr_begin ( pthread_attr_t *attr )  {
  if(DEBUG)  printf("  Begin timing threads:  ");

  usleep(200000);  tmr_thread( &tmr_io,    attr, fcn_io    );
  usleep(200000);  tmr_thread( &tmr_flag,  attr, fcn_flag  );
  usleep(200000);  tmr_thread( &tmr_imu,   attr, fcn_imu   );
  usleep(200000);  tmr_thread( &tmr_stab,  attr, fcn_stab  );
  usleep(200000);  tmr_thread( &tmr_ins,   attr, fcn_ins   );
  //usleep(200000);  tmr_thread( &tmr_nav,   attr, fcn_nav   );
  usleep(200000);  tmr_thread( &tmr_gcstx, attr, fcn_gcstx );
  usleep(200000);  tmr_thread( &tmr_gcsrx, attr, fcn_gcsrx );

  if(DEBUG) {
    tmr_thread( &tmr_debug, attr, fcn_debug );
    printf("\n\n");
  }

  return;
}
*/

/**
 *  tmr_exit
 *  Cleanly exits the timing threads.
 */
/*
void tmr_exit ( void )  {
  printf("Close timing threads:  ");

  // Exit GCSRX thread
  if( pthread_join ( tmr_gcsrx.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'gcsrx' thread. \n" );
  if(DEBUG)  printf( "gcsrx " );

  // Exit GCSTX thread
  if( pthread_join ( tmr_gcstx.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'gcstx' thread. \n" );
  if(DEBUG)  printf( "gcstx " );

  // Exit navigation thread
  //if( pthread_join ( tmr_nav.id, NULL ) )
    //printf( "Error (tmr_exit): Failed to exit 'nav' thread. \n" );
  //if(DEBUG)  printf( "ins " );

  // Exit inertial nav sys thread
  if( pthread_join ( tmr_ins.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'ins' thread. \n" );
  if(DEBUG)  printf( "ins " );

  // Exit stabilization thread
  if( pthread_join ( tmr_stab.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'stab' thread. \n" );
  if(DEBUG)  printf( "stab " );

  // Exit IMU thread
  if( pthread_join ( tmr_imu.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'imu' thread. \n" );
  if(DEBUG)  printf( "imu " );

  // Exit program execution flags thread
  if( pthread_join ( tmr_flag.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'flag' thread. \n" );
  if(DEBUG)  printf( "flag " );

  // Exit input/output thread
  if( pthread_join ( tmr_io.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'sio' thread. \n" );
  if(DEBUG)  printf( "io " );

  // Exit debugging thread
  if(DEBUG) {
  if( pthread_join ( tmr_debug.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'debug' thread. \n" );
    printf( "debug \n" );
  }

  return;
}
*/

/**
 *  tmr_thread
 *  Create a new pthread to run the timer. 
 */
/*
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
*/

/**
 *  tmr_create
 *  Creates the timer file descriptor within a thread.
 */
/*
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
  itval.it_value.tv_nsec = 10 * 1000 * 1000;

  // Enable the timer
  if( timerfd_settime ( tmr->fd, 0, &itval, NULL ) )
    printf( "Error (tmr_create): Failed to enable the '%s' timer. \n", tmr->name );

  return;
}
*/

/**
 *  tmr_pause
 *  Implements the pause before starting the next loop.
 */
/*
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
*/

/**
 *  tmr_start
 *  Start code for a timer loop.
 */
/*
void tmr_start ( timer_struct *tmr )  {

  // Get current time
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );

  // Assign start time to thread
  tmr->start_sec  = timeval.tv_sec;
  tmr->start_usec = timeval.tv_nsec / 1000;

  return;
}
*/

/**
 *  tmr_finish
 *  Finish code for a timer loop.
 */
/*
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
*/

/**
 *  fcn_io
 *  Function handler for the input/output timing thread.
 */
/*
void *fcn_io (  )  {
  tmr_create(&tmr_io);
  while (running) {
    tmr_start(&tmr_io);
    io_update();
    tmr_finish(&tmr_io);
    if (datalog.enabled)  log_record(LOG_IO);
    tmr_pause(&tmr_io);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

/**
 *  fcn_flag
 *  Function handler for the program execution flag timing thread.
 */
/*
void *fcn_flag (  )  {
  tmr_create(&tmr_flag);
  while (running) {
    tmr_start(&tmr_flag);
    flag_update();
    tmr_finish(&tmr_flag);
    tmr_pause(&tmr_flag);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

/**
 *  fcn_imu
 *  Function handler for dual IMU timing thread.
 */
/*
void *fcn_imu (  )  {
  tmr_create(&tmr_imu);
  while (running) {
    tmr_start(&tmr_imu);
    if (!datalog.saving) {
      if (IMUA_ENABLED)  imu_update(&imuA);
      if (IMUB_ENABLED)  imu_update(&imuB);
      imu_state();
      ekf_update();
    }
    tmr_finish(&tmr_imu);
    if (datalog.enabled)  log_record(LOG_IMU);
    tmr_pause(&tmr_imu);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

/**
 *  fcn_stab
 *  Function handler for the stabilization timing thread.
 */
/*
void *fcn_stab (  )  {
  tmr_create(&tmr_stab);
  while (running) {
    tmr_start(&tmr_stab);
    stab_update();
    tmr_finish(&tmr_stab);
    if (datalog.enabled)  log_record(LOG_STAB);
    tmr_pause(&tmr_stab);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

/**
 *  fcn_ins
 *  Function handler for the inertial navigation system timing thread.
 */
/*
void *fcn_ins (  )  {
  tmr_create(&tmr_ins);
  while (running) {
    tmr_start(&tmr_ins);
    ins_update();
    ekf_gain();
    tmr_finish(&tmr_ins);
    if (datalog.enabled)  log_record(LOG_INS);
    tmr_pause(&tmr_ins);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

/**
 *  fcn_nav
 *  Function handler for the navigation timing thread.
 */
/*
void *fcn_nav (  )  {
  tmr_create(&tmr_nav);
  while (running) {
    tmr_start(&tmr_nav);
    nav_update();
    tmr_finish(&tmr_nav);
    if (datalog.enabled)  log_record(LOG_NAV);
    tmr_pause(&tmr_nav);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

/**
 *  fcn_gcstx
 *  Function handler for the GCS transmission timing thread.
 */
/*
void *fcn_gcstx (  )  {
  tmr_create(&tmr_gcstx);
  while (running) {
    tmr_start(&tmr_gcstx);
    gcs_tx();
    tmr_finish(&tmr_gcstx);
    tmr_pause(&tmr_gcstx);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

/**
 *  fcn_gcsrx
 *  Function handler for the GCS receiver timing thread.
 */
/*
void *fcn_gcsrx (  )  {
  tmr_create(&tmr_gcsrx);
  while (running) {
    tmr_start(&tmr_gcsrx);
    gcs_rx();
    tmr_finish(&tmr_gcsrx);
    tmr_pause(&tmr_gcsrx);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

/**
 *  fcn_debug
 *  Function handler for the debugging timing thread.
 */
/*
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
*/


