

#include "timer.h"
#include <limits.h>
#include <stdio.h>
#include <sys/timerfd.h>
#include <unistd.h>
//#include "adapt.h"
#include "ahrs.h"
//#include "ekf.h"
#include "flag.h"
#include "gcs.h"
//#include "gps.h"
#include "imu.h"
#include "io.h"
#include "log.h"
#include "stab.h"
#include "sys.h"


/**
 *  tmr_init
 *  Initializes the various timing thread functions.
 */
void tmr_init ( void )  {
  if(DEBUG)  printf("Initializing timing threads \n");
  pthread_attr_t attr;
  tmr_mutex();
  tmr_setup();
  tmr_attr(&attr);
  tmr_begin(&attr);
  return;
}


/**
 *  tmr_mutex
 *  Initializes the mutex locks for the shared data.
 */
void tmr_mutex ( void )  {
  if(DEBUG)  printf("  Establish mutex locks \n");
  pthread_mutex_init( &input.mutex,  NULL );
  pthread_mutex_init( &output.mutex, NULL );
  pthread_mutex_init( &mutex_i2c1,   NULL );
  pthread_mutex_init( &mutex_i2c2,   NULL );
  pthread_mutex_init( &imuA.mutex,   NULL );
  pthread_mutex_init( &gyrA.mutex,   NULL );
  pthread_mutex_init( &accA.mutex,   NULL );
  pthread_mutex_init( &magA.mutex,   NULL );
  pthread_mutex_init( &ahrsA.mutex,  NULL );
  pthread_mutex_init( &imuB.mutex,   NULL );
  pthread_mutex_init( &gyrB.mutex,   NULL );
  pthread_mutex_init( &accB.mutex,   NULL );
  pthread_mutex_init( &magB.mutex,   NULL );
  pthread_mutex_init( &ahrsB.mutex,  NULL );
  pthread_mutex_init( &stab.mutex,   NULL );
  pthread_mutex_init( &pidX.mutex,   NULL );
  pthread_mutex_init( &pidY.mutex,   NULL );
  pthread_mutex_init( &pidZ.mutex,   NULL );
  pthread_mutex_init( &gcs.mutex,    NULL );
  //pthread_mutex_init( &adaptR.mutex, NULL );
  //pthread_mutex_init( &adaptP.mutex, NULL );
  //pthread_mutex_init( &ekf.mutex,    NULL );
  //pthread_mutex_init( &gps.mutex,    NULL );
  return;
}


/**
 *  tmr_setup
 *  Assign timing thread parameters to a data structure.
 */
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
  tmr_imu.per  = 1000000 / HZ_IMU_FAST;

  // Stabilization timer
  tmr_stab.name = "stab";
  tmr_stab.prio = PRIO_STAB;
  tmr_stab.per  = 1000000 / HZ_STAB;

  // AHRS timer
  //tmr_ahrs.name = "ahrs";
  //tmr_ahrs.prio = PRIO_AHRS;
  //tmr_ahrs.per  = 1000000 / HZ_AHRS;

  // EKF timer
  //tmr_ekf.name  = "ekf";
  //tmr_ekf.prio  = PRIO_EKF;
  //tmr_ekf.per   = 1000000 / HZ_EKF;

  // GPS timer
  //tmr_gps.name = "gps";
  //tmr_gps.prio = PRIO_GPS;
  //tmr_gps.per  = 1000000 / HZ_GPS;

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


/**
 *  tmr_attr
 *  Sets the attributes of the timing threads.
 */
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


/**
 *  tmr_begin
 *  Function that begins each of the timing loop threads.
 */
void tmr_begin ( pthread_attr_t *attr )  {
  if(DEBUG)  printf("  Begin timing threads:  ");

  tmr_thread( &tmr_io,    attr, fcn_io    );  usleep(100000);
  tmr_thread( &tmr_flag,  attr, fcn_flag  );  usleep(100000);
  tmr_thread( &tmr_imu,   attr, fcn_imu   );  usleep(100000);
  tmr_thread( &tmr_stab,  attr, fcn_stab  );  usleep(100000);
  //tmr_thread( &tmr_ekf,   attr, fcn_ekf   );  usleep(100000);
  //tmr_thread( &tmr_gps,   attr, fcn_gps   );  usleep(100000);
  tmr_thread( &tmr_gcstx, attr, fcn_gcstx );  usleep(100000);
  tmr_thread( &tmr_gcsrx, attr, fcn_gcsrx );  usleep(100000);

  if(DEBUG) {
    tmr_thread( &tmr_debug, attr, fcn_debug );
    printf("\n\n");
  }

  return;
}


/**
 *  tmr_exit
 *  Cleanly exits the timing threads.
 */
void tmr_exit ( void )  {
  printf("Close timing threads:  ");

  // Destroy mutex locks
  pthread_mutex_destroy(&input.mutex);
  pthread_mutex_destroy(&output.mutex);
  pthread_mutex_destroy(&mutex_i2c1);
  pthread_mutex_destroy(&mutex_i2c2);
  pthread_mutex_destroy(&imuA.mutex);
  pthread_mutex_destroy(&gyrA.mutex);
  pthread_mutex_destroy(&accA.mutex);
  pthread_mutex_destroy(&magA.mutex);
  pthread_mutex_destroy(&ahrsA.mutex);
  pthread_mutex_destroy(&imuB.mutex);
  pthread_mutex_destroy(&gyrB.mutex);
  pthread_mutex_destroy(&accB.mutex);
  pthread_mutex_destroy(&magB.mutex);
  pthread_mutex_destroy(&ahrsB.mutex);
  pthread_mutex_destroy(&stab.mutex);
  pthread_mutex_destroy(&pidX.mutex);
  pthread_mutex_destroy(&pidY.mutex);
  pthread_mutex_destroy(&pidZ.mutex);
  pthread_mutex_destroy(&gcs.mutex);
  //pthread_mutex_destroy(&adaptR.mutex);
  //pthread_mutex_destroy(&adaptP.mutex);
  //pthread_mutex_destroy(&ekf.mutex);
  //pthread_mutex_destroy(&gps.mutex);

  // Exit GCSRX thread
  if( pthread_join ( tmr_gcsrx.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'gcsrx' thread. \n" );
  if(DEBUG)  printf( "gcsrx " );

  // Exit GCSTX thread
  if( pthread_join ( tmr_gcstx.id, NULL ) )
    printf( "Error (tmr_exit): Failed to exit 'gcstx' thread. \n" );
  if(DEBUG)  printf( "gcstx " );

  // Exit GPS thread
  //if( pthread_join ( tmr_gps.id, NULL ) )
  //  printf( "Error (tmr_exit): Failed to exit 'gps' thread. \n" );
  //if(DEBUG)  printf( "gps " );

  // Exit EKF thread
  //if( pthread_join ( tmr_ekf.id, NULL ) )
  //  printf( "Error (tmr_exit): Failed to exit 'ekf' thread. \n" );
  //if(DEBUG)  printf( "ekf " ); 

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


/**
 *  tmr_thread
 *  Create a new pthread to run the timer. 
 */
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


/**
 *  tmr_create
 *  Creates the timer file descriptor within a thread.
 */
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


/**
 *  tmr_pause
 *  Implements the pause before starting the next loop.
 */
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


/**
 *  tmr_start
 *  Start code for a timer loop.
 */
void tmr_start ( timer_struct *tmr )  {

  // Get current time
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );

  // Assign start time to thread
  tmr->start_sec  = timeval.tv_sec;
  tmr->start_usec = timeval.tv_nsec / 1000;

  return;
}


/**
 *  tmr_finish
 *  Finish code for a timer loop.
 */
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


/**
 *  fcn_io
 *  Function handler for the input/output timing thread.
 */
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


/**
 *  fcn_flag
 *  Function handler for the program execution flag timing thread.
 */
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


/**
 *  fcn_imu
 *  Function handler for dual IMU timing thread.
 */
void *fcn_imu (  )  {
  tmr_create(&tmr_imu);
  while (running) {
    tmr_start(&tmr_imu);
    if (!datalog.saving) {  
      if (IMUA_ENABLED)  {  imu_update(&imuA);  ahrs_update( &ahrsA, &imuA );  }
      if (IMUB_ENABLED)  {  imu_update(&imuB);  ahrs_update( &ahrsB, &imuB );  }
    }
    tmr_finish(&tmr_imu);
    if (datalog.enabled) {
      if (IMUA_ENABLED)  {  log_record(LOG_IMUA);  log_record(LOG_AHRSA);  }
      if (IMUB_ENABLED)  {  log_record(LOG_IMUB);  log_record(LOG_AHRSB);  }
    }
    tmr_pause(&tmr_imu);
  }
  pthread_exit(NULL);
  return NULL;
}


/**
 *  fcn_ahrs
 *  Function handler for the AHRS timing thread.
 */
/*
void *fcn_ahrs (  )  {
  tmr_create(&tmr_ahrs);
  while (running) {
    tmr_start(&tmr_ahrs);
    if (!datalog.saving) {
      if (IMUA_ENABLED)  ahrs_update( &ahrsA, &imuA );
      if (IMUB_ENABLED)  ahrs_update( &ahrsB, &imuB );
    }
    tmr_finish(&tmr_ahrs);
    if (datalog.enabled) {
      if (IMUA_ENABLED)  log_record(LOG_AHRSA);
      if (IMUB_ENABLED)  log_record(LOG_AHRSB);
    }
    tmr_pause(&tmr_ahrs);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

/**
 *  fcn_stab
 *  Function handler for the stabilization timing thread.
 */
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


/**
 *  fcn_ekf
 *  Function handler for the Extended Kalman Filter timing thread.
 */
/*
void *fcn_ekf (  )  {
  tmr_create(&tmr_ekf);
  while (running) {
    tmr_start(&tmr_ekf);
    if (!datalog.saving)  ekf_update();
    tmr_finish(&tmr_ekf);
    if (datalog.enabled)  log_record(LOG_EKF);
    tmr_pause(&tmr_ekf);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

/**
 *  fcn_gps
 *  Function handler for the GPS timing thread.
 */
/*
void *fcn_gps (  )  {
  tmr_create(&tmr_gps);
  while (running) {
    tmr_start(&tmr_gps);
    gps_update();
    tmr_finish(&tmr_gps);
    if (datalog.enabled)  log_record(LOG_GPS);
    tmr_pause(&tmr_gps);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

/**
 *  fcn_gcstx
 *  Function handler for the GCS transmission timing thread.
 */
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


/**
 *  fcn_gcsrx
 *  Function handler for the GCS receiver timing thread.
 */
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


/**
 *  fcn_debug
 *  Function handler for the debugging timing thread.
 */
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



