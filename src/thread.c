
//============================================================
//  thread.c
//  Justin M Selfridge
//============================================================
#include "thread.h"


  // Mutex initialization
  //pthread_mutex_init( &mutex_imu,    NULL );
  //pthread_mutex_init( &mutex_fusion, NULL );
  //pthread_mutex_init( &mutex_sysio,  NULL );


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

/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_init
//  Initializes a new thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_init ( thread_struct* thr, pthread_attr_t* attr, void* fcn )  {
  if(DEBUG)  printf( "  Running '%s' \n", thr->name );

  // Declare thread priority
  struct sched_param param;
  param.sched_priority = thr->priority;

  // Assign thread priority and attributes
  if( pthread_attr_setschedparam( attr, &param ) )
    printf( "Error (thr_init): Failed to set '%s' priority. \n", thr->name );

  // Create thread
  int myint = 8;
  if( pthread_create ( &thr->id, attr, fcn, &myint ) )
    printf( "Error (thr_init): Failed to create '%s' thread. \n", thr->name );
  printf( "created \n" );

  return;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_periodic
//  Establishes the periodic attributes for a thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_periodic ( tmr_struct *tmr )  {

  // Local variables
  time_t sec;
  long   nsec;
  struct itimerspec itval;

  // Create the timer
  tmr->fd = timerfd_create ( CLOCK_MONOTONIC, 0 );
  if ( tmr->fd == -1 )
    printf( "Error (thread_periodic): Failed to create timer. \n" );

  // Determine time values
  sec  = tmr->period / 1000000;
  nsec = ( tmr->period % 1000000 ) * 1000;

  // Set interval duration
  itval.it_interval.tv_sec  = sec;
  itval.it_interval.tv_nsec = nsec;

  // Set start value
  itval.it_value.tv_sec  = 0;
  itval.it_value.tv_nsec = 200 * 1000 * 1000;

  // Enable the timer
  if( timerfd_settime ( tmr->fd, 0, &itval, NULL ) )
    printf( "Error (thread_periodic): Failed to enable the timer. \n" );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_pause
//  Implements the pause before starting the next loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_pause ( tmr_struct* tmr )  {

  // Local variables
  unsigned long long missed;

  // Wait for timer event and obtain number of "missed" loops
  if( read( tmr->fd, &missed, sizeof(missed) ) == -1 )
    printf( "Error (thr_pause): Failed to read timer file. \n" );

  // Play around with the "missed" feature some more...
  //if ( missed > 0 )  {  tmr->missed += (missed - 1);  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_start
//  Start code for a thread loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_start ( tmr_struct *tmr )  {

  // Get current time
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );

  // Assign start time to thread
  tmr->start_sec  = timeval.tv_sec;
  tmr->start_usec = timeval.tv_nsec / 1000;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_finish
//  Finish code for a thread loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_finish ( tmr_struct *tmr )  {

  // Get current time
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );

  // Assign finish time to thread
  tmr->finish_sec  = timeval.tv_sec;
  tmr->finish_usec = timeval.tv_nsec / 1000;

  // Adjust for rollover
  if ( tmr->finish_sec == tmr->start_sec )  tmr->dur = 0;
  else                                      tmr->dur = 1000000;

  // Calculate timing metrics
  tmr->dur += tmr->finish_usec - tmr->start_usec;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thr_exit
//  Cleanly exits the threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thr_exit ( tmr_struct* tmr )  {
  void *status;
  if( pthread_join ( tmr->id, &status ) )
    printf( "Error (thread_exit): Failed to exit '%s' thread. \n", tmr->name );
  if(DEBUG)  printf( "  Status %ld for '%s' thread \n", (long)status, tmr->name );
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_gyr
//  Function handler for the 'gyroscope' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void* fcn_gyr ( void* arg ) {
  if(DEBUG)  printf("  Running 'gyro' thread \n");

  // Demux argument pointers
  struct gyr_arg_struct* gyr_arg     = arg;
  struct sensor_struct*  gyr_sensor  = gyr_arg->gyr_sensor;
  struct tmr_struct*     gyr_tmr     = gyr_arg->gyr_tmr;

  // Execute timing loop
  thr_periodic(gyr_tmr);
  while (running) {
    thr_start(gyr_tmr);
    imu_gyr(gyr_sensor);

    float timestamp = (float)( gyr_tmr->start_sec + ( gyr_tmr->start_usec / 1000000.0f ) );
    fprintf( datalog, "%011.6f \n", timestamp );
    /*
    printf("%d %d %d    %f %f %f \n", \
      gyr_sensor->raw[0],   gyr_sensor->raw[1],   gyr_sensor->raw[2], \
      gyr_sensor->calib[0], gyr_sensor->calib[1], gyr_sensor->calib[2] );  fflush(stdout);
    */
    thr_finish(gyr_tmr);
    thr_pause(gyr_tmr);
  }

  // Exit timing thread
  pthread_exit(NULL);
  return NULL;
}

void imu_gyr ( sensor_struct* gyr_sensor ) {
  static short raw = 0;
  if ( raw >= 100 ) raw = 0;
  raw++;
  float calib = raw * 3.0;
  gyr_sensor->raw[0]    = raw;
  gyr_sensor->raw[1]    = raw + 10;
  gyr_sensor->raw[2]    = raw + 20;
  gyr_sensor->calib[0]  = calib;
  gyr_sensor->calib[1]  = calib + 100.0;
  gyr_sensor->calib[2]  = calib + 200.0;
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_debug
//  Function handler for the 'debug' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void* fcn_debug ( void* arg ) {
  if(DEBUG)  printf("  Running 'debug' thread \n");

  // Demux argument pointers
  struct debug_arg_struct* debug_arg   = arg;
  struct tmr_struct*       debug_tmr   = debug_arg->debug_tmr;
  struct sensor_struct*    gyr_sensor  = debug_arg->gyr_sensor;

  // Execute timing loop
  thr_periodic(debug_tmr);
  while (running) {
    thr_start(debug_tmr);
    sys_debug(debug_tmr,gyr_sensor);
    thr_finish(debug_tmr);
    thr_pause(debug_tmr);
  }

  // Exit timing thread
  pthread_exit(NULL);
  return NULL;
}












/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  fcn_imu
//  Function handler for the 'imu' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void* fcn_imu ( thread_struct* thr )  {
  if(DEBUG)  printf("  Is that an IMU... or are you happy to see me?... \n");

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
*/

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



