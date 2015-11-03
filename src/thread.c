
//============================================================
//  thread.c
//  Justin M Selfridge
//============================================================
#include "thread.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_init
//  Initializes the various threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thread_init ( void )  {
  if(DEBUG)  printf("Initializing threads \n");

  // Local variables
  pthread_attr_t attr;
  struct sched_param param;

  // Gyroscope
  thr_gyro.priority  =     99;
  thr_gyro.pin       =     60;
  thr_gyro.period    =   5000;

  // Accelerometer
  thr_acc.priority   =     99;
  thr_acc.pin        =     50;
  thr_acc.period     =   5000;

  // Magnetometer
  thr_mag.priority   =     98;
  thr_mag.pin        =     48;
  thr_mag.period     =  20000;

  // Stabilization
  //thr_stab.priority  =     XX;
  //thr_stab.pin       =     XX;
  //thr_stab.period    =  XXXXX;

  // Navigation
  //thr_nav.priority   =     XX;
  //thr_nav.pin        =     XX;
  //thr_nav.period     =  XXXXX;

  // Telemetry
  //thr_telem.priority =     XX;
  //thr_telem.pin      =     XX;
  //thr_telem.period   =  XXXXX;

  // Debugging
  thr_debug.priority =     94;
  thr_debug.pin      =     00;
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

  // Initialize 'gyroscope' thread
  param.sched_priority = thr_gyro.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'gyro' priority." );
  sys.ret = pthread_create ( &thr_gyro.id, &attr, thread_gyro, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'gyro' thread." );

  // Initialize 'accelerometer' thread
  param.sched_priority = thr_acc.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'acc' priority." );
  sys.ret = pthread_create ( &thr_acc.id, &attr, thread_acc, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'acc' thread." );

  // Initialize 'magnetometer' thread
  param.sched_priority = thr_mag.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'mag' priority." );
  sys.ret = pthread_create ( &thr_mag.id, &attr, thread_mag, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'mag' thread." );

  // Initialize 'stabilization' thread
  //param.sched_priority = thr_stab.priority;
  //sys.ret = pthread_attr_setschedparam( &attr, &param );
  //sys_err( sys.ret, "Error (thread_init): Failed to set 'stab' priority." );
  //sys.ret = pthread_create ( &thr_stab.id, &attr, thread_stab, (void *)NULL );
  //sys_err( sys.ret, "Error (thread_init): Failed to create 'stab' thread." );

  // Initialize 'navigation' thread
  //param.sched_priority = thr_nav.priority;
  //sys.ret = pthread_attr_setschedparam( &attr, &param );
  //sys_err( sys.ret, "Error (thread_init): Failed to set 'nav' priority." );
  //sys.ret = pthread_create ( &thr_nav.id, &attr, thread_nav, (void *)NULL );
  //sys_err( sys.ret, "Error (thread_init): Failed to create 'nav' thread." );

  // Initialize 'telemetry' thread
  //param.sched_priority = thr_telem.priority;
  //sys.ret = pthread_attr_setschedparam( &attr, &param );
  //sys_err( sys.ret, "Error (thread_init): Failed to set 'telem' priority." );
  //sys.ret = pthread_create ( &thr_telem.id, &attr, thread_telem, (void *)NULL );
  //sys_err( sys.ret, "Error (thread_init): Failed to create 'telem' thread." );

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
//  thread_periodic
//  Establishes the periodic attributes for a thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thread_periodic ( thread_struct *thr )  {

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
//  thread_pause
//  Implements the pause before starting the next loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thread_pause ( thread_struct *thr )  {

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
//  thread_start
//  Start code for a thread loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thread_start ( thread_struct *thr )  {

  // Get current time
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );

  // Assign start time to thread
  thr->start_sec  = timeval.tv_sec;
  thr->start_usec = timeval.tv_nsec / 1000;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_finish
//  Finish code for a thread loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thread_finish ( thread_struct *thr )  {

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
//  thread_exit
//  Cleanly exits the threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thread_exit ( void )  {
  printf("Closing threads  \n");
  void *status;

  // Exit 'gyroscope' thread
  sys.ret = pthread_join ( thr_gyro.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'gyro' thread." );
  if(DEBUG)  printf( "  Status %ld for 'gyroscope' thread \n", (long)status );

  // Exit 'accelerometer' thread
  sys.ret = pthread_join ( thr_acc.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'acc' thread." );
  if(DEBUG)  printf( "  Status %ld for 'accelerometer' thread \n", (long)status );

  // Exit 'magnetometer' thread
  sys.ret = pthread_join ( thr_mag.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'mag' thread." );
  if(DEBUG)  printf( "  Status %ld for 'magnetometer' thread \n", (long)status );

  // Exit 'stabilization' thread
  //sys.ret = pthread_join ( thr_stab.id, &status );
  //sys_err( sys.ret, "Error (thread_exit): Failed to exit 'stab' thread." );
  //if(DEBUG)  printf( "  Status %ld for 'stabilization' thread \n", (long)status );

  // Exit 'navigation' thread
  //sys.ret = pthread_join ( thr_nav.id, &status );
  //sys_err( sys.ret, "Error (thread_exit): Failed to exit 'nav' thread." );
  //if(DEBUG)  printf( "  Status %ld for 'navigation' thread \n", (long)status );

  // Exit 'telemetry' thread
  //sys.ret = pthread_join ( thr_telem.id, &status );
  //sys_err( sys.ret, "Error (thread_exit): Failed to exit 'telem' thread." );
  //if(DEBUG)  printf( "  Status %ld for 'telemetry' thread \n", (long)status );

  // Exit 'debug' thread
  if(DEBUG) {
  sys.ret = pthread_join ( thr_debug.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'debug' thread." );
  printf( "  Status %ld for 'debug' thread \n", (long)status );
  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_gyro
//  Run the 'gyroscope' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *thread_gyro ( )  {
  printf("  Running 'gyroscope' thread \n");
  gpio_export(thr_gyro.pin);
  gpio_set_dir( thr_gyro.pin, OUTPUT_PIN );
  thread_periodic (&thr_gyro);
  while (sys.running) {
    thread_start(&thr_gyro);
    gpio_set_val( thr_gyro.pin, HIGH );
    imu_gyro(&imu1);
    gpio_set_val( thr_gyro.pin, LOW );
    thread_finish(&thr_gyro);
    log_write(LOG_GYRO);
    thread_pause(&thr_gyro);
  }
  pthread_exit(NULL);
  return NULL;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_acc
//  Run the 'accelerometer' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *thread_acc ( )  {
  printf("  Running 'accelerometer' thread \n");
  gpio_export(thr_acc.pin);
  gpio_set_dir( thr_acc.pin, OUTPUT_PIN );
  thread_periodic (&thr_acc);
  while (sys.running) {
    thread_start(&thr_acc);
    gpio_set_val( thr_acc.pin, HIGH );
    imu_acc(&imu1);
    gpio_set_val( thr_acc.pin, LOW );
    thread_finish(&thr_acc);
    log_write(LOG_ACC);
    thread_pause(&thr_acc);
  }
  pthread_exit(NULL);
  return NULL;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_mag
//  Run the 'magnetomter' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *thread_mag ( )  {
  printf("  Running 'magnetometer' thread \n");
  gpio_export(thr_mag.pin);
  gpio_set_dir( thr_mag.pin, OUTPUT_PIN );
  thread_periodic (&thr_mag);
  while (sys.running) {
    thread_start(&thr_mag);
    gpio_set_val( thr_mag.pin, HIGH );
    imu_mag(&imu1);
    gpio_set_val( thr_mag.pin, LOW );
    thread_finish(&thr_mag);
    log_write(LOG_MAG);
    thread_pause(&thr_mag);
  }
  pthread_exit(NULL);
  return NULL;
}

/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_stab
//  Run the 'stabilization' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *thread_stab ( )  {
  printf("  Running 'stabilization' thread \n");
  gpio_export(thr_stab.pin);
  gpio_set_dir( thr_stab.pin, OUTPUT_PIN );
  thread_periodic (&thr_stab);
  while (sys.running) {
    thread_start(&thr_stab);
    gpio_set_val( thr_stab.pin, HIGH );

    //--- FUNCTION ---//
    imu_raw(&imu1);
    //usleep(10);
    //--- FUNCTION ---//

    gpio_set_val( thr_stab.pin, LOW );
    thread_finish(&thr_stab);
    log_write();
    thread_pause(&thr_stab);
  }
  pthread_exit(NULL);
  return NULL;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_nav
//  Run the 'navigation' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *thread_nav ( )  {
  printf("  Running 'navigation' thread \n");
  gpio_export(thr_nav.pin);
  gpio_set_dir( thr_nav.pin, OUTPUT_PIN );
  thread_periodic (&thr_nav);
  while (sys.running) {
    gpio_set_val( thr_nav.pin, HIGH );
    usleep(5000);
    gpio_set_val( thr_nav.pin, LOW );
    thread_pause(&thr_nav);
  }
  pthread_exit(NULL);
  return NULL;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_telem
//  Run the 'telemetry' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *thread_telem ( )  {
  printf("  Running 'telemetry' thread \n");
  gpio_export(thr_telem.pin);
  gpio_set_dir( thr_telem.pin, OUTPUT_PIN );
  thread_periodic (&thr_telem);
  while (sys.running) {
    gpio_set_val( thr_telem.pin, HIGH );
    usleep(5000);
    gpio_set_val( thr_telem.pin, LOW );
    thread_pause(&thr_telem);
  }
  pthread_exit(NULL);
  return NULL;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_debug
//  Run the 'magnetomter' thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void *thread_debug ( )  {
  printf("  Running 'debug' thread \n");
  thread_periodic (&thr_debug);
  while (sys.running) {
    thread_start(&thr_debug);

    usleep(10000);
    sys_debug();

    thread_finish(&thr_debug);
    thread_pause(&thr_debug);
  }
  pthread_exit(NULL);
  return NULL;
}



