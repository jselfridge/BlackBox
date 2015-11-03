
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

  // Stabilization
  thr_stab.pin       =     60;
  thr_stab.period    = 200000;
  thr_stab.priority  =     90;

  // Navigation
  thr_nav.pin        =     50;
  thr_nav.period     =   5000;
  thr_nav.priority   =     80;

  // Telemetry
  thr_telem.pin      =     51;
  thr_telem.period   =  10000;
  thr_telem.priority =     70;

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

  // Initialize 'stabilization' thread
  param.sched_priority = thr_stab.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'stab' priority." );
  sys.ret = pthread_create ( &thr_stab.id, &attr, thread_stab, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'stab' thread." );

  // Initialize 'navigation' thread
  param.sched_priority = thr_nav.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'nav' priority." );
  sys.ret = pthread_create ( &thr_nav.id, &attr, thread_nav, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'nav' thread." );

  // Initialize 'telemetry' thread
  param.sched_priority = thr_telem.priority;
  sys.ret = pthread_attr_setschedparam( &attr, &param );
  sys_err( sys.ret, "Error (thread_init): Failed to set 'telem' priority." );
  sys.ret = pthread_create ( &thr_telem.id, &attr, thread_telem, (void *)NULL );
  sys_err( sys.ret, "Error (thread_init): Failed to create 'telem' thread." );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_periodic
//  Establishes the periodic attributes for a thread.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thread_periodic ( thread_struct *thr, int *fd )  {

  //--- Debug ---
  printf("FD (perodic_before): %d \n", *fd);

  // Local variables
  unsigned int sec, nsec;
  struct itimerspec itval;

  // Create the timer
  *fd = timerfd_create ( CLOCK_MONOTONIC, 0 );
  sys_err( *fd == -1, "Error (thread_periodic): Failed to create timer." );

  //--- Debug ---
  printf("FD (perodic_after): %d \n", *fd);

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
  sys.ret = timerfd_settime ( *fd, 0, &itval, NULL );
  sys_err( sys.ret, "Error (thread_periodic): Failed to enable the timer." );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_pause
//  Implements the pause before starting the next loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thread_pause ( thread_struct *thr, int *fd )  {

  // Local variables
  ulong missed;

  //--- Debug ---
  printf("FD (pause): %d \n", *fd);

  // Wait for timer event and obtain number of "missed" loops
  //sys.ret = read( 3, &missed, sizeof(missed) );
  sys.ret = read( *fd, &missed, sizeof(missed) );
  //sys_err( sys.ret == -1, "Error (thread_pause): Failed to read timer file.");

  // Play around with the "missed" feature some more...
  //if ( thr->missed > 0 )  {  thr->missed += (missed - 1);  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  thread_exit
//  Cleanly exits the threads.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thread_exit ( )  {
  printf("Closing threads  \n");
  void *status;

  // Exit 'stabilization' thread
  sys.ret = pthread_join ( thr_stab.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'stab' thread." );
  if(DEBUG)  printf( "  Status %ld for 'stabilization' thread \n", (long)status );

  // Exit 'navigation' thread
  sys.ret = pthread_join ( thr_nav.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'nav' thread." );
  if(DEBUG)  printf( "  Status %ld for 'navigation' thread \n", (long)status );

  // Exit 'telemetry' thread
  sys.ret = pthread_join ( thr_telem.id, &status );
  sys_err( sys.ret, "Error (thread_exit): Failed to exit 'telem' thread." );
  if(DEBUG)  printf( "  Status %ld for 'telemetry' thread \n", (long)status );

  return;
}


//----------------------------------------
void *thread_stab ( )  {
  printf("  Running 'stabilization' thread \n");
  //struct loop_info signal_info;
  //int fd, len;
  //char buf[64];
  //int period = 500;

  //--- Debug ---
  int FD = 12;
  printf("FD (stab_before): %d \n", FD);
  thread_periodic ( &thr_stab, &FD );
  printf("FD (stab_after): %d \n", FD);


  while (sys.running) {
    led_on(0);
    usleep(100000);

    //len = snprintf( buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", 60 );
    //if( len <=0 )  {  printf("Error (gpio_set_val): Failed to assign path.");  }
    //fd = open( buf, O_WRONLY );
    //if( fd <0 )  {  printf("Error (gpio_set_val): Failed to open file.");  }
    //if (on) { write( fd, "0", 2 ); on=0; }
    //else    { write( fd, "1", 2 ); on=1; }
    //close(fd);

    printf("  Another 'stabilization' loop \n");
    led_off(0);
    //usleep(1000000);
    thread_pause ( &thr_stab, &FD );
  }

  //pthread_exit(NULL);
  printf("  Finished 'stabilization' thread \n");
  return NULL;
}


//----------------------------------------
void *thread_nav ( )  {
  printf("  Running 'navigation' thread \n");
  //struct loop_info signal_info;
  //int fd, len;
  //char buf[64];
  //int period = 500;
  //loop_init ( period, &signal_info );
  //while (running) {
    //len = snprintf( buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", 60 );
    //if( len <=0 )  {  printf("Error (gpio_set_val): Failed to assign path.");  }
    //fd = open( buf, O_WRONLY );
    //if( fd <0 )  {  printf("Error (gpio_set_val): Failed to open file.");  }
    //if (on) { write( fd, "0", 2 ); on=0; }
    //else    { write( fd, "1", 2 ); on=1; }
    //close(fd);
    //loop_wait (&signal_info);
  //}
  //pthread_exit(NULL);
  printf("  Finished 'navigation' thread \n");
  return NULL;
}


//----------------------------------------
void *thread_telem ( )  {
  printf("  Running 'telemetry' thread \n");
  //struct loop_info signal_info;
  //int fd, len;
  //char buf[64];
  //int period = 500;
  //loop_init ( period, &signal_info );
  //while (running) {
    //len = snprintf( buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", 60 );
    //if( len <=0 )  {  printf("Error (gpio_set_val): Failed to assign path.");  }
    //fd = open( buf, O_WRONLY );
    //if( fd <0 )  {  printf("Error (gpio_set_val): Failed to open file.");  }
    //if (on) { write( fd, "0", 2 ); on=0; }
    //else    { write( fd, "1", 2 ); on=1; }
    //close(fd);
    //loop_wait (&signal_info);
  //}
  //pthread_exit(NULL);
  printf("  Finished 'telemetry' thread \n");
  return NULL;
}



