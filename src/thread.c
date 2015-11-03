
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

  // DEBUGGING
  //int value;
  //printf( "SCHED_FIFO:  %d \n", SCHED_FIFO  );
  //sys.ret = pthread_attr_getschedpolicy( &attr, &value );
  //printf("BEFORE:  ret: %d   state: %d \n", sys.ret, value );

  // Local variables
  pthread_attr_t attr;
  struct sched_param param;

  // Stabilization (400Hz)
  thr_stab.pin       =     60;
  thr_stab.period    =   2500;
  thr_stab.priority  =     90;
  thr_stab.missed    =      0;

  // Navigation (100Hz)
  thr_nav.pin        =      0;
  thr_nav.period     =  10000;
  thr_nav.priority   =     80;
  thr_nav.missed     =      0;

  // Telemetry (10Hz)
  thr_telem.pin      =      0;
  thr_telem.period   = 100000;
  thr_telem.priority =     70;
  thr_telem.missed   =      0;

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

  // DEBUGGING
  //sys.ret = pthread_attr_getschedpolicy( &attr, &value );
  //printf("AFTER:  ret: %d   state: %d \n", sys.ret, value );

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
  return NULL;
}



