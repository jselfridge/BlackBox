
//============================================================
//  sys.c
//  Justin M Selfridge
//============================================================
#include "sys.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_init
//  Initializes the system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_init ( void )  {
  if(DEBUG)  printf("Initializing system \n");

  // Establish exit condition
  if(DEBUG)  printf("  Set system exit condition \n");
  struct sigaction sys_run;
  running = true;
  memset( &sys_run, 0, sizeof(sys_run) );
  sys_run.sa_handler = &sys_exit;
  if( sigaction( SIGINT, &sys_run, NULL ) == -1 )
    printf( "Error (sys_init): Function 'sigaction' failed. \n" );

  // Establish realtime priority
  if(DEBUG)  printf("  Establish realtime priority \n");
  struct sched_param sp;
  sp.sched_priority = 98;
  if( sched_setscheduler( 0, SCHED_FIFO, &sp ) == -1 )
    printf( "Error (sys_init): Function 'sched_setscheduler' failed. \n" );

  // Prefault memory stack
  if(DEBUG)  printf("  Prefault memory stack \n");
  int i;
  char *buffer;   
  buffer = malloc(SYS_STACK);
  for ( i=0; i<SYS_STACK; i += sysconf(_SC_PAGESIZE) )  {  buffer[i] = 0;  }
  free(buffer);

  // Lock and reserve memory
  if(DEBUG)  printf("  Lock and reserve memory \n");
  if ( mlockall( MCL_CURRENT | MCL_FUTURE ) )
    printf( "Error (sys_init): Failed to lock memory. \n" );
  mallopt( M_TRIM_THRESHOLD, -1 );
  mallopt( M_MMAP_MAX, 0 );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_debug
//  Prints system debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_debug ( void )  {

  // Start debugging display
  printf("\r");  fflush(stdout);

  // Datalog status
  if (datalog.enabled)  printf(" Log %s: ", datalog.dir );
  else                  printf(" - - - -  ");
  fflush(stdout);

  // Time values
  float timestamp = (float) ( tmr_debug.start_sec + ( tmr_debug.start_usec / 1000000.0f ) - datalog.offset );
  printf("%6.1f    ", timestamp );  fflush(stdout);

  // Select data for display
//pthread_mutex_lock(&mutex_imu);    sys_imu();    pthread_mutex_unlock(&mutex_imu);
  pthread_mutex_lock(&mutex_sio);    sys_sio();    pthread_mutex_unlock(&mutex_sio);

  // Complete debugging display 
  printf("  "); fflush(stdout);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_imu
//  Prints IMU debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_imu ( void )  {
  ushort i;

  // Gyroscope data
  //for ( i=0; i<3; i++ )  printf("%06d ",   gyr.raw[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%09.2f ", gyr.avg[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%06.3f ", gyr.cal[i] );  printf("   ");  fflush(stdout);

  // Accelerometer data
  //for ( i=0; i<3; i++ )  printf("%06d ",   acc.raw[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%09.2f ", acc.avg[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%06.3f ", acc.cal[i] );  printf("   ");  fflush(stdout);

  // Magnetometer data
  //for ( i=0; i<3; i++ )  printf("%04d ",   mag.raw[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%07.2f ", mag.avg[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%06.3f ", mag.cal[i] );  printf("   ");  fflush(stdout);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_sio
//  Prints system input/output values to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_sio ( void )  {
  ushort i;

  // Input signals
  //for ( i=0; i<10; i++ )  printf("%05d ",   input.reg[i]  );  printf("   ");  fflush(stdout);
  for ( i=0; i<10; i++ )  printf("%04d ",   input.pwm[i]  );  printf("   ");  fflush(stdout);
  //for ( i=0; i<10; i++ )  printf("%07.4f ", input.norm[i] );  printf("   ");  fflush(stdout);

  // Output signals
  //for ( i=0; i<10; i++ )  printf("%05d ",   output.reg[i]  );  printf("   ");  fflush(stdout);
  //for ( i=0; i<10; i++ )  printf("%04d ",   output.pwm[i]  );  printf("   ");  fflush(stdout);
  //for ( i=0; i<10; i++ )  printf("%07.4f ", output.norm[i] );  printf("   ");  fflush(stdout);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_exit
//  Code that runs prior to exiting the system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_exit (  )  {

  // Change exit status
  running = false;
  usleep(200000);

  // Exit subsystems
  if(DEBUG)  printf("\n\n--- Exit BlackBox program --- \n");
  usleep(100000);
  tmr_exit();
  sio_exit();
  imu_exit();
  log_exit();

  // Shut everything down
  if(DEBUG)  printf("Program complete \n");
  led_off(LED_IMU);  led_off(LED_PRU);  led_off(LED_LOG);  led_off(LED_MOT);
  if( sigaction( SIGINT, &sys_signal, NULL ) == -1 )
    printf( "Error (sys_exit): Function 'sigaction' failed. \n" );
  kill( 0, SIGINT );

  return;
}



