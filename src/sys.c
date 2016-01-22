
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

  // Loop counter
  ushort i;

  // Time values
  printf("\r");  fflush(stdout);
  //if (datalog.enabled)  printf(" %s: ", datalog.dir );
  //else                  printf(" - - - -  ");

  float timestamp = (float)( tmr_debug.start_sec + ( tmr_debug.start_usec / 1000000.0f ) ); //- datalog.offset );
  printf("%6.1f    ", timestamp );  fflush(stdout);

  // Gyroscope data
  pthread_mutex_lock(&gyr_mutex);
  for ( i=0; i<3; i++ )  printf("%06d ",   gyr.raw[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%09.2f ", gyr.avg[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%06.3f ", gyr.cal[i] );  printf("   ");
  pthread_mutex_unlock(&gyr_mutex);

  // Accelerometer data
  pthread_mutex_lock(&acc_mutex);
  for ( i=0; i<3; i++ )  printf("%06d ",   acc.raw[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%09.2f ", acc.avg[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%06.3f ", acc.cal[i] );  printf("   ");
  pthread_mutex_unlock(&acc_mutex);

  // Magnetometer data
  pthread_mutex_lock(&mag_mutex);
  for ( i=0; i<3; i++ )  printf("%04d ",   mag.raw[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%07.2f ", mag.avg[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%06.3f ", mag.cal[i] );  printf("   ");
  pthread_mutex_unlock(&mag_mutex);

  // Complete debugging display 
  printf("  "); fflush(stdout);

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
  imu_exit();

  // Under development
  //pru_exit();
  //log_exit();  //--- DEBUG ---//

  // Shut everything down
  if(DEBUG)  printf("Program complete \n");
  led_off(LED_IMU);  led_off(LED_PRU);  led_off(LED_LOG);  led_off(LED_MOT);
  if( sigaction( SIGINT, &sys_signal, NULL ) == -1 )
    printf( "Error (sys_exit): Function 'sigaction' failed. \n" );
  kill( 0, SIGINT );

  return;
}



