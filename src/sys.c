
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
  sys_imu();
  sys_ahr();
  //sys_sio();

  // Complete debugging display 
  printf("  "); fflush(stdout);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_imu
//  Prints IMU debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_imu ( void )  {

  // Loop counter
  ushort i;

  /*// Raw data
  pthread_mutex_lock(&mutex_raw);
  for ( i=0; i<3; i++ )  printf("%6d ",   gyr.raw[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%6d ",   acc.raw[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%4d ",   mag.raw[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_raw); */

  /*// Averaged data
  pthread_mutex_lock(&mutex_avg);
  for ( i=0; i<3; i++ )  printf("%9.2f ", gyr.avg[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%9.2f ", acc.avg[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%7.2f ", mag.avg[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_avg); */

  // Calibrated data
  pthread_mutex_lock(&mutex_cal);
  for ( i=0; i<3; i++ )  printf("%6.3f ", gyr.cal[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", acc.cal[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", mag.cal[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_cal);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_ahr
//  Prints AHR debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_ahr ( void )  {

  // Loop counter
  ushort i;

  /*// Quaternion data
  pthread_mutex_lock(&mutex_quat);
  for ( i=0; i<4; i++ )  printf("%7.4f ", ahr.quat[i]  );  printf("   ");  fflush(stdout);
  for ( i=0; i<4; i++ )  printf("%7.4f ", ahr.dquat[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_quat);*/

  // Euler data
  pthread_mutex_lock(&mutex_eul);
  for ( i=0; i<3; i++ )  printf("%7.2f ", ahr.eul[i]  * (180.0/PI) );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%7.2f ", ahr.deul[i] * (180.0/PI) );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_eul); 

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_sio
//  Prints system input/output values to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_sio ( void )  {

  // Loop counter
  ushort i;

  // Input signals
  pthread_mutex_lock(&mutex_input);
  for ( i=0; i<4; i++ )  printf("%05d ",   input.reg[i]  );  printf("   ");  fflush(stdout);
  for ( i=0; i<4; i++ )  printf("%04d ",   input.pwm[i]  );  printf("   ");  fflush(stdout);
  for ( i=0; i<4; i++ )  printf("%07.4f ", input.norm[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_input);

  /*// Output signals
  pthread_mutex_lock(&mutex_output);
  for ( i=0; i<4; i++ )  printf("%05d ",   output.reg[i]  );  printf("   ");  fflush(stdout);
  for ( i=0; i<6; i++ )  printf("%04d ",   output.pwm[i]  );  printf("   ");  fflush(stdout);
  for ( i=0; i<4; i++ )  printf("%07.4f ", output.norm[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_output); */

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
  log_exit();
  ahr_exit();
  imu_exit();
  flg_exit();
  sio_exit();

  // Shut everything down
  if(DEBUG)  printf("Program complete \n");
  if( sigaction( SIGINT, &sys_signal, NULL ) == -1 )
    printf( "Error (sys_exit): Function 'sigaction' failed. \n" );
  kill( 0, SIGINT );

  return;
}



