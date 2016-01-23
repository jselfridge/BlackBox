
//============================================================
//  sys.c
//  Justin M Selfridge
//============================================================
#include "sys.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_init
//  Initializes the system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_init (  )  {
  if(DEBUG)  printf("Initializing system \n");
  running = true;

<<<<<<< HEAD
  // Setup exit condition
=======
  // Establish exit condition
  if(DEBUG)  printf("  Set system exit condition \n");
>>>>>>> altlog
  struct sigaction sys_run;
  running = true;
  memset( &sys_run, 0, sizeof(sys_run) );
<<<<<<< HEAD
  sys_run.sa_handler = &sys_flag;
=======
  sys_run.sa_handler = &sys_exit;
>>>>>>> altlog
  if( sigaction( SIGINT, &sys_run, NULL ) == -1 )
    printf( "Error (sys_init): Function 'sigaction' failed. \n" );

  // Establish realtime priority
<<<<<<< HEAD
  struct sched_param sp;
  printf("  Establishing realtime priority \n");
=======
  if(DEBUG)  printf("  Establish realtime priority \n");
  struct sched_param sp;
>>>>>>> altlog
  sp.sched_priority = 98;
  if( sched_setscheduler( 0, SCHED_FIFO, &sp ) == -1 )
    printf( "Error (sys_init): Function 'sched_setscheduler' failed. \n" );

<<<<<<< HEAD
  // Lock current and future memroy
  printf("  Locking current and future memory \n");
  if( mlockall( MCL_CURRENT | MCL_FUTURE ) )
    printf( "Error (sys_init): Failed to lock memory." );
  mallopt( M_TRIM_THRESHOLD, -1 );
  mallopt( M_MMAP_MAX, 0 );

  // Prefault the memory stack
  long i; 
  char *buffer;
  printf("  Prefaulting memory stack \n");
  buffer = malloc(SYS_STACK);
  for ( i=0; i<SYS_STACK; i += sysconf(_SC_PAGESIZE) )  buffer[i] = 0;
  free(buffer);
=======
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
>>>>>>> altlog

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_debug
//  Prints system debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
<<<<<<< HEAD
void sys_debug ( tmr_struct* tmr_debug, sensor_struct* gyr_sensor )  {

  // Loop counter
  ushort i;

  // Datalog file
  printf("\r");  fflush(stdout);
  //if (datalog.enabled)  printf(" %s: ", datalog.dir );
  //else                  printf(" - - - -  ");

  // Time values
  float timestamp = (float)( tmr_debug->start_sec + ( tmr_debug->start_usec / 1000000.0f ) ); //- datalog.offset );
  printf("%6.1f    ", timestamp );  fflush(stdout);

  // Debugging sensor values
  for ( i=0; i<3; i++ )  printf("%06d ", gyr_sensor->raw[i] );  printf("   ");
  for ( i=0; i<3; i++ )  printf("%6.3f ", gyr_sensor->calib[i] );  printf("   ");
  

  /*
  // System Input/Output values
  //pthread_mutex_lock(&mutex_sysio);
  //for ( i=0; i<4; i++ )  printf("%04d ", sys.input[i]  );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%04d ", sys.output[i] );  printf("   ");
  //pthread_mutex_unlock(&mutex_sysio);

  // Raw sensor values
  //for ( i=0; i<3; i++ )  printf("%06d ", imu1.rawGyr[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%06d ", imu1.rawAcc[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%04d ", imu1.rawMag[i] );  printf("   ");

  // Filtered sensor values
  //for ( i=0; i<3; i++ )  printf("%09.2f ", imu1.avgGyr[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%09.2f ", imu1.avgAcc[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%07.2f ", imu1.avgMag[i] );  printf("   ");

  // Calibrated sensor values
  //pthread_mutex_lock(&mutex_imu);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", imu1.calGyr[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%6.3f ", imu1.calAcc[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%6.3f ", imu1.calMag[i] );  printf("   ");
  //pthread_mutex_unlock(&mutex_imu);

  // Data fusion values
  //pthread_mutex_lock(&mutex_fusion);
  //for ( i=0; i<4; i++ )  printf("%6.3f ", imu1.Quat[i]              );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%6.1f ", imu1.Eul[i]  *(180.0f/PI) );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%6.1f ", imu1.dEul[i] *(180.0f/PI) );  printf("   ");
  //pthread_mutex_unlock(&mutex_fusion);

  // Control values
  //printf( "%6.3f      ", ctrl.heading*(180.0/PI) );
  //for ( i=0; i<4; i++ )  printf( "%5.2f ", ctrl.norm[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf( "%05.2f ", ctrl.err[X][i]*(180.0f/PI) );  printf("   ");
  //for ( i=0; i<3; i++ )  printf( "%05.2f ", ctrl.err[Y][i]*(180.0f/PI) );  printf("   ");
  //for ( i=0; i<3; i++ )  printf( "%05.2f ", ctrl.err[Z][i]*(180.0f/PI) );  printf("   ");
  //for ( i=0; i<3; i++ )  printf( "%07.2f ", ctrl.input[i] *(180.0f/PI) );  printf("   ");
  */

  // Finish print loop
  fflush(stdout);
=======
void sys_debug ( void )  {

  // Start debugging display
  printf("\r");  fflush(stdout);

  // Datalog status
  if (datalog.enabled)  printf(" Log %s: ", datalog.dir );
  else                  printf(" - - - -  ");
  fflush(stdout);

  // Time values
  float timestamp = (float) ( tmr_debug.start_sec + ( tmr_debug.start_usec / 1000000.0f ) ); //- datalog.offset );
  printf("%6.1f    ", timestamp );  fflush(stdout);

  // Select data for display
  pthread_mutex_lock(&mutex_imu);    sys_imu();    pthread_mutex_unlock(&mutex_imu);

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
>>>>>>> altlog

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_flag
//  Change boolean flag to signal the end of the program.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
<<<<<<< HEAD
void sys_flag (  )  {
  running = false;
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_exit
//  Code that runs prior to exiting the system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_exit (  )  {
  if(DEBUG)  printf("Program complete \n");
  if( sigaction( SIGINT, &sys_signal, NULL ) == -1 )
    printf( "Error (sys_exit): Function 'sigaction' failed." );
  kill( 0, SIGINT );
=======
void sys_exit (  )  {

  // Change exit status
  running = false;
  usleep(200000);

  // Exit subsystems
  if(DEBUG)  printf("\n\n--- Exit BlackBox program --- \n");
  usleep(100000);
  tmr_exit();
  imu_exit();
  log_exit();

  // Under development
  //pru_exit();

  // Shut everything down
  if(DEBUG)  printf("Program complete \n");
  led_off(LED_IMU);  led_off(LED_PRU);  led_off(LED_LOG);  led_off(LED_MOT);
  if( sigaction( SIGINT, &sys_signal, NULL ) == -1 )
    printf( "Error (sys_exit): Function 'sigaction' failed. \n" );
  kill( 0, SIGINT );

>>>>>>> altlog
  return;
}



