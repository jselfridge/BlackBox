
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
  /*
  // Loop counter
  ushort i;

  // Time values
  printf("\r");  fflush(stdout);
  if (datalog.enabled)  printf(" %s: ", datalog.dir );
  else                  printf(" - - - -  ");

  float timestamp = (float)( thr_debug.start_sec + ( thr_debug.start_usec / 1000000.0f ) - datalog.offset );
  printf("%6.1f    ", timestamp );  fflush(stdout);
  //printf("%09ld  ", t.start_nano );  fflush(stdout);
  //printf("%09ld  ", t.dur );  fflush(stdout);
  //printf("%4.2f  ", t.percent );  fflush(stdout);
  //if (t.percent<1.0) {  printf("_    ");  fflush(stdout);  }
  //else               {  printf("X    ");  fflush(stdout);  }

  // Input/Output values
  //for ( i=0; i<6; i++ )  printf("%06.1f ", sys.input[i]  );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%06.1f ", sys.output[i] );  printf("   ");

  // Raw sensor values - IMU1
  //for ( i=0; i<3; i++ )  printf("%06d ", imu1.rawGyr[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%06d ", imu1.rawAcc[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%04d ", imu1.rawMag[i] );  printf("   ");

  // Raw sensor values - IMU2
  //for ( i=0; i<3; i++ )  printf("%06d ", imu2.rawGyr[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%04d ", imu2.rawMag[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%06d ", imu2.rawAcc[i] );  printf("   ");

  // Filtered sensor values - IMU1
  //for ( i=0; i<3; i++ )  printf("%09.2f ", imu1.avgGyr[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%09.2f ", imu1.avgAcc[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%07.2f ", imu1.avgMag[i] );  printf("   ");

  // Filtered sensor values - IMU2
  //for ( i=0; i<3; i++ )  printf("%09.2f ", imu2.avgGyr[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%09.2f ", imu2.avgAcc[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%07.2f ", imu2.avgMag[i] );  printf("   ");

  // Calibrated sensor values - IMU1
  //pthread_mutex_lock(&mutex_cal);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", imu1.calGyr[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%6.3f ", imu1.calAcc[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%6.3f ", imu1.calMag[i] );  printf("   ");
  //pthread_mutex_unlock(&mutex_cal);

  // Calibrated sensor values - IMU2
  //for ( i=0; i<3; i++ )  printf("%7.4f ", imu2.calGyr[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%7.4f ", imu2.calAcc[i] );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%7.4f ", imu2.calMag[i] );  printf("   ");

  // Data fusion values - IMU1
  pthread_mutex_lock(&mutex_fusion);
  for ( i=0; i<4; i++ )  printf("%6.3f ", imu1.Quat[i]              );  printf("   ");
  for ( i=0; i<3; i++ )  printf("%6.1f ", imu1.Eul[i]  *(180.0f/PI) );  printf("   ");
  for ( i=0; i<3; i++ )  printf("%6.1f ", imu1.dEul[i] *(180.0f/PI) );  printf("   ");
  pthread_mutex_unlock(&mutex_fusion);

  // Data fusion values - IMU2
  //for ( i=0; i<4; i++ )  printf("%6.3f ", imu2.Quat[i]              );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%6.3f ", imu2.dQuat[i]             );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%6.1f ", imu2.Eul[i]  *(180.0f/PI) );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%6.1f ", imu2.dEul[i] *(180.0f/PI) );  printf("   ");

  // Data fusion convergence
  //for ( i=0; i<4; i++ )  printf("%6.3f ", imu1.Prev[i] );  printf("   ");  

  // Control values
  //printf( "%6.3f      ", ctrl.heading*(180.0/PI) );
  //for ( i=0; i<3; i++ )  printf( "%05.2f ", ctrl.err[X][i]*(180.0f/PI) );  printf("   ");
  //for ( i=0; i<3; i++ )  printf( "%05.2f ", ctrl.err[Y][i]*(180.0f/PI) );  printf("   ");
  //for ( i=0; i<3; i++ )  printf( "%05.2f ", ctrl.err[Z][i]*(180.0f/PI) );  printf("   ");
  //for ( i=0; i<3; i++ )  printf( "%07.2f ", ctrl.input[i] *(180.0f/PI) );  printf("   ");

  // Finish print loop
  printf("    ");
  fflush(stdout);
  */
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
  //tmr_exit();
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



