
//============================================================
//  sys.c
//  Justin M Selfridge
//============================================================
#include "sys.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_err
//  If error condition is true, prints a warning and exits.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_err ( bool cond, char* msg )  {
  if (cond) {  fprintf( stderr, "%s\n\n", msg );  exit(1);  }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_init
//  Initializes the system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_init ( void )  {
  if(DEBUG)  printf("Initializing system \n");

  // Initialize struct values
  sys.running = true;
  sys.ret = 0;

  // Establish exit condition
  struct sigaction sys_run;
  if(DEBUG)  printf("  Setting system exit condition \n");
  memset( &sys_run, 0, sizeof(sys_run) );
  sys_run.sa_handler = &sys_exit;
  sys.ret = sigaction( SIGINT, &sys_run, NULL );
  sys_err( sys.ret == -1, "Error (sys_init): Function 'sigaction' failed." );

  // Lock and reserve memory
  if(DEBUG)  printf("  Locking and reserving memory \n");
  sys.ret = mlockall( MCL_CURRENT | MCL_FUTURE );
  sys_err( sys.ret, "Error (sys_init): Failed to lock memory." );
  mallopt( M_TRIM_THRESHOLD, -1 );
  mallopt( M_MMAP_MAX, 0 );
  sys_memory(SYS_STACK);

  // Initialize subsystems
  imu_init();
  //pru_init();
  //ctrl_init();
  log_init();  //~~~ DEBUGGING ~~~//
  thr_init();

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_debug
//  Prints system debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_debug (  )  {

  // Loop counter
  ushort i;

  // Datalog file
  printf("\r");  fflush(stdout);
  if (datalog.enabled)  printf(" %s: ", datalog.dir );
  else                  printf(" - - - -  ");

  // Time values
  float timestamp = (float)( thr_debug.start_sec + ( thr_debug.start_usec / 1000000.0f ) - datalog.offset );
  printf("%6.1f    ", timestamp );  fflush(stdout);

  // System Input/Output values
  //pthread_mutex_lock(&mutex_sysio);
  //for ( i=0; i<4; i++ )  printf("%04d ", sys.input[i]  );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%04d ", sys.output[i] );  printf("   ");
  //pthread_mutex_unlock(&mutex_sysio);

  // Raw sensor values
  for ( i=0; i<3; i++ )  printf("%06d ", imu1.rawGyr[i] );  printf("   ");
  for ( i=0; i<3; i++ )  printf("%06d ", imu1.rawAcc[i] );  printf("   ");
  for ( i=0; i<3; i++ )  printf("%04d ", imu1.rawMag[i] );  printf("   ");

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

  // Finish print loop
  printf("    ");
  fflush(stdout);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_exit
//  Code that runs prior to exiting the system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_exit (  )  {
  sys.running = false;
  usleep(500000);
  if(DEBUG)  printf("\n\n--- Exit BlackBox program --- \n");
  log_exit();  //~~~ DEBUGGING ~~~//
  thr_exit();
  imu_exit();
  //pru_exit();
  //ctrl_exit();
  led_off(LED_IMU);  led_off(LED_PRU);  led_off(LED_LOG);  led_off(LED_MOT);
  if(DEBUG)  printf("Program complete \n");
  sys.ret = sigaction( SIGINT, &sys_signal, NULL );
  sys_err( sys.ret == -1, "Error (sys_exit): Function 'sigaction' failed." );
  kill( 0, SIGINT );
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_memory
//  Reserves a block of memory exclusively for the system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_memory ( int size )  {
  int i;
  char *buffer;   
  buffer = malloc(size);
  for ( i=0; i<size; i += sysconf(_SC_PAGESIZE) )  {  buffer[i] = 0;  }
  free(buffer);
  return;
}



