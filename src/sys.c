
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

  // Setup exit condition
  struct sigaction sys_run;
  if(DEBUG)  printf("  Setting system exit condition \n");
  memset( &sys_run, 0, sizeof(sys_run) );
  sys_run.sa_handler = &sys_flag;
  if( sigaction( SIGINT, &sys_run, NULL ) == -1 )
    printf( "Error (sys_init): Function 'sigaction' failed. \n" );

  // Establish realtime priority
  struct sched_param sp;
  printf("  Establishing realtime priority \n");
  sp.sched_priority = 98;
  if( sched_setscheduler( 0, SCHED_FIFO, &sp ) == -1 )
    printf( "Error (sys_init): Function 'sched_setscheduler' failed. \n" );

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

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_debug
//  Prints system debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_debug ( thread_struct* thr )  {

  // Loop counter
  //static ushort i;

  // Datalog file
  printf("\r");  fflush(stdout);
  //if (datalog.enabled)  printf(" %s: ", datalog.dir );
  //else                  printf(" - - - -  ");

  // Time values
  float timestamp = (float)( thr->start_sec + ( thr->start_usec / 1000000.0f ) ); //- datalog.offset );
  printf("%6.1f    ", timestamp );  fflush(stdout);
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

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_flag
//  Change boolean flag to signal the end of the program.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
  return;
}



