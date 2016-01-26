
//============================================================
//  log.c
//  Justin M Selfridge
//============================================================
#include "log.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_init
//  Initalizes the datalog structure attributes.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_init ( void )  {
  if(DEBUG)  printf("Initializing log parameters \n");

  // DEBUGGING PLACEMENT (enabled by flags in the future)
  datalog.enabled = true;

  // Set counters to zero
  if(DEBUG)  printf("  Clear counters \n");
  log_gyr.count    = 0;
  log_acc.count    = 0;
  log_mag.count    = 0;
  log_input.count  = 0;
  log_output.count = 0;

  // Establish datalog limits
  if(DEBUG)  printf("  Establish datalog limits \n");
  log_gyr.limit    = MAX_LOG_DUR * HZ_IMU_FAST;
  log_acc.limit    = MAX_LOG_DUR * HZ_IMU_FAST;
  log_mag.limit    = MAX_LOG_DUR * HZ_IMU_SLOW;
  log_input.limit  = MAX_LOG_DUR * HZ_SIO;
  log_output.limit = MAX_LOG_DUR * HZ_SIO;

  // Allocate memory for 'gyr' storage arrays
  if(DEBUG)  printf("  Allocate 'gyr' mem \n");
  log_gyr.time =  malloc( sizeof(float) * log_gyr.limit     );
  log_gyr.dur  =  malloc( sizeof(ulong) * log_gyr.limit     );
  log_gyr.raw  =  malloc( sizeof(short) * log_gyr.limit * 3 );
  log_gyr.avg  =  malloc( sizeof(float) * log_gyr.limit * 3 );
  log_gyr.cal  =  malloc( sizeof(float) * log_gyr.limit * 3 );

  // Allocate memory for 'acc' storage arrays
  if(DEBUG)  printf("  Allocate 'acc' mem \n");
  log_acc.time =  malloc( sizeof(float) * log_acc.limit     );
  log_acc.dur  =  malloc( sizeof(ulong) * log_acc.limit     );
  log_acc.raw  =  malloc( sizeof(short) * log_acc.limit * 3 );
  log_acc.avg  =  malloc( sizeof(float) * log_acc.limit * 3 );
  log_acc.cal  =  malloc( sizeof(float) * log_acc.limit * 3 );

  // Allocate memory for 'mag' storage arrays
  if(DEBUG)  printf("  Allocate 'mag' mem \n");
  log_mag.time =  malloc( sizeof(float) * log_mag.limit     );
  log_mag.dur  =  malloc( sizeof(ulong) * log_mag.limit     );
  log_mag.raw  =  malloc( sizeof(short) * log_mag.limit * 3 );
  log_mag.avg  =  malloc( sizeof(float) * log_mag.limit * 3 );
  log_mag.cal  =  malloc( sizeof(float) * log_mag.limit * 3 );

  // Allocate memory for 'input' storage arrays
  if(DEBUG)  printf("  Allocate 'input' mem \n");
  log_input.time =  malloc( sizeof(float)  * log_input.limit      );
  log_input.reg  =  malloc( sizeof(ushort) * log_input.limit * 10 );
  log_input.pwm  =  malloc( sizeof(ushort) * log_input.limit * 10 );
  log_input.norm =  malloc( sizeof(double) * log_input.limit * 10 );

  // Allocate memory for 'output' storage arrays
  if(DEBUG)  printf("  Allocate 'output' mem \n");
  log_output.time =  malloc( sizeof(float)  * log_output.limit      );
  log_output.reg  =  malloc( sizeof(ushort) * log_output.limit * 10 );
  log_output.pwm  =  malloc( sizeof(ushort) * log_output.limit * 10 );
  log_output.norm =  malloc( sizeof(double) * log_output.limit * 10 );

  // Allocate dir/path/file memory
  datalog.dir  = malloc(16);
  datalog.path = malloc(32);
  char *file   = malloc(64);

  // Find next available log directory
  ushort i = 0;
  while (true) {
    i++;
    if      ( i<10   )  sprintf( datalog.dir, "00%d", i );
    else if ( i<100  )  sprintf( datalog.dir, "0%d",  i );
    else if ( i<1000 )  sprintf( datalog.dir, "%d",   i );
    else    printf( "Error (log_XX): Exceeded maximum number of log directories. \n" );
    sprintf( file, "../Log/%s/notes.txt", datalog.dir );
    if ( access( file , F_OK ) == -1 )  break;
  }

  // Determine start second
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );
  datalog.offset = timeval.tv_sec;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_exit
//  Closes the data log files.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_exit ( void )  {
  if(DEBUG)  printf("Close log files \n");
  led_blink( LED_LOG, 500, 500 );

  // Local ariables
  char *file = malloc(64);
  FILE *fnote, *fgyr, *facc, *fmag, *fin, *fout;
  ushort i;
  ulong row;

  // Create new directory
  sprintf( datalog.path, "../Log/%s/", datalog.dir );
  mkdir( datalog.path, 222 );

  // Create notes datalog file
  sprintf( file, "%snotes.txt", datalog.path );
  fnote = fopen( file, "w" );
  if( fnote == NULL )  printf( "Error (log_XX): Cannot open 'notes' file. \n" );
  fprintf( fnote, " Assign some system parameteres like gains, or telemetry waypoint updates... " );

  // Create gyroscope datalog file
  sprintf( file, "%sgyr.txt", datalog.path );
  fgyr = fopen( file, "w" );
  if( fgyr == NULL )  printf( "Error (log_XX): Cannot open 'gyr' file. \n" );
  fprintf( fgyr,
    "       Gtime    Gdur   \
    Grx     Gry     Grz       \
    Gax        Gay        Gaz     \
    Gcx      Gcy      Gcz      ");

  // Loop through gyroscope data
  for ( row = 0; row < log_gyr.count; row++ ) {
    fprintf( fgyr, "\n %011.6f  %06d    ", log_gyr.time[row], log_gyr.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( fgyr, "%06d  ",   log_gyr.raw[ row*3 +i ] );   fprintf( fgyr, "   " );
    for ( i=0; i<3; i++ )  fprintf( fgyr, "%09.2f  ", log_gyr.avg[ row*3 +i ] );   fprintf( fgyr, "   " );
    for ( i=0; i<3; i++ )  fprintf( fgyr, "%07.4f  ", log_gyr.cal[ row*3 +i ] );   fprintf( fgyr, "   " );
  }

  // Create accelerometer datalog file
  sprintf( file, "%sacc.txt", datalog.path );
  facc = fopen( file, "w" );
  if( facc == NULL )  printf( "Error (log_XX): Cannot open 'acc' file. \n" );
  fprintf( facc, 
    "       Atime    Adur   \
    Arx     Ary     Arz       \
    Aax        Aay        Aaz     \
    Acx      Acy      Acz      ");

  // Loop through accelerometer data
  for ( row = 0; row < log_acc.count; row++ ) {
    fprintf( facc, "\n %011.6f  %06d    ", log_acc.time[row], log_acc.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( facc, "%06d  ",   log_acc.raw[ row*3 +i ] );   fprintf( facc, "   " );
    for ( i=0; i<3; i++ )  fprintf( facc, "%09.2f  ", log_acc.avg[ row*3 +i ] );   fprintf( facc, "   " );
    for ( i=0; i<3; i++ )  fprintf( facc, "%07.4f  ", log_acc.cal[ row*3 +i ] );   fprintf( facc, "   " );
  }

  // Create magnetometer datalog file
  sprintf( file, "%smag.txt", datalog.path );
  fmag = fopen( file, "w" );
  if( fmag == NULL )  printf( "Error (log_XX): Cannot open 'mag' file. \n" );
  fprintf( fmag,
    "       Mtime    Mdur   \
    Mrx     Mry     Mrz       \
    Max        May        Maz     \
    Mcx      Mcy      Mcz      ");

  // Loop through magnetometer data
  for ( row = 0; row < log_mag.count; row++ ) {
    fprintf( fmag, "\n %011.6f  %06d    ", log_mag.time[row], log_mag.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( fmag, "%06d  ",   log_mag.raw[ row*3 +i ] );   fprintf( fmag, "   " );
    for ( i=0; i<3; i++ )  fprintf( fmag, "%09.2f  ", log_mag.avg[ row*3 +i ] );   fprintf( fmag, "   " );
    for ( i=0; i<3; i++ )  fprintf( fmag, "%07.4f  ", log_mag.cal[ row*3 +i ] );   fprintf( fmag, "   " );
  }

  // Create input datalog file
  sprintf( file, "%sinput.txt", datalog.path );
  fin = fopen( file, "w" );
  if( fin == NULL )  printf( "Error (log_XX): Cannot open 'input' file. \n" );
  fprintf( fin, "       Itime      I0    I1    I2    I3    I4    I5    I6    I7    I8    I9");

  // Loop through input data
  for ( row = 0; row < log_input.count; row++ ) {
    fprintf( fin, "\n %011.6f    ", log_input.time[row] );
    //for ( i=0; i<4; i++ )  fprintf( fin, "%05d  ",  log_input.reg  [ row*10 +i ] );   fprintf( fin, "   " );
    for ( i=0; i<4; i++ )  fprintf( fin, "%04d  ",  log_input.pwm  [ row*10 +i ] );   fprintf( fin, "   " );
    //for ( i=0; i<4; i++ )  fprintf( fin, "%7.4f  ", log_input.norm [ row*10 +i ] );   fprintf( fin, "   " );
  }

  // Create output datalog file
  sprintf( file, "%soutput.txt", datalog.path );
  fout = fopen( file, "w" );
  if( fout == NULL )  printf( "Error (log_XX): Cannot open 'input' file. \n" );
  fprintf( fout, "       Otime      O0    O1    O2    O3    O4    O5    O6    O7    O8    O9");

  // Loop through output data
  for ( row = 0; row < log_output.count; row++ ) {
    fprintf( fout, "\n %011.6f    ", log_output.time[row] );
    //for ( i=0; i<4; i++ )  fprintf( fout, "%05d  ",  log_output.reg  [ row*10 +i ] );   fprintf( fout, "   " );
    for ( i=0; i<4; i++ )  fprintf( fout, "%04d  ",  log_output.pwm  [ row*10 +i ] );   fprintf( fout, "   " );
    //for ( i=0; i<4; i++ )  fprintf( fout, "%7.4f  ", log_output.norm [ row*10 +i ] );   fprintf( fout, "   " );
  }

  // Close files
  fclose(fnote);
  fclose(fgyr);
  fclose(facc);
  fclose(fmag);
  fclose(fin);
  fclose(fout);

  // Free memory for 'gyr'
  if(DEBUG)  printf("  Free 'gyr' memory \n");
  free(log_gyr.time);
  free(log_gyr.dur);
  free(log_gyr.raw);
  free(log_gyr.avg);
  free(log_gyr.cal);

  // Free memory for 'acc'
  if(DEBUG)  printf("  Free 'acc' memory \n");
  free(log_acc.time);
  free(log_acc.dur);
  free(log_acc.raw);
  free(log_acc.avg);
  free(log_acc.cal);

  // Free memory for 'mag'
  if(DEBUG)  printf("  Free 'mag' memory \n");
  free(log_mag.time);
  free(log_mag.dur);
  free(log_mag.raw);
  free(log_mag.avg);
  free(log_mag.cal);

  // Free memory for 'input'
  if(DEBUG)  printf("  Free 'input' memory \n");
  free(log_input.time);
  free(log_input.reg);
  free(log_input.pwm);
  free(log_input.norm);

  // Free memory for 'output'
  if(DEBUG)  printf("  Free 'output' memory \n");
  free(log_output.time);
  free(log_output.reg);
  free(log_output.pwm);
  free(log_output.norm);

  led_off( LED_LOG );
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_record
//  Records the data to the log file.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_record ( enum log_index index )  {

  // Local variables
  ushort i;
  ulong  row;
  float  timestamp;

  // Jump to appropriate log 
  switch(index) {


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Record IMU data
  case LOG_IMU :

    timestamp = (float) ( tmr_imu.start_sec + ( tmr_imu.start_usec / 1000000.0f ) ) - datalog.offset;
    pthread_mutex_lock(&mutex_imu);
    
    // Gyroscope data
    if ( log_gyr.count <= log_gyr.limit ) {
      row = log_gyr.count;
      log_gyr.time[row] = timestamp;
      log_gyr.dur[row]  = tmr_imu.dur;
      for ( i=0; i<3; i++ )  log_gyr.raw[ row*3 +i ] = gyr.raw[i];
      for ( i=0; i<3; i++ )  log_gyr.avg[ row*3 +i ] = gyr.avg[i];
      for ( i=0; i<3; i++ )  log_gyr.cal[ row*3 +i ] = gyr.cal[i];
      log_gyr.count++;
    }

    // Accelerometer data
    if ( log_acc.count <= log_acc.limit ) {
      row = log_acc.count;
      log_acc.time[row] = timestamp;
      log_acc.dur[row]  = tmr_imu.dur;
      for ( i=0; i<3; i++ )  log_acc.raw[ row*3 +i ] = acc.raw[i];
      for ( i=0; i<3; i++ )  log_acc.avg[ row*3 +i ] = acc.avg[i];
      for ( i=0; i<3; i++ )  log_acc.cal[ row*3 +i ] = acc.cal[i];
      log_acc.count++;
    }

    // Magnetometer data
    if( imu.getmag && ( log_mag.count < log_mag.limit) ) {
      row = log_mag.count;
      log_mag.time[row] = timestamp;
      log_mag.dur[row]  = tmr_imu.dur;
      for ( i=0; i<3; i++ )  log_mag.raw[ row*3 +i ] = mag.raw[i];
      for ( i=0; i<3; i++ )  log_mag.avg[ row*3 +i ] = mag.avg[i];
      for ( i=0; i<3; i++ )  log_mag.cal[ row*3 +i ] = mag.cal[i];
      log_mag.count++;
    }

    pthread_mutex_unlock(&mutex_imu);
    return;


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Record AHRS data
  case LOG_AHRS :
    return;


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Record system input/output data
  case LOG_SIO :

    timestamp = (float) ( tmr_sio.start_sec + ( tmr_sio.start_usec / 1000000.0f ) ) - datalog.offset;
    pthread_mutex_lock(&mutex_sio);

    // Input data
    if ( log_input.count <= log_input.limit ) {
      row = log_input.count;
      log_input.time[row] = timestamp;
      //for ( i=0; i<4; i++ )  log_input.reg  [ row*10 +i ] = input.reg[i];
      for ( i=0; i<4; i++ )  log_input.pwm  [ row*10 +i ] = input.pwm[i];
      //for ( i=0; i<4; i++ )  log_input.norm [ row*10 +i ] = input.norm[i];
      log_input.count++;
    }

    // Output data
    if ( log_output.count <= log_output.limit ) {
      row = log_output.count;
      log_output.time[row] = timestamp;
      //for ( i=0; i<4; i++ )  log_output.reg  [ row*10 +i ] = output.reg[i];
      for ( i=0; i<4; i++ )  log_output.pwm  [ row*10 +i ] = output.pwm[i];
      //for ( i=0; i<4; i++ )  log_output.norm [ row*10 +i ] = output.norm[i];
      log_output.count++;
    }

    pthread_mutex_unlock(&mutex_sio);
    return;

  default :
    return;
  
  }
}



