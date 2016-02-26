
//============================================================
//  log.c
//  Justin M Selfridge
//============================================================
#include "log.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_init
//  Runs on start up to initalize the datalog attributes.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_init ( void )  {
  if(DEBUG)  printf("Initializing log parameters \n");

  // Establish datalog limits
  if(DEBUG)  printf("  Establish datalog limits \n");
  log_input.limit  = MAX_LOG_DUR * HZ_SIO;
  log_output.limit = MAX_LOG_DUR * HZ_SIO;
  log_gyrA.limit   = MAX_LOG_DUR * HZ_IMU_FAST;
  log_accA.limit   = MAX_LOG_DUR * HZ_IMU_FAST;
  log_magA.limit   = MAX_LOG_DUR * HZ_IMU_SLOW;
  log_gyrB.limit   = MAX_LOG_DUR * HZ_IMU_FAST;
  log_accB.limit   = MAX_LOG_DUR * HZ_IMU_FAST;
  log_magB.limit   = MAX_LOG_DUR * HZ_IMU_SLOW;
  log_ahrs.limit   = MAX_LOG_DUR * HZ_AHRS;
  log_gps.limit    = MAX_LOG_DUR * HZ_GPS;
  //log_ctrl.limit   = MAX_LOG_DUR * HZ_CTRL;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_open
//  Prepares the system for the next datalog sequence.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_open ( void )  {

  // Clear counters for new session
  log_input.count  = 0;
  log_output.count = 0;
  log_gyrA.count   = 0;
  log_accA.count   = 0;
  log_magA.count   = 0;
  log_gyrB.count   = 0;
  log_accB.count   = 0;
  log_magB.count   = 0;
  log_ahrs.count   = 0;
  log_gps.count    = 0;
  //log_ctrl.count   = 0;

  // Input signal storage
  log_input.time =  malloc( sizeof(float)  * log_input.limit      );
  log_input.reg  =  malloc( sizeof(ushort) * log_input.limit * 10 );
  log_input.pwm  =  malloc( sizeof(ushort) * log_input.limit * 10 );
  log_input.norm =  malloc( sizeof(float)  * log_input.limit * 10 );

  // Output signal storage
  log_output.time =  malloc( sizeof(float)  * log_output.limit      );
  log_output.reg  =  malloc( sizeof(ushort) * log_output.limit * 10 );
  log_output.pwm  =  malloc( sizeof(ushort) * log_output.limit * 10 );
  log_output.norm =  malloc( sizeof(float)  * log_output.limit * 10 );

  // IMU A Storage
  if(USE_IMUA) {

  // Gyroscope A storage
  log_gyrA.time =  malloc( sizeof(float) * log_gyrA.limit     );
  log_gyrA.dur  =  malloc( sizeof(ulong) * log_gyrA.limit     );
  log_gyrA.raw  =  malloc( sizeof(short) * log_gyrA.limit * 3 );
  log_gyrA.avg  =  malloc( sizeof(float) * log_gyrA.limit * 3 );
  log_gyrA.cal  =  malloc( sizeof(float) * log_gyrA.limit * 3 );

  // Accelerometer A storage
  log_accA.time =  malloc( sizeof(float) * log_accA.limit     );
  log_accA.dur  =  malloc( sizeof(ulong) * log_accA.limit     );
  log_accA.raw  =  malloc( sizeof(short) * log_accA.limit * 3 );
  log_accA.avg  =  malloc( sizeof(float) * log_accA.limit * 3 );
  log_accA.cal  =  malloc( sizeof(float) * log_accA.limit * 3 );

  // Magnetometer A storage
  log_magA.time =  malloc( sizeof(float) * log_magA.limit     );
  log_magA.dur  =  malloc( sizeof(ulong) * log_magA.limit     );
  log_magA.raw  =  malloc( sizeof(short) * log_magA.limit * 3 );
  log_magA.avg  =  malloc( sizeof(float) * log_magA.limit * 3 );
  log_magA.cal  =  malloc( sizeof(float) * log_magA.limit * 3 );

  }

  // IMU B Storage
  if(USE_IMUB) {

  // Gyroscope B storage
  log_gyrB.time =  malloc( sizeof(float) * log_gyrB.limit     );
  log_gyrB.dur  =  malloc( sizeof(ulong) * log_gyrB.limit     );
  log_gyrB.raw  =  malloc( sizeof(short) * log_gyrB.limit * 3 );
  log_gyrB.avg  =  malloc( sizeof(float) * log_gyrB.limit * 3 );
  log_gyrB.cal  =  malloc( sizeof(float) * log_gyrB.limit * 3 );

  // Accelerometer B storage
  log_accB.time =  malloc( sizeof(float) * log_accB.limit     );
  log_accB.dur  =  malloc( sizeof(ulong) * log_accB.limit     );
  log_accB.raw  =  malloc( sizeof(short) * log_accB.limit * 3 );
  log_accB.avg  =  malloc( sizeof(float) * log_accB.limit * 3 );
  log_accB.cal  =  malloc( sizeof(float) * log_accB.limit * 3 );

  // Magnetometer B storage
  log_magB.time =  malloc( sizeof(float) * log_magB.limit     );
  log_magB.dur  =  malloc( sizeof(ulong) * log_magB.limit     );
  log_magB.raw  =  malloc( sizeof(short) * log_magB.limit * 3 );
  log_magB.avg  =  malloc( sizeof(float) * log_magB.limit * 3 );
  log_magB.cal  =  malloc( sizeof(float) * log_magB.limit * 3 );

  }

  // Attitude and Heading Reference System storage
  log_ahrs.time  =  malloc( sizeof(float) * log_ahrs.limit     );
  log_ahrs.dur   =  malloc( sizeof(ulong) * log_ahrs.limit     );
  log_ahrs.quat  =  malloc( sizeof(float) * log_ahrs.limit * 4 );
  log_ahrs.dquat =  malloc( sizeof(float) * log_ahrs.limit * 4 );
  log_ahrs.eul   =  malloc( sizeof(float) * log_ahrs.limit * 3 );
  log_ahrs.deul  =  malloc( sizeof(float) * log_ahrs.limit * 3 );
  log_ahrs.bias  =  malloc( sizeof(float) * log_ahrs.limit * 3 );
  log_ahrs.fx    =  malloc( sizeof(float) * log_ahrs.limit     );
  log_ahrs.fz    =  malloc( sizeof(float) * log_ahrs.limit     );

  // Global Positioning System storage
  log_gps.time   =  malloc( sizeof(float) * log_gps.limit );
  log_gps.dur    =  malloc( sizeof(ulong) * log_gps.limit );

  /*// Controller parameter storage
  log_ctrl.time =  malloc( sizeof(float) * log_ctrl.limit     );
  log_ctrl.dur  =  malloc( sizeof(ulong) * log_ctrl.limit     );
  log_ctrl.perr =  malloc( sizeof(float) * log_ctrl.limit * 3 );
  log_ctrl.ierr =  malloc( sizeof(float) * log_ctrl.limit * 3 );
  log_ctrl.derr =  malloc( sizeof(float) * log_ctrl.limit * 3 );
  log_ctrl.cmd  =  malloc( sizeof(float) * log_ctrl.limit * 4 );
  */

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
  datalog.setup = true;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_close
//  Completes a datalog session by writing out collected data.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_close ( void )  {

  // Inidcate the download is in progress
  datalog.saving = true;
  led_blink( LED_LOG, 200, 200 );
  usleep(200000);

  // Local variables
  char *file = malloc(64);
  FILE *fnote, *fin, *fout, *fgyrA, *faccA, *fmagA, *fgyrB, *faccB, *fmagB, *fahrs, *fgps; // *fctl;
  ushort i;
  ulong row;

  // Create new directory
  sprintf( datalog.path, "../Log/%s/", datalog.dir );
  mkdir( datalog.path, 222 );

  // Create notes datalog file
  sprintf( file, "%snotes.txt", datalog.path );
  fnote = fopen( file, "w" );
  if( fnote == NULL )  printf( "Error (log_close): Cannot generate 'notes' file. \n" );
  fprintf( fnote, " Assign some system parameteres like gains, or telemetry waypoint updates... " );

  // Create input datalog file
  sprintf( file, "%sinput.txt", datalog.path );
  fin = fopen( file, "w" );
  if( fin == NULL )  printf( "Error (log_close): Cannot generate 'input' file. \n" );
  fprintf( fin,  "       Itime         I1       I2       I3       I4       I5       I6       I7       I8       I9       I0" );

  // Loop through input data
  for ( row = 0; row < log_input.count; row++ ) {
    fprintf( fin, "\n %011.6f    ", log_input.time[row] );
    for ( i=0; i<10; i++ )  fprintf( fin, "%7.4f  ", log_input.norm [ row*10 +i ] );   fprintf( fin, "   " );
  }

  // Free input memory
  free(log_input.time);
  free(log_input.reg);
  free(log_input.pwm);
  free(log_input.norm);

  // Create output datalog file
  sprintf( file, "%soutput.txt", datalog.path );
  fout = fopen( file, "w" );
  if( fout == NULL )  printf( "Error (log_close): Cannot generate 'output' file. \n" );
  fprintf( fout, "       Otime         O1       O2       O3       O4       O5       O6       O7       O8       O9       O0" );

  // Loop through output data
  for ( row = 0; row < log_output.count; row++ ) {
    fprintf( fout, "\n %011.6f    ", log_output.time[row] );
    for ( i=0; i<10; i++ )  fprintf( fout, "%7.4f  ", log_output.norm [ row*10 +i ] );   fprintf( fout, "   " );
  }

  // Free output memory
  free(log_output.time);
  free(log_output.reg);
  free(log_output.pwm);
  free(log_output.norm);

  // IMU A datalogs
  if(USE_IMUA)  {

  // Create gyroscope A datalog file
  sprintf( file, "%sgyrA.txt", datalog.path );
  fgyrA = fopen( file, "w" );
  if( fgyrA == NULL )  printf( "Error (log_close): Cannot generate 'gyrA' file. \n" );
  fprintf( fgyrA,
    "       Gtime    Gdur   \
    Grx     Gry     Grz       \
    Gax        Gay        Gaz     \
    Gcx      Gcy      Gcz");

  // Loop through gyroscope data
  for ( row = 0; row < log_gyrA.count; row++ ) {
    fprintf( fgyrA, "\n %011.6f  %06ld    ", log_gyrA.time[row], log_gyrA.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( fgyrA, "%06d  ",   log_gyrA.raw[ row*3 +i ] );   fprintf( fgyrA, "   " );
    for ( i=0; i<3; i++ )  fprintf( fgyrA, "%09.2f  ", log_gyrA.avg[ row*3 +i ] );   fprintf( fgyrA, "   " );
    for ( i=0; i<3; i++ )  fprintf( fgyrA, "%07.4f  ", log_gyrA.cal[ row*3 +i ] );   fprintf( fgyrA, "   " );
  }

  // Free gyroscope memory
  free(log_gyrA.time);
  free(log_gyrA.dur);
  free(log_gyrA.raw);
  free(log_gyrA.avg);
  free(log_gyrA.cal);

  // Create accelerometer A datalog file
  sprintf( file, "%saccA.txt", datalog.path );
  faccA = fopen( file, "w" );
  if( faccA == NULL )  printf( "Error (log_close): Cannot generate 'accA' file. \n" );
  fprintf( faccA, 
    "       Atime    Adur   \
    Arx     Ary     Arz       \
    Aax        Aay        Aaz     \
    Acx      Acy      Acz");

  // Loop through accelerometer data
  for ( row = 0; row < log_accA.count; row++ ) {
    fprintf( faccA, "\n %011.6f  %06ld    ", log_accA.time[row], log_accA.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( faccA, "%06d  ",   log_accA.raw[ row*3 +i ] );   fprintf( faccA, "   " );
    for ( i=0; i<3; i++ )  fprintf( faccA, "%09.2f  ", log_accA.avg[ row*3 +i ] );   fprintf( faccA, "   " );
    for ( i=0; i<3; i++ )  fprintf( faccA, "%07.4f  ", log_accA.cal[ row*3 +i ] );   fprintf( faccA, "   " );
  }

  // Free accelerometer memory
  free(log_accA.time);
  free(log_accA.dur);
  free(log_accA.raw);
  free(log_accA.avg);
  free(log_accA.cal);

  // Create magnetometer A datalog file
  sprintf( file, "%smagA.txt", datalog.path );
  fmagA = fopen( file, "w" );
  if( fmagA == NULL )  printf( "Error (log_close): Cannot generate 'magA' file. \n" );
  fprintf( fmagA,
    "       Mtime    Mdur   \
    Mrx     Mry     Mrz       \
    Max        May        Maz     \
    Mcx      Mcy      Mcz");

  // Loop through magnetometer data
  for ( row = 0; row < log_magA.count; row++ ) {
    fprintf( fmagA, "\n %011.6f  %06ld    ", log_magA.time[row], log_magA.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( fmagA, "%06d  ",   log_magA.raw[ row*3 +i ] );   fprintf( fmagA, "   " );
    for ( i=0; i<3; i++ )  fprintf( fmagA, "%09.2f  ", log_magA.avg[ row*3 +i ] );   fprintf( fmagA, "   " );
    for ( i=0; i<3; i++ )  fprintf( fmagA, "%07.4f  ", log_magA.cal[ row*3 +i ] );   fprintf( fmagA, "   " );
  }

  // Free magnetometer memory
  free(log_magA.time);
  free(log_magA.dur);
  free(log_magA.raw);
  free(log_magA.avg);
  free(log_magA.cal);

  }

  // IMU B datalog
  if (USE_IMUB)  {

  // Create gyroscope B datalog file
  sprintf( file, "%sgyrB.txt", datalog.path );
  fgyrB = fopen( file, "w" );
  if( fgyrB == NULL )  printf( "Error (log_close): Cannot generate 'gyrB' file. \n" );
  fprintf( fgyrB,
    "       Gtime    Gdur   \
    Grx     Gry     Grz       \
    Gax        Gay        Gaz     \
    Gcx      Gcy      Gcz");

  // Loop through gyroscope data
  for ( row = 0; row < log_gyrB.count; row++ ) {
    fprintf( fgyrB, "\n %011.6f  %06ld    ", log_gyrB.time[row], log_gyrB.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( fgyrB, "%06d  ",   log_gyrB.raw[ row*3 +i ] );   fprintf( fgyrB, "   " );
    for ( i=0; i<3; i++ )  fprintf( fgyrB, "%09.2f  ", log_gyrB.avg[ row*3 +i ] );   fprintf( fgyrB, "   " );
    for ( i=0; i<3; i++ )  fprintf( fgyrB, "%07.4f  ", log_gyrB.cal[ row*3 +i ] );   fprintf( fgyrB, "   " );
  }

  // Free gyroscope memory
  free(log_gyrB.time);
  free(log_gyrB.dur);
  free(log_gyrB.raw);
  free(log_gyrB.avg);
  free(log_gyrB.cal);

  // Create accelerometer B datalog file
  sprintf( file, "%saccB.txt", datalog.path );
  faccB = fopen( file, "w" );
  if( faccB == NULL )  printf( "Error (log_close): Cannot generate 'accB' file. \n" );
  fprintf( faccB, 
    "       Atime    Adur   \
    Arx     Ary     Arz       \
    Aax        Aay        Aaz     \
    Acx      Acy      Acz");

  // Loop through accelerometer data
  for ( row = 0; row < log_accB.count; row++ ) {
    fprintf( faccB, "\n %011.6f  %06ld    ", log_accB.time[row], log_accB.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( faccB, "%06d  ",   log_accB.raw[ row*3 +i ] );   fprintf( faccB, "   " );
    for ( i=0; i<3; i++ )  fprintf( faccB, "%09.2f  ", log_accB.avg[ row*3 +i ] );   fprintf( faccB, "   " );
    for ( i=0; i<3; i++ )  fprintf( faccB, "%07.4f  ", log_accB.cal[ row*3 +i ] );   fprintf( faccB, "   " );
  }

  // Free accelerometer memory
  free(log_accB.time);
  free(log_accB.dur);
  free(log_accB.raw);
  free(log_accB.avg);
  free(log_accB.cal);

  // Create magnetometer B datalog file
  sprintf( file, "%smagB.txt", datalog.path );
  fmagB = fopen( file, "w" );
  if( fmagB == NULL )  printf( "Error (log_close): Cannot generate 'magB' file. \n" );
  fprintf( fmagB,
    "       Mtime    Mdur   \
    Mrx     Mry     Mrz       \
    Max        May        Maz     \
    Mcx      Mcy      Mcz");

  // Loop through magnetometer data
  for ( row = 0; row < log_magB.count; row++ ) {
    fprintf( fmagB, "\n %011.6f  %06ld    ", log_magB.time[row], log_magB.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( fmagB, "%06d  ",   log_magB.raw[ row*3 +i ] );   fprintf( fmagB, "   " );
    for ( i=0; i<3; i++ )  fprintf( fmagB, "%09.2f  ", log_magB.avg[ row*3 +i ] );   fprintf( fmagB, "   " );
    for ( i=0; i<3; i++ )  fprintf( fmagB, "%07.4f  ", log_magB.cal[ row*3 +i ] );   fprintf( fmagB, "   " );
  }

  // Free magnetometer memory
  free(log_magB.time);
  free(log_magB.dur);
  free(log_magB.raw);
  free(log_magB.avg);
  free(log_magB.cal);

  }

  // Create attitude and heading reference system datalog file
  sprintf( file, "%sahrs.txt", datalog.path );
  fahrs = fopen( file, "w" );
  if( fahrs == NULL )  printf( "Error (log_close): Cannot generate 'ahrs' file. \n" );
  fprintf( fahrs,
    "       Rtime    Rdur     \
    Qw       Qx       Qy       Qz     \
    dQw      dQx      dQy      dQz      \
    Ex       Ey       Ez     \
    dEx      dEy      dEz      \
    bx       by       bz      \
    fx       fz");

  // Loop through attitude and heading reference system data
  for ( row = 0; row < log_ahrs.count; row++ ) {
    fprintf( fahrs, "\n %011.6f  %06ld    ", log_ahrs.time[row], log_ahrs.dur[row] );
    for ( i=0; i<4; i++ )  fprintf( fahrs, "%07.4f  ", log_ahrs.quat  [ row*4 +i ] );  fprintf( fahrs, "   " );
    for ( i=0; i<4; i++ )  fprintf( fahrs, "%07.4f  ", log_ahrs.dquat [ row*4 +i ] );  fprintf( fahrs, "   " );
    for ( i=0; i<3; i++ )  fprintf( fahrs, "%07.4f  ", log_ahrs.eul   [ row*3 +i ] );  fprintf( fahrs, "   " );
    for ( i=0; i<3; i++ )  fprintf( fahrs, "%07.4f  ", log_ahrs.deul  [ row*3 +i ] );  fprintf( fahrs, "   " );
    for ( i=0; i<3; i++ )  fprintf( fahrs, "%07.4f  ", log_ahrs.bias  [ row*3 +i ] );  fprintf( fahrs, "   " );
    fprintf( fahrs, "%07.4f  ", log_ahrs.fx[ row +i ] );
    fprintf( fahrs, "%07.4f  ", log_ahrs.fz[ row +i ] );
    fprintf( fahrs, "   " );
  }

  // Free attitude/heading memory
  free(log_ahrs.time);
  free(log_ahrs.dur);
  free(log_ahrs.quat);
  free(log_ahrs.dquat);
  free(log_ahrs.eul);
  free(log_ahrs.deul);
  free(log_ahrs.bias);
  free(log_ahrs.fx);
  free(log_ahrs.fz);

  // Create GPS datalog file
  sprintf( file, "%sgps.txt", datalog.path );
  fgps = fopen( file, "w" );
  if( fgps == NULL )  printf( "Error (log_close): Cannot generate 'gps' file. \n" );
  fprintf( fgps,
    "       Gtime    Gdur     ");

  // Loop through GPS data
  for ( row = 0; row < log_gps.count; row++ ) {
    fprintf( fgps, "\n %011.6f  %06ld    ", log_gps.time[row], log_gps.dur[row] );
    //for ( i=0; i<4; i++ )  fprintf( fahrs, "%07.4f  ", log_ahrs.quat  [ row*4 +i ] );  fprintf( fahrs, "   " );
    fprintf( fgps, "   " );
  }

  // Free attitude/heading memory
  free(log_gps.time);
  free(log_gps.dur);

  /*// Create controller datalog file
  sprintf( file, "%sctrl.txt", datalog.path );
  fctl = fopen( file, "w" );
  if( fctl == NULL )  printf( "Error (log_close): Cannot generate 'ctrl' file. \n" );
  fprintf( fctl,
    "       Ctime    Cdur    \
    EPX      EPY      EPZ     \
    EIX      EIY      EIZ     \
    EDX      EDY      EDZ      \
    CX       CY       CZ       CT" );

  // Loop through controller data
  for ( row = 0; row < log_ctrl.count; row++ ) {
    fprintf( fctl, "\n %011.6f  %06ld    ", log_ctrl.time[row], log_ctrl.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( fctl, "%07.4f  ",  log_ctrl.perr[ row*3 +i ] );   fprintf( fctl, "   " );
    for ( i=0; i<3; i++ )  fprintf( fctl, "%07.4f  ",  log_ctrl.ierr[ row*3 +i ] );   fprintf( fctl, "   " );
    for ( i=0; i<3; i++ )  fprintf( fctl, "%07.4f  ",  log_ctrl.derr[ row*3 +i ] );   fprintf( fctl, "   " );
    for ( i=0; i<4; i++ )  fprintf( fctl, "%07.4f  ",  log_ctrl.cmd [ row*4 +i ] );   fprintf( fctl, "   " );
  }
  */
  /*// Free controller memory
  free(log_ctrl.time);
  free(log_ctrl.dur);
  free(log_ctrl.perr);
  free(log_ctrl.ierr);
  free(log_ctrl.derr);
  free(log_ctrl.cmd);
  */

  // Close files
  fclose(fnote);
  fclose(fin);
  fclose(fout);
  if (USE_IMUA)  {
    fclose(fgyrA);
    fclose(faccA);
    fclose(fmagA);
  }
  if (USE_IMUB)  {
    fclose(fgyrB);
    fclose(faccB);
    fclose(fmagB);
  }
  fclose(fahrs);
  fclose(fgps);
  //fclose(fctl);

  // Switch datalog setup flag
  datalog.setup = false;
  datalog.saving = false;
  led_off( LED_LOG );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_exit
//  Closes the data log files.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_exit ( void )  {
  if(DEBUG)  printf("Close logs \n");
  // Add code as needed...
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
  // Record system input/output data
  case LOG_SIO :

    timestamp = (float) ( tmr_sio.start_sec + ( tmr_sio.start_usec / 1000000.0f ) ) - datalog.offset;

    // Input data
    pthread_mutex_lock(&mutex_input);
    if ( log_input.count < log_input.limit ) {
      row = log_input.count;
      log_input.time[row] = timestamp;
      for ( i=0; i<10; i++ )  log_input.norm [ row*10 +i ] = input.norm[i];
      log_input.count++;
    }
    pthread_mutex_unlock(&mutex_input);

    // Output data
    pthread_mutex_lock(&mutex_output);
    if ( log_output.count < log_output.limit ) {
      row = log_output.count;
      log_output.time[row] = timestamp;
      for ( i=0; i<10; i++ )  log_output.norm [ row*10 +i ] = output.norm[i];
      log_output.count++;
    }
    pthread_mutex_unlock(&mutex_output);

    return;


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Record IMUA data
  case LOG_IMUA :

    timestamp = (float) ( tmr_imuA.start_sec + ( tmr_imuA.start_usec / 1000000.0f ) ) - datalog.offset;
    
    // Gyroscope A data
    pthread_mutex_lock(&mutex_gyrA);
    if ( log_gyrA.count < log_gyrA.limit ) {
      row = log_gyrA.count;
      log_gyrA.time[row] = timestamp;
      log_gyrA.dur[row]  = tmr_imuA.dur;
      for ( i=0; i<3; i++ )  log_gyrA.raw[ row*3 +i ] = gyrA.raw[i];
      for ( i=0; i<3; i++ )  log_gyrA.avg[ row*3 +i ] = gyrA.avg[i];
      for ( i=0; i<3; i++ )  log_gyrA.cal[ row*3 +i ] = gyrA.cal[i];
      log_gyrA.count++;
    }
    pthread_mutex_unlock(&mutex_gyrA);

    // Accelerometer A data
    pthread_mutex_lock(&mutex_accA);
    if ( log_accA.count < log_accA.limit ) {
      row = log_accA.count;
      log_accA.time[row] = timestamp;
      log_accA.dur[row]  = tmr_imuA.dur;
      for ( i=0; i<3; i++ )  log_accA.raw[ row*3 +i ] = accA.raw[i];
      for ( i=0; i<3; i++ )  log_accA.avg[ row*3 +i ] = accA.avg[i];
      for ( i=0; i<3; i++ )  log_accA.cal[ row*3 +i ] = accA.cal[i];
      log_accA.count++;
    }
    pthread_mutex_unlock(&mutex_accA);

    // Magnetometer A data
    pthread_mutex_lock(&mutex_magA);
    if( imuA.getmag && ( log_magA.count < log_magA.limit) ) {
      row = log_magA.count;
      log_magA.time[row] = timestamp;
      log_magA.dur[row]  = tmr_imuA.dur;
      for ( i=0; i<3; i++ )  log_magA.raw[ row*3 +i ] = magA.raw[i];
      for ( i=0; i<3; i++ )  log_magA.avg[ row*3 +i ] = magA.avg[i];
      for ( i=0; i<3; i++ )  log_magA.cal[ row*3 +i ] = magA.cal[i];
      log_magA.count++;
    }
    pthread_mutex_unlock(&mutex_magA);

    return;


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Record IMUB data
  case LOG_IMUB :

    timestamp = (float) ( tmr_imuB.start_sec + ( tmr_imuB.start_usec / 1000000.0f ) ) - datalog.offset;
    
    // Gyroscope B data
    pthread_mutex_lock(&mutex_gyrB);
    if ( log_gyrB.count < log_gyrB.limit ) {
      row = log_gyrB.count;
      log_gyrB.time[row] = timestamp;
      log_gyrB.dur[row]  = tmr_imuB.dur;
      for ( i=0; i<3; i++ )  log_gyrB.raw[ row*3 +i ] = gyrB.raw[i];
      for ( i=0; i<3; i++ )  log_gyrB.avg[ row*3 +i ] = gyrB.avg[i];
      for ( i=0; i<3; i++ )  log_gyrB.cal[ row*3 +i ] = gyrB.cal[i];
      log_gyrB.count++;
    }
    pthread_mutex_unlock(&mutex_gyrB);

    // Accelerometer B data
    pthread_mutex_lock(&mutex_accB);
    if ( log_accB.count < log_accB.limit ) {
      row = log_accB.count;
      log_accB.time[row] = timestamp;
      log_accB.dur[row]  = tmr_imuB.dur;
      for ( i=0; i<3; i++ )  log_accB.raw[ row*3 +i ] = accB.raw[i];
      for ( i=0; i<3; i++ )  log_accB.avg[ row*3 +i ] = accB.avg[i];
      for ( i=0; i<3; i++ )  log_accB.cal[ row*3 +i ] = accB.cal[i];
      log_accB.count++;
    }
    pthread_mutex_unlock(&mutex_accB);

    // Magnetometer B data
    pthread_mutex_lock(&mutex_magB);
    if( imuB.getmag && ( log_magB.count < log_magB.limit) ) {
      row = log_magB.count;
      log_magB.time[row] = timestamp;
      log_magB.dur[row]  = tmr_imuB.dur;
      for ( i=0; i<3; i++ )  log_magB.raw[ row*3 +i ] = magB.raw[i];
      for ( i=0; i<3; i++ )  log_magB.avg[ row*3 +i ] = magB.avg[i];
      for ( i=0; i<3; i++ )  log_magB.cal[ row*3 +i ] = magB.cal[i];
      log_magB.count++;
    }
    pthread_mutex_unlock(&mutex_magB);

    return;


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Record AHRS data
  case LOG_AHRS :

    timestamp = (float) ( tmr_ahrs.start_sec + ( tmr_ahrs.start_usec / 1000000.0f ) ) - datalog.offset;

    if ( log_ahrs.count < log_ahrs.limit ) {
      row = log_ahrs.count;
      log_ahrs.time[row] = timestamp;
      log_ahrs.dur[row]  = tmr_ahrs.dur;

      pthread_mutex_lock(&mutex_quat);
      for ( i=0; i<4; i++ )  log_ahrs.quat  [ row*4 +i ] = ahrs.quat  [i];
      for ( i=0; i<4; i++ )  log_ahrs.dquat [ row*4 +i ] = ahrs.dquat [i];
      pthread_mutex_unlock(&mutex_quat);

      pthread_mutex_lock(&mutex_eul);
      for ( i=0; i<3; i++ )  log_ahrs.eul   [ row*3 +i ] = ahrs.eul   [i];
      for ( i=0; i<3; i++ )  log_ahrs.deul  [ row*3 +i ] = ahrs.deul  [i];
      pthread_mutex_unlock(&mutex_eul);

      pthread_mutex_lock(&mutex_ahrs);
      for ( i=0; i<3; i++ )  log_ahrs.bias  [ row*3 +i ] = ahrs.bias  [i];
                             log_ahrs.fx    [ row   +i ] = ahrs.fx;
                             log_ahrs.fz    [ row   +i ] = ahrs.fz;
      pthread_mutex_unlock(&mutex_ahrs);

      log_ahrs.count++;
    }

    return;


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Record GPS data
  case LOG_GPS :

    timestamp = (float) ( tmr_gps.start_sec + ( tmr_gps.start_usec / 1000000.0f ) ) - datalog.offset;

    if ( log_gps.count < log_gps.limit ) {
      row = log_gps.count;
      log_gps.time[row] = timestamp;
      log_gps.dur[row]  = tmr_gps.dur;

      pthread_mutex_lock(&mutex_gps);
      //for ( i=0; i<4; i++ )  log_ahrs.quat  [ row*4 +i ] = ahrs.quat  [i];
      //for ( i=0; i<4; i++ )  log_ahrs.dquat [ row*4 +i ] = ahrs.dquat [i];
      pthread_mutex_unlock(&mutex_gps);

      log_gps.count++;
    }

    return;


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Record CTRL data
  //case LOG_CTL :
    /*
    timestamp = (float) ( tmr_ctrl.start_sec + ( tmr_ctrl.start_usec / 1000000.0f ) ) - datalog.offset;

    if ( log_ctrl.count < log_ctrl.limit ) {
      row = log_ctrl.count;
      log_ctrl.time[row] = timestamp;
      log_ctrl.dur[row]  = tmr_ctrl.dur;
      for ( i=0; i<3; i++ )  log_ctrl.perr[ row*3 +i ] = ctrl.perr[i];
      for ( i=0; i<3; i++ )  log_ctrl.ierr[ row*3 +i ] = ctrl.ierr[i];
      for ( i=0; i<3; i++ )  log_ctrl.derr[ row*3 +i ] = ctrl.derr[i];
      for ( i=0; i<4; i++ )  log_ctrl.cmd [ row*4 +i ] = ctrl.cmd[i];
      log_ctrl.count++;
    }
    */
    //return;


  default :
    return;
  
  }
}



