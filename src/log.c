
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
  datalog.open     = false;
  datalog.enabled  = true;  //-- DEBUG --//  false;
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_write
//  Top level function for writing data log commands.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_write ( enum log_index index )  {
  if (datalog.enabled) {  if (!datalog.open)  log_open();  log_record(index);  }
  else                 {  if (datalog.open)   log_exit();  }
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_open
//  Opens the next sequential log file and populates the header.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_open ( void )  {

  // Local varaibales
  int i = 0;
  char *file;

  // Allocate memory
  datalog.dir  = malloc(16);
  datalog.path = malloc(32);
  file         = malloc(64);

  // Find next available log directory
  while (true) {
    i++;
    if      ( i<10   )  sprintf( datalog.dir, "Log_00%d", i );
    else if ( i<100  )  sprintf( datalog.dir, "Log_0%d",  i );
    else if ( i<1000 )  sprintf( datalog.dir, "Log_%d",   i );
    else  sys_err( true, "Error (log_init): Exceeded maximum number of log directories." );
    sprintf( file, "./log/%s/notes.txt", datalog.dir );
    if ( access( file , F_OK ) == -1 )  break;
  }

  // Create new directory
  sprintf( datalog.path, "./log/%s/", datalog.dir );
  mkdir( datalog.path, 222 );
  datalog.open = true;

  // Create notes file
  sprintf( file, "%snotes.txt", datalog.path );
  datalog.note = fopen( file, "w" );
  sys_err( datalog.note == NULL, "Error (log_init): Cannot open 'note' file. \n" );
  fprintf( datalog.note, "%s \n", datalog.dir );
  fprintf( datalog.note, "Add some more content... \n");
  fprintf( datalog.note, "About the system parameters... ");
  fclose(datalog.note);

  // Create gyroscope datalog file
  sprintf( file, "%sgyr.txt", datalog.path );
  datalog.gyr = fopen( file, "w" );
  sys_err( datalog.gyr == NULL, "Error (log_init): Cannot open 'gyro' file. \n" );
  fprintf( datalog.gyr, 
    " Gtime,       Gdur,  \
    Grx1,   Gry1,   Grz1,  \
    Gax1,      Gay1,      Gaz1      \
    Gcx1,    Gcy1,    Gcz1   ");

  // Create accelerometer datalog file
  sprintf( file, "%sacc.txt", datalog.path );
  datalog.acc = fopen( file, "w" );
  sys_err( datalog.acc == NULL, "Error (log_init): Cannot open 'acc' file. \n" );
  fprintf( datalog.acc, 
    " Atime,       Adur,  \
    Arx1,   Ary1,   Arz1,  \
    Aax1,      Aay1,      Aaz1,     \
    Acx1,    Acy1,    Acz1,   ");

  /*
  // Create magnetometer datalog file
  sprintf( file, "%smag.txt", datalog.path );
  datalog.mag = fopen( file, "w" );
  sys_err( datalog.mag == NULL, "Error (log_init): Cannot open 'mag' file. \n" );
  fprintf( datalog.mag, 
    " Mtime,       Mdur,  \
    Mrx1, Mry1, Mrz1,\
    Max1,    May1,    Maz1,   \
    Mcx1,    Mcy1,    Mcz1,   ");
  */
  /*
  // Create data fusion datalog file
  sprintf( file, "%sfusion.txt", datalog.path );
  datalog.fusion = fopen( file, "w" );
  sys_err( datalog.fusion == NULL, "Error (log_init): Cannot open 'fusion' file. \n" );
  fprintf( datalog.fusion, 
    " Ftime,       Fdur,  \
    Qw1,     Qx1,     Qy1,     Qz1,    \
    Ex1,     Ey1,     Ez1,  \
    dEx1, dEy1, dEz1,\
    XXXX,    XXXX,    XXXX,   ");
  */

  // Determine start second
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );
  datalog.offset = timeval.tv_sec;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_record
//  Records the data to the log file.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_record ( enum log_index index )  {

  // Local variables
  ushort i;
  float timestamp;

  // Jump to appropriate log 
  switch(index) {

  // Record 'gyroscope' datalog
  case LOG_GYR :
    timestamp = (float)( thr_imu.start_sec + ( thr_imu.start_usec / 1000000.0f ) - datalog.offset );
    fprintf( datalog.gyr, "\n %011.6f, %06ld,    ", timestamp, thr_imu.dur );
    for ( i=0; i<3; i++ )  fprintf( datalog.gyr, "%06d, ",   imu1.rawGyr[i] );   fprintf( datalog.gyr, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.gyr, "%09.2f, ", imu1.avgGyr[i] );   fprintf( datalog.gyr, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.gyr, "%07.4f, ", imu1.calGyr[i] );   fprintf( datalog.gyr, "   " );
    return;

  // Record 'accelerometer' datalog
  case LOG_ACC :
    timestamp = (float)( thr_imu.start_sec + ( thr_imu.start_usec / 1000000.0f ) - datalog.offset );
    fprintf( datalog.acc, "\n %011.6f, %06ld,    ", timestamp, thr_imu.dur );
    for ( i=0; i<3; i++ )  fprintf( datalog.acc, "%06d, ",   imu1.rawAcc[i] );  fprintf( datalog.acc, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.acc, "%09.2f, ", imu1.avgAcc[i] );  fprintf( datalog.acc, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.acc, "%07.4f, ", imu1.calAcc[i] );  fprintf( datalog.acc, "   " );
    return;
  /*
  // Record 'magnetometer' datalog
  case LOG_MAG :
    timestamp = (float)( thr_imu.start_sec + ( thr_imu.start_usec / 1000000.0f ) - datalog.offset );
    fprintf( datalog.mag, "\n %011.6f, %06ld,    ", timestamp, thr_imu.dur );
    for ( i=0; i<3; i++ )  fprintf( datalog.mag, "%04d, ",   imu1.rawMag[i] );  fprintf( datalog.mag, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.mag, "%07.2f, ", imu1.avgMag[i] );  fprintf( datalog.mag, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.mag, "%07.4f, ", imu1.calMag[i] );  fprintf( datalog.mag, "   " );
    return;
  */
  /*
  // Record 'data fusion' datalog
  case LOG_FUSION :
    timestamp = (float)( thr_fusion.start_sec + ( thr_fusion.start_usec / 1000000.0f ) - datalog.offset );
    fprintf( datalog.fusion, "\n %011.6f, %06ld,    ", timestamp, thr_fusion.dur );
    for ( i=0; i<4; i++ )  fprintf( datalog.fusion, "%07.4f, ", imu1.Quat[i] );  fprintf( datalog.fusion, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.fusion, "%07.4f, ", imu1.Eul[i]  );  fprintf( datalog.fusion, "   " );
    //for ( i=0; i<3; i++ )  fprintf( datalog.mag, "%07.4f, ", imu1.calMag[i] );  fprintf( datalog.mag, "   " );
    return;
  */

  default :
    return;
  }

}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_exit
//  Closes the data log file.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_exit ( void )  {

  // Close files
  fclose(datalog.gyr);
  fclose(datalog.acc);
  //fclose(datalog.mag);
  //fclose(datalog.fusion);

  // Adjust flag
  datalog.open = false;

  return;
}



