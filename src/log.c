
//============================================================
//  log.c
//  Justin M Selfridge
//============================================================
#include "log.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_write
//  Top level function for writing data log commands.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_write ( void )  {
  if (datalog.enabled) {  if (!datalog.open)  log_init();  log_record();  }
  else                 {  if (datalog.open)   log_exit();  }
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_init
//  Initalizes next sequential log file and populates the header.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_init ( void )  {
  if(DEBUG)  printf("Initializing log file: \t");

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
  if(DEBUG)  printf( "%s \n\n", datalog.dir );

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
  sprintf( file, "%sgyro.txt", datalog.path );
  datalog.gyro = fopen( file, "w" );
  sys_err( datalog.gyro == NULL, "Error (log_init): Cannot open 'gyro' file. \n" );
  fprintf( datalog.gyro, " Gtime,       Gdur,    Gperc, G,    Grx1,   Gry1,   Grz1,       ");

  // Create accelerometer datalog file
  sprintf( file, "%sacc.txt", datalog.path );
  datalog.acc = fopen( file, "w" );
  sys_err( datalog.acc == NULL, "Error (log_init): Cannot open 'acc' file. \n" );
  fprintf( datalog.acc, "timeA, rAx1, rAy1, rAz, ");

  // Create magnetometer datalog file
  sprintf( file, "%smag.txt", datalog.path );
  datalog.mag = fopen( file, "w" );
  sys_err( datalog.mag == NULL, "Error (log_init): Cannot open 'mag' file. \n" );
  fprintf( datalog.mag, "timeM, rMx1, rMy1, rMz1, ");

  // Determine start second
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );
  datalog.offset = timeval.tv_sec;

  return;
}


/*
//--- ORIGINAL ---//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_init
//  Initalizes next sequential log file and populates the header.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_init ( void )  {
  if(DEBUG)  printf("Initializing log file: \t");

  // Find next available log file
  int i = 0;
  datalog.name = malloc(32);
  while (true) {
    i++;
    if      ( i<10   )  sprintf( datalog.name, "./log/Log_00%d.txt", i );
    else if ( i<100  )  sprintf( datalog.name, "./log/Log_0%d.txt",  i );
    else if ( i<1000 )  sprintf( datalog.name, "./log/Log_%d.txt",   i );
    else  sys_err( true, "Error (log_init): Exceeded maximum number of log files." );
    if ( access( datalog.name, F_OK ) == -1 )  break;
  }
  if(DEBUG)  printf( "%s \n\n", datalog.name );

  // Open log file
  datalog.file = fopen( datalog.name, "w" );
  sys_err( datalog.file == NULL, "Error (log_init): Cannot open log file. \n" );
  datalog.open = true;

  // Timing header
  fprintf( datalog.file, " timestamp,   dur,     perc,  X,    ");

  // Input/Output header
  //fprintf( datalog.file, "R1,     R2,     R3,     R4,     R5,     R6,        ");
  //fprintf( datalog.file, "M1,     M2,     M3,     M4,        ");

  // IMU1 header
  //fprintf( datalog.file, "rMx1, rMy1, rMz1,    ");
  //fprintf( datalog.file, "rAx1,   rAy1,   rAz1,      ");
  fprintf( datalog.file, "rGx1,   rGy1,   rGz1,      ");
  //fprintf( datalog.file, "rQo1,         rQx1,         rQy1,         rQz1,            ");
  //fprintf( datalog.file, "aMx1,    aMy1,    aMz1,       ");
  //fprintf( datalog.file, "aAx1,      aAy1,      aAz1,         ");
  //fprintf( datalog.file, "aGx1,      aGy1,      aGz1,          ");
  //fprintf( datalog.file, "nMx1,    nMy1,    nMz1,       ");
  //fprintf( datalog.file, "nAx1,    nAy1,    nAz1,       ");
  //fprintf( datalog.file, "nGx1,    nGy1,    nGz1,       ");
  //fprintf( datalog.file, "bx1,       by1,       bz1,          fx1,     fz1,        ");
  //fprintf( datalog.file, "Qo1,     Qx1,     Qy1,     Qz1,        ");
  //fprintf( datalog.file, "dQo1,    dQx1,    dQy1,    dQz1,       ");
  //fprintf( datalog.file, "Ex1,     Ey1,     Ez1,        ");
  //fprintf( datalog.file, "dEx1,    dEy1,    dEz1,       ");

  // IMU2 header
  //fprintf( datalog.file, "rMx2, rMy2, rMz2,     ");
  //fprintf( datalog.file, "rAx2,   rAy2,   rAz2,      ");
  //fprintf( datalog.file, "rGx2,   rGy2,   rGz2,      ");
  //fprintf( datalog.file, "rQo2,         rQx2,         rQy2,         rQz2,            ");
  //fprintf( datalog.file, "aMx2,     aMy2,     aMz2,        ");
  //fprintf( datalog.file, "aAx2,     aAy2,     aAz2,        ");
  //fprintf( datalog.file, "aGx2,     aGy2,     aGz2,        ");
  //fprintf( datalog.file, "nMx2,    nMy2,    nMz2,       ");
  //fprintf( datalog.file, "nAx2,    nAy2,    nAz2,       ");
  //fprintf( datalog.file, "nGx2,    nGy2,    nGz2,       ");
  //fprintf( datalog.file, "bx2,       by2,       bz2,          fx2,     fz2,        ");
  //fprintf( datalog.file, "Qo2,     Qx2,     Qy2,     Qz2,        ");
  //fprintf( datalog.file, "dQo2,    dQx2,    dQy2,    dQz2,       ");
  //fprintf( datalog.file, "Ex2,     Ey2,     Ez2,        ");
  //fprintf( datalog.file, "dEx2,    dEy2,    dEz2,       ");

  // Control header
  //fprintf( datalog.file, "head,      ");
  //fprintf( datalog.file, "XPerr,  XIerr,  XDerr,     ");
  //fprintf( datalog.file, "YPerr,  YIerr,  YDerr,     ");
  //fprintf( datalog.file, "ZPerr,  ZIerr,  ZDerr,    ");
  //fprintf( datalog.file, "Xin,     Yin,     Zin,     Tin,  ");

  // Determine start second
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );
  datalog.offset = timeval.tv_sec;  

  return;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_record
//  Records the data to the log file.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_record ( void )  {

  // Increment counter
  ushort i;

  // Record the timestamp
  float timestamp = (float)( thr_gyro.start_sec + ( thr_gyro.start_usec / 1000000.0f ) - datalog.offset );
  fprintf( datalog.gyro, "\n %011.6f, %06ld, %6.3f, ", 
    timestamp, thr_gyro.dur, thr_gyro.perc );
  if (thr_gyro.perc<1.0)  fprintf( datalog.gyro, "_,    ");
  else                    fprintf( datalog.gyro, "X,    ");

  // Inputs and outputs
  //for ( i=0; i<6; i++ )  fprintf( datalog.file, "%06.1f, ", sys.input[i]  );     fprintf( datalog.file, "   " );
  //for ( i=0; i<4; i++ )  fprintf( datalog.file, "%06.1f, ", sys.output[i] );     fprintf( datalog.file, "   " );
 
  // Raw sensors data (IMU1)
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%04d, ",   imu1.rawMag[i]  );   fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%06d, ",   imu1.rawAcc[i]  );   fprintf( datalog.file, "   " );
  for ( i=0; i<3; i++ )  fprintf( datalog.gyro, "%06d, ",   imu1.rawGyro[i] );   fprintf( datalog.gyro, "   " );
  //for ( i=0; i<4; i++ )  fprintf( datalog.file, "%012ld, ", imu1.rawQuat[i] );   fprintf( datalog.file, "   " );

  // Moving average data (IMU1)
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%07.2f, ", imu1.avgMag[i]  );   fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%09.2f, ", imu1.avgAcc[i]  );   fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%09.2f, ", imu1.avgGyro[i] );   fprintf( datalog.file, "   " );

  // Normalized sensor data (IMU1)
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%7.4f, ",  imu1.normMag[i]  );  fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%7.4f, ",  imu1.normAcc[i]  );  fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%7.4f, ",  imu1.normGyro[i] );  fprintf( datalog.file, "   " );

  // Internal fusion algorithm values (IMU1)
  //for ( i=0; i<3; i++ )  
  //fprintf( datalog.file, "%9.6f, ", imu1.bias[i] );
  //fprintf( datalog.file, "   " );
  //fprintf( datalog.file, "%7.4f, ", imu1.fx );
  //fprintf( datalog.file, "%7.4f, ", imu1.fz );
  //fprintf( datalog.file, "   " );

  // Fused sensor data (IMU1)
  //for ( i=0; i<4; i++ )  fprintf( datalog.file, "%7.4f, ",  imu1.Quat[i]  );     fprintf( datalog.file, "   " );
  //for ( i=0; i<4; i++ )  fprintf( datalog.file, "%7.4f, ",  imu1.dQuat[i] );     fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%7.4f, ",  imu1.Eul[i]   );     fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%7.4f, ",  imu1.dEul[i]  );     fprintf( datalog.file, "   " );

  // Raw sensors data (IMU2)
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%04d, ",   imu2.rawMag[i]  );  fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%06d, ",   imu2.rawAcc[i]  );  fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%06d, ",   imu2.rawGyro[i] );  fprintf( datalog.file, "   " );
  //for ( i=0; i<4; i++ )  fprintf( datalog.file, "%012ld, ", imu2.rawQuat[i] );  fprintf( datalog.file, "   " );

  // Moving average data (IMU2)
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%8.2f, ",   imu2.avgMag[i]  );  fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%8.2f, ",   imu2.avgAcc[i]  );  fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%8.2f, ",   imu2.avgGyro[i] );  fprintf( datalog.file, "   " );

  // Calibrated sensors data (IMU2)
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%7.4f, ", imu2.normMag[i]  );  fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%7.4f, ", imu2.normAcc[i]  );  fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%7.4f, ", imu2.normGyro[i] );  fprintf( datalog.file, "   " );

  // Internal fusion algorithm values (IMU2)
  //for ( i=0; i<3; i++ )  
  //fprintf( datalog.file, "%9.6f, ", imu2.bias[i] );
  //fprintf( datalog.file, "   " );
  //fprintf( datalog.file, "%7.4f, ", imu2.fx );
  //fprintf( datalog.file, "%7.4f, ", imu2.fz );
  //fprintf( datalog.file, "   " );

  // Fused sensor data (IMU2)
  //for ( i=0; i<4; i++ )  fprintf( datalog.file, "%7.4f, ", imu2.Quat[i]  );  fprintf( datalog.file, "   " );
  //for ( i=0; i<4; i++ )  fprintf( datalog.file, "%7.4f, ", imu2.dQuat[i] );  fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%7.4f, ", imu2.Eul[i]   );  fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%7.4f, ", imu2.dEul[i]  );  fprintf( datalog.file, "   " );

  // Control values
  //                       fprintf( datalog.file, "%6.3f,    ", ctrl.heading   );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%6.3f, ",    ctrl.err[X][i] );  fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%6.3f, ",    ctrl.err[Y][i] );  fprintf( datalog.file, "   " );
  //for ( i=0; i<3; i++ )  fprintf( datalog.file, "%6.3f, ",    ctrl.err[Z][i] );  fprintf( datalog.file, "   " );
  //for ( i=0; i<4; i++ )  fprintf( datalog.file, "%07.2f, ",   ctrl.input[i]  );  fprintf( datalog.file, "   " );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_exit
//  Closes the data log file.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_exit ( void )  {
  if(DEBUG)  printf("Closing log file \n");

  // Close files 
  fclose(datalog.gyro);
  //fclose(datalog.acc);
  //fclose(datalog.mag);

  // Free resources
  //free(datalog.note);
  //free(datalog.gyro);
  //free(datalog.acc);
  //free(datalog.mag);

  // Adjust flag
  datalog.open = false;

  return;
}



