
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
  if (uav.logdata) {  if (!uav.fileopen)  log_init();  log_record();  }
  else             {  if (uav.fileopen)   log_exit();  }
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_init
//  Generates the next sequential log file and populates the header.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_init ( void )  {
  if(DEBUG)  printf("Initializing log file: \t");

  // Find next available log file
  int i = 0;
  uav.filename = malloc(32);
  while (true) {
    i++;
    if      ( i<10   )  sprintf( uav.filename, "../DataLogs/Log_00%d.txt", i );
    else if ( i<100  )  sprintf( uav.filename, "../DataLogs/Log_0%d.txt",  i );
    else if ( i<1000 )  sprintf( uav.filename, "../DataLogs/Log_%d.txt",   i );
    else  uav_err( true, "Error (log_init): Exceeded maximum number of log files." );
    if ( access( uav.filename, F_OK ) == -1 )  break;
  }
  if(DEBUG)  printf( "%s \n\n", uav.filename );

  // Open log file
  uav.logfile = fopen( uav.filename, "w" );
  uav_err( uav.logfile == NULL, "Error (log_init): Cannot open log file. \n" );
  uav.fileopen = true;

  // Timing header
  fprintf( uav.logfile, " time,    start,     dur,        perc,  X,    ");

  // Input/Output header
  fprintf( uav.logfile, "R1,   R2,   R3,   R4,   R5,   R6,      ");
  fprintf( uav.logfile, "M1,   M2,   M3,   M4,       ");

  // MPU1 header
  fprintf( uav.logfile, "rMx1,   rMy1,   rMz1,      ");
  fprintf( uav.logfile, "rAx1,   rAy1,   rAz1,      ");
  fprintf( uav.logfile, "rGx1,   rGy1,   rGz1,      ");
  fprintf( uav.logfile, "rQo1,         rQx1,         rQy1,         rQz1,            ");
  fprintf( uav.logfile, "nMx1,    nMy1,    nMz1,       ");
  fprintf( uav.logfile, "nAx1,    nAy1,    nAz1,       ");
  fprintf( uav.logfile, "nGx1,    nGy1,    nGz1,       ");
  fprintf( uav.logfile, "bx1,       by1,       bz1,          fx1,     fz1,        ");
  fprintf( uav.logfile, "Qo1,     Qx1,     Qy1,     Qz1,        ");
  fprintf( uav.logfile, "dQo1,    dQx1,    dQy1,    dQz1,       ");
  fprintf( uav.logfile, "Ex1,     Ey1,     Ez1,        ");
  fprintf( uav.logfile, "dEx1,    dEy1,    dEz1,      ");

  // MPU2 header
  //fprintf( uav.logfile, "nMx2,    nMy2,    nMz2,       ");
  //fprintf( uav.logfile, "nAx2,    nAy2,    nAz2,       ");
  //fprintf( uav.logfile, "nGx2,    nGy2,    nGz2,       ");
  //fprintf( uav.logfile, "bx2,       by2,       bz2,          fx2,     fz2,        ");
  //fprintf( uav.logfile, "Qo2,     Qx2,     Qy2,     Qz2,        ");
  //fprintf( uav.logfile, "dQo2,    dQx2,    dQy2,    dQz2,       ");
  //fprintf( uav.logfile, "Ex2,     Ey2,     Ez2,        ");
  //fprintf( uav.logfile, "dEx2,    dEy2,    dEz2,      ");

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_exit
//  Closes the data log file.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_exit ( void )  {
  fclose(uav.logfile);
  free(uav.filename);
  uav.fileopen = false;
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  log_record
//  Records the data to the log file.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void log_record ( void )  {

  // Increment counter
  ushort i;

  // Time values
  fprintf( uav.logfile, "\n %07.3f, %09ld, %09ld, %6.3f, ", 
    t.runtime, t.start_nano, t.dur, t.percent );
  if (t.percent<1.0)  fprintf( uav.logfile, "_,    ");
  else                fprintf( uav.logfile, "X,    ");

  // Inputs and outputs
  for ( i=0; i<6; i++ )  fprintf( uav.logfile, "%04d, ", uav.radio[i]  );  fprintf( uav.logfile, "   " );
  for ( i=0; i<4; i++ )  fprintf( uav.logfile, "%04d, ", uav.servo[i]  );  fprintf( uav.logfile, "   " );

  // Raw sensors data (MPU1)
  for ( i=0; i<3; i++ )  fprintf( uav.logfile, "%06d, ",   mpu1.rawMag[i]  );  fprintf( uav.logfile, "   " );
  for ( i=0; i<3; i++ )  fprintf( uav.logfile, "%06d, ",   mpu1.rawAcc[i]  );  fprintf( uav.logfile, "   " );
  for ( i=0; i<3; i++ )  fprintf( uav.logfile, "%06d, ",   mpu1.rawGyro[i] );  fprintf( uav.logfile, "   " );
  for ( i=0; i<4; i++ )  fprintf( uav.logfile, "%012ld, ", mpu1.rawQuat[i] );  fprintf( uav.logfile, "   " );

  // Calibrated sensors data (MPU1)
  for ( i=0; i<3; i++ )  fprintf( uav.logfile, "%7.4f, ", mpu1.normMag[i]  );  fprintf( uav.logfile, "   " );
  for ( i=0; i<3; i++ )  fprintf( uav.logfile, "%7.4f, ", mpu1.normAcc[i]  );  fprintf( uav.logfile, "   " );
  for ( i=0; i<3; i++ )  fprintf( uav.logfile, "%7.4f, ", mpu1.normGyro[i] );  fprintf( uav.logfile, "   " );

  // Internal fusion algorithm values (MPU1)
  for ( i=0; i<3; i++ )  
  fprintf( uav.logfile, "%9.6f, ", mpu1.bias[i] );
  fprintf( uav.logfile, "   " );
  fprintf( uav.logfile, "%7.4f, ", mpu1.fx );
  fprintf( uav.logfile, "%7.4f, ", mpu1.fz );
  fprintf( uav.logfile, "   " );

  // Fused sensor data (MPU1)
  for ( i=0; i<4; i++ )  fprintf( uav.logfile, "%7.4f, ", mpu1.Quat[i]  );  fprintf( uav.logfile, "   " );
  for ( i=0; i<4; i++ )  fprintf( uav.logfile, "%7.4f, ", mpu1.dQuat[i] );  fprintf( uav.logfile, "   " );
  for ( i=0; i<3; i++ )  fprintf( uav.logfile, "%7.4f, ", mpu1.Eul[i]   );  fprintf( uav.logfile, "   " );
  for ( i=0; i<3; i++ )  fprintf( uav.logfile, "%7.4f, ", mpu1.dEul[i]  );  fprintf( uav.logfile, "   " );

  // Calibrated sensors data (MPU2)
  //for ( i=0; i<3; i++ )  fprintf( uav.logfile, "%7.4f, ", mpu2.normMag[i]  );  fprintf( uav.logfile, "   " );
  //for ( i=0; i<3; i++ )  fprintf( uav.logfile, "%7.4f, ", mpu2.normAcc[i]  );  fprintf( uav.logfile, "   " );
  //for ( i=0; i<3; i++ )  fprintf( uav.logfile, "%7.4f, ", mpu2.normGyro[i] );  fprintf( uav.logfile, "   " );

  // Internal fusion algorithm values (MPU2)
  //for ( i=0; i<3; i++ )  
  //fprintf( uav.logfile, "%9.6f, ", mpu2.bias[i] );
  //fprintf( uav.logfile, "   " );
  //fprintf( uav.logfile, "%7.4f, ", mpu2.fx );
  //fprintf( uav.logfile, "%7.4f, ", mpu2.fz );
  //fprintf( uav.logfile, "   " );

  // Fused sensor data (MPU2)
  //for ( i=0; i<4; i++ )  fprintf( uav.logfile, "%7.4f, ", mpu2.Quat[i]  );  fprintf( uav.logfile, "   " );
  //for ( i=0; i<4; i++ )  fprintf( uav.logfile, "%7.4f, ", mpu2.dQuat[i] );  fprintf( uav.logfile, "   " );
  //for ( i=0; i<3; i++ )  fprintf( uav.logfile, "%7.4f, ", mpu2.Eul[i]   );  fprintf( uav.logfile, "   " );
  //for ( i=0; i<3; i++ )  fprintf( uav.logfile, "%7.4f, ", mpu2.dEul[i]  );  fprintf( uav.logfile, "   " );

  return;
}



