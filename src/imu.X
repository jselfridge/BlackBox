
//============================================================
//  imu.c
//  Justin M Selfridge
//============================================================
#include "imu.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_init
//  Initializes the MPU sensors.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
<<<<<<< HEAD:src/imu.X
void imu_init ( void )  {
  if(DEBUG)  printf( "Initializing IMU \n" );
  led_blink( LED_IMU, 500, 500 );
  imu1.id = 1;  imu1.addr = 0x68;  imu_setup(&imu1);
  //imu2.id = 2;  imu2.addr = 0x69;  imu_setup(&imu2);
=======
void imu_init (  )  {
  if(DEBUG)  printf( "Initializing IMU \n" );

  // Start initialization
  led_blink( LED_IMU, 500, 500 );
  imu.id   = 1;
  imu.addr = 0x68;
  imu.gyr  = &gyr;
  imu.acc  = &acc;
  imu.mag  = &mag;

  // Init functions
  imu_param();
  imu_getcal();
  imu_setic();

  // IMU warmup period
  usleep(500000);
>>>>>>> altlog:src/imu.c
  led_on(LED_IMU);
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_setup
//  Setup functions for each MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
<<<<<<< HEAD:src/imu.X
void imu_setup ( imu_struct* imu )  {
  imu_param(imu);
  imu_getcal(imu);
  imu_setic(imu);
=======
void imu_exit ( void )  {
  if(DEBUG)  printf("Close IMU \n");
  // Add IMU exit code here...
>>>>>>> altlog:src/imu.c
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_param
//  Assign parameters to an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_param (  )  {

<<<<<<< HEAD:src/imu.X
  if(DEBUG) {  printf( "  Assigning IMU%d parameters ", imu->id );  fflush(stdout);  }
  linux_set_i2c_bus(1);
=======
  if(DEBUG) {  printf("  Assign IMU parameters ");  fflush(stdout);  }
  linux_set_i2c_bus(imu.id);
>>>>>>> altlog:src/imu.c

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_init_master(NULL) )
    printf( "Error (imu_param): 'mpu_init_master' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_sensors( INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS ) )
    printf( "Error (imu_param): 'mpu_set_sensors' failed. \n" );

<<<<<<< HEAD:src/imu.X
  sys.ret = mpu_set_sample_rate(HZ_IMU);
=======
>>>>>>> altlog:src/imu.c
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_sample_rate(HZ_IMU_FAST) )
    printf( "Error (imu_param): 'mpu_set_sample_rate' failed. \n" );

<<<<<<< HEAD:src/imu.X
  sys.ret = mpu_set_compass_sample_rate(HZ_ATT);
=======
>>>>>>> altlog:src/imu.c
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_compass_sample_rate(HZ_IMU_SLOW) )
    printf( "Error (imu_param): 'mpu_set_compass_sample_rate' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_gyro_fsr(GYR_FSR) )
    printf( "Error (imu_param): 'mpu_set_gyro_fsr' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_accel_fsr(ACC_FSR) )
    printf( "Error (imu_param): 'mpu_set_accel_fsr' failed. \n" );

  if(DEBUG)  printf(" complete \n");
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_getcal
//  Gets the calibration parameters for the MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
<<<<<<< HEAD:src/imu.X
void imu_getcal ( imu_struct* imu )  {
  if(DEBUG)  printf( "  IMU%d calibration values: \n", imu->id );
=======
void imu_getcal (  )  {
  if(DEBUG)  printf("  IMU calibration values: \n");
>>>>>>> altlog:src/imu.c

  // Local variables
  int i;
  FILE* f;
  char buff [32];  memset( buff, 0, sizeof(buff) );
  char path [32];  memset( path, 0, sizeof(path) );

<<<<<<< HEAD:src/imu.X
  // Set magnetometer offset
  sprintf( path, "/root/BlackBox/cal/imu%d/moffset", imu->id );
=======
  // Set acceleration bias
  sprintf( path, "cal/acc/bias" );
>>>>>>> altlog:src/imu.c
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File 'acc/bias' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    acc.bias[i] = atoi(buff);
  }
  fclose(f);

<<<<<<< HEAD:src/imu.X
  // Set magnetometer range
  sprintf( path, "/root/BlackBox/cal/imu%d/mrange", imu->id );
=======
  // Set acceleration range
  sprintf( path, "cal/acc/range" );
>>>>>>> altlog:src/imu.c
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File 'acc/range' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    acc.range[i] = atoi(buff);
  }
  fclose(f);

<<<<<<< HEAD:src/imu.X
  // Set acceleration offset
  sprintf( path, "/root/BlackBox/cal/imu%d/aoffset", imu->id );
=======
  // Set magnetometer bias
  sprintf( path, "cal/mag/bias" );
>>>>>>> altlog:src/imu.c
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File 'mag/bias' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    mag.bias[i] = atoi(buff);
  }
  fclose(f);

<<<<<<< HEAD:src/imu.X
  // Set acceleration range
  sprintf( path, "/root/BlackBox/cal/imu%d/arange", imu->id );
=======
  // Set magnetometer range
  sprintf( path, "cal/mag/range" );
>>>>>>> altlog:src/imu.c
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File 'mag/range' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    mag.range[i] = atoi(buff);
  }
  fclose(f);

  // Display calibration values
  if(DEBUG) {
    printf("    abias  arange    mbias  mrange \n");
    for ( i=0; i<3; i++ ) {
      printf("     ");
      printf( "%04d  ", acc.bias[i]  );
      printf( "%06d     ", acc.range[i] );
      printf( "%04d   ", mag.bias[i]  );
      printf( "%04d  \n", mag.range[i] );
    } 
  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_setic
//  Sets the initial conditions for the MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
<<<<<<< HEAD:src/imu.X
void imu_setic ( imu_struct* imu )  {
  if(DEBUG)  printf( "  Setting IMU%d initial conditions \n", imu->id );

  // Local vsariable
  ushort i, j;

  // Determine timing loops
  sys_err ( ( HZ_IMU % HZ_ATT != 0 ), "Error (imu_setic): 'HZ_IMU' must be a multiple of 'HZ_ATT'."   );
  imu->count = 0;
  imu->loops = HZ_IMU / HZ_ATT;

  // Assign sample rates
  imu->hz_imu = HZ_IMU;
  imu->hz_att = HZ_ATT;

  // Calculate time steps
  imu->dt_imu = 1.0/HZ_IMU;
  imu->dt_att = 1.0/HZ_ATT;

  // Assign low pass filter cutoff
  imu->lpf_gyr = GYR_LPF;
  imu->lpf_acc = ACC_LPF;
  imu->lpf_mag = MAG_LPF;

  // Determine time constants
  if ( GYR_LPF !=0.0 )  imu->tc_gyr = 1.0/GYR_LPF;  else imu->tc_gyr = 0.0;
  if ( ACC_LPF !=0.0 )  imu->tc_acc = 1.0/ACC_LPF;  else imu->tc_acc = 0.0;
  if ( MAG_LPF !=0.0 )  imu->tc_mag = 1.0/MAG_LPF;  else imu->tc_mag = 0.0;

  // Calculate filter gain values
  imu->gain_gyr = imu->dt_imu / ( imu->tc_gyr + imu->dt_imu );
  imu->gain_acc = imu->dt_imu / ( imu->tc_acc + imu->dt_imu );
  imu->gain_mag = imu->dt_att / ( imu->tc_mag + imu->dt_att );

  // Display settings
  if (DEBUG) {
    printf("    IMU ( %4d hz | %5.3f sec ) \n",  imu->hz_imu, imu->dt_imu );
    printf("    Att ( %4d hz | %5.3f sec ) \n",  imu->hz_att, imu->dt_att );
    printf("    Gyr:    LPF: %6.2f    TC: %5.2f    gain: %7.4f \n", imu->lpf_gyr, imu->tc_gyr, imu->gain_gyr );
    printf("    Acc:    LPF: %6.2f    TC: %5.2f    gain: %7.4f \n", imu->lpf_acc, imu->tc_acc, imu->gain_acc );
    printf("    Mag:    LPF: %6.2f    TC: %5.2f    gain: %7.4f \n", imu->lpf_mag, imu->tc_mag, imu->gain_mag );
  }

  // Clear moving average history
  for ( i=0; i<3; i++ ) {
    for ( j=0; j<GYR_HIST; j++ )  imu->histGyr[i][j] = 0;
    for ( j=0; j<ACC_HIST; j++ )  imu->histAcc[i][j] = 0;
    for ( j=0; j<MAG_HIST; j++ )  imu->histMag[i][j] = 0;
  }

  // Data fusion variables
  imu->fx = 0.5;  imu->fz = 0.866;
  for ( i=0; i<4; i++ ) {
    imu->Quat[i]  = 0;
    imu->dQuat[i] = 0;
    if (i<3) {
      imu->Eul[i]  = 0;
      imu->dEul[i] = 0;
      imu->bias[i] = 0;
    }
  }
  imu->Quat[0] = 1;
=======
void imu_setic (  )  {
  if(DEBUG)  printf("  Set IMU initial conditions \n");

  // Assign loop counter values
  if ( HZ_IMU_FAST % HZ_IMU_SLOW != 0 )
    printf( "  *** WARNING ***  Slow loop must divide evenly into fast loop. \n" );
  imu.loops  = HZ_IMU_FAST / HZ_IMU_SLOW;
  imu.count  = 0;
  imu.getmag = false;

  // Calculate time steps
  float gyr_dt, acc_dt, mag_dt;
  gyr_dt = 1.0 / HZ_IMU_FAST;
  acc_dt = 1.0 / HZ_IMU_FAST;
  mag_dt = 1.0 / HZ_IMU_SLOW;

  // Determine time constants
  float gyr_tc, acc_tc, mag_tc;
  if ( GYR_LPF != 0.0 )  gyr_tc = 1.0/GYR_LPF;  else  gyr_tc = 0.0;
  if ( ACC_LPF != 0.0 )  acc_tc = 1.0/ACC_LPF;  else  acc_tc = 0.0;
  if ( MAG_LPF != 0.0 )  mag_tc = 1.0/MAG_LPF;  else  mag_tc = 0.0;

  // Calculate filter gain values
  gyr.gain = gyr_dt / ( gyr_tc + gyr_dt );
  acc.gain = acc_dt / ( acc_tc + acc_dt );
  mag.gain = mag_dt / ( mag_tc + mag_dt );

  // Display settings
  if (DEBUG) {
    printf("    |  GYR  |  HZ %4d  |  DT %5.3f  |  LPF %6.2f  |  TC %5.2f  |  gain %7.4f  |\n", \
	   HZ_IMU_FAST, gyr_dt, GYR_LPF, gyr_tc, gyr.gain );
    printf("    |  ACC  |  HZ %4d  |  DT %5.3f  |  LPF %6.2f  |  TC %5.2f  |  gain %7.4f  |\n", \
	   HZ_IMU_FAST, acc_dt, ACC_LPF, acc_tc, acc.gain );
    printf("    |  MAG  |  HZ %4d  |  DT %5.3f  |  LPF %6.2f  |  TC %5.2f  |  gain %7.4f  |\n", \
	   HZ_IMU_SLOW, mag_dt, MAG_LPF, mag_tc, mag.gain );
  }
>>>>>>> altlog:src/imu.c

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_data
//  Obtain new IMU sensor data.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_data (  )  {

  // Local variables
<<<<<<< HEAD:src/imu.X
  //ushort i, j, k;
  //float g, a; //, m;
  //bool mag;
=======
  ushort i, j, k;
  float g, a, m;

  // Declare data history arrays
  static short ghist[3][GYR_HIST];
  static short ahist[3][ACC_HIST];
  static short mhist[3][MAG_HIST];
>>>>>>> altlog:src/imu.c

  // Try using local variables for history...

  /*
  // Increment counter
  imu.getmag = false;
  if ( imu.count == 0 ) {
    imu.getmag = true;
    imu.count = imu.loops;
  }
  imu.count--;

  // Lock IMU data
  pthread_mutex_lock(&mutex_imu);

  // Sample IMU
  if( mpu_get_gyro_reg( gyr.raw, NULL ) )
    printf( "Error (imu_data): 'mpu_get_gyro_reg' failed. \n" );
  if( mpu_get_accel_reg( acc.raw, NULL ) )
    printf( "Error (imu_data): 'mpu_get_accel_reg' failed. \n" );
  if(imu.getmag){
  if( mpu_get_compass_reg( mag.raw, NULL ) )
    printf( "Error (imu_data): 'mpu_get_compass_reg' failed. \n" );
  }
<<<<<<< HEAD:src/imu.X
  */
  /*
  // Sample IMU
  sys.ret = mpu_get_gyro_reg( imu->rawGyr, NULL );
  sys_err( sys.ret, "Error (imu_mems): 'mpu_get_gyro_reg' failed." );
  sys.ret = mpu_get_accel_reg( imu->rawAcc, NULL );
  sys_err( sys.ret, "Error (imu_mems): 'mpu_get_accel_reg' failed." );
  //if(mag) {
  //sys.ret = mpu_get_compass_reg( imu->rawMag, NULL );
  //sys_err( sys.ret, "Error (imu_mems): 'mpu_get_compass_reg' failed." );
  //}
  */

  //~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Gyroscope

  //for ( i=0; i<3; i++ ) { 
  //imu->rawGyr[i] = imu->rawGyr[i] + 2; 
  //if ( imu->rawGyr[i] > 10 )  imu->rawGyr[i] = -10;
  //}
  /*
  k = GYR_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  imu->histGyr[i][j-1] = imu->histGyr[i][j];
    imu->histGyr[i][k-1] = imu->rawGyr[i];
    g = (float) (imu->histGyr[i][0]);
    for ( j=1; j<k; j++ )  g = g + imu->gain_gyr * (float) ( imu->histGyr[i][j] - g );
    imu->avgGyr[i] = g;
=======

  // Gyroscope low pass filter
  k = GYR_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  ghist[i][j-1] = ghist[i][j];
    ghist[i][k-1] = gyr.raw[i];
    g = (float) (ghist[i][0]);
    for ( j=1; j<k; j++ )  g = g + gyr.gain * (float) ( ghist[i][j] - g );
    gyr.avg[i] = g;
>>>>>>> altlog:src/imu.c
  }
  */

<<<<<<< HEAD:src/imu.X
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Accelerometer

  //for ( i=0; i<3; i++ ) { 
  //imu->rawAcc[i] = imu->rawAcc[i] + 5; 
  //if ( imu->rawAcc[i] > 50 )  imu->rawAcc[i] = -50;
  //}
  /*
  k = ACC_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  imu->histAcc[i][j-1] = imu->histAcc[i][j];
    imu->histAcc[i][k-1] = imu->rawAcc[i];
    a = (float) (imu->histAcc[i][0]);
    for ( j=1; j<k; j++ )  a = a + imu->gain_acc * (float) ( imu->histAcc[i][j] - a );
    imu->avgAcc[i] = a;
  }
  */
  /*
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Magnetometer
  if(mag) {

  //for ( i=0; i<3; i++ ) { 
  //imu->rawMag[i] = imu->rawMag[i] + 10; 
  //if ( imu->rawMag[i] > 100 )  imu->rawMag[i] = -100;
  //}

  k = MAG_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  imu->histMag[i][j-1] = imu->histMag[i][j];
    imu->histMag[i][k-1] = imu->rawMag[i];
    m = (float) (imu->histMag[i][0]);
    for ( j=1; j<k; j++ )  m = m + imu->gain_mag * (float) ( imu->histMag[i][j] - m );
    imu->avgMag[i] = m;
  }
  }
  */

  // Lock 'calibrated' variables
  //pthread_mutex_lock(&mutex_imu);
  /*
  // Scale and orient gyroscope readings
  imu->calGyr[X] =  imu->avgGyr[Y] * GYR_SCALE;
  imu->calGyr[Y] =  imu->avgGyr[X] * GYR_SCALE;
  imu->calGyr[Z] = -imu->avgGyr[Z] * GYR_SCALE;

  // Shift and orient accelerometer readings
  imu->calAcc[X] = ( -imu->avgAcc[Y] - imu->aoffset[Y] ) / (double)imu->arange[Y];
  imu->calAcc[Y] = ( -imu->avgAcc[X] - imu->aoffset[X] ) / (double)imu->arange[X];
  imu->calAcc[Z] = (  imu->avgAcc[Z] - imu->aoffset[Z] ) / (double)imu->arange[Z];
  */
  /*
  // Shift and orient magnetometer readings
  if (mag) {
  imu->calMag[X] = ( imu->avgMag[X] - imu->moffset[X] ) / (double)imu->mrange[X];
  imu->calMag[Y] = ( imu->avgMag[Y] - imu->moffset[Y] ) / (double)imu->mrange[Y];
  imu->calMag[Z] = ( imu->avgMag[Z] - imu->moffset[Z] ) / (double)imu->mrange[Z];
=======
  // Accelerometer low pass filter
  k = ACC_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  ahist[i][j-1] = ahist[i][j];
    ahist[i][k-1] = acc.raw[i];
    a = (float) (ahist[i][0]);
    for ( j=1; j<k; j++ )  a = a + acc.gain * (float) ( ahist[i][j] - a );
    acc.avg[i] = a;
  }

  // Magnetometer low pass filter
  if(imu.getmag) {
  k = MAG_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  mhist[i][j-1] = mhist[i][j];
    mhist[i][k-1] = mag.raw[i];
    m = (float) (mhist[i][0]);
    for ( j=1; j<k; j++ )  m = m + mag.gain * (float) ( mhist[i][j] - m );
    mag.avg[i] = m;
  }}

  // Shift and orient gyroscope readings (add bias)
  gyr.cal[X] =   ( gyr.avg[Y] - 0 ) * GYR_SCALE;
  gyr.cal[Y] =   ( gyr.avg[X] - 0 ) * GYR_SCALE;
  gyr.cal[Z] = - ( gyr.avg[Z] - 0 ) * GYR_SCALE;

  // Shift and orient accelerometer readings
  acc.cal[X] = - ( acc.avg[Y] - acc.bias[Y] ) / (double) (acc.range[Y]);
  acc.cal[Y] = - ( acc.avg[X] - acc.bias[X] ) / (double) (acc.range[X]);
  acc.cal[Z] =   ( acc.avg[Z] - acc.bias[Z] ) / (double) (acc.range[Z]);

  // Shift and orient magnetometer readings
  if(imu.getmag) {
  mag.cal[X] = ( mag.avg[X] - mag.bias[X] ) / (double) (mag.range[X]);
  mag.cal[Y] = ( mag.avg[Y] - mag.bias[Y] ) / (double) (mag.range[Y]);
  mag.cal[Z] = ( mag.avg[Z] - mag.bias[Z] ) / (double) (mag.range[Z]);
>>>>>>> altlog:src/imu.c
  }
  */

<<<<<<< HEAD:src/imu.X
  // Unlock 'calibrated' variables
  //pthread_mutex_unlock(&mutex_imu);
=======
  // Unlock IMU data
  pthread_mutex_unlock(&mutex_imu);
>>>>>>> altlog:src/imu.c

  return;
}








/*
  // Data fusion variables
  imu->fx = 0.5;  imu->fz = 0.866;
  for ( i=0; i<4; i++ ) {
    imu->Prev[i]  = 0;
    imu->Quat[i]  = 0;
    imu->dQuat[i] = 0;
    if (i<3) {
      imu->Eul[i]  = 0;
      imu->dEul[i] = 0;
      imu->bias[i] = 0;
    }
  }
  imu->Prev[0] = 1;
  imu->Quat[0] = 1;
*/


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_fusion
//  Applies sensor data fusion algorithm. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void imu_fusion ( imu_struct* imu )  {

  // Local variables
  unsigned short i;
  double norm;

  // Quaternion index
  unsigned short w=0, x=1, y=2, z=3;

  // Get values from mpu structure
  double q[4], m[3], a[3], g[3], b[3], fx, fz, dt;
  fx = imu->fx;  fz = imu->fz;  dt = imu->dt_att;
  //pthread_mutex_lock(&mutex_imu);
  for ( i=0; i<4; i++ ) {
    q[i] = imu->Quat[i];
    if (i<3) {
      g[i] = imu->calGyr[i];
      a[i] = imu->calAcc[i];
      m[i] = imu->calMag[i];
      b[i] = imu->bias[i];
    }
  }
  //pthread_mutex_unlock(&mutex_imu);

  // Normalize magnetometer
  norm = 0.0;
  for ( i=0; i<3; i++ )  norm += m[i] * m[i];
  norm = sqrt(norm);
  for ( i=0; i<3; i++ )  m[i] /= norm;

  // Normalize accelerometer
  norm = 0.0;
  for ( i=0; i<3; i++ )  norm += a[i] * a[i];
  norm = sqrt(norm);
  for ( i=0; i<3; i++ )  a[i] /= norm;

  // Define auxiliary vectors
  double halfq[4], twoq[4], twom[3], twofxq[4], twofzq[4], twofx, twofz;
  twofx = 2.0f * fx;  twofz = 2.0f * fz;
  for ( i=0; i<4; i++ ) {
    halfq[i]  =  0.5f * q[i];
    twoq[i]   =  2.0f * q[i];
    twofxq[i] = twofx * q[i];
    twofzq[i] = twofz * q[i];
    if(i<3)  twom[i] = 2.0f * m[i];
  }

  // Quaternion multiplication values 
  double qwx, qwy, qwz, qxy, qxz, qyz, qxx, qyy, qzz;
  qwx = q[w] * q[x];  qwy = q[w] * q[y];  qwz = q[w] * q[z];
  qxy = q[x] * q[y];  qxz = q[x] * q[z];  qyz = q[y] * q[z];
  qxx = q[x] * q[x];  qyy = q[y] * q[y];  qzz = q[z] * q[z];

  // Calculate objective function
  double F1, F2, F3, F4, F5, F6;
  F1 =        twoq[x] * q[z]        - twoq[w] * q[y]               - a[X];
  F2 =        twoq[w] * q[x]        + twoq[y] * q[z]               - a[Y];
  F3 = 1.0f - twoq[x] * q[x]        - twoq[y] * q[y]               - a[Z];
  F4 = twofx * ( 0.5f - qyy - qzz ) + twofz * (        qxz - qwy ) - m[X]; 
  F5 = twofx * (        qxy - qwz ) + twofz * (        qwx + qyz ) - m[Y];
  F6 = twofx * (        qwy + qxz ) + twofz * ( 0.5f - qxx - qyy ) - m[Z];

  // Calculate jacobian matrix
  double J11J24, J12J23, J13J22, J14J21, J32, J33;
  double J41, J42, J43, J44;
  double J51, J52, J53, J54;
  double J61, J62, J63, J64;
  J11J24 = twoq[y];
  J12J23 = 2.0f * q[z];
  J13J22 = twoq[w];
  J14J21 = twoq[x];
  J32 = 2.0f * J14J21;
  J33 = 2.0f * J11J24;
  J41 = twofzq[y];
  J42 = twofzq[z];
  J43 = 2.0f * twofxq[y] + twofzq[w]; 
  J44 = 2.0f * twofxq[z] - twofzq[x];
  J51 = twofxq[z] - twofzq[x];
  J52 = twofxq[y] + twofzq[w];
  J53 = twofxq[x] + twofzq[z];
  J54 = twofxq[w] - twofzq[y];
  J61 = twofxq[y];
  J62 = twofxq[z] - 2.0f * twofzq[x];
  J63 = twofxq[w] - 2.0f * twofzq[y];
  J64 = twofxq[x];

  // Quaternion gradient from objective function
  double qdf[4];
  qdf[w] = - J11J24 * F1 + J14J21 * F2            - J41 * F4 - J51 * F5 + J61 * F6;
  qdf[x] =   J12J23 * F1 + J13J22 * F2 - J32 * F3 + J42 * F4 + J52 * F5 + J62 * F6;
  qdf[y] = - J13J22 * F1 + J12J23 * F2 - J33 * F3 - J43 * F4 + J53 * F5 + J63 * F6;
  qdf[z] =   J14J21 * F1 + J11J24 * F2            - J44 * F4 - J54 * F5 + J64 * F6;

  // Normalize gradient
  norm = 0.0;
  for ( i=0; i<4; i++ )  norm += qdf[i] * qdf[i];
  norm = sqrt(norm);
  for ( i=0; i<4; i++ )  qdf[i] /= norm;

  // Compute gyroscope error
  double err[3];
  err[X] = twoq[w] * qdf[x] - twoq[x] * qdf[w] - twoq[y] * qdf[z] + twoq[z] * qdf[y];
  err[Y] = twoq[w] * qdf[y] + twoq[x] * qdf[z] - twoq[y] * qdf[w] - twoq[z] * qdf[x];
  err[Z] = twoq[w] * qdf[z] - twoq[x] * qdf[y] + twoq[y] * qdf[x] - twoq[z] * qdf[w];

  // Adjust for gyroscope baises
  for ( i=0; i<3; i++ ) {
    b[i] += err[i] * dt * IMU_ZETA;
    g[i] -= b[i];
  }

  // Quaternion derivative from rate gyro
  double qdr[4];
  qdr[w] = -halfq[x] * g[X] - halfq[y] * g[Y] - halfq[z] * g[Z];
  qdr[x] =  halfq[w] * g[X] + halfq[y] * g[Z] - halfq[z] * g[Y];
  qdr[y] =  halfq[w] * g[Y] - halfq[x] * g[Z] + halfq[z] * g[X];
  qdr[z] =  halfq[w] * g[Z] + halfq[x] * g[Y] - halfq[y] * g[X];

  // Fused quaternion and derivative values
  double qd[4];
  for ( i=0; i<4; i++ ) {
    qd[i] = qdr[i] - ( IMU_BETA * qdf[i] );
    q[i] += qd[i] * dt;
  }

  // Normalise quaternion
  norm = 0.0;
  for ( i=0; i<4; i++ )  norm += q[i] * q[i];
  norm = sqrt(norm);
  for ( i=0; i<4; i++ )  q[i] /= norm;

  // Compute earth frame flux
  float h[3];
  h[X] = twom[X] * ( 0.5f - qyy - qzz ) + twom[Y] * ( qxy - qwz ) + twom[Z] * ( qxz + qwy );
  h[Y] = twom[X] * ( qxy + qwz ) + twom[Y] * ( 0.5f - qxx - qzz ) + twom[Z] * ( qyz - qwx ); 
  h[Z] = twom[X] * ( qxz - qwy ) + twom[Y] * ( qyz + qwx ) + twom[Z] * ( 0.5f - qxx - qyy );

  // Adjust flux vector
  fx = sqrt( ( h[X] * h[X] ) + ( h[Y] * h[Y] ) );
  fz = h[Z];

  // Calculate euler angles
  double e[3];
  e[X] = atan2 ( ( 2* ( qwx + qyz ) ), ( 1- 2* ( qxx + qyy ) ) ) - R_BIAS;
  e[Y] = asin  (   2* ( qwy - qxz ) )                            - P_BIAS;
  e[Z] = atan2 ( ( 2* ( qwz + qxy ) ), ( 1- 2* ( qyy + qzz ) ) ) - Y_BIAS;

  // Update imu structure
  //pthread_mutex_lock(&mutex_fusion);
  imu->fx = fx;  imu->fz = fz;
  for ( i=0; i<4; i++ ) {
    imu->Quat[i]  = q[i];
    imu->dQuat[i] = qd[i];
    if(i<3) {
      imu->Eul[i]  = e[i];
      imu->dEul[i] = g[i];
      imu->bias[i] = b[i];
    }
  }
  //pthread_mutex_unlock(&mutex_fusion);

  return;
}
*/


  /*
  //~~~  MOVE TO 'IMU_CONV' FUNCTION  ~~~//
  
  double temp[3];
  for( i=0; i<3; i++ )  temp[i] = imu->Eul[i] - imu->Prev[i];

  // Norm of delta
  norm = 0.0;
  for ( i=0; i<3; i++ )  norm += temp[i] * temp[i];
  norm = sqrt(norm);

  // Debugging statements
  printf("E: %7.4f %7.4f %7.4f    ", imu->Eul[0], imu->Eul[1], imu->Eul[2] );
  printf("D: %7.4f %7.4f %7.4f    ", temp[0], temp[1], temp[2] );
  printf("N: %7.4f", norm);
  //printf("E: %7.4f  A: %7.4f    ", imu->Eul[0], temp );
  printf("\n");
  fflush(stdout);

  // Assign previous value
  for ( i=0; i<3; i++ )  imu->Prev[i] = imu->Eul[i];

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
  */


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_exit
//  Terminate an IMU device.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_exit ( void )  {
  if(DEBUG)  printf("Closing IMU \n");
  return;
}



