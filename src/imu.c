
//============================================================
//  imu.c
//  Justin M Selfridge
//============================================================
#include "imu.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_init
//  Initializes the MPU sensors.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_init ( void )  {

  if(DEBUG)  printf( "Initializing IMU \n" );

  led_blink( LED_IMU, 500, 500 );

  imu1.id = 1;  imu1.addr = 0x68;  imu_setup(&imu1);
  //imu2.id = 2;  imu2.addr = 0x69;  imu_setup(&imu2);

  led_on(LED_IMU);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_setup
//  Setup functions for each MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_setup ( imu_struct* imu )  {

  // Init functions
  imu_param(imu);
  imu_getcal(imu);
  imu_setic(imu);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_param
//  Assign parameters to an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_param ( imu_struct* imu )  {

  if(DEBUG) {  printf( "  Assigning IMU%d parameters ", imu->id );  fflush(stdout);  }
  linux_set_i2c_bus(1);

  sys.ret = mpu_init_master(NULL);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_init_master' failed." );

  sys.ret = mpu_set_sensors( INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS );
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_set_sensors' failed." );

  sys.ret = mpu_set_sample_rate(FAST_HZ);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_set_sample_rate' failed." );

  sys.ret = mpu_set_compass_sample_rate(SLOW_HZ);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_set_compass_sample_rate' failed." );

  sys.ret = mpu_set_gyro_fsr(GYR_FSR);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_set_gyro_fsr' failed." );

  sys.ret = mpu_set_accel_fsr(ACC_FSR);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_set_accel_fsr' failed." );

  if(DEBUG)  printf(" complete \n");
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_getcal
//  Gets the calibration parameters for the MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_getcal ( imu_struct* imu )  {
  if(DEBUG)  printf( "  IMU%d calibration values: \n", imu->id );

  // Local variables
  int i;
  FILE* f;
  char buff [32];
  char path [32];
  memset( buff, 0, sizeof(buff) );

  // Set magnetometer offset
  sprintf( path, "cal/imu%d/moffset", imu->id );
  f = fopen( path, "r" );
  sys_err( !f, "Error (imu_getcal): File 'moffset' not found." );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->moffset[i] = atoi(buff);
  }
  fclose(f);

  // Set magnetometer range
  sprintf( path, "cal/imu%d/mrange", imu->id );
  f = fopen( path, "r" );
  sys_err( !f, "Error (imu_getcal): File 'mrange' not found." );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->mrange[i] = atoi(buff);
  }
  fclose(f);

  // Set acceleration offset
  sprintf( path, "cal/imu%d/aoffset", imu->id );
  f = fopen( path, "r" );
  sys_err( !f, "Error (imu_getcal): File 'aoffset' not found." );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->aoffset[i] = atoi(buff);
  }
  fclose(f);

  // Set acceleration range
  sprintf( path, "cal/imu%d/arange", imu->id );
  f = fopen( path, "r" );
  sys_err( !f, "Error (imu_getcal): File 'arange' not found." );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->arange[i] = atoi(buff);
  }
  fclose(f);

  // Display calibration values
  if(DEBUG) {
    printf("    moffset  mrange  aoffset  arange \n");
    for ( i=0; i<3; i++ ) {
      printf("      ");
      printf( "%04d    ", imu->moffset[i] );
      printf( "%04d    ", imu->mrange[i]  );
      printf( "%04d    ", imu->aoffset[i] );
      printf( "%06d  \n", imu->arange[i]  );
    } 
  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_setic
//  Sets the initial conditions for the MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_setic ( imu_struct* imu )  {
  if(DEBUG)  printf( "  Setting IMU%d initial conditions \n", imu->id );

  // Local vsariable
  ushort i, j;

  // Determine timing loops
  sys_err ( ( FAST_HZ % SLOW_HZ   != 0 ), "Error (imu_setic): 'FAST_HZ' must be a multiple of 'SLOW_HZ'."   );
  sys_err ( ( SLOW_HZ % FUSION_HZ != 0 ), "Error (imu_setic): 'SLOW_HZ' must be a multiple of 'FUSION_HZ'." );
  imu->count = 0;
  imu->loops = FAST_HZ / SLOW_HZ;

  // Assign sample rates
  imu->gyr_hz = FAST_HZ;
  imu->acc_hz = FAST_HZ;
  imu->mag_hz = SLOW_HZ;
  imu->fus_hz = FUSION_HZ;

  // Calculate time steps
  imu->gyr_dt = 1.0/FAST_HZ;
  imu->acc_dt = 1.0/FAST_HZ;
  imu->mag_dt = 1.0/SLOW_HZ;
  imu->fus_dt = 1.0/FUSION_HZ;

  // Assign low pass filter cutoff
  imu->gyr_lpf = GYR_LPF;
  imu->acc_lpf = ACC_LPF;
  imu->mag_lpf = MAG_LPF;

  // Determine time constants
  if ( GYR_LPF !=0.0 )  imu->gyr_tc = 1.0/GYR_LPF;  else imu->gyr_tc = 0.0;
  if ( ACC_LPF !=0.0 )  imu->acc_tc = 1.0/ACC_LPF;  else imu->acc_tc = 0.0;
  if ( MAG_LPF !=0.0 )  imu->mag_tc = 1.0/MAG_LPF;  else imu->mag_tc = 0.0;

  // Calculate filter gain values
  imu->gyr_gain = imu->gyr_dt / ( imu->gyr_tc + imu->gyr_dt );
  imu->acc_gain = imu->acc_dt / ( imu->acc_tc + imu->acc_dt );
  imu->mag_gain = imu->mag_dt / ( imu->mag_tc + imu->mag_dt );

  // Display IMU settings
  if (DEBUG) {
    printf("    Gyroscope:       \
    HZ: %4d    DT: %5.3f    LPF: %6.2f    TC: %5.2f    gain: %7.4f  \n", \
    imu->gyr_hz, imu->gyr_dt, imu->gyr_lpf, imu->gyr_tc, imu->gyr_gain );
    printf("    Accelerometer:   \
    HZ: %4d    DT: %5.3f    LPF: %6.2f    TC: %5.2f    gain: %7.4f  \n", \
    imu->acc_hz, imu->acc_dt, imu->acc_lpf, imu->acc_tc, imu->acc_gain );
    printf("    Magnetometer:    \
    HZ: %4d    DT: %5.3f    LPF: %6.2f    TC: %5.2f    gain: %7.4f  \n", \
    imu->mag_hz, imu->mag_dt, imu->mag_lpf, imu->mag_tc, imu->mag_gain );
    printf("    Data Fusion:     \
    HZ: %4d    DT: %5.3f  \n", \
    imu->fus_hz, imu->fus_dt );
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

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_data
//  Obtain new IMU device data.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_data ( imu_struct* imu )  {

  // Local variables
  ushort i, j, k;
  float g, a, m;
  bool mag;

  // Increment counter
  mag = false;
  imu->count++;
  if ( imu->count == imu->loops ) {
    mag = true;
    imu->count = 0;
  }

  // Gyroscope
  sys.ret = mpu_get_gyro_reg( imu->rawGyr, NULL );
  sys_err( sys.ret, "Error (imu_mems): 'mpu_get_gyro_reg' failed." );
  k = GYR_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  imu->histGyr[i][j-1] = imu->histGyr[i][j];
    imu->histGyr[i][k-1] = imu->rawGyr[i];
    g = (float) (imu->histGyr[i][0]);
    for ( j=1; j<k; j++ )  g = g + imu->gyr_gain * (float) ( imu->histGyr[i][j] - g );
    imu->avgGyr[i] = g;
  }

  // Accelerometer
  sys.ret = mpu_get_accel_reg( imu->rawAcc, NULL );
  sys_err( sys.ret, "Error (imu_mems): 'mpu_get_accel_reg' failed." );
  k = ACC_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  imu->histAcc[i][j-1] = imu->histAcc[i][j];
    imu->histAcc[i][k-1] = imu->rawAcc[i];
    a = (float) (imu->histAcc[i][0]);
    for ( j=1; j<k; j++ )  a = a + imu->acc_gain * (float) ( imu->histAcc[i][j] - a );
    imu->avgAcc[i] = a;
  }

  // Magnetometer
  if(mag) {
  sys.ret = mpu_get_compass_reg( imu->rawMag, NULL );
  sys_err( sys.ret, "Error (imu_mems): 'mpu_get_compass_reg' failed." );
  k = MAG_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  imu->histMag[i][j-1] = imu->histMag[i][j];
    imu->histMag[i][k-1] = imu->rawMag[i];
    m = (float) (imu->histMag[i][0]);
    for ( j=1; j<k; j++ )  m = m + imu->mag_gain * (float) ( imu->histMag[i][j] - m );
    imu->avgMag[i] = m;
  }
  }

  // Lock 'calibrated' variables
  pthread_mutex_lock(&mutex_cal);  

  // Scale and orient gyroscope readings
  imu->calGyr[X] = -imu->avgGyr[Y] * GYR_SCALE;
  imu->calGyr[Y] = -imu->avgGyr[X] * GYR_SCALE;
  imu->calGyr[Z] = -imu->avgGyr[Z] * GYR_SCALE;

  // Shift and orient accelerometer readings
  imu->calAcc[X] = ( imu->avgAcc[Y] - imu->aoffset[Y] ) / (double)imu->arange[Y];
  imu->calAcc[Y] = ( imu->avgAcc[X] - imu->aoffset[X] ) / (double)imu->arange[X];
  imu->calAcc[Z] = ( imu->avgAcc[Z] - imu->aoffset[Z] ) / (double)imu->arange[Z];

  // Shift and orient magnetometer readings
  if (mag) {
  imu->calMag[X] = -( imu->avgMag[X] - imu->moffset[X] ) / (double)imu->mrange[X];
  imu->calMag[Y] = -( imu->avgMag[Y] - imu->moffset[Y] ) / (double)imu->mrange[Y];
  imu->calMag[Z] =  ( imu->avgMag[Z] - imu->moffset[Z] ) / (double)imu->mrange[Z];
  }

  // Unlock 'calibrated' variables
  pthread_mutex_unlock(&mutex_cal);  

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_fusion
//  Applies sensor data fusion algorithm. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_fusion ( imu_struct* imu )  {

  // Local variables
  unsigned short i;
  double norm;

  // Quaternion index
  unsigned short w=0, x=1, y=2, z=3;

  // Get values from mpu structure
  double q[4], m[3], a[3], g[3], b[3], fx, fz, dt;
  fx = imu->fx;  fz = imu->fz;  dt = imu->fus_dt;
  pthread_mutex_lock(&mutex_cal);
  for ( i=0; i<4; i++ ) {
    q[i] = imu->Quat[i];
    if (i<3) {
      g[i] = imu->calGyr[i];
      a[i] = imu->calAcc[i];
      m[i] = imu->calMag[i];
      b[i] = imu->bias[i];
    }
  }
  pthread_mutex_unlock(&mutex_cal);

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
  pthread_mutex_lock(&mutex_fusion);
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
  pthread_mutex_unlock(&mutex_fusion);

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

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_exit
//  Terminate an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_exit ( void )  {
  if(DEBUG)  printf("Closing IMU \n");
  //sys.ret = mpu_set_dmp_state(0);
  //sys_err( sys.ret, "Error (imu_exit): 'mpu_set_dmp_state' failed." );
  return;
}



