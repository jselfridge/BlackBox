
//============================================================
//  imu.c
//  Justin M Selfridge
//============================================================
#include "imu.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_init
//  Initializes an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_init ( imu_struct* imu )  {

  // Start initialization
  char bus[1];
  sprintf( bus, "%d", imu->bus );
  if(DEBUG)  printf( "Initializing IMU%s \n", bus );
  led_blink( LED_MPU, 500, 500 );

  // Init functions
  imu_param(imu);
  //imu_setcal(imu);
  //imu_conv(imu);
  //imu_setic(imu);

  // Indicate init completed
  led_on(LED_MPU);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_exit
//  Terminate an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_exit ( void )  {
  if(DEBUG)  printf("  Closing IMU \n");
  sys.ret = mpu_set_dmp_state(0);
  sys_err( sys.ret, "Error (imu_exit): 'mpu_set_dmp_state' failed." );
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_param
//  Assign parameters to an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_param ( imu_struct* imu )  {

  // Modify this later
  //const signed char R[9] = { 1,0,0, 0,1,0, 0,0,1 };

  if(DEBUG) {  printf("  Assigning IMU parameters ");  fflush(stdout);  }
  linux_set_i2c_bus(imu->bus);

  sys.ret = mpu_init_master(NULL);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_init_master' failed." );

  sys.ret = mpu_set_sensors( INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS );
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_set_sensors' failed." );

  sys.ret = mpu_configure_fifo( INV_XYZ_GYRO | INV_XYZ_ACCEL );
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_configure_fifo' failed." );

  sys.ret = mpu_set_sample_rate(1000);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_set_sample_rate' failed." );

  sys.ret = mpu_set_compass_sample_rate(100);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_set_compass_sample_rate' failed." );

  //sys.ret = mpu_set_lpf(5);
  //if(DEBUG) {  printf(".");  fflush(stdout);  }
  //sys_err( sys.ret, "Error (imu_init): 'mpu_set_lpf' failed." );

  //sys.ret = dmp_load_motion_driver_firmware();
  //if(DEBUG) {  printf(".");  fflush(stdout);  }
  //sys_err( sys.ret, "Error (imu_init): 'dmp_load_motion_driver_firmware' failed." );

  //sys.ret = dmp_set_orientation( mpu_orient(mpu->rot) );
  //sys.ret = dmp_set_orientation( imu_orient(R) );
  //if(DEBUG) {  printf(".");  fflush(stdout);  }
  //sys_err( sys.ret, "Error (imu_init): 'dmp_set_orientation' failed." );

  //sys.ret = dmp_enable_feature( 
    //DMP_FEATURE_6X_LP_QUAT | 
    //DMP_FEATURE_SEND_RAW_ACCEL | 
    //DMP_FEATURE_SEND_CAL_GYRO | 
    //DMP_FEATURE_GYRO_CAL );
  //if(DEBUG) {  printf(".");  fflush(stdout);  }
  //sys_err( sys.ret, "Error (imu_init): 'dmp_enable_feature' failed." );

  //sys.ret = dmp_set_fifo_rate(200);
  //if(DEBUG) {  printf(".");  fflush(stdout);  }
  //sys_err( sys.ret, "Error (imu_init): 'dmp_set_fifo_rate' failed." );

  //sys.ret = mpu_set_dmp_state(1);
  //if(DEBUG) {  printf(".");  fflush(stdout);  }
  //sys_err( sys.ret, "Error (imu_init): 'mpu_set_dmp_state' failed." );

  sys.ret = mpu_set_gyro_fsr(500);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_set_gyro_fsr' failed." );

  sys.ret = mpu_set_accel_fsr(4);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_set_accel_fsr' failed." );

  if(DEBUG)  printf(" complete \n");
  return;
}

/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_setcal
//  Sets the calibration parameters for the MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_setcal ( imu_struct* imu )  {
  if(DEBUG)  printf("  IMU calibration values: \n");

  // Local variables
  int i;
  FILE* f;
  char buff [32];
  char path [32];
  memset( buff, 0, sizeof(buff) );

  // Set magnetometer offset
  sprintf( path, "cal/imu%d/moffset", imu->bus );
  f = fopen( path, "r" );
  if (!f)  {  printf("Error (imu_setcal): File 'moffset' not found. \n");  return;  }
  for ( i=0; i<3; i++ ) {
    if ( !fgets( buff, 32, f ) )  printf("Error: Failed to read 'moffset' file. \n");
    imu->moffset[i] = atoi(buff);
  }
  fclose(f);

  // Set magnetometer range
  sprintf( path, "cal/imu%d/mrange", imu->bus );
  f = fopen( path, "r" );
  if (!f)  {  printf("Error (imu_setcal): File 'mrange' not found. \n");  return;  }
  for ( i=0; i<3; i++ ) {
    if ( !fgets( buff, 32, f ) )  printf("Error: Failed to read 'mrange' file. \n");
    imu->mrange[i] = atoi(buff);
  }
  fclose(f);

  // Set acceleration offset
  sprintf( path, "cal/imu%d/aoffset", imu->bus );
  f = fopen( path, "r" );
  if (!f)  {  printf("Error (imu_setcal): File 'aoffset' not found. \n");  return;  }
  for ( i=0; i<3; i++ ) {
    if ( !fgets( buff, 32, f ) )  printf("Error: Failed to read 'aoffset' file. \n");
    imu->aoffset[i] = atoi(buff);
  }
  fclose(f);

  // Set acceleration range
  sprintf( path, "cal/imu%d/arange", imu->bus );
  f = fopen( path, "r" );
  if (!f)  {  printf("Error (imu_setcal): File 'arange' not found. \n");  return;  }
  for ( i=0; i<3; i++ ) {
    if ( !fgets( buff, 32, f ) )  printf("Error: Failed to read 'arange' file. \n");
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
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_conv
//  Allows the sensor heading to converge after initialization. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_conv ( imu_struct* imu )  {
  if(DEBUG)  printf("  Determining IMU heading:    "); 
*/
  /*
  double magx, magy;
  ushort loop = 1000;
  while(loop--) {  mpu_raw();  mpu_cal();  }
  magx = mpu.calMag[X];  magy = mpu.calMag[Y];
  heading = -atan2(magy,magx);
  printf("magx: %6.3f   magy: %6.3f   angle: %6.3f \n", magx, magy, heading*(180.0f/PI) );
  led_on(LED_MPU);
  */
  /*
  led_blink( LED_MPU, 500, 500 );
  while( counter < FREQ ) {
    mpu_sample();
    curr_head = mpu.Eul[Z];
    factor = curr_head / prev_head;
    if ( fabs(1.0-factor) < DT/30.0 ) counter++;
    prev_head = curr_head;
    if(DEBUG) {  printf("\b\b\b\b\b\b\b\b");  printf( "%8.3f", curr_head*(180.0f/PI) );  fflush(stdout);  }
    usleep(DT*1000000);
  }
  heading = curr_head;
  if(DEBUG) {  printf(" ...... locked \n");  fflush(stdout);  }
  led_on(LED_MPU);
  */
/*
  ctrl.heading = -45* (PI/180.0);
  if(DEBUG)  printf("hard coded as %6.3f \n", ctrl.heading*(180.0/PI) );

  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_setic
//  Sets the initial conditions for the MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_setic ( imu_struct* imu )  {
  if(DEBUG)  printf("  Setting IMU initial conditions \n");

  unsigned short i=1;
  imu->fx = 0.5;  imu->fz = 0.866;
  for ( i=0; i<4; i++ ) {
    imu->dQuat[i] = 0;
    if (i<3) {
      imu->Eul[i]  = 0;
      imu->dEul[i] = 0;
      imu->bias[i] = 0;
    }
  }
  imu->Quat[0] = 1;
  imu->Eul[Z] = ctrl.heading;

  imu->Quat[0] = cos(ctrl.heading/2);
  imu->Quat[1] = 0;
  imu->Quat[2] = 0;
  imu->Quat[3] = sin(ctrl.heading/2);

  imu->weight[0]  = 0.203125;
  imu->weight[1]  = 0.203125;
  imu->weight[2]  = 0.125000;
  imu->weight[3]  = 0.125000;
  imu->weight[4]  = 0.078125;
  imu->weight[5]  = 0.078125;
  imu->weight[6]  = 0.046875;
  imu->weight[7]  = 0.046875;
  imu->weight[8]  = 0.031250;
  imu->weight[9]  = 0.031250;
  imu->weight[10] = 0.015625;
  imu->weight[11] = 0.015625;

  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_avail
//  Check the MPU interupt to see if new data is avialable.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int imu_avail ( void )  {
  short status;
  ret = mpu_get_int_status(&status);
  sys_err( ret<0, "Error (imu_avail): 'mpu_get_int_status' failed." );
  return ( status == ( MPU_INT_STATUS_DATA_READY | MPU_INT_STATUS_DMP | MPU_INT_STATUS_DMP_0 ) );
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_clear
//  Clears the FIFO buffer on the MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*
int imu_clear ( void )  {
  unsigned char ii;
  unsigned char data;
  for (ii = 0; ii < st.hw->num_reg; ii++) {
    if (ii == st.reg->fifo_r_w || ii == st.reg->mem_r_w)
      continue;
    if (i2c_read(st.hw->addr, ii, 1, &data))
      return -1;
    log_i("%#5x: %#5x\r\n", ii, data);
  }
  return 0;
}
*/

/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_raw
//  Obtains raw data from MPU sensor and maps to body frame.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_raw ( imu_struct* imu )  {

  // Local variables 
  short sensors;
  unsigned char more = 1;

  // Check for new data
  if ( imu_avail() ) {

    // Obtain magnetometer values
    ret = mpu_get_compass_reg( imu->rawMag, &imu->magTime );
    sys_err( ret<0, "Error (imu_raw): 'mpu_get_compass_reg' failed." );

    // Obatin gyro, acc, and quat values
    while (more) {
      ret = dmp_read_fifo( imu->rawGyro, imu->rawAcc, imu->rawQuat, &imu->dmpTime, &sensors, &more );
      sys_err( ret<0, "Error (imu_raw): 'dmp_read_fifo' failed. ");

    }
  }
  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_avg
//  Applies a moving average to the raw data.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_avg ( imu_struct* imu )  {

  // Local variables
  unsigned short i, j;

  // Loop through elements
  for ( i=0; i<3; i++ ) {

    // Shift raw data history
    for ( j=11; j>0; j-- ) {
      imu->histMag[i][j]  = imu->histMag[i][j-1];
      imu->histAcc[i][j]  = imu->histAcc[i][j-1];
      imu->histGyro[i][j] = imu->histGyro[i][j-1];
    }
    
    // Assign new data
    imu->histMag[i][0]  = imu->rawMag[i];
    imu->histAcc[i][0]  = imu->rawAcc[i];
    imu->histGyro[i][0] = imu->rawGyro[i];

    // Reset moving average
    imu->avgMag[i]  = 0;
    imu->avgAcc[i]  = 0;
    imu->avgGyro[i] = 0;

    // Calculate new moving average
    for ( j=0; j<12; j++ ) {
      imu->avgMag[i]  += imu->histMag[i][j]  * imu->weight[j];
      imu->avgAcc[i]  += imu->histAcc[i][j]  * imu->weight[j];
      imu->avgGyro[i] += imu->histGyro[i][j] * imu->weight[j];
    }

  }
  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_norm
//  Generates normalized sensor data.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_norm ( imu_struct* imu )  {

  // Shift and orient magnetometer readings
  imu->normMag[X] = -( imu->avgMag[X] - imu->moffset[X] ) / (double)imu->mrange[X];
  imu->normMag[Y] = -( imu->avgMag[Y] - imu->moffset[Y] ) / (double)imu->mrange[Y];
  imu->normMag[Z] =  ( imu->avgMag[Z] - imu->moffset[Z] ) / (double)imu->mrange[Z];

  // Shift and orient accelerometer readings
  imu->normAcc[X] = ( imu->avgAcc[Y] - imu->aoffset[Y] ) / (double)imu->arange[Y];
  imu->normAcc[Y] = ( imu->avgAcc[X] - imu->aoffset[X] ) / (double)imu->arange[X];
  imu->normAcc[Z] = ( imu->avgAcc[Z] - imu->aoffset[Z] ) / (double)imu->arange[Z];

  // Scale and orient gyro readings
  imu->normGyro[X] = -imu->avgGyro[Y] * GYRO_SCALE;
  imu->normGyro[Y] = -imu->avgGyro[X] * GYRO_SCALE;
  imu->normGyro[Z] = -imu->avgGyro[Z] * GYRO_SCALE;

  // Normalize raw quaternion values
  double mag = 0.0;  unsigned short i = 1;
  for ( i=0; i<4; i++ )  mag += imu->rawQuat[i] * imu->rawQuat[i];
  mag = sqrt(mag);
  for ( i=0; i<4; i++ )  imu->normQuat[i] = imu->rawQuat[i] / mag;

  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_fusion
//  Applies sensor data fusion algorithm. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_fusion ( imu_struct* imu )  {

  // Local variables
  unsigned short i;
  double mag;

  // Quaternion index
  unsigned short w=0, x=1, y=2, z=3;

  // Get values from mpu structure
  double q[4], m[3], a[3], g[3], b[3], fx, fz;
  fx = imu->fx;  fz = imu->fz;
  for ( i=0; i<4; i++ ) {
    q[i] = imu->Quat[i];
    if (i<3) {
      m[i] = imu->normMag[i];
      a[i] = imu->normAcc[i];
      g[i] = imu->normGyro[i];
      b[i] = imu->bias[i];
    }
  }

  // Normalize magnetometer
  mag = 0.0;
  for ( i=0; i<3; i++ )  mag += m[i] * m[i];
  mag = sqrt(mag);
  for ( i=0; i<3; i++ )  m[i] /= mag;

  // Normalize accelerometer
  mag = 0.0;
  for ( i=0; i<3; i++ )  mag += a[i] * a[i];
  mag = sqrt(mag);
  for ( i=0; i<3; i++ )  a[i] /= mag;

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

  // Quaternion  multiplication values 
  double qwx, qwy, qwz, qxy, qxz, qyz, qxx, qyy, qzz;
  qwx = q[w] * q[x];  qwy = q[w] * q[y];  qwz = q[w] * q[z];
  qxy = q[x] * q[y];  qxz = q[x] * q[z];  qyz = q[y] * q[z];
  qxx = q[x] * q[x];  qyy = q[y] * q[y];  qzz = q[z] * q[z];

  // Calculate objective function
  double F1, F2, F3, F4, F5, F6;
  F1 =        twoq[x] * q[z] - twoq[w] * q[y] - a[X];
  F2 =        twoq[w] * q[x] + twoq[y] * q[z] - a[Y];
  F3 = 1.0f - twoq[x] * q[x] - twoq[y] * q[y] - a[Z];
  F4 = twofx * ( 0.5f - q[y] * q[y] - q[z] * q[z] ) 
     + twofz * ( qxz - qwy ) - m[X]; 
  F5 = twofx * ( q[x] * q[y] - q[w] * q[z] ) 
     + twofz * ( q[w] * q[x] + q[y] * q[z] ) - m[Y];
  F6 = twofx * ( qwy + qxz )
     + twofz * ( 0.5f - q[x] * q[x] - q[y] * q[y] ) - m[Z];

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

  // Normalise gradient (create function)
  mag = 0.0;
  for ( i=0; i<4; i++ )  mag += qdf[i] * qdf[i];
  mag = sqrt(mag);
  for ( i=0; i<4; i++ )  qdf[i] /= mag;

  // Compute gyroscope error
  double err[3];
  err[X] = twoq[w] * qdf[x] - twoq[x] * qdf[w] - twoq[y] * qdf[z] + twoq[z] * qdf[y];
  err[Y] = twoq[w] * qdf[y] + twoq[x] * qdf[z] - twoq[y] * qdf[w] - twoq[z] * qdf[x];
  err[Z] = twoq[w] * qdf[z] - twoq[x] * qdf[y] + twoq[y] * qdf[x] - twoq[z] * qdf[w];

  // Adjust for gyroscope baises
  for ( i=0; i<3; i++ ) {
    b[i] += err[i] * SYS_DT * IMU_ZETA;
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
    q[i] += qd[i] * SYS_DT;
  }

  // Normalise quaternion
  mag = 0.0;
  for ( i=0; i<4; i++ )  mag += q[i] * q[i];
  mag = sqrt(mag);
  for ( i=0; i<4; i++ )  q[i] /= mag;

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

  // Update mpu structure
  imu->fx = fx;  imu->fz = fz;
  for ( i=0; i<4; i++ ) {
    imu->Quat[i] = q[i];
    imu->dQuat[i] = qd[i];
    if(i<3) {
      imu->Eul[i]  = e[i];
      imu->dEul[i] = g[i];
      imu->bias[i] = b[i];
    }
  }

  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_sample
//  Generates a sample of the MPU sensor data.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_sample ( imu_struct* imu )  {
  imu_raw(imu);
  imu_avg(imu);
  imu_norm(imu);
  imu_fusion(imu);
  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_row_map
//  Maps rows so the DMP can define proper orientation.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
short imu_row_map ( const signed char* row )  {
  short b;
  if      ( row[0] >0 )  b = 0;
  else if ( row[0] <0 )  b = 4;
  else if ( row[1] >0 )  b = 1;
  else if ( row[1] <0 )  b = 5;
  else if ( row[2] >0 )  b = 2;
  else if ( row[2] <0 )  b = 6;
  else                   b = 7;
  return b;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_orient
//  Returns a scalar the DMP uses to define orientation.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
short imu_orient ( const signed char* mtx )  {
  short scalar;
  scalar  = imu_row_map(mtx);
  scalar |= imu_row_map(mtx + 3) << 3;
  scalar |= imu_row_map(mtx + 6) << 6;
  return scalar;
}
*/


