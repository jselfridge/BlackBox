
//============================================================
//  mpu.c
//  Justin M Selfridge
//============================================================
#include "mpu.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  mpu_init
//  Initializes an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void mpu_init ( mpu_struct* mpu )  {

  // Start initialization
  char bus[1];
  sprintf( bus, "%d", mpu->bus );
  if(DEBUG)  printf( "Initializing MPU%s \n", bus );
  led_blink( LED_MPU, 500, 500 );

  // Init functions
  mpu_param(mpu);
  mpu_setcal(mpu);
  mpu_conv(mpu);
  mpu_setic(mpu);

  // Indicate init completed
  led_on(LED_MPU);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  mpu_exit
//  Terminate an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void mpu_exit ( void )  {
  if(DEBUG)  printf("  Closing MPU \n");
  ret = mpu_set_dmp_state(0);
  sys_err( ret, "Error (mpu_exit): 'mpu_set_dmp_state' failed." );
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  mpu_param
//  Assign parameters to an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void mpu_param ( mpu_struct* mpu )  {

  if(DEBUG) {  printf("  Assigning MPU parameters ");  fflush(stdout);  }
  linux_set_i2c_bus(mpu->bus);

  ret = mpu_init_master(NULL);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( ret, "Error (mpu_init): 'mpu_init_master' failed." );

  ret = mpu_set_sensors( INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS );
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( ret, "Error (mpu_init): 'mpu_set_sensors' failed." );

  ret = mpu_configure_fifo( INV_XYZ_GYRO | INV_XYZ_ACCEL );
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( ret, "Error (mpu_init): 'mpu_configure_fifo' failed." );

  ret = mpu_set_sample_rate((int)SYS_FREQ);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( ret, "Error (mpu_init): 'mpu_set_sample_rate' failed." );

  ret = mpu_set_compass_sample_rate((int)SYS_FREQ);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( ret, "Error (mpu_init): 'mpu_set_compass_sample_rate' failed." );

  ret = mpu_set_lpf(5);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( ret, "Error (mpu_init): 'mpu_set_lpf' failed." );

  ret = dmp_load_motion_driver_firmware();
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( ret, "Error (mpu_init): 'dmp_load_motion_driver_firmware' failed." );

  ret = dmp_set_orientation( mpu_orient(mpu->rot) );
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( ret, "Error (mpu_init): 'dmp_set_orientation' failed." );

  ret = dmp_enable_feature( 
          DMP_FEATURE_6X_LP_QUAT | 
          DMP_FEATURE_SEND_RAW_ACCEL | 
	  DMP_FEATURE_SEND_CAL_GYRO | 
          DMP_FEATURE_GYRO_CAL );
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( ret, "Error (mpu_init): 'dmp_enable_feature' failed." );

  ret = dmp_set_fifo_rate(SYS_FREQ);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( ret, "Error (mpu_init): 'dmp_set_fifo_rate' failed." );

  ret = mpu_set_dmp_state(1);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( ret, "Error (mpu_init): 'mpu_set_dmp_state' failed." );

  ret = mpu_set_gyro_fsr(500);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( ret, "Error (mpu_init): 'mpu_set_gyro_fsr' failed." );

  ret = mpu_set_accel_fsr(4);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( ret, "Error (mpu_init): 'mpu_set_accel_fsr' failed." );

  if(DEBUG)  printf(" complete \n");
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  mpu_setcal
//  Sets the calibration parameters for the MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void mpu_setcal ( mpu_struct* mpu )  {
  if(DEBUG)  printf("  MPU calibration values: \n");

  // Local variables
  int i;
  FILE* f;
  char buff [32];
  char path [32];
  memset( buff, 0, sizeof(buff) );

  // Set magnetometer offset
  sprintf( path, "cal/mpu%d/moffset", mpu->bus );
  f = fopen( path, "r" );
  if (!f)  {  printf("Error (mpu_setcal): File 'moffset' not found. \n");  return;  }
  for ( i=0; i<3; i++ ) {
    if ( !fgets( buff, 32, f ) )  printf("Error: Failed to read 'moffset' file. \n");
    mpu->moffset[i] = atoi(buff);
  }
  fclose(f);

  // Set magnetometer range
  sprintf( path, "cal/mpu%d/mrange", mpu->bus );
  f = fopen( path, "r" );
  if (!f)  {  printf("Error (mpu_setcal): File 'mrange' not found. \n");  return;  }
  for ( i=0; i<3; i++ ) {
    if ( !fgets( buff, 32, f ) )  printf("Error: Failed to read 'mrange' file. \n");
    mpu->mrange[i] = atoi(buff);
  }
  fclose(f);

  // Set acceleration offset
  sprintf( path, "cal/mpu%d/aoffset", mpu->bus );
  f = fopen( path, "r" );
  if (!f)  {  printf("Error (mpu_setcal): File 'aoffset' not found. \n");  return;  }
  for ( i=0; i<3; i++ ) {
    if ( !fgets( buff, 32, f ) )  printf("Error: Failed to read 'aoffset' file. \n");
    mpu->aoffset[i] = atoi(buff);
  }
  fclose(f);

  // Set acceleration range
  sprintf( path, "cal/mpu%d/arange", mpu->bus );
  f = fopen( path, "r" );
  if (!f)  {  printf("Error (mpu_setcal): File 'arange' not found. \n");  return;  }
  for ( i=0; i<3; i++ ) {
    if ( !fgets( buff, 32, f ) )  printf("Error: Failed to read 'arange' file. \n");
    mpu->arange[i] = atoi(buff);
  }
  fclose(f);

  // Display calibration values
  if(DEBUG) {
  printf("    moffset  mrange  aoffset  arange \n");
  for ( i=0; i<3; i++ ) {
    printf("      ");
    printf( "%04d    ", mpu->moffset[i] );
    printf( "%04d    ", mpu->mrange[i]  );
    printf( "%04d    ", mpu->aoffset[i] );
    printf( "%06d  \n", mpu->arange[i]  );
  } 
  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  mpu_conv
//  Allows the sensor heading to converge after initialization. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void mpu_conv ( mpu_struct* mpu )  {
  if(DEBUG)  printf("  Determining MPU heading:    "); 

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

  heading = -PI/4;
  if(DEBUG)  printf("hard coded as %6.3f \n", heading*(180.0/PI) );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  mpu_setic
//  Sets the initial conditions for the MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void mpu_setic ( mpu_struct* mpu )  {
  if(DEBUG)  printf("  Setting MPU initial conditions \n");

  unsigned short i=1;
  mpu->fx = 0.5;  mpu->fz = 0.866;
  for ( i=0; i<4; i++ ) {
    mpu->dQuat[i] = 0;
    if (i<3) {
      mpu->Eul[i]  = 0;
      mpu->dEul[i] = 0;
      mpu->bias[i] = 0;
    }
  }
  mpu->Quat[0] = 1;
  mpu->Eul[Z] = heading;

  mpu->Quat[0] = cos(heading/2);
  mpu->Quat[1] = 0;
  mpu->Quat[2] = 0;
  mpu->Quat[3] = sin(heading/2);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  mpu_avail
//  Check the MPU interupt to see if new data is avialable.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int mpu_avail ( void )  {
  short status;
  ret = mpu_get_int_status(&status);
  sys_err( ret<0, "Error (mpu_avail): 'mpu_get_int_status' failed." );
  return ( status == ( MPU_INT_STATUS_DATA_READY | MPU_INT_STATUS_DMP | MPU_INT_STATUS_DMP_0 ) );
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  mpu_clear
//  Clears the FIFO buffer on the MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*
int mpu_avail ( void )  {
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


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  mpu_raw
//  Obtains raw data from MPU sensor and maps to body frame.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void mpu_raw ( mpu_struct* mpu )  {

  // Local variables 
  short sensors;
  unsigned char more = 1;

  // Check for new data
  if ( mpu_avail() ) {

    // Obtain magnetometer values
    ret = mpu_get_compass_reg( mpu->rawMag, &mpu->magTime );
    sys_err( ret<0, "Error (mpu_raw): 'mpu_get_compass_reg' failed." );

    // Obatin gyro, acc, and quat values
    while (more) {
      ret = dmp_read_fifo( mpu->rawGyro, mpu->rawAcc, mpu->rawQuat, &mpu->dmpTime, &sensors, &more );
      sys_err( ret<0, "Error (mpu_raw): 'dmp_read_fifo' failed. ");
    }

  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  mpu_norm
//  Generates normalized sensor data.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void mpu_cal ( mpu_struct* mpu )  {

  // Shift and orient magnetometer readings
  mpu->normMag[X] = -( mpu->rawMag[X] - mpu->moffset[X] ) / (double)mpu->mrange[X];
  mpu->normMag[Y] = -( mpu->rawMag[Y] - mpu->moffset[Y] ) / (double)mpu->mrange[Y];
  mpu->normMag[Z] =  ( mpu->rawMag[Z] - mpu->moffset[Z] ) / (double)mpu->mrange[Z];

  // Shift and orient accelerometer readings
  mpu->normAcc[X] = ( mpu->rawAcc[Y] - mpu->aoffset[Y] ) / (double)mpu->arange[Y];
  mpu->normAcc[Y] = ( mpu->rawAcc[X] - mpu->aoffset[X] ) / (double)mpu->arange[X];
  mpu->normAcc[Z] = ( mpu->rawAcc[Z] - mpu->aoffset[Z] ) / (double)mpu->arange[Z];

  // Scale and orient gyro readings
  mpu->normGyro[X] = -mpu->rawGyro[Y] * GYRO_SCALE;
  mpu->normGyro[Y] = -mpu->rawGyro[X] * GYRO_SCALE;
  mpu->normGyro[Z] = -mpu->rawGyro[Z] * GYRO_SCALE;

  // Normalize raw quaternion values
  double mag = 0.0;  unsigned short i = 1;
  for ( i=0; i<4; i++ )  mag += mpu->rawQuat[i] * mpu->rawQuat[i];
  mag = sqrt(mag);
  for ( i=0; i<4; i++ )  mpu->normQuat[i] = mpu->rawQuat[i] / mag;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  mpu_fusion
//  Applies sensor data fusion algorithm. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void mpu_fusion ( mpu_struct* mpu )  {

  // Local variables
  unsigned short i;
  double mag;

  // Quaternion index
  unsigned short w=0, x=1, y=2, z=3;

  // Get values from mpu structure
  double q[4], m[3], a[3], g[3], b[3], fx, fz;
  fx = mpu->fx;  fz = mpu->fz;
  for ( i=0; i<4; i++ ) {
    q[i] = mpu->Quat[i];
    if (i<3) {
      m[i] = mpu->normMag[i];
      a[i] = mpu->normAcc[i];
      g[i] = mpu->normGyro[i];
      b[i] = mpu->bias[i];
    }
  }

  // Normalize magnetometer (create function)
  mag = 0.0;
  for ( i=0; i<3; i++ )  mag += m[i] * m[i];
  mag = sqrt(mag);
  for ( i=0; i<3; i++ )  m[i] /= mag;

  // Normalize accelerometer (create function)
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
    b[i] += err[i] * SYS_DT * MPU_ZETA;
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
    qd[i] = qdr[i] - ( MPU_BETA * qdf[i] );
    q[i] += qd[i] * SYS_DT;
  }

  // Normalise quaternion (create function)
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
  e[X] = atan2 ( ( 2* ( qwx + qyz ) ), ( 1- 2* ( qxx + qyy ) ) );
  e[Y] = asin  (   2* ( qwy - qxz ) );
  e[Z] = atan2 ( ( 2* ( qwz + qxy ) ), ( 1- 2* ( qyy + qzz ) ) );

  // Update mpu structure
  mpu->fx = fx;  mpu->fz = fz;
  for ( i=0; i<4; i++ ) {
    mpu->Quat[i] = q[i];
    mpu->dQuat[i] = qd[i];
    if(i<3) {
      mpu->Eul[i]  = e[i];
      mpu->dEul[i] = g[i];
      mpu->bias[i] = b[i];
    }
  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  mpu_sample
//  Generates a sample of the MPU sensor data.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void mpu_sample ( mpu_struct* mpu )  {
  mpu_raw(mpu);
  mpu_cal(mpu);
  mpu_fusion(mpu);
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  mpu_row_map
//  Maps rows so the DMP can define proper orientation.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
short mpu_row_map ( const signed char* row )  {
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


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  mpu_orient
//  Returns a scalar the DMP uses to define orientation.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
short mpu_orient ( const signed char* mtx )  {
  short scalar;
  scalar  = mpu_row_map(mtx);
  scalar |= mpu_row_map(mtx + 3) << 3;
  scalar |= mpu_row_map(mtx + 6) << 6;
  return scalar;
}



