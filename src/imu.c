
//============================================================
//  imu.c
//  Justin M Selfridge
//============================================================
#include "imu.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_init
//  Initializes an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_init ( imu_struct* imu, ushort bus )  {

  // Start initialization
  imu->bus = bus;
  if(DEBUG)  printf( "Initializing IMU%d \n", bus );
  led_blink( LED_MPU, 500, 500 );

  // Init functions
  imu_param(imu);
  imu_setcal(imu);
  //imu_conv(imu);
  imu_setic(imu);

  // Indicate init completed
  led_on(LED_MPU);

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


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_param
//  Assign parameters to an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_param ( imu_struct* imu )  {

  if(DEBUG) {  printf("  Assigning IMU parameters ");  fflush(stdout);  }
  linux_set_i2c_bus(imu->bus);

  sys.ret = mpu_init_master(NULL);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_init_master' failed." );

  sys.ret = mpu_set_sensors( INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS );
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_set_sensors' failed." );

  sys.ret = mpu_set_sample_rate(MEMS_HZ);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_set_sample_rate' failed." );

  sys.ret = mpu_set_compass_sample_rate(COMP_HZ);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_set_compass_sample_rate' failed." );

  sys.ret = mpu_set_gyro_fsr(GYRO_FSR);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_set_gyro_fsr' failed." );

  sys.ret = mpu_set_accel_fsr(ACC_FSR);
  if(DEBUG) {  printf(".");  fflush(stdout);  }
  sys_err( sys.ret, "Error (imu_init): 'mpu_set_accel_fsr' failed." );

  // Modify this later
  //const signed char R[9] = { 1,0,0, 0,1,0, 0,0,1 };

  //sys.ret = mpu_configure_fifo( INV_XYZ_GYRO | INV_XYZ_ACCEL );
  //if(DEBUG) {  printf(".");  fflush(stdout);  }
  //sys_err( sys.ret, "Error (imu_init): 'mpu_configure_fifo' failed." );

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

  //gpio_export(I2C1_INT_PIN);
  //gpio_set_dir( I2C1_INT_PIN, INPUT_PIN );
  //gpio_set_edge( I2C1_INT_PIN, "falling" );

  if(DEBUG)  printf(" complete \n");
  return;
}


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
  sys_err( !f, "Error (imu_setcal): File 'moffset' not found." );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->moffset[i] = atoi(buff);
  }
  fclose(f);

  // Set magnetometer range
  sprintf( path, "cal/imu%d/mrange", imu->bus );
  f = fopen( path, "r" );
  sys_err( !f, "Error (imu_setcal): File 'mrange' not found." );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->mrange[i] = atoi(buff);
  }
  fclose(f);

  // Set acceleration offset
  sprintf( path, "cal/imu%d/aoffset", imu->bus );
  f = fopen( path, "r" );
  sys_err( !f, "Error (imu_setcal): File 'aoffset' not found." );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->aoffset[i] = atoi(buff);
  }
  fclose(f);

  // Set acceleration range
  sprintf( path, "cal/imu%d/arange", imu->bus );
  f = fopen( path, "r" );
  sys_err( !f, "Error (imu_setcal): File 'arange' not found." );
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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_setic
//  Sets the initial conditions for the MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_setic ( imu_struct* imu )  {
  if(DEBUG)  printf("  Setting IMU initial conditions \n");

  // Local vsariable
  ushort i, j;

  // Assign sample rates
  imu->mems_hz   = MEMS_HZ;
  imu->comp_hz   = COMP_HZ;
  imu->fusion_hz = FUSION_HZ;

  // Calculate time steps
  imu->mems_dt   = 1.0/MEMS_HZ;
  imu->comp_dt   = 1.0/COMP_HZ;
  imu->fusion_dt = 1.0/FUSION_HZ;

  // Assign low pass filter cutoff
  imu->mems_lpf = MEMS_LPF;
  imu->comp_lpf = COMP_LPF;

  // Determine time constants
  if ( MEMS_LPF !=0.0 )  imu->mems_tc = 1.0/MEMS_LPF;  else imu->mems_tc = 0.0;
  if ( COMP_LPF !=0.0 )  imu->comp_tc = 1.0/COMP_LPF;  else imu->comp_tc = 0.0;

  // Calculate filter gain values
  imu->mems_gain = imu->mems_dt / ( imu->mems_tc + imu->mems_dt );
  imu->comp_gain = imu->comp_dt / ( imu->comp_tc + imu->comp_dt );

  // Display IMU settings
  if (DEBUG) {
    printf("    MEMS device    \
    HZ: %4d    DT: %5.3f    LPF: %6.2f    TC: %5.2f    gain: %7.4f  \n", \
    imu->mems_hz, imu->mems_dt, imu->mems_lpf, imu->mems_tc, imu->mems_gain );
    printf("    COMP device    \
    HZ: %4d    DT: %5.3f    LPF: %6.2f    TC: %5.2f    gain: %7.4f  \n", \
    imu->comp_hz, imu->comp_dt, imu->comp_lpf, imu->comp_tc, imu->comp_gain );
    printf("    Data Fusion:   \
    HZ: %4d    DT: %5.3f  \n", \
    imu->fusion_hz, imu->fusion_dt ); 
  }

  // Clear moving average history
  for ( i=0; i<3; i++ ) {
    for ( j=0; j<MEMS_HIST; j++ )  imu->histGyro[i][j] = 0;
    for ( j=0; j<MEMS_HIST; j++ )  imu->histAcc[i][j]  = 0;
    for ( j=0; j<COMP_HIST; j++ )  imu->histMag[i][j]  = 0;
  }

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

  //imu->Eul[Z] = ctrl.heading;
  //imu->Quat[0] = cos(ctrl.heading/2);
  //imu->Quat[1] = 0;
  //imu->Quat[2] = 0;
  //imu->Quat[3] = sin(ctrl.heading/2);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_avail
//  Checks for new available data. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool imu_avail ( imu_struct* imu )  {

  // Local variables
  short status;
  bool ret;

  // Ping status register
  sys.ret = mpu_get_int_status(&status);
  sys_err( sys.ret<0, "Error (imu_avail): 'mpu_get_int_status' failed." );
  ret = ( status == ( MPU_INT_STATUS_DATA_READY | MPU_INT_STATUS_DMP ) ) ;

  return ret;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_mems
//  Processes MEMS device data.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_mems ( imu_struct* imu )  {

  // Local variables
  ushort i, j, k = MEMS_HIST;
  float g, a;

  // Get raw data
  sys.ret = mpu_get_gyro_reg( imu->rawGyro, NULL );
  sys_err( sys.ret, "Error (imu_mems): 'mpu_get_gyro_reg' failed." );
  sys.ret = mpu_get_accel_reg( imu->rawAcc, NULL );
  sys_err( sys.ret, "Error (imu_mems): 'mpu_get_accel_reg' failed." );

  // Cycle through elements
  for ( i=0; i<3; i++ ) {

    // Advance history data
    for ( j=1; j<k; j++ ) {  imu->histGyro[i][j-1] = imu->histGyro[i][j];  imu->histAcc[i][j-1] = imu->histAcc[i][j];  }

    // Assign current value
    imu->histGyro[i][k-1] = imu->rawGyro[i];  imu->histAcc[i][k-1] = imu->rawAcc[i];

    // Initialize filter value
    g = (float) (imu->histGyro[i][0]);  a = (float) (imu->histAcc[i][0]);

    // Implement low-pass filter
    for ( j=1; j<k; j++ ) {  g = g + imu->mems_gain * (float) ( imu->histGyro[i][j] - g );  a = a + imu->mems_gain * (float) ( imu->histAcc[i][j] - a );  }

    // Store averaged values
    imu->avgGyro[i] = g;  imu->avgAcc[i] = a;

  }

  // Scale and orient gyroscope readings
  imu->calGyro[X] = -imu->avgGyro[Y] * GYRO_SCALE;
  imu->calGyro[Y] = -imu->avgGyro[X] * GYRO_SCALE;
  imu->calGyro[Z] = -imu->avgGyro[Z] * GYRO_SCALE;

  // Shift and orient accelerometer readings
  imu->calAcc[X] = ( imu->avgAcc[Y] - imu->aoffset[Y] ) / (double)imu->arange[Y];
  imu->calAcc[Y] = ( imu->avgAcc[X] - imu->aoffset[X] ) / (double)imu->arange[X];
  imu->calAcc[Z] = ( imu->avgAcc[Z] - imu->aoffset[Z] ) / (double)imu->arange[Z];

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_comp
//  Processes compass device data.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_comp ( imu_struct* imu )  {

  // Local variables
  ushort i, j, k = COMP_HIST;
  float m;

  // Get raw data
  sys.ret = mpu_get_compass_reg( imu->rawMag, NULL );
  sys_err( sys.ret, "Error (imu_comp): 'mpu_get_compass_reg' failed." );

  // Cycle through elements
  for ( i=0; i<3; i++ ) {

    // Advance history data
    for ( j=1; j<k; j++ ) {  imu->histMag[i][j-1] = imu->histMag[i][j];  }

    // Assign current value
    imu->histMag[i][k-1] = imu->rawMag[i];

    // Initialize filter value
    m = (float) (imu->histMag[i][0]);

    // Implement low-pass filter
    for ( j=1; j<k; j++ ) {  m = m + imu->comp_gain * (float) ( imu->histMag[i][j] - m );  }

    // Store averaged values
    imu->avgMag[i] = m;

  }

  // Shift and orient magnetometer readings
  imu->calMag[X] = -( imu->avgMag[X] - imu->moffset[X] ) / (double)imu->mrange[X];
  imu->calMag[Y] = -( imu->avgMag[Y] - imu->moffset[Y] ) / (double)imu->mrange[Y];
  imu->calMag[Z] =  ( imu->avgMag[Z] - imu->moffset[Z] ) / (double)imu->mrange[Z];

  return;
}


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
  double q[4], m[3], a[3], g[3], b[3], fx, fz, dt;
  fx = imu->fx;  fz = imu->fz;  dt = imu->fusion_dt;
  for ( i=0; i<4; i++ ) {
    q[i] = imu->Quat[i];
    if (i<3) {
      m[i] = imu->calMag[i];
      a[i] = imu->calAcc[i];
      g[i] = imu->calGyro[i];
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

  // Quaternion multiplication values 
  double qwx, qwy, qwz, qxy, qxz, qyz, qxx, qyy, qzz;
  qwx = q[w] * q[x];  qwy = q[w] * q[y];  qwz = q[w] * q[z];
  qxy = q[x] * q[y];  qxz = q[x] * q[z];  qyz = q[y] * q[z];
  qxx = q[x] * q[x];  qyy = q[y] * q[y];  qzz = q[z] * q[z];

  // Calculate objective function
  double F1, F2, F3, F4, F5, F6;
  F1 =        twoq[x] * q[z] - twoq[w] * q[y] - a[X];
  F2 =        twoq[w] * q[x] + twoq[y] * q[z] - a[Y];
  F3 = 1.0f - twoq[x] * q[x] - twoq[y] * q[y] - a[Z];
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


