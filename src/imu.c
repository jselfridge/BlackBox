
//============================================================
//  imu.c
//  Justin M Selfridge
//============================================================
#include "imu.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_init
//  Initializes an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_init (  )  {
  if(DEBUG)  printf( "Initializing IMU \n" );

  // Start initialization
  led_blink( LED_IMU, 200, 200 );

  // IMUA struct values
  imuA.bus  = 1;
  imuA.addr = 0x68;
  imuA.gyr  = &gyrA;
  imuA.acc  = &accA;
  imuA.mag  = &magA;

  // IMUB struct values
  imuB.bus  = 2;
  imuB.addr = 0x68;
  imuB.gyr  = &gyrB;
  imuB.acc  = &accB;
  imuB.mag  = &magB;

  // Open the I2C buses
  i2c_init( &(imuA.fd), imuA.bus, imuA.addr );
  i2c_init( &(imuB.fd), imuB.bus, imuB.addr );

  // Init functions
  imu_param();
  imu_getcal();
  imu_setic();

  // IMU warmup period
  usleep(500000);
  led_on(LED_IMU);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_exit
//  Terminate an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_exit ( void )  {
  if(DEBUG)  printf("Close IMU \n");
  i2c_exit( &(imuA.fd) );
  i2c_exit( &(imuB.fd) );
  led_off(LED_IMU);
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_param
//  Assign parameters to an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_param (  )  {

  if(DEBUG) {  printf("  Assign IMU parameters ");  fflush(stdout);  }

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_init( imuA.fd, NULL ) )
    printf( "Error (imu_param): 'mpu_init' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_sensors( imuA.fd, INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS ) )
    printf( "Error (imu_param): 'mpu_set_sensors' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_sample_rate( imuA.fd, HZ_IMU_FAST ) )
    printf( "Error (imu_param): 'mpu_set_sample_rate' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_compass_sample_rate( imuA.fd, HZ_IMU_SLOW ) )
    printf( "Error (imu_param): 'mpu_set_compass_sample_rate' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_gyro_fsr( imuA.fd, GYR_FSR ) )
    printf( "Error (imu_param): 'mpu_set_gyro_fsr' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_accel_fsr( imuA.fd, ACC_FSR ) )
    printf( "Error (imu_param): 'mpu_set_accel_fsr' failed. \n" );

  if(DEBUG)  printf(" complete \n");
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_getcal
//  Gets the calibration parameters for the MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_getcal (  )  {
  if(DEBUG)  printf("  IMU calibration values: \n");

  // Local variables
  int i;
  FILE* f;
  char buff [32];  memset( buff, 0, sizeof(buff) );
  char path [32];  memset( path, 0, sizeof(path) );

  // Set acceleration bias
  sprintf( path, "../Param/board/bias/accA" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'accA bias' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    accA.bias[i] = atoi(buff);
  }
  fclose(f);

  // Set acceleration range
  sprintf( path, "../Param/board/range/accA" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'accA range' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    accA.range[i] = atoi(buff);
  }
  fclose(f);

  // Set magnetometer bias
  sprintf( path, "../Param/board/bias/magA" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'magA bias' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    magA.bias[i] = atoi(buff);
  }
  fclose(f);

  // Set magnetometer range
  sprintf( path, "../Param/board/range/magA" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'magA range' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    magA.range[i] = atoi(buff);
  }
  fclose(f);

  // Set gyro bias
  sprintf( path, "../Param/board/bias/gyrA" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'gyrA bias' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    gyrA.bias[i] = atoi(buff);
  }
  fclose(f);

  // Display calibration values
  if(DEBUG) {
    printf("   abiasA arangeA   mbiasA mrangeA   gbiasA \n");
    for ( i=0; i<3; i++ ) {
      printf("     ");
      printf( "%4d    ",   accA.bias[i]  );
      printf( "%4d     ",  accA.range[i] );
      printf( "%4d    ",   magA.bias[i]  );
      printf( "%4d     ",  magA.range[i] );
      printf( "%4d  \n",   gyrA.bias[i]  );
    } 
  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_setic
//  Sets the initial conditions for the MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_setic (  )  {
  if(DEBUG)  printf("  Set IMU initial conditions \n");

  // Assign loop counter values
  if ( HZ_IMU_FAST % HZ_IMU_SLOW != 0 )
    printf( "  *** WARNING ***  Slow loop must divide evenly into fast loop. \n" );
  imuA.loops  = HZ_IMU_FAST / HZ_IMU_SLOW;
  imuA.count  = 0;
  imuA.getmag = false;

  // Calculate time steps
  double gyr_dt, acc_dt, mag_dt;
  gyr_dt = 1.0 / HZ_IMU_FAST;
  acc_dt = 1.0 / HZ_IMU_FAST;
  mag_dt = 1.0 / HZ_IMU_SLOW;

  // Determine time constants
  double gyr_tc, acc_tc, mag_tc;
  if ( GYR_LPF != 0.0 )  gyr_tc = 1.0 / ( 2.0 * PI * GYR_LPF );  else  gyr_tc = 0.0;
  if ( ACC_LPF != 0.0 )  acc_tc = 1.0 / ( 2.0 * PI * ACC_LPF );  else  acc_tc = 0.0;
  if ( MAG_LPF != 0.0 )  mag_tc = 1.0 / ( 2.0 * PI * MAG_LPF );  else  mag_tc = 0.0;

  // Calculate filter gain values
  gyrA.gain = gyr_dt / ( gyr_tc + gyr_dt );
  accA.gain = acc_dt / ( acc_tc + acc_dt );
  magA.gain = mag_dt / ( mag_tc + mag_dt );

  // Display settings
  if (DEBUG) {
    printf("    |  GYR  |  HZ %4d  |  DT %5.3f  |  LPF %6.2f  |  TC %5.2f  |  gain %7.4f  |\n", \
	   HZ_IMU_FAST, gyr_dt, GYR_LPF, gyr_tc, gyrA.gain );
    printf("    |  ACC  |  HZ %4d  |  DT %5.3f  |  LPF %6.2f  |  TC %5.2f  |  gain %7.4f  |\n", \
	   HZ_IMU_FAST, acc_dt, ACC_LPF, acc_tc, accA.gain );
    printf("    |  MAG  |  HZ %4d  |  DT %5.3f  |  LPF %6.2f  |  TC %5.2f  |  gain %7.4f  |\n", \
	   HZ_IMU_SLOW, mag_dt, MAG_LPF, mag_tc, magA.gain );
  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_update
//  Update system with new IMU sensor data.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_update ( void )  {

  // Local variables
  ushort i, j, k;
  double g, a, m;

  // Array index
  ushort x=0, y=1, z=2;

  // Local IMU struct arrays
  short  Graw[3], Araw[3], Mraw[3];
  double Gavg[3], Aavg[3], Mavg[3];
  double Gcal[3], Acal[3], Mcal[3];

  // Declare data history arrays
  static short Ghist[3][GYR_HIST];
  static short Ahist[3][ACC_HIST];
  static short Mhist[3][MAG_HIST];

  // Increment counter
  imuA.getmag = false;
  if ( imuA.count == 0 ) {
    imuA.getmag = true;
    imuA.count = imuA.loops;
  }
  imuA.count--;

  // Sample IMU
  if( mpu_get_gyro_reg( imuA.fd, Graw, NULL ) )
    printf( "Error (imu_data): 'mpu_get_gyro_reg' failed. \n" );
  if( mpu_get_accel_reg( imuA.fd, Araw, NULL ) )
    printf( "Error (imu_data): 'mpu_get_accel_reg' failed. \n" );
  if(imuA.getmag){
    if( mpu_get_compass_reg( imuA.fd, Mraw, NULL ) )
    printf( "Error (imu_data): 'mpu_get_compass_reg' failed. \n" );
  } 

  // Gyroscope low pass filter
  k = GYR_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  Ghist[i][j-1] = Ghist[i][j];
    Ghist[i][k-1] = Graw[i];
    g = (float) (Ghist[i][0]);
    for ( j=1; j<k; j++ )  g = g + gyrA.gain * (float) ( Ghist[i][j] - g );
    Gavg[i] = g;
  }

  // Accelerometer low pass filter
  k = ACC_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  Ahist[i][j-1] = Ahist[i][j];
    Ahist[i][k-1] = Araw[i];
    a = (float) (Ahist[i][0]);
    for ( j=1; j<k; j++ )  a = a + accA.gain * (float) ( Ahist[i][j] - a );
    Aavg[i] = a;
  }

  // Magnetometer low pass filter
  if(imuA.getmag) {
  k = MAG_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  Mhist[i][j-1] = Mhist[i][j];
    Mhist[i][k-1] = Mraw[i];
    m = (float) (Mhist[i][0]);
    for ( j=1; j<k; j++ )  m = m + magA.gain * (float) ( Mhist[i][j] - m );
    Mavg[i] = m;
  }}

  // Shift and orient gyroscope readings
  Gcal[x] =   ( Gavg[y] - gyrA.bias[y] ) * GYR_SCALE;
  Gcal[y] =   ( Gavg[x] - gyrA.bias[x] ) * GYR_SCALE;
  Gcal[z] = - ( Gavg[z] - gyrA.bias[z] ) * GYR_SCALE;

  // Shift and orient accelerometer readings
  Acal[x] =   ( Aavg[y] - accA.bias[y] ) / (double) (accA.range[y]);
  Acal[y] =   ( Aavg[x] - accA.bias[x] ) / (double) (accA.range[x]);
  Acal[z] = - ( Aavg[z] - accA.bias[z] ) / (double) (accA.range[z]);

  // Shift and orient magnetometer readings
  if(imuA.getmag) {
  Mcal[x] = ( Mavg[x] - magA.bias[x] ) / (double) (magA.range[x]);
  Mcal[y] = ( Mavg[y] - magA.bias[y] ) / (double) (magA.range[y]);
  Mcal[z] = ( Mavg[z] - magA.bias[z] ) / (double) (magA.range[z]);
  }

  // Push gyroscope values to data structure
  pthread_mutex_lock(&mutex_gyrA);
  for ( i=0; i<3; i++ )  {
    gyrA.raw[i] = Graw[i];
    gyrA.avg[i] = Gavg[i];
    gyrA.cal[i] = Gcal[i];
  }
  pthread_mutex_unlock(&mutex_gyrA);

  // Push accerometer values to data structure
  pthread_mutex_lock(&mutex_accA);
  for ( i=0; i<3; i++ )  {
    accA.raw[i] = Araw[i];
    accA.avg[i] = Aavg[i];
    accA.cal[i] = Acal[i];
  }
  pthread_mutex_unlock(&mutex_accA);

  // Push magnetometer values to data structure
  if(imuA.getmag) {
  pthread_mutex_lock(&mutex_magA);
  for ( i=0; i<3; i++ )  {
    magA.raw[i] = Mraw[i];
    magA.avg[i] = Mavg[i];
    magA.cal[i] = Mcal[i];
  }
  pthread_mutex_unlock(&mutex_magA);
  }

  return;
}



