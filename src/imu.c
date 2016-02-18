
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

  // IMU struct values
  imu.id   = 2;
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
  led_on(LED_IMU);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_exit
//  Terminate an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_exit ( void )  {
  if(DEBUG)  printf("Close IMU \n");
  led_off(LED_IMU);
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_param
//  Assign parameters to an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_param (  )  {

  if(DEBUG) {  printf("  Assign IMU parameters ");  fflush(stdout);  }
  linux_set_i2c_bus(imu.id);

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_init_master(NULL) )
    printf( "Error (imu_param): 'mpu_init_master' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_sensors( INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS ) )
    printf( "Error (imu_param): 'mpu_set_sensors' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_sample_rate(HZ_IMU_FAST) )
    printf( "Error (imu_param): 'mpu_set_sample_rate' failed. \n" );

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
void imu_getcal (  )  {
  if(DEBUG)  printf("  IMU calibration values: \n");

  // Local variables
  int i;
  FILE* f;
  char buff [32];  memset( buff, 0, sizeof(buff) );
  char path [32];  memset( path, 0, sizeof(path) );

  // Set acceleration bias
  sprintf( path, "../Param/board/bias/acc1" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'acc1 bias' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    acc.bias[i] = atoi(buff);
  }
  fclose(f);

  // Set acceleration range
  sprintf( path, "../Param/board/range/acc1" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'acc1 range' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    acc.range[i] = atoi(buff);
  }
  fclose(f);

  // Set magnetometer bias
  sprintf( path, "../Param/board/bias/mag1" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'mag1 bias' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    mag.bias[i] = atoi(buff);
  }
  fclose(f);

  // Set magnetometer range
  sprintf( path, "../Param/board/range/mag1" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'mag1 range' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    mag.range[i] = atoi(buff);
  }
  fclose(f);

  // Set gyro bias
  sprintf( path, "../Param/board/bias/gyr1" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'gyr1 bias' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    gyr.bias[i] = atoi(buff);
  }
  fclose(f);

  // Display calibration values
  if(DEBUG) {
    printf("   a1bias a1range   m1bias m1range   g1bias \n");
    for ( i=0; i<3; i++ ) {
      printf("     ");
      printf( "%4d    ",   acc.bias[i]  );
      printf( "%4d     ",  acc.range[i] );
      printf( "%4d    ",   mag.bias[i]  );
      printf( "%4d     ",  mag.range[i] );
      printf( "%4d  \n",   gyr.bias[i]  );
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
  imu.loops  = HZ_IMU_FAST / HZ_IMU_SLOW;
  imu.count  = 0;
  imu.getmag = false;

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
  imu.getmag = false;
  if ( imu.count == 0 ) {
    imu.getmag = true;
    imu.count = imu.loops;
  }
  imu.count--;

  // Sample IMU
  if( mpu_get_gyro_reg( Graw, NULL ) )
    printf( "Error (imu_data): 'mpu_get_gyro_reg' failed. \n" );
  if( mpu_get_accel_reg( Araw, NULL ) )
    printf( "Error (imu_data): 'mpu_get_accel_reg' failed. \n" );
  if(imu.getmag){
  if( mpu_get_compass_reg( Mraw, NULL ) )
    printf( "Error (imu_data): 'mpu_get_compass_reg' failed. \n" );
  } 

  // Gyroscope low pass filter
  k = GYR_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  Ghist[i][j-1] = Ghist[i][j];
    Ghist[i][k-1] = Graw[i];
    g = (float) (Ghist[i][0]);
    for ( j=1; j<k; j++ )  g = g + gyr.gain * (float) ( Ghist[i][j] - g );
    Gavg[i] = g;
  }

  // Accelerometer low pass filter
  k = ACC_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  Ahist[i][j-1] = Ahist[i][j];
    Ahist[i][k-1] = Araw[i];
    a = (float) (Ahist[i][0]);
    for ( j=1; j<k; j++ )  a = a + acc.gain * (float) ( Ahist[i][j] - a );
    Aavg[i] = a;
  }

  // Magnetometer low pass filter
  if(imu.getmag) {
  k = MAG_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  Mhist[i][j-1] = Mhist[i][j];
    Mhist[i][k-1] = Mraw[i];
    m = (float) (Mhist[i][0]);
    for ( j=1; j<k; j++ )  m = m + mag.gain * (float) ( Mhist[i][j] - m );
    Mavg[i] = m;
  }}

  // Shift and orient gyroscope readings
  Gcal[x] =   ( Gavg[y] - gyr.bias[y] ) * GYR_SCALE;
  Gcal[y] =   ( Gavg[x] - gyr.bias[x] ) * GYR_SCALE;
  Gcal[z] = - ( Gavg[z] - gyr.bias[z] ) * GYR_SCALE;

  // Shift and orient accelerometer readings
  Acal[x] =   ( Aavg[y] - acc.bias[y] ) / (double) (acc.range[y]);
  Acal[y] =   ( Aavg[x] - acc.bias[x] ) / (double) (acc.range[x]);
  Acal[z] = - ( Aavg[z] - acc.bias[z] ) / (double) (acc.range[z]);

  // Shift and orient magnetometer readings
  if(imu.getmag) {
  Mcal[x] = ( Mavg[x] - mag.bias[x] ) / (double) (mag.range[x]);
  Mcal[y] = ( Mavg[y] - mag.bias[y] ) / (double) (mag.range[y]);
  Mcal[z] = ( Mavg[z] - mag.bias[z] ) / (double) (mag.range[z]);
  }

  // Push gyroscope values to data structure
  pthread_mutex_lock(&mutex_gyr);
  for ( i=0; i<3; i++ )  {
    gyr.raw[i] = Graw[i];
    gyr.avg[i] = Gavg[i];
    gyr.cal[i] = Gcal[i];
  }
  pthread_mutex_unlock(&mutex_gyr);

  // Push accerometer values to data structure
  pthread_mutex_lock(&mutex_acc);
  for ( i=0; i<3; i++ )  {
    acc.raw[i] = Araw[i];
    acc.avg[i] = Aavg[i];
    acc.cal[i] = Acal[i];
  }
  pthread_mutex_unlock(&mutex_acc);

  // Push magnetometer values to data structure
  if(imu.getmag) {
  pthread_mutex_lock(&mutex_mag);
  for ( i=0; i<3; i++ )  {
    mag.raw[i] = Mraw[i];
    mag.avg[i] = Mavg[i];
    mag.cal[i] = Mcal[i];
  }
  pthread_mutex_unlock(&mutex_mag);
  }

  return;
}



