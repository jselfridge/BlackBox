
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

  // Setup IMUA
  if (USE_IMUA)  {

    // Struct values
    imuA.id   = 'A';
    imuA.bus  = 1;
    imuA.addr = 0x68;
    imuA.gyr  = &gyrA;
    imuA.acc  = &accA;
    imuA.mag  = &magA;

    // Setup functions
    i2c_init( &(imuA.fd), imuA.bus, imuA.addr );
    imu_param(&imuA);
    imu_getcal(&imuA);
    imu_setic(&imuA);

  }

  // Setup IMUB
  if (USE_IMUB)  {

    // Struct values
    imuB.id   = 'B';
    imuB.bus  = 2;
    imuB.addr = 0x68;
    imuB.gyr  = &gyrB;
    imuB.acc  = &accB;
    imuB.mag  = &magB;

    // Setup functions
    i2c_init( &(imuB.fd), imuB.bus, imuB.addr );
    imu_param(&imuB);
    imu_getcal(&imuB);
    imu_setic(&imuB);

  }

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
  if(USE_IMUA)  i2c_exit( &(imuA.fd) );
  if(USE_IMUB)  i2c_exit( &(imuB.fd) );
  led_off(LED_IMU);
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_param
//  Assign parameters to an MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_param ( imu_struct *imu )  {

  if(DEBUG) {  printf("  Assign IMU%c parameters ", imu->id );  fflush(stdout);  }

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_init( imu->fd, NULL ) )
    printf( "Error (imu_param): 'mpu_init' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_sensors( imu->fd, INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS ) )
    printf( "Error (imu_param): 'mpu_set_sensors' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_sample_rate( imu->fd, HZ_IMU_FAST ) )
    printf( "Error (imu_param): 'mpu_set_sample_rate' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_compass_sample_rate( imu->fd, HZ_IMU_SLOW ) )
    printf( "Error (imu_param): 'mpu_set_compass_sample_rate' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_gyro_fsr( imu->fd, GYR_FSR ) )
    printf( "Error (imu_param): 'mpu_set_gyro_fsr' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_accel_fsr( imu->fd, ACC_FSR ) )
    printf( "Error (imu_param): 'mpu_set_accel_fsr' failed. \n" );

  if(DEBUG)  printf(" complete \n");
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_getcal
//  Gets the calibration parameters for the MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_getcal ( imu_struct *imu )  {
  if(DEBUG)  printf( "  IMU%c calibration values: \n", imu->id );

  // Local variables
  int i;
  FILE* f;
  char buff [32];  memset( buff, 0, sizeof(buff) );
  char path [32];  memset( path, 0, sizeof(path) );

  // Set acceleration bias
  sprintf( path, "../Param/board/bias/acc%c", imu->id );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'acc%c bias' not found. \n", imu->id );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->acc->bias[i] = atoi(buff);
  }
  fclose(f);

  // Set acceleration range
  sprintf( path, "../Param/board/range/acc%c", imu->id );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'acc%c range' not found. \n", imu->id );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->acc->range[i] = atoi(buff);
  }
  fclose(f);

  // Set magnetometer bias
  sprintf( path, "../Param/board/bias/mag%c", imu->id );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'mag%c bias' not found. \n", imu->id );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->mag->bias[i] = atoi(buff);
  }
  fclose(f);

  // Set magnetometer range
  sprintf( path, "../Param/board/range/mag%c", imu->id );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'mag%c range' not found. \n", imu->id );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->mag->range[i] = atoi(buff);
  }
  fclose(f);

  // Set gyro bias
  sprintf( path, "../Param/board/bias/gyr%c", imu->id );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'gyr%c bias' not found. \n", imu->id );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->gyr->bias[i] = atoi(buff);
  }
  fclose(f);

  // Display calibration values
  if(DEBUG) {
    printf("   abias%c arange%c   mbias%c mrange%c   gbias%c \n", imu->id, imu->id, imu->id, imu->id, imu->id );
    for ( i=0; i<3; i++ ) {
      printf("     ");
      printf( "%4d    ",   imu->acc->bias[i]  );
      printf( "%4d     ",  imu->acc->range[i] );
      printf( "%4d    ",   imu->mag->bias[i]  );
      printf( "%4d     ",  imu->mag->range[i] );
      printf( "%4d  \n",   imu->gyr->bias[i]  );
    } 
  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_setic
//  Sets the initial conditions for the MPU sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_setic ( imu_struct *imu )  {
  if(DEBUG)  printf( "  Set IMU%c initial conditions \n", imu->id );

  // Assign loop counter values
  if ( HZ_IMU_FAST % HZ_IMU_SLOW != 0 )
    printf( "  *** WARNING ***  Slow loop must divide evenly into fast loop. \n" );
  imu->loops  = HZ_IMU_FAST / HZ_IMU_SLOW;
  imu->count  = 0;
  imu->getmag = false;

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
  imu->gyr->gain = gyr_dt / ( gyr_tc + gyr_dt );
  imu->acc->gain = acc_dt / ( acc_tc + acc_dt );
  imu->mag->gain = mag_dt / ( mag_tc + mag_dt );

  // Display settings
  if (DEBUG) {
    printf("    |  GYR  |  HZ %4d  |  DT %5.3f  |  LPF %6.2f  |  TC %5.2f  |  gain %7.4f  |\n", \
	   HZ_IMU_FAST, gyr_dt, GYR_LPF, gyr_tc, imu->gyr->gain );
    printf("    |  ACC  |  HZ %4d  |  DT %5.3f  |  LPF %6.2f  |  TC %5.2f  |  gain %7.4f  |\n", \
	   HZ_IMU_FAST, acc_dt, ACC_LPF, acc_tc, imu->acc->gain );
    printf("    |  MAG  |  HZ %4d  |  DT %5.3f  |  LPF %6.2f  |  TC %5.2f  |  gain %7.4f  |\n", \
	   HZ_IMU_SLOW, mag_dt, MAG_LPF, mag_tc, imu->mag->gain );
  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  imu_update
//  Update system with new IMU sensor data.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void imu_update ( imu_struct *imu )  {

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
  imu->getmag = false;
  if ( imu->count == 0 ) {
    imu->getmag = true;
    imu->count = imu->loops;
  }
  imu->count--;

  // Sample IMU
  if( mpu_get_gyro_reg( imu->fd, Graw, NULL ) )
    printf( "Error (imu_data): 'mpu_get_gyro_reg' failed. \n" );
  if( mpu_get_accel_reg( imu->fd, Araw, NULL ) )
    printf( "Error (imu_data): 'mpu_get_accel_reg' failed. \n" );
  if(imu->getmag){
    if( mpu_get_compass_reg( imu->fd, Mraw, NULL ) )
    printf( "Error (imu_data): 'mpu_get_compass_reg' failed. \n" );
  } 

  // Gyroscope low pass filter
  k = GYR_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  Ghist[i][j-1] = Ghist[i][j];
    Ghist[i][k-1] = Graw[i];
    g = (float) (Ghist[i][0]);
    for ( j=1; j<k; j++ )  g = g + imu->gyr->gain * (float) ( Ghist[i][j] - g );
    Gavg[i] = g;
  }

  // Accelerometer low pass filter
  k = ACC_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  Ahist[i][j-1] = Ahist[i][j];
    Ahist[i][k-1] = Araw[i];
    a = (float) (Ahist[i][0]);
    for ( j=1; j<k; j++ )  a = a + imu->acc->gain * (float) ( Ahist[i][j] - a );
    Aavg[i] = a;
  }

  // Magnetometer low pass filter
  if(imu->getmag) {
  k = MAG_HIST;
  for ( i=0; i<3; i++ ) {
    for ( j=1; j<k; j++ )  Mhist[i][j-1] = Mhist[i][j];
    Mhist[i][k-1] = Mraw[i];
    m = (float) (Mhist[i][0]);
    for ( j=1; j<k; j++ )  m = m + imu->mag->gain * (float) ( Mhist[i][j] - m );
    Mavg[i] = m;
  }}

  // Shift and orient gyroscope readings
  Gcal[x] =   ( Gavg[y] - imu->gyr->bias[y] ) * GYR_SCALE;
  Gcal[y] =   ( Gavg[x] - imu->gyr->bias[x] ) * GYR_SCALE;
  Gcal[z] = - ( Gavg[z] - imu->gyr->bias[z] ) * GYR_SCALE;

  // Shift and orient accelerometer readings
  Acal[x] =   ( Aavg[y] - imu->acc->bias[y] ) / (double) (imu->acc->range[y]);
  Acal[y] =   ( Aavg[x] - imu->acc->bias[x] ) / (double) (imu->acc->range[x]);
  Acal[z] = - ( Aavg[z] - imu->acc->bias[z] ) / (double) (imu->acc->range[z]);

  // Shift and orient magnetometer readings
  if(imu->getmag) {
  Mcal[x] = ( Mavg[x] - imu->mag->bias[x] ) / (double) (imu->mag->range[x]);
  Mcal[y] = ( Mavg[y] - imu->mag->bias[y] ) / (double) (imu->mag->range[y]);
  Mcal[z] = ( Mavg[z] - imu->mag->bias[z] ) / (double) (imu->mag->range[z]);
  }

  // Push gyroscope values to data structure
  if ( imu->bus == 1 )  pthread_mutex_lock(&mutex_gyrA);
  if ( imu->bus == 2 )  pthread_mutex_lock(&mutex_gyrB);
  for ( i=0; i<3; i++ )  {
    imu->gyr->raw[i] = Graw[i];
    imu->gyr->avg[i] = Gavg[i];
    imu->gyr->cal[i] = Gcal[i];
  }
  if ( imu->bus == 1 )  pthread_mutex_unlock(&mutex_gyrA);
  if ( imu->bus == 2 )  pthread_mutex_unlock(&mutex_gyrB);

  // Push accerometer values to data structure
  if ( imu->bus == 1 )  pthread_mutex_lock(&mutex_accA);
  if ( imu->bus == 2 )  pthread_mutex_lock(&mutex_accB);
  for ( i=0; i<3; i++ )  {
    imu->acc->raw[i] = Araw[i];
    imu->acc->avg[i] = Aavg[i];
    imu->acc->cal[i] = Acal[i];
  }
  if ( imu->bus == 1 )  pthread_mutex_unlock(&mutex_accA);
  if ( imu->bus == 2 )  pthread_mutex_unlock(&mutex_accB);

  // Push magnetometer values to data structure
  if(imu->getmag) {
  if ( imu->bus == 1 )  pthread_mutex_lock(&mutex_magA);
  if ( imu->bus == 2 )  pthread_mutex_lock(&mutex_magB);
  for ( i=0; i<3; i++ )  {
    imu->mag->raw[i] = Mraw[i];
    imu->mag->avg[i] = Mavg[i];
    imu->mag->cal[i] = Mcal[i];
  }
  if ( imu->bus == 1 )  pthread_mutex_unlock(&mutex_magA);
  if ( imu->bus == 2 )  pthread_mutex_unlock(&mutex_magB);
  }

  return;
}



