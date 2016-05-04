

#include "imu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "filter.h"
#include "i2c.h"
#include "led.h"
#include "mpu.h"
#include "sys.h"
#include "timer.h"


/**
 *  imu_init
 *  Initializes an MPU sensor.
 */
void imu_init (  )  {
  if(DEBUG)  printf( "Initializing IMU \n" );

  // Start initialization
  led_blink( LED_IMU, 200, 200 );

  // Check timing conditions
  if ( HZ_IMU_FAST % HZ_IMU_SLOW != 0 )
    printf( "  *** WARNING ***  Slow loop must divide evenly into fast loop. \n" );
  uint loops = HZ_IMU_FAST / HZ_IMU_SLOW;

  // Setup IMUA
  if (IMUA_ENABLED)  {

    // I2C parameters
    imuA.id   = 'A';
    imuA.bus  = 1;
    imuA.addr = 0x68;

    // Loop counter values
    imuA.count  = 0;
    imuA.loops  = loops;
    imuA.getmag = false;

    // IMU data structures
    imuA.gyr  = &gyrA;
    imuA.acc  = &accA;
    imuA.mag  = &magA;

    // Setup functions
    i2c_init( &(imuA.fd), imuA.bus, imuA.addr );
    imu_param(&imuA);
    imu_getcal(&imuA);

  }

  // Setup IMUB
  if (IMUB_ENABLED)  {

    // I2C parameters
    imuB.id   = 'B';
    imuB.bus  = 2;
    imuB.addr = 0x68;

    // Loop counter values
    imuB.count  = 0;
    imuB.loops  = loops;
    imuB.getmag = false;

    // IMU data structures
    imuB.gyr  = &gyrB;
    imuB.acc  = &accB;
    imuB.mag  = &magB;

    // Setup functions
    i2c_init( &(imuB.fd), imuB.bus, imuB.addr );
    imu_param(&imuB);
    imu_getcal(&imuB);

  }

  // Setup shared IMU
  if ( IMUA_ENABLED && IMUB_ENABLED )  {

    // IMU data structures
    imu.gyr  = &gyr;
    imu.acc  = &acc;
    imu.mag  = &mag;

  }

  // IMU warmup period
  usleep(300000);
  led_on(LED_IMU);

  return;
}


/**
 *  imu_exit
 *  Terminate an MPU sensor.
 */
void imu_exit ( void )  {
  if(DEBUG)  printf("Close IMU \n");
  if(IMUA_ENABLED)  i2c_exit( &(imuA.fd) );
  if(IMUB_ENABLED)  i2c_exit( &(imuB.fd) );
  led_off(LED_IMU);
  return;
}


/**
 *  imu_param
 *  Assign parameters to an MPU sensor.
 */
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


/**
 *  imu_getcal
 *  Gets the calibration parameters for the MPU sensor.
 */
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


/**
 *  imu_update
 *  Update system with new IMU sensor data.
 */
void imu_update ( imu_struct *imu )  {

  // Loop counter
  ushort i;

  // Array index
  ushort x=0, y=1, z=2;

  // Local IMU struct arrays
  short  Gr[3], Ar[3], Mr[3];
  double Gs[3], As[3], Ms[3];
  double Gf[3], Af[3], Mf[3];

  // Increment counter
  imu->getmag = false;
  if ( imu->count == 0 ) {
    imu->getmag = true;
    imu->count = imu->loops;
  }
  imu->count--;

  // Raw IMU data
  if( mpu_get_gyro_reg( imu->fd, Gr, NULL ) )
    printf( "Error (imu_data): 'mpu_get_gyro_reg' failed. \n" );
  if( mpu_get_accel_reg( imu->fd, Ar, NULL ) )
    printf( "Error (imu_data): 'mpu_get_accel_reg' failed. \n" );
  if(imu->getmag){
    if( mpu_get_compass_reg( imu->fd, Mr, NULL ) )
    printf( "Error (imu_data): 'mpu_get_compass_reg' failed. \n" );
  } 

  // Scaled gyroscope readings
  Gs[x] = - ( Gr[x] - imu->gyr->bias[x] ) * GYR_SCALE;
  Gs[y] =   ( Gr[y] - imu->gyr->bias[y] ) * GYR_SCALE;
  Gs[z] = - ( Gr[z] - imu->gyr->bias[z] ) * GYR_SCALE;

  // Scaled accelerometer readings
  As[x] = - ( Ar[x] - imu->acc->bias[x] ) / (double) (imu->acc->range[x]);
  As[y] =   ( Ar[y] - imu->acc->bias[y] ) / (double) (imu->acc->range[y]);
  As[z] = - ( Ar[z] - imu->acc->bias[z] ) / (double) (imu->acc->range[z]);

  // Scaled magnetometer readings
  if(imu->getmag) {
  Ms[x] = - ( Mr[y] - imu->mag->bias[y] ) / (double) (imu->mag->range[y]);
  Ms[y] =   ( Mr[x] - imu->mag->bias[x] ) / (double) (imu->mag->range[x]);
  Ms[z] =   ( Mr[z] - imu->mag->bias[z] ) / (double) (imu->mag->range[z]);
  }

  // IMUA low pass filter
  if ( imu->id == 'A' )  {
    filter_lpf ( &filter_gyrA, Gs, Gf );
    filter_lpf ( &filter_accA, As, Af );
    if (imu->getmag)  {
    filter_lpf ( &filter_magA, Ms, Mf );
    }
  }

  // IMUB low pass filter
  if ( imu->id == 'B' )  {
    filter_lpf ( &filter_gyrB, Gs, Gf );
    filter_lpf ( &filter_accB, As, Af );
    if (imu->getmag)  {
    filter_lpf ( &filter_magB, Ms, Mf );
    }
  }

  // Push gyroscope values to data structure
  if ( imu->bus == 1 )  pthread_mutex_lock(&mutex_gyrA);
  if ( imu->bus == 2 )  pthread_mutex_lock(&mutex_gyrB);
  for ( i=0; i<3; i++ )  {
    imu->gyr->raw[i]    = Gr[i];
    imu->gyr->scaled[i] = Gs[i];
    imu->gyr->filter[i] = Gf[i];
  }
  if ( imu->bus == 1 )  pthread_mutex_unlock(&mutex_gyrA);
  if ( imu->bus == 2 )  pthread_mutex_unlock(&mutex_gyrB);

  // Push accerometer values to data structure
  if ( imu->bus == 1 )  pthread_mutex_lock(&mutex_accA);
  if ( imu->bus == 2 )  pthread_mutex_lock(&mutex_accB);
  for ( i=0; i<3; i++ )  {
    imu->acc->raw[i]    = Ar[i];
    imu->acc->scaled[i] = As[i];
    imu->acc->filter[i] = Af[i];
  }
  if ( imu->bus == 1 )  pthread_mutex_unlock(&mutex_accA);
  if ( imu->bus == 2 )  pthread_mutex_unlock(&mutex_accB);

  // Push magnetometer values to data structure
  if(imu->getmag) {
  if ( imu->bus == 1 )  pthread_mutex_lock(&mutex_magA);
  if ( imu->bus == 2 )  pthread_mutex_lock(&mutex_magB);
  for ( i=0; i<3; i++ )  {
    imu->mag->raw[i]    = Mr[i];
    imu->mag->scaled[i] = Ms[i];
    imu->mag->filter[i] = Mf[i];
  }
  if ( imu->bus == 1 )  pthread_mutex_unlock(&mutex_magA);
  if ( imu->bus == 2 )  pthread_mutex_unlock(&mutex_magB);
  }

  return;
}



