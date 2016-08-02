

#include "imu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "ahrs.h"
#include "i2c.h"
#include "led.h"
#include "mpu.h"
#include "sys.h"
#include "timer.h"


/**
 *  imu_init
 *  Initializes an MPU sensor.
 */
void imu_init ( void )  {
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

    // Low pass filter values
    imuA.gyr  = &gyrA;  imuA.gyr->lpf[0] = 1.00;  imuA.gyr->lpf[1] = 1.00;  imuA.gyr->lpf[2] = 1.00;    // 0.50
    imuA.acc  = &accA;  imuA.acc->lpf[0] = 1.00;  imuA.acc->lpf[1] = 1.00;  imuA.acc->lpf[2] = 1.00;    // 0.10
    imuA.mag  = &magA;  imuA.mag->lpf[0] = 1.00;  imuA.mag->lpf[1] = 1.00;  imuA.mag->lpf[2] = 1.00;    // 0.80

    // Complimentary filter values
    imuA.comp  = 0.99;
    imuA.roll  = 0.00;
    imuA.pitch = 0.00; 

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

    // Low pass filter values
    imuB.gyr  = &gyrB;  imuB.gyr->lpf[0] = 1.00;  imuB.gyr->lpf[1] = 1.00;  imuB.gyr->lpf[2] = 1.00;    // 0.50
    imuB.acc  = &accB;  imuB.acc->lpf[0] = 1.00;  imuB.acc->lpf[1] = 1.00;  imuB.acc->lpf[2] = 1.00;    // 0.10
    imuB.mag  = &magB;  imuB.mag->lpf[0] = 1.00;  imuB.mag->lpf[1] = 1.00;  imuB.mag->lpf[2] = 1.00;    // 0.80

    // Complimentary filter values
    imuB.comp  = 0.99;
    imuB.roll  = 0.00;
    imuB.pitch = 0.00; 

    // Setup functions
    i2c_init( &(imuB.fd), imuB.bus, imuB.addr );
    imu_param(&imuB);
    imu_getcal(&imuB);

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
  FILE *f;
  char buff [32];  memset( buff, 0, sizeof(buff) );
  char path [32];  memset( path, 0, sizeof(path) );

  // Set accelerometer bias
  sprintf( path, "../Param/board/bias/acc%c", imu->id );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'acc%c bias' not found. \n", imu->id );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->acc->bias[i] = atoi(buff);
  }
  fclose(f);

  // Set accelerometer range
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

  // Local IMU arrays
  short  Gr[3], Ar[3], Mr[3];    // Raw
  double Gs[3], As[3], Ms[3];    // Scaled
  double Gp[3], Ap[3], Mp[3];    // Previous
  double Gg[3], Ag[3], Mg[3];    // LPF Gain
  double Gf[3], Af[3], Mf[3];    // Filtered

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

  // Get gyroscope values
  pthread_mutex_lock( &(imu->gyr->mutex) );
  for ( i=0; i<3; i++ )  {  Gp[i] = imu->gyr->filter[i];  Gg[i] = imu->gyr->lpf[i];  }
  pthread_mutex_unlock( &(imu->gyr->mutex) );

  // Get accerometer values
  pthread_mutex_lock( &(imu->acc->mutex) );
  for ( i=0; i<3; i++ )  {  Ap[i] = imu->acc->filter[i];  Ag[i] = imu->acc->lpf[i];  }
  pthread_mutex_unlock( &(imu->acc->mutex) );

  // Get magnetometer values
  if(imu->getmag) {
  pthread_mutex_lock( &(imu->mag->mutex) );
  for ( i=0; i<3; i++ )  {  Mp[i] = imu->mag->filter[i];  Mg[i] = imu->mag->lpf[i];  }
  pthread_mutex_unlock( &(imu->mag->mutex) );
  }

  // Low pass filter
  for ( i=0; i<3; i++ )  {
    Gf[i] = Gp[i] + Gg[i] * ( Gs[i] - Gp[i] );
    Af[i] = Ap[i] + Ag[i] * ( As[i] - Ap[i] );
    if (imu->getmag)  {
    Mf[i] = Mp[i] + Mg[i] * ( Ms[i] - Mp[i] );
    }
  }

  // Complimentary filter variables
  double dt, gain, R, P;

  // Get complimentary filter values
  dt = 1.0 / HZ_IMU_FAST;
  pthread_mutex_lock( &(imu->mutex) );
  gain = imu->comp;
  R = imu->roll;
  P = imu->pitch;
  pthread_mutex_unlock( &(imu->mutex) );

  // Complimentary filter attitude angles
  R = gain * ( R + Gs[0] * dt ) + ( 1.0 - gain ) * atan2( -As[1], -As[2] );
  P = gain * ( P + Gs[1] * dt ) + ( 1.0 - gain ) * atan2(  As[0], -As[2] );

  // Push gyroscope values to data structure
  pthread_mutex_lock( &(imu->gyr->mutex) );
  for ( i=0; i<3; i++ )  {
    imu->gyr->raw[i]    = Gr[i];
    imu->gyr->scaled[i] = Gs[i];
    imu->gyr->filter[i] = Gf[i];
  }
  pthread_mutex_unlock( &(imu->gyr->mutex) );

  // Push accerometer values to data structure
  pthread_mutex_lock( &(imu->acc->mutex) );
  for ( i=0; i<3; i++ )  {
    imu->acc->raw[i]    = Ar[i];
    imu->acc->scaled[i] = As[i];
    imu->acc->filter[i] = Af[i];
  }
  pthread_mutex_unlock( &(imu->acc->mutex) );

  // Push magnetometer values to data structure
  if(imu->getmag) {
  pthread_mutex_lock( &(imu->mag->mutex) );
  for ( i=0; i<3; i++ )  {
    imu->mag->raw[i]    = Mr[i];
    imu->mag->scaled[i] = Ms[i];
    imu->mag->filter[i] = Mf[i];
  }
  pthread_mutex_unlock( &(imu->mag->mutex) );
  }

  // Push complimentary filter values to data structure
  pthread_mutex_lock( &(imu->mutex) );
  imu->roll  = R;
  imu->pitch = P;
  pthread_mutex_unlock( &(imu->mutex) );

  return;
}


/**
 *  imu_state
 *  Calculate the rotational state from various signals and filters
 */
void imu_state ( void )  {

  // Local variables
  ushort i;
  double att[3];
  double ang[3];

  // Zero out states
  for ( i=0; i<3; i++ )  {  att[i] = 0.0;  ang[i] = 0.0;  }

  // IMUA states
  if (IMUA_ENABLED)  {

    // Comp filter values (no yaw value)
    pthread_mutex_lock(&imuA.mutex);
    att[0] += imuA.roll;
    att[1] += imuA.pitch;
    pthread_mutex_unlock(&imuA.mutex);

    // Loop through states
    for ( i=0; i<3; i++ )  {

      // LPF gyro values
      pthread_mutex_lock(&gyrA.mutex);
      ang[i] += gyrA.filter[i];
      pthread_mutex_unlock(&gyrA.mutex);

      // AHRS values
      //pthread_mutex_lock(&ahrsA.mutex);
      //att[i] += ahrsA.eul[i];
      //ang[i] += ahrsA.deul[i];
      //pthread_mutex_unlock(&ahrsA.mutex);

    }
  }

  // IMUB states
  if (IMUB_ENABLED)  {

    // Comp filter values (no yaw value)
    pthread_mutex_lock(&imuB.mutex);
    att[0] += imuB.roll;
    att[1] += imuB.pitch;
    pthread_mutex_unlock(&imuB.mutex);

    // Loop through states
    for ( i=0; i<3; i++ )  {

      // LPF gyro values
      pthread_mutex_lock(&gyrB.mutex);
      ang[i] += gyrB.filter[i];
      pthread_mutex_unlock(&gyrB.mutex);

      // AHRS values
      //pthread_mutex_lock(&ahrsB.mutex);
      //att[i] += ahrsB.eul[i];
      //ang[i] += ahrsB.deul[i];
      //pthread_mutex_unlock(&ahrsB.mutex);

    }
  }

  // Average all the data sources
  //if ( IMUA_ENABLED && IMUB_ENABLED )  {  for ( i=0; i<3; i++ )  {  att[i] /= 4.0;  ang[i] /= 4.0;  }  }
  //else                                 {  for ( i=0; i<3; i++ )  {  att[i] /= 2.0;  ang[i] /= 2.0;  }  }
  for ( i=0; i<3; i++ )  {  att[i] /= 2.0;  ang[i] /= 2.0;  }

  // Correction b/c comp filter has no yaw value  
  //att[2] *= 2.0; 

  // Push to data structure
  pthread_mutex_lock(&rot.mutex);
  for ( i=0; i<3; i++ ) rot.att[i] = att[i];
  for ( i=0; i<3; i++ ) rot.ang[i] = ang[i];
  pthread_mutex_unlock(&rot.mutex);

  return;
}



