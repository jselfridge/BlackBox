

#include "filter.h"


/**
 *  filter_init
 *  Initializes the system signal filters.
 */
void filter_init (  )  {

  if(DEBUG)  printf( "Initializing filters \n" );

  /*
  // Start initialization
  led_blink( LED_IMU, 200, 200 );

  // Setup IMUA
  if (IMUA_ENABLED)  {

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
  if (IMUB_ENABLED)  {

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
  */
  return;
}


/**
 *  filter_exit
 *  Terminate the system filters.
 */
void filter_exit ( void )  {
  if(DEBUG)  printf("Close filters \n");
  // Add code as needed...
  return;
}


/**
 *  imu_setic
 *  Sets the initial conditions for the MPU sensor.
 */
/*void imu_setic ( imu_struct *imu )  {
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
*/

/**
 *  filter_lpf
 *  Run a signal through a low pass filter.
 */
double filter_lpf ( filter_struct *signal, double val )  {

  // Local variables
  ushort i;
  double f;

  // Shift stored data
  for ( i=1; i < signal->hist; i++ )  signal->data[i-1] = signal->data[i];

  // Assign newest data value
  signal->data [ signal->hist -1 ] = val;

  // Initialize starting value
  f = signal->data[0];

  // Loop through sequence
  for ( i=1; i < signal->hist; i++ )  f = f + signal->gain * ( signal->data[i] - f );

  // Return filtered result
  return f;

}



