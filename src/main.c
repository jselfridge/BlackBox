

#include "main.h"


//temp
#include <stdlib.h>

/**
 *  main
 *  Primary code that runs the UAV avionics.
 */
int main ( void )  {

  // Begin program
  if(DEBUG)  printf("\n--- Begin BlackBox program ---\n");
  led_off(LED_SIO);  led_off(LED_IMU);  led_off(LED_LOG);  led_off(LED_MOT);
  sys_init();
  flag_init();
  io_init();
  filter_init();
  //imu_init();
  //ahrs_init();
  //gps_init();
  //gcs_init();
  //ctrl_init();
  //log_init();
  //---  DEBUGGING  ---//
  //log_open();
  //datalog.enabled = true;
  //-------------------//
  //tmr_init();

  // Run program
  //while(running)  usleep(100000);




  //---  DEBUGGING  ---//
  printf("\n\nFilter debugging...\n");

  //printf("%5.3f %5.1f %f \n", filter_gyrA.dt, filter_gyrA.freq, filter_gyrA.gain );
  //filter_freq( &filter_gyrA, 100.0 );
  //printf("%5.3f %5.1f %f \n", filter_gyrA.dt, filter_gyrA.freq, filter_gyrA.gain );

  filter_gyrA.data = malloc( 2 );
  //free(filter_gyrA.data);
  //filter_gyrA.data[0] = 4.0;
  //filter_gyrA.data[1] = 3.0;
  //printf("%f %f \n", filter_gyrA.data[0], filter_gyrA.data[1] );
  printf("%d \n", sizeof(filter_gyrA.data[3]) );

  //-------------------//




  // Exit program
  if(DEBUG)  printf("\n\n--- Exit BlackBox program --- \n");
  //tmr_exit();
  //--  DEBUGGING  --//
  //datalog.enabled = false;
  //log_close();
  //-----------------//
  //log_exit();
  //ctrl_exit();
  //gcs_exit();
  //gps_exit();
  //ahrs_exit();
  //imu_exit();
  filter_exit();
  io_exit();
  flag_exit();
  sys_exit();

  return 0;

}



