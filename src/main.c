

#include "main.h"


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
  imu_init();
  ahrs_init();
  gps_init();
  gcs_init();
  ctrl_init();
  log_init();
  //---  DEBUGGING  ---//
  log_open();
  datalog.enabled = true;
  //-------------------//
  tmr_init();

  // Run program
  while(running)  usleep(100000);

  // Exit program
  if(DEBUG)  printf("\n\n--- Exit BlackBox program --- \n");
  tmr_exit();
  //--  DEBUGGING  --//
  datalog.enabled = false;
  log_close();
  //-----------------//
  log_exit();
  ctrl_exit();
  gcs_exit();
  gps_exit();
  ahrs_exit();
  imu_exit();
  filter_exit();
  io_exit();
  flag_exit();
  sys_exit();

  return 0;

}



