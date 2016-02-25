
//============================================================
//  main.c
//  Justin M Selfridge
//============================================================
#include "main.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  main
//  Primary code that runs the UAV avionics.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main ( void )  {

  // Begin the program
  if(DEBUG)  printf("\n--- Begin BlackBox program ---\n");
  led_off(LED_SIO);  led_off(LED_IMU);  led_off(LED_LOG);  led_off(LED_MOT);

  // Initialize subsystems
  sys_init();
  sio_init();
  flag_init();
  imu_init();
  ahrs_init();
  uart_init();
  gps_init();
  //ctrl_init();
  //log_init();
  //--  DEBUGGING  --//
  //log_open();
  //datalog.enabled = true;
  //-----------------//
  tmr_init();

  // Wait for exit condition
  while(running)  usleep(100000);

  // Run exit functions
  sys_exit();

  return 0;

}



