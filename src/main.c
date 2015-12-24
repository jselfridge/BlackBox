
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
  if(DEBUG)  printf("\n--- Begin BlackBox program ---\n");

  // Set LED indicators
  led_off(LED_IMU);  led_off(LED_PRU);  led_off(LED_LOG);  led_off(LED_MOT);

  // Initialize subsystems
  sys_init();
  imu_init(&imu1,1);
  log_init();
  //pru_init();
  //ctrl_init();
  thr_init();


  // Continuous loop and then exit
  while(sys.running);
  sys_exit();

  return 0;
}



