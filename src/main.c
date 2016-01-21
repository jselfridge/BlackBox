
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
  led_on(LED_IMU);  led_on(LED_PRU);  led_on(LED_LOG);  led_on(LED_MOT);

  // Initialize subsystems
  sys_init();

  //imu_init(&imu1,1);
  //log_init();
  //pru_init();
  //ctrl_init();
  //thr_init();

  // Wait for exit condition
  while(running)  usleep(100000);

  return 0;
}



