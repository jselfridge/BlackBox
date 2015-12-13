
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
  led_on(LED_MPU);  led_on(LED_PRU);  led_on(LED_LOG);  led_on(LED_MOT);

  // Initialize subsystems
  sys_init();
  log_init();
  imu_init(&imu1,1);
  thread_init();

  //timer_init();
  //pru_init();
  //imu_init(&imu2);
  //ctrl_init();
  //timer_begin();

  // Continuous loop and then exit
  while(sys.running);
  sys_exit();

  return 0;
}



