

#include "main.h"


/**
 *  main
 *  Primary code that runs the UAV avionics.
 */
int main ( void )  {

  // Begin program
  if (DEBUG)  printf("\n--- Begin BlackBox program ---\n");
  sys_init();
  io_init();
  flag_init();
  imu_init();
  //stab_init();
  gcs_init();
  log_init();
  tmr_init();

  // Run program
  if (DEBUG)  {  log_start();  datalog.enabled = true;  led_on(LED_LOG);  }
  while(running)  usleep(100000);
  if (DEBUG)  {  datalog.enabled = false;  log_finish();  led_off(LED_LOG);  }

  // Exit program
  if (DEBUG)  printf("\n\n--- Exit BlackBox program --- \n");
  tmr_exit();
  log_exit();
  gcs_exit();
  //stab_exit();
  imu_exit();
  flag_exit();
  io_exit();
  sys_exit();

  return 0;

}



