

#include "main.h"

/**
 * @brief       This is the main program for the BlackBox program.
 * @param[in]   None
 * @return      None
 */
int main ( void )
{
  printf("\n\nHello World!\n\n");
  return 0;
}


/**
 *  main
 *  Primary code that runs the UAV avionics.
 */
/*
int main ( void )  {

  // Begin program
  if (DEBUG)  printf("\n--- Begin BlackBox program ---\n");
  sys_init();
  io_init();
  flag_init();
  imu_init();
  ekf_init();
  stab_init();
  ins_init();
  gcs_init();
  log_init();
  tmr_init();

  // Run program
  //if (DEBUG)  {  log_start();  datalog.enabled = true;  led_on(LED_LOG);  }
  while(running)  usleep(100000);
  //if (DEBUG)  {  datalog.enabled = false;  log_finish();  led_off(LED_LOG);  }

  // Exit program
  if (DEBUG)  printf("\n\n--- Exit BlackBox program --- \n");
  tmr_exit();
  log_exit();
  gcs_exit();
  ins_exit();
  stab_exit();
  ekf_exit();
  imu_exit();
  flag_exit();
  io_exit();
  sys_exit();

  return 0;

}
*/


