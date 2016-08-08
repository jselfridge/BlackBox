

#include "main.h"


/**
 *  main
 *  Primary code that runs the UAV avionics.
 */
int main ( void )  {

  // Begin program
  if(DEBUG)  printf("\n--- Begin BlackBox program ---\n");

  printf("invSqrt 2: %f \n", invSqrt(2.0) );

  sys_init();
  io_init();
  flag_init();
  imu_init();
  ahrs_init();
  stab_init();
  gcs_init();
  log_init();
  tmr_init();

  //---  DEBUGGING  ---//
  log_start();
  datalog.enabled = true;
  //-------------------//

  // Run program
  while(running)  usleep(100000);

  //--  DEBUGGING  --//
  datalog.enabled = false;
  log_finish();
  //-----------------//

  // Exit program
  if(DEBUG)  printf("\n\n--- Exit BlackBox program --- \n");
  tmr_exit();
  log_exit();
  gcs_exit();
  stab_exit();
  ahrs_exit();
  imu_exit();
  flag_exit();
  io_exit();
  sys_exit();

  return 0;

}



