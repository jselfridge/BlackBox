
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
  if(DEBUG)  printf("Begin BlackBox program \n");

  // Initial structure values
  datalog.open     = false;
  datalog.enabled  = true;  // Change back to FALSE after debugging
  //mpu1.bus = 1;
  //mpu2.bus = 2;

  // Initialize subsystems
  sys_init();
  timer_init();  // Pass process into function
  //pru_init();
  //mpu_init(&mpu1);
  //mpu_init(&mpu2);
  //ctrl_init();
  timer_begin();
  while(1);
  //while(uav.running) {}
  //uav_exit();
  return 0;
}



