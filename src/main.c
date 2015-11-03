
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

  // Set LED indicators
  led_off(LED_MPU);  led_off(LED_PRU);  led_off(LED_LOG);  led_off(LED_MOT);

  // Initial structure values
  sys.running      = true;
  datalog.open     = false;
  datalog.enabled  = true;  // DEBUGGING!! false;
//  imu1.bus = 1;  //imu1.rot[0] = 1;  imu1.rot[4] = 1;  imu1.rot[8] = 1;  // Add this later
  //imu2.bus = 2;  //imu2.rot[0] = 1;  imu2.rot[4] = 1;  imu2.rot[8] = 1;

  // Initialize subsystems
  sys_init();
  timer_init();
//  pru_init();
//  imu_init(&imu1);
  //imu_init(&imu2);
//  ctrl_init();
  timer_begin();

  // Switch power to buses (relocate this code???)
  //ushort IO_SWITCH = 60;
  //gpio_export(IO_SWITCH);
  //gpio_set_dir( IO_SWITCH, OUTPUT_PIN );
  //gpio_set_val( IO_SWITCH, HIGH );
  //if(DEBUG)  printf("Switching on power to I/O bus \n");

  // Run eternal loop
  log_init();  // DEBUGGING!!!!
  while(sys.running);

  // Exit program
  sys_exit();
  return 0;

}



