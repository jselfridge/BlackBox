
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

  // Initial structure values   === Move to ***_init() functions ===
  datalog.open     = false;
  datalog.enabled  = true;  //--- DEBUG ---// false;
  imu1.bus = 2;  //imu1.rot[0] = 1;  imu1.rot[4] = 1;  imu1.rot[8] = 1;  // Add this later
  //imu2.bus = 2;  //imu2.rot[0] = 1;  imu2.rot[4] = 1;  imu2.rot[8] = 1;

  // Initialize subsystems
  sys_init();
  log_init();  //--- DEBUG ---//
  imu_init(&imu1);
  thread_init();



  //timer_init();
  //pru_init();
  //imu_init(&imu2);
  //ctrl_init();
  //timer_begin();

  // Switch power to buses (relocate this code???)
  //ushort IO_SWITCH = 60;
  //gpio_export(IO_SWITCH);
  //gpio_set_dir( IO_SWITCH, OUTPUT_PIN );
  //gpio_set_val( IO_SWITCH, HIGH );
  //if(DEBUG)  printf("Switching on power to I/O bus \n");




  // Continuous loop and then exit
  while(sys.running);
  sys_exit();

  return 0;
}



