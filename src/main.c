
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
  //sys_init();
  //sio_init();
  //flg_init();
  //imu_init();
  //ahr_init();
  uart_init();
  //ctl_init();
  //log_init();

  //--  DEBUGGING  --//
  //log_open();
  //datalog.enabled = true;
  //-----------------//

  //tmr_init();

  // Wait for exit condition
  //while(running)  usleep(100000);


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  BEGIN UART DEMO

  // Send serial data
  char buf_tx[64] = "Hello there!";
  int i = write( uart1.fd, &buf_tx, 20 );
  printf("i: %d  buf_tx: %s \n", i, buf_tx );

  // Read serial data
  char buf_rx[64];
  memset( &buf_rx, 0, sizeof(buf_rx) );
  int j = read( uart2.fd, &buf_rx, sizeof(buf_rx) );
  tcflush( uart2.fd, TCIOFLUSH );
  printf("j: %d  buf_rx: %s \n", j, buf_rx );

  uart_exit();

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  END UART DEMO

  // Run exit functions
  sys_exit();

  return 0;
}



