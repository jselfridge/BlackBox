
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
  sys_init();
  sio_init();
  flg_init();
  imu_init();
  ahr_init();
  uart_init();
  gps_init();
  //ctl_init();
  //log_init();
  //--  DEBUGGING  --//
  //log_open();
  //datalog.enabled = true;
  //-----------------//
  //tmr_init();

  // Wait for exit condition
  //while(running)  usleep(100000);

  // Run exit functions
  sys_exit();

  return 0;

  /*
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  BEGIN UART DEMO

  //int i, j, m, n;
  char buf_tx[64] = "Hello there, I am a bit of serial data that I wish to be read.";
  char buf_rx[64];

  //memset( &buf_tx, 0, sizeof(buf_tx) );
  memset( &buf_rx, 0, sizeof(buf_rx) );

  // Send serial data
  sprintf( uart1.txdata, buf_tx );
  uart_update(&uart1);

  // Read serial data
  uart_update(&uart2);
  sprintf( buf_rx, uart2.rxdata );

  printf("buf_tx: %s \n", buf_tx );
  printf("buf_rx: %s \n", buf_rx );

  uart_exit();

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  END UART DEMO
  */

}



