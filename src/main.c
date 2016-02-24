
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

  int i, j, m, n;
  char buf_tx1[128] = "Hello there, I am a bit of serial data that I wish to be read.";
  char buf_rx1[128];
  char buf_tx2[128] = "Blah.";
  char buf_rx2[128];

  memset( &buf_rx1, 0, sizeof(buf_rx1) );
  memset( &buf_rx2, 0, sizeof(buf_rx2) );

  // Send serial data
  i = write( uart1.fd, &buf_tx1, 64 );
  usleep(i*100);
  j = read( uart1.fd, &buf_rx1, sizeof(buf_rx1) );
  tcflush( uart1.fd, TCIOFLUSH );

  // Read serial data
  m = write( uart2.fd, &buf_tx2, 64 );
  usleep(m*100);
  n = read( uart2.fd, &buf_rx2, sizeof(buf_rx2) );
  tcflush( uart2.fd, TCIOFLUSH );

  printf("i:%d buf_tx1: %s \n", i, buf_tx1 );
  printf("j:%d buf_rx1: %s \n", j, buf_rx1 );
  printf("m:%d buf_tx2: %s \n", m, buf_tx2 );
  printf("n:%d buf_rx2: %s \n", n, buf_rx2 );

  uart_exit();

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  END UART DEMO

  // Run exit functions
  sys_exit();

  return 0;
}



