
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

  //int uart1_fd;  //, uart2_fd;
  /*
  // Configure UART1
  printf("Configure UART1 \n");
  struct termios uart1;
  memset( &uart1, 0, sizeof(uart1) );
  uart1.c_iflag     = 0;
  uart1.c_oflag     = 0;
  uart1.c_cflag     = CS8 | CREAD | CLOCAL;
  uart1.c_lflag     = 0;
  uart1.c_cc[VTIME] = 0;
  uart1.c_cc[VMIN]  = 1;
  */
  /*
  // Configure UART2
  printf("Configure UART2 \n");
  struct termios uart2;
  memset( &uart2, 0, sizeof(uart2) );
  uart2.c_iflag     = 0;
  uart2.c_oflag     = 0;
  uart2.c_cflag     = CS8 | CREAD | CLOCAL;
  uart2.c_lflag     = 0;
  uart2.c_cc[VTIME] = 0;
  uart2.c_cc[VMIN]  = 1;
  */

  // Open file descriptors
  //uart1_fd = open ( "/dev/ttyO1", O_RDWR | O_NOCTTY );
  //sys_err( uart1_fd <0, "Error (BLAH): Couldn't open UART1." );
  //uart2_fd = open ( "/dev/ttyO2", O_RDWR | O_NOCTTY );
  //sys_err( uart2_fd <0, "Error (BLAH): Couldn't open UART2." );

  // Set baud rates
  //sys.ret = cfsetispeed( &uart1, B115200 );
  //sys_err( sys.ret <0, "Error (BLAH): Couldn't set UART1 buad rate." );
  //sys.ret = cfsetispeed( &uart2, B115200 );
  //sys_err( sys.ret <0, "Error (BLAH): Couldn't set UART2 buad rate." );

  // Assign attributes
  //sys.ret = tcsetattr( uart1_fd, TCSAFLUSH, &uart1 );
  //sys_err( sys.ret <0, "Error (BLAH): Failed to assign UART1 parameters." );
  //sys.ret = tcsetattr( uart2_fd, TCSAFLUSH, &uart2 );
  //sys_err( sys.ret <0, "Error (BLAH): Failed to assign UART2 parameters." );

  // Send serial data
  //char buf_tx[64] = "Hello there!";
  //int i = write( uart1_fd, &buf_tx, 10 );
  //printf("i: %d  buf_tx: %s \n", i, buf_tx );

  // Obtain serial data
  //char buf_rx[64];
  //memset( &buf_rx, 0, sizeof(buf_rx) );
  //int j = read( uart1_fd, &buf_rx, sizeof(buf_rx) );
  //tcflush( uart1_fd, TCIOFLUSH );
  //printf("j: %d  buf_rx: %s \n", j, buf_rx );

  // Close file descriptors
  //sys.ret = close(uart1_fd);
  //sys_err( sys.ret<0, "Error (BLAH): Couldn't close UART1." );
  //sys.ret = close(uart2_fd);
  //sys_err( sys.ret<0, "Error (BLAH): Couldn't close UART2." );





  // Set LED indicators
  led_off(LED_IMU);  led_off(LED_PRU);  led_off(LED_LOG);  led_off(LED_MOT);

  // Initialize subsystems
  sys_init();
  imu_init();
  log_init();
  pru_init();
  //param_init();
  //ctrl_init();
  thr_init();

  // Continuous loop and then exit
  while(sys.running);
  sys_exit();

  return 0;
}



