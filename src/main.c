
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

  
  // UART data structure
  typedef struct uart_struct {
    int      fd;
    char     path[16];
    struct termios  config;
  } uart_struct;
  uart_struct uartA;
  uart_struct uartB;

  int ret;

  sprintf( uartA.path, "/dev/ttyO1" );
  sprintf( uartB.path, "/dev/ttyO2" );

  // Configure UARTA
  printf("Configure UARTA \n");
  memset( &(uartA.config), 0, sizeof( &(uartA.config) ) );
  uartA.config.c_iflag     = 0;
  uartA.config.c_oflag     = 0;
  uartA.config.c_cflag     = CS8 | CREAD | CLOCAL;
  uartA.config.c_lflag     = 0;
  uartA.config.c_cc[VTIME] = 0;
  uartA.config.c_cc[VMIN]  = 1;

  // Configure UARTB
  printf("Configure UARTB \n");
  memset( &(uartB.config), 0, sizeof( &(uartB.config) ) );
  uartB.config.c_iflag     = 0;
  uartB.config.c_oflag     = 0;
  uartB.config.c_cflag     = CS8 | CREAD | CLOCAL;
  uartB.config.c_lflag     = 0;
  uartB.config.c_cc[VTIME] = 0;
  uartB.config.c_cc[VMIN]  = 1;

  // Open file descriptors
  uartA.fd = open ( uartA.path, O_RDWR | O_NOCTTY );
  if ( uartA.fd <0 )  printf( "Error (uart): Couldn't open UART1. \n" );
  uartB.fd = open ( uartB.path, O_RDWR | O_NOCTTY );
  if ( uartB.fd <0 )  printf( "Error (uart): Couldn't open UART2. \n" );

  // Set baud rates
  ret = cfsetispeed( &(uartA.config), B115200 );
  if (ret<0)  printf( "Error (uart): Couldn't set UART1 buad rate. \n" );
  ret = cfsetispeed( &(uartB.config), B115200 );
  if (ret<0)  printf( "Error (uart): Couldn't set UART2 buad rate. \n" );

  // Assign attributes
  ret = tcsetattr( uartA.fd, TCSAFLUSH, &(uartA.config) );
  if (ret<0)  printf( "Error (uart): Failed to assign UART1 parameters. \n" );
  ret = tcsetattr( uartB.fd, TCSAFLUSH, &(uartB.config) );
  if (ret<0)  printf( "Error (uart): Failed to assign UART2 parameters. \n" );

  // Send serial data
  char buf_tx[64] = "Hello there!";
  int i = write( uartA.fd, &buf_tx, 20 );
  printf("i: %d  buf_tx: %s \n", i, buf_tx );

  // Read serial data
  char buf_rx[64];
  memset( &buf_rx, 0, sizeof(buf_rx) );
  int j = read( uartB.fd, &buf_rx, sizeof(buf_rx) );
  tcflush( uartB.fd, TCIOFLUSH );
  printf("j: %d  buf_rx: %s \n", j, buf_rx );

  // Close file descriptors
  ret = close(uartA.fd);
  if (ret<0)  printf( "Error (uart): Couldn't close UART1. \n" );
  ret = close(uartB.fd);
  if (ret<0)  printf( "Error (uart): Couldn't close UART2. \n" );

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  END UART DEMO




  // Run exit functions
  sys_exit();

  return 0;
}



