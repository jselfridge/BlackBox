
//============================================================
//  uart.c
//  Justin M Selfridge
//============================================================
#include "uart.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  uart_init
//  Initializes the UART communication channels.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void uart_init ( void )  {
  if(DEBUG)  printf( "Initializing serial communications \n" );

  // Set IDs
  uart1.id = 1;
  uart2.id = 2;
  uart4.id = 4;
  uart5.id = 5;

  // Specify device paths
  sprintf( uart1.path, "/dev/ttyO1" );
  sprintf( uart2.path, "/dev/ttyO2" );
  sprintf( uart4.path, "/dev/ttyO4" );
  sprintf( uart5.path, "/dev/ttyO5" );

  // Setup each UART device
  if (DEBUG)  printf("  Configured:  " );
  if (UART1_ENABLED)  uart_setup(&uart1);
  if (UART2_ENABLED)  uart_setup(&uart2);
  if (UART4_ENABLED)  uart_setup(&uart4);
  if (UART5_ENABLED)  uart_setup(&uart5);
  if (DEBUG)  printf("\n");

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  uart_setup
//  Setup the UART parameters.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void uart_setup ( uart_struct *uart )  {
  if (DEBUG)  printf("UART%d ", uart->id );
 
  // Assign settings
  struct termios settings;
  memset( &settings, 0, sizeof( &settings ) );
  settings.c_iflag     = 0;
  settings.c_oflag     = 0;
  settings.c_cflag     = CS8 | CREAD | CLOCAL;
  settings.c_lflag     = 0;
  settings.c_cc[VTIME] = 5;
  settings.c_cc[VMIN]  = 0;
  uart->param = settings;

  // Open the file descriptor
  uart->fd = open ( uart->path, O_RDWR | O_NOCTTY );  //| O_NONBLOCK );
  if ( uart->fd <0 )  printf( "Error (uart): Couldn't open UART%d. \n", uart->id );

  // Set baud rate
  if ( cfsetispeed( &(uart->param), B9600 ) <0 )
    printf( "Error (uart): Couldn't set UART%d buad rate. \n", uart->id );

  // Assign parameters to device
  if ( tcsetattr( uart->fd, TCSAFLUSH, &(uart->param) ) <0 )
    printf( "Error (uart): Failed to assign UART%d parameters. \n", uart->id );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  uart_update
//  Read and write serial data on the UART device.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void uart_update ( uart_struct *uart )  {
  int w = write( uart->fd, uart->txdata, 64 );
  usleep(w*200);
  //int r = read( uart->fd, uart->rxdata, sizeof(char)*64 );
  read( uart->fd, uart->rxdata, sizeof(char)*64 );
  tcflush( uart->fd, TCIOFLUSH );
  return;
}

 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  uart_exit
//  Close the UART file descriptors.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void uart_exit ( void )  {
  if (DEBUG)  printf("Close UART \n" );
  if (UART1_ENABLED)  if ( close(uart1.fd) <0 )  printf( "Error (uart_exit): Couldn't close UART1. \n" ); 
  if (UART2_ENABLED)  if ( close(uart2.fd) <0 )  printf( "Error (uart_exit): Couldn't close UART2. \n" ); 
  if (UART4_ENABLED)  if ( close(uart4.fd) <0 )  printf( "Error (uart_exit): Couldn't close UART4. \n" ); 
  if (UART5_ENABLED)  if ( close(uart5.fd) <0 )  printf( "Error (uart_exit): Couldn't close UART5. \n" ); 
  return;
}



