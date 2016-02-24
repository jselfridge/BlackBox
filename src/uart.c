
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
  uart_setup(&uart1);
  uart_setup(&uart2);
  uart_setup(&uart4);
  uart_setup(&uart5);
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
  settings.c_cc[VTIME] = 0;
  settings.c_cc[VMIN]  = 1;
  uart->param = settings;

  // Open the file descriptor
  uart->fd = open ( uart->path, O_RDWR | O_NOCTTY );
  if ( uart->fd <0 )  printf( "Error (uart): Couldn't open UART%d. \n", uart->id );

  // Set baud rate
  if ( cfsetispeed( &(uart->param), B115200 ) <0 )
    printf( "Error (uart): Couldn't set UART%d buad rate. \n", uart->id );

  // Assign parameters to device
  if ( tcsetattr( uart->fd, TCSAFLUSH, &(uart->param) ) <0 )
    printf( "Error (uart): Failed to assign UART%d parameters. \n", uart->id );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  uart_exit
//  Close the UART file descriptors.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void uart_exit ( void )  {
  if (DEBUG)  printf("Exit UARTs \n" );
  if ( close(uart1.fd) <0 )  printf( "Error (uart_exit): Couldn't close UART1. \n" ); 
  if ( close(uart2.fd) <0 )  printf( "Error (uart_exit): Couldn't close UART2. \n" ); 
  if ( close(uart4.fd) <0 )  printf( "Error (uart_exit): Couldn't close UART4. \n" ); 
  if ( close(uart5.fd) <0 )  printf( "Error (uart_exit): Couldn't close UART5. \n" ); 
  return;
}



