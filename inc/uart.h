
//============================================================
//  uart.h
//  Justin M Selfridge
//============================================================
#ifndef _UART_H_
#define _UART_H_
#include <main.h>


// Define statements


// UART structure
typedef struct uart_struct {
  uint   id;
  int    fd;
  char   path   [16];
  char   txdata [128];
  char   rxdata [128];
  struct termios param;
} uart_struct;
uart_struct uart1;
uart_struct uart2;
uart_struct uart4;
uart_struct uart5;


// UART functions
void  uart_init    ( void );
void  uart_setup   ( uart_struct *uart );
void  uart_update  ( uart_struct *uart );
void  uart_exit    ( void );


#endif



