
//============================================================
//  gpio.h
//  Justin M Selfridge
//============================================================
#ifndef _GPIO_H_
#define _GPIO_H_
#include <main.h>


// Local definitions
#define GPIO_PATH  "/sys/class/gpio"
#define GPIO_BUF     64


// Enumerations
enum PIN_DIR {
  INPUT_PIN  = 0,
  OUTPUT_PIN = 1
};
enum PIN_VAL {
  LOW  = 0,
  HIGH = 1
};


// GPIO functions
void  gpio_export    ( uint gpio );
void  gpio_unexport  ( uint gpio );
void  gpio_setdir    ( uint gpio, enum PIN_DIR dir );
void  gpio_setval    ( uint gpio, enum PIN_VAL val );
void  gpio_getval    ( uint gpio, uint *val );
void  gpio_setedge   ( uint gpio, char *edge );
int   gpio_fdopen    ( uint gpio );
void  gpio_fdclose   ( int  fd );


#endif



