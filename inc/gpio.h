
//============================================================
//  gpio.h
//  Justin M Selfridge
//============================================================
#ifndef _GPIO_H_
#define _GPIO_H_
#include <main.h>


// Local definitions
#define GPIO_PATH  "/sys/class/gpio"
#define MAX_BUF     64


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
void  gpio_export    ( unsigned int gpio );
void  gpio_unexport  ( unsigned int gpio );
void  gpio_set_dir   ( unsigned int gpio, enum PIN_DIR dir );
void  gpio_set_val   ( unsigned int gpio, enum PIN_VAL val );
void  gpio_get_val   ( unsigned int gpio, unsigned int *val );
void  gpio_set_edge  ( unsigned int gpio, char *edge );
int   gpio_fd_open   ( unsigned int gpio );
void  gpio_fd_close  ( int fd );


#endif



