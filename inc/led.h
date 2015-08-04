
//============================================================
//  led.h
//  Justin M Selfridge
//============================================================
#ifndef _LED_H_
#define _LED_H_
#include <uav.h>


// Custom definitions
#define LED_PATH   "/sys/class/leds/beaglebone:green:usr"
#define MAX_BUF     64


// LED functions
int   led_rmtrig     ( unsigned int index );
int   led_on         ( unsigned int index );
int   led_off        ( unsigned int index );
int   led_blink      ( unsigned int index, unsigned int on, unsigned int off );


#endif



