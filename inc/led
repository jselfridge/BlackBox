
//============================================================
//  led.h
//  Justin M Selfridge
//============================================================
#ifndef _LED_H_
#define _LED_H_
#include <main.h>


// Custom definitions
#define LED_PATH   "/sys/class/leds/beaglebone:green:usr"
#define LED_BUF    64


// Enumerations
enum LED {
  LED_MOT = 0,
  LED_LOG = 1,
  LED_IMU = 2,
  LED_SIO = 3
};


// LED functions
int   led_rmtrig     ( uint index );
int   led_on         ( uint index );
int   led_off        ( uint index );
int   led_blink      ( uint index, uint on, uint off );


#endif



