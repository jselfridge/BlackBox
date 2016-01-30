
//============================================================
//  ctrl.h
//  Justin M Selfridge
//============================================================
#ifndef _CTRL_H_
#define _CTRL_H_
#include <main.h>


// Define misc items
//#define CTRL_HZ     100
//#define DEG2RAD     (PI/180.0f)
//#define RAD2DEG     (180.0f/PI)
//#define STICK_HOLD  3.0
//#define I_RESET     0.25


// Define motor channel index
//#define MOT_FR  0
//#define MOT_BL  1
//#define MOT_FL  2
//#define MOT_BR  3


// Axes index
//#define X  0
//#define Y  1
//#define Z  2
//#define T  3


// PID index
//#define P  0
//#define I  1
//#define D  2


// Define range values
//#define R_RANGE   0.50f
//#define P_RANGE   0.50f
//#define Y_RANGE   1.50f


// Define throttle gains
//#define T_RANGE    100
//#define T_MIN     1500
//#define T_MAX     1650
//#define T_TILT   30.0f


// Full scale ranges
//#define MIN   0
//#define MAX   1
//#define LEFT  0
//#define RIGHT 1
//#define DOWN  0
//#define UP    1


// CTRL structure
typedef struct ctrl_struct {
  ushort  blah[3];
} ctrl_struct;
ctrl_struct ctrl;


// CTRL functions
void  ctl_init    ( void );
void  ctl_exit    ( void );
void  ctl_exec    ( void );

//void  ctl_disarm  ( void );
//void  ctl_law     ( void );
//void  ctl_ref     ( void );
//void  ctl_flags   ( void );
//void  ctl_switch  ( void );
//void  ctl_pid     ( void );
//void  ctl_limit   ( void );


#endif



