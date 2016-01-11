
//============================================================
//  ctrl.h
//  Justin M Selfridge
//============================================================
#ifndef _CTRL_H_
#define _CTRL_H_
#include <main.h>


// Define misc items
#define CTRL_HZ     100
#define DEG2RAD     (PI/180.0f)
#define RAD2DEG     (180.0f/PI)
#define STICK_HOLD  3.0
#define I_RESET     0.25


// Define radio channel index
#define CH_1    0
#define CH_2    1
#define CH_3    2
#define CH_4    3
#define CH_5    4
#define CH_6    5
#define CH_7    6
#define CH_8    7


// Define state index
#define CH_R    0    //  Roll
#define CH_P    1    //  Pitch
#define CH_Y    2    //  Yaw
#define CH_T    3    //  Throttle
#define CH_S    4    //  Switch
#define CH_D    5    //  Dial
#define CH_A    6    //  Auxiliary


// Define motor channel index
#define MOT_FR  0
#define MOT_BL  1
#define MOT_FL  2
#define MOT_BR  3


// Axes index
#define X  0
#define Y  1
#define Z  2
#define T  3


// PID index
#define P  0
#define I  1
#define D  2


// Define range values
#define R_RANGE   0.50f
#define P_RANGE   0.50f
#define Y_RANGE   1.50f


// Define throttle gains
#define T_RANGE    100
#define T_MIN     1500
#define T_MAX     1650
#define T_TILT   30.0f


// Full scale ranges
#define MIN   0
#define MAX   1
#define LEFT  0
#define RIGHT 1
#define DOWN  0
#define UP    1


// CTRL structure
typedef struct {
  ushort  hz;
  float   dt;
  double  norm[10];
  double  range[4];
  double  ref[4];
  double  heading;
  double  err[3][3];
  double  gain[3][3];
  double  input[4];
  uint    fullStick[4][2];
  uint    stickHold;
  bool    motorsArmed;
} ctrl_struct;
ctrl_struct ctrl;


// CTRL functions
//void  ctrl_init    ( void );
//void  ctrl_exit    ( void );
//void  ctrl_disarm  ( void );
//void  ctrl_law     ( void );
//void  ctrl_ref     ( void );
//void  ctrl_flags   ( void );
//void  ctrl_switch  ( void );
//void  ctrl_pid     ( void );
//void  ctrl_limit   ( void );


#endif



