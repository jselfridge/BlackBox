
//============================================================
//  ctrl.h
//  Justin M Selfridge
//============================================================
#ifndef _CTRL_H_
#define _CTRL_H_
#include <main.h>


// Define misc items
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


// Define roll gains
#define R_RANGE   0.50f
#define R_KP    150.00f
#define R_KI      0.00f
#define R_KD     40.00f


// Define pitch gains
#define P_RANGE   0.50f
#define P_KP    150.00f
#define P_KI      0.00f
#define P_KD     40.00f


// Define yaw gains
#define Y_RANGE   2.00f
#define Y_KP    200.00f
#define Y_KI      0.00f
#define Y_KD     60.00f


// Define throttle gains
#define T_RANGE    100
#define T_MIN     1500
#define T_MAX     1650
#define T_TILT   12.0f


// Full scale ranges
#define MIN   0
#define MAX   1
#define LEFT  0
#define RIGHT 1
#define DOWN  0
#define UP    1


// Global variables
double  norm[10];
double  range[4];
double  ref[4];
double  heading;
double  R_KIerr;
double  P_KIerr;
double  Y_KIerr;
uint    fullStick[4][2];
uint    stickHold;
bool    motorsArmed;


// CTRL functions
void  ctrl_init    ( void );
void  ctrl_disarm  ( void );
void  ctrl_law     ( void );
void  ctrl_ref     ( void );
void  ctrl_flags   ( void );
void  ctrl_switch  ( void );
void  ctrl_pid     ( void );
void  ctrl_limit   ( void );


#endif



