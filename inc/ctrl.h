
//============================================================
//  ctrl.h
//  Justin M Selfridge
//============================================================
#ifndef _CTRL_H_
#define _CTRL_H_
#include <main.h>


// Define angle conversions
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

// Roll and pitch tests
// Flight01:   R0.5, P200, D25
// Flight02:   R0.5, P150, D30
// Flight03:   R0.5, P150, D35
// Flight04:   R0.5, P175, D35
// Flight05:   R0.5, P150, D35

#define R_RANGE   0.50f
#define R_KP    150.00f
#define R_KI      0.00f
#define R_KD     35.00f

#define P_RANGE   0.50f
#define P_KP    150.00f
#define P_KI      0.00f
#define P_KD     35.00f

// Yaw tests
// Flight06 values were used during Flight01-05.
// Flight06:   R3.0, P200, D60
// Flight07:   R2.0, P200, D60

#define Y_RANGE   2.00f
#define Y_KP    200.00f
#define Y_KI      0.00f
#define Y_KD     60.00f

// Throttle tests
// Flight01: T00 (bug in code, no good flights)
// Flight02: T00 (stable flight, good data)
// Flight03: T20 (very transient at takeoff)
// Flight04: T10 (very stable flight, moving on to doublets)
// Flight05: T12

// Define throttle values
// Min is around 1450
// Detrmine max in field
#define T_HOVER   1550
#define T_RANGE    400
#define T_TUNE     100
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
void    ctrl_init    ( void );
void    ctrl_disarm  ( void );
void    ctrl_law     ( void );
void    ctrl_ref     ( void );
void    ctrl_flags   ( void );
void    ctrl_switch  ( void );
void    ctrl_pid     ( void );
void    ctrl_limit   ( void );


#endif



