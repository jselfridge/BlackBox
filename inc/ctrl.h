
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
#define IRESET     0.25


// Define motor channel index
//#define MOT_FR  0
//#define MOT_BL  1
//#define MOT_FL  2
//#define MOT_BR  3


// Define range values (enumerate?)
// Maximum reference commands
// R: rad  P: rad  Y: rad/s  T:??
#define R_RANGE   0.50
#define P_RANGE   0.50
#define Y_RANGE   1.50
#define T_RANGE   0.00


// Define throttle gains
//#define T_MIN     1500
//#define T_MAX     1650
//#define T_TILT   30.0f


// Full scale ranges (enumerate???)
//#define MIN   0
//#define MAX   1
//#define LEFT  0
//#define RIGHT 1
//#define DOWN  0
//#define UP    1


// CTRL structure
typedef struct ctrl_struct {
  float   dt;
  double  scale [4];
  double  pgain [3];
  double  igain [3];
  double  dgain [3];
  double  perr  [3];
  double  ierr  [3];
  double  derr  [3];
  double  cmd   [4];
} ctrl_struct;
ctrl_struct ctrl;


// CTRL functions
void  ctl_init    ( void );
void  ctl_exit    ( void );
void  ctl_exec    ( void );
void  ctl_pid     ( void );
void  ctl_debug   ( void );

#endif



