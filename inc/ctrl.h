
//============================================================
//  ctrl.h
//  Justin M Selfridge
//============================================================
#ifndef _CTRL_H_
#define _CTRL_H_
#include <main.h>


// Define misc items
#define IRESET    0.25
#define SYSTEM    "plane"


// Define motor channel index (enumerate??)
#define MOT_FR  0
#define MOT_BL  1
#define MOT_FL  2
#define MOT_BR  3


// Define range values
// Maximum reference commands
// R: rad  P: rad  Y: rad/s  T:%thrust
#define R_RANGE   0.50
#define P_RANGE   0.50
#define Y_RANGE   1.50
#define T_RANGE   1.00


// Define throttle gains
// MIN: 0.00    MAX: 0.30    TILT: 1.00
#define TMIN   0.00
#define TMAX   0.00
#define TILT   0.00


// Quad PID Gains
// P: 0.30    I: 0.15    D: 0.06
#define QUAD_PX    0.00
#define QUAD_PY    0.00
#define QUAD_PZ    0.00
#define QUAD_IX    0.00
#define QUAD_IY    0.00
#define QUAD_IZ    0.00
#define QUAD_DX    0.00
#define QUAD_DY    0.00
#define QUAD_DZ    0.00


// Plane PD Gains
// P: ????    I: ????    D: ????
#define PLANE_PX    0.00
#define PLANE_PY    0.00
#define PLANE_PZ    0.00
#define PLANE_DX    0.00
#define PLANE_DY    0.00
#define PLANE_DZ    0.00


// CTRL structure
typedef struct ctrl_struct {
  double  dt;
  double  scale [4];
  double  pgain [3];
  double  igain [3];
  double  dgain [3];
  double  perr  [3];
  double  ierr  [3];
  double  derr  [3];
  double  cmd   [4];
  double  heading;
} ctrl_struct;
ctrl_struct ctrl;


// CTRL functions
void    ctl_init    ( void );
void    ctl_exit    ( void );
void    ctl_exec    ( void );
void    ctl_quad    ( void );
void    ctl_plane   ( void );
void    ctl_debug   ( void );

#endif



