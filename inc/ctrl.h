

#ifndef CTRL_H
#define CTRL_H


#define IRESET    0.25
#define SYSTEM    "quad"


/*

#define QUAD_OFF0  -1.0
#define QUAD_OFF1  -1.0
#define QUAD_OFF2   0.0
#define QUAD_OFF3   0.0
#define QUAD_OFF4  -1.0
#define QUAD_OFF5  -1.0
#define QUAD_OFF6   0.0
#define QUAD_OFF7   0.0
#define QUAD_OFF8   0.0
#define QUAD_OFF9   0.0

#define QUAD_FL  0
#define QUAD_BL  1
#define QUAD_BR  4
#define QUAD_FR  5

#define QUAD_X_RANGE   0.50
#define QUAD_Y_RANGE   0.50
#define QUAD_Z_RANGE   1.50
#define QUAD_T_RANGE   0.50

#define QUAD_TMIN   0.00
#define QUAD_TMAX   0.30
#define QUAD_TILT   0.00

#define QUAD_PX    0.33
#define QUAD_PY    0.33
#define QUAD_PZ    0.00
#define QUAD_IX    0.00
#define QUAD_IY    0.00
#define QUAD_IZ    0.00
#define QUAD_DX    0.10
#define QUAD_DY    0.10
#define QUAD_DZ    0.00

*/

/*

// Define 'plane' disarm values (same for any plane)
#define PLANE_OFF0   0.0
#define PLANE_OFF1   0.0
#define PLANE_OFF2   0.0
#define PLANE_OFF3   0.0
#define PLANE_OFF4  -1.0
#define PLANE_OFF5   0.0
#define PLANE_OFF6   0.0
#define PLANE_OFF7   0.0
#define PLANE_OFF8   0.0
#define PLANE_OFF9   0.0

// Define 'plane' channel index
#define PLANE_ELEV  0
#define PLANE_RUDD  1
#define PLANE_THRL  4

// Define 'plane' range values
// R: rad/s    P: rad/s    Y: rad/s    T: ?thrust?
//#define PLANE_R_RANGE   1.00
//#define PLANE_P_RANGE   1.00
//#define PLANE_Y_RANGE   1.00
#define PLANE_T_RANGE   1.00

// Define 'plane' throttle gains
#define PLANE_TMIN   0.00
#define PLANE_TMAX   0.00

// Plane PD Gains
// P: ????    I: ????    D: ????
//#define PLANE_PX    0.00
//#define PLANE_PY    0.00
//#define PLANE_PZ    0.00
//#define PLANE_DX    0.00
//#define PLANE_DY    0.00
//#define PLANE_DZ    0.00

*/


typedef struct ctrl_struct {
  double  dt;
  double  off   [10];
  double  scale [4];
  double  thrl  [3];
  double  pgain [3];
  double  igain [3];
  double  dgain [3];
  double  perr  [3];
  double  ierr  [3];
  double  derr  [3];
  double  cmd   [4];
  double  bank;
  double  climb;
  double  heading;
} ctrl_struct;
ctrl_struct ctrl;


void    ctrl_init    ( void );
void    ctrl_exit    ( void );

//void    ctrl_update  ( void );
//void    ctrl_quad    ( void );
//void    ctrl_plane   ( void );
//void    ctrl_disarm  ( void );


#endif



