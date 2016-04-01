

#ifndef _CTRL_H_
#define _CTRL_H_
#include <main.h>


// Define misc items
#define IRESET    0.25
#define SYSTEM    "quad"


// Define 'quad' disarm values
#define QUAD_OFF0  -1.0
#define QUAD_OFF1  -1.0
#define QUAD_OFF2  -1.0
#define QUAD_OFF3  -1.0
#define QUAD_OFF4   0.0
#define QUAD_OFF5   0.0
#define QUAD_OFF6   0.0
#define QUAD_OFF7   0.0
#define QUAD_OFF8   0.0
#define QUAD_OFF9   0.0


// Define 'quad' motor channel index (smae for any quad)
#define QUAD_FR  0
#define QUAD_BL  1
#define QUAD_FL  2
#define QUAD_BR  3


// Define 'quad' range values (vehicle specific values)
// X: rad    Y: rad    Z: rad/s    T: ?thrust?
#define QUAD_X_RANGE   0.50
#define QUAD_Y_RANGE   0.50
#define QUAD_Z_RANGE   1.50
#define QUAD_T_RANGE   0.50


// Define 'quad' throttle gains (vehicle specific values)
// MIN: 0.00    MAX: 0.30    TILT: 1.00
#define QUAD_TMIN   0.00
#define QUAD_TMAX   0.30
#define QUAD_TILT   0.00


// Define 'quad' PID Gains (vehicle specific values)
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


/*// Define 'plane' disarm values (same for any plane)
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
*/

/*// Define 'plane' channel index
#define PLANE_ELEV  0
#define PLANE_RUDD  1
#define PLANE_THRL  4
*/

/*// Define 'plane' range values
// R: rad/s    P: rad/s    Y: rad/s    T: ?thrust?
//#define PLANE_R_RANGE   1.00
//#define PLANE_P_RANGE   1.00
//#define PLANE_Y_RANGE   1.00
#define PLANE_T_RANGE   1.00
*/

/*// Define 'plane' throttle gains
#define PLANE_TMIN   0.00
#define PLANE_TMAX   0.00
*/

// Plane PD Gains
// P: ????    I: ????    D: ????
//#define PLANE_PX    0.00
//#define PLANE_PY    0.00
//#define PLANE_PZ    0.00
//#define PLANE_DX    0.00
//#define PLANE_DY    0.00
//#define PLANE_DZ    0.00


// CTRL structure
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


// CTRL functions
void    ctrl_init    ( void );
void    ctrl_exit    ( void );
void    ctrl_update  ( void );
void    ctrl_quad    ( void );
//void    ctrl_plane   ( void );
void    ctrl_disarm  ( void );


#endif



