

#include "stab.h"
#include <stdio.h>
#include "imu.h"
#include "io.h"
#include "sys.h"
#include "timer.h"


static void    stab_quad    ( void );
//static double  stab_pid     ( pid_struct *pid, double xp, double xd, double ref, bool reset );
//static double  stab_adapt   ( adapt_struct *adapt, double p, double d, double r, bool areset );
static void    stab_disarm  ( void );


/**
 *  stab_init
 *  Initializes the stabilization control loop.
 */
void stab_init ( void )  {
  if (DEBUG)  printf("Initializing stabilization \n");

  // Set timing value
  double dt = 1.0 / HZ_STAB;;
  stab.dt = dt;
  sfX.dt  = dt;
  sfY.dt  = dt;
  sfZ.dt  = dt;

  // Asign disarming array values
  stab.off[0] = -1.0;
  stab.off[1] = -1.0;
  stab.off[2] = -1.0;
  stab.off[3] = -1.0;
  stab.off[4] = -1.0;
  stab.off[5] = -1.0;
  stab.off[6] = -1.0;
  stab.off[7] = -1.0;
  stab.off[8] = -1.0;
  stab.off[9] = -1.0;

  // Reference ranges (TODO: Move into radio or transmitter function)
  stab.range[CH_R] = 0.6;
  stab.range[CH_P] = 0.6;
  stab.range[CH_Y] = 2.0;
  stab.range[CH_T] = 0.3;

  // Throttle values (TODO: Move into radio or transmitter function)
  stab.thrl[0] = -0.30;  // Tmin
  stab.thrl[1] = -0.15;  // Tmax
  stab.thrl[2] =  0.00;  // Ttilt

  // Wrap values of pi
  sfX.wrap = true;
  sfY.wrap = true;
  sfZ.wrap = true;

  /*  -- ORIGINAL --
  // Roll (X) gain values
  pidX.pgain = 0.085;
  pidX.igain = 0.000;
  pidX.dgain = 0.055;
  adaptX.Gp  = 0.0;
  adaptX.Gd  = 0.0;
  //adaptX.Gr  = 0.0;
  adaptX.G   = 0.0;
  adaptX.kp  = pidX.pgain;
  adaptX.kd  = pidX.dgain;
  //adaptX.kr  =  pidX.pgain;
  adaptX.k   = 0.0;
  adaptX.kp_prev = adaptX.kp;
  adaptX.kd_prev = adaptX.kd;
  //adaptX.kr_prev = adaptX.kr;

  // Pitch (Y) gain values
  pidY.pgain = 0.085;
  pidY.igain = 0.000;
  pidY.dgain = 0.055;

  // Yaw (Z) gain values
  pidZ.pgain = 0.085;
  pidZ.igain = 0.000;
  pidZ.dgain = 0.055;
  */

  return;
}


/**
 *  stab_exit
 *  Exits the stabilization controller code.
 */
void stab_exit ( void )  {
  if (DEBUG)  printf("Close stabilization \n");
  // Add exit code as needed...
  return;
}


/**
 *  stab_update
 *  Executes the stabilization update loop.
 */
void stab_update ( void )  {

  if (armed)  stab_quad();
  else        stab_disarm();

  return;
}


/**
 *  stab_quad
 *  Apply stabilization control to quadrotor system.
 */
void stab_quad ( void )  {
 
  // Local variables
  ushort i;
  ushort x=0, y=1, z=2, t=3;
  double att[3], ang[3], in[4], ref[4], cmd[4], out[4];
  double dt, trange, tmin, tmax, tilt, heading, dial;
  //bool reset;

  // Obtain states
  pthread_mutex_lock(&rot.mutex);
  for ( i=0; i<3; i++ )  {  att[i] = rot.att[i];  ang[i] = rot.ang[i];  }
  pthread_mutex_unlock(&rot.mutex);

  // Obtain inputs
  pthread_mutex_lock(&input.mutex);
  for ( i=0; i<4; i++ )  in[i] = input.norm[i];
  dial = input.norm[CH_D];
  pthread_mutex_unlock(&input.mutex);

  // Obtain stabilization parameters
  pthread_mutex_lock(&stab.mutex);
  dt      = stab.dt;
  tmin    = stab.thrl[0];
  tmax    = stab.thrl[1];
  tilt    = stab.thrl[2];
  trange  = stab.range[3];
  heading = stab.heading;
  pthread_mutex_unlock(&stab.mutex);

  // Calculate reference signals
  for ( i=0; i<4; i++ )  ref[i] = in[i] * stab.range[i];

  // Determine desired heading
  if ( in[CH_T] > -0.9 && fabs(in[CH_Y]) > 0.1 )  heading += ref[CH_Y] * dt;
  while ( heading >   M_PI )  heading -= 2.0 * M_PI;
  while ( heading <= -M_PI )  heading += 2.0 * M_PI;




  cmd[x] = 0.0;
  cmd[y] = 0.0;
  cmd[z] = 0.0;


  // -- ORIGINAL -- //
  // Calculate Roll command
  //reset = ( in[CH_R] < -IRESET || in[CH_R] > IRESET || in[CH_T] < -0.9 );
  //cmd[x] = stab_pid( &pidX, att[0], ang[0], ref[CH_R], reset );
  //reset = ( in[CH_T] < -0.2 );
  //cmd[x] = stab_adapt( &adaptX, att[0], ang[0], ref[CH_R], reset );
  // Calculate Pitch command
  //reset = ( in[CH_P] < -IRESET || in[CH_P] > IRESET || in[CH_T] < -0.9 );
  //cmd[y] = stab_pid( &pidY, att[1], ang[1], ref[CH_P], reset );
  // Calculate Yaw command
  //reset = ( in[CH_Y] < -IRESET || in[CH_Y] > IRESET || in[CH_T] < -0.9 );
  //cmd[z] = stab_pid( &pidZ, att[2], ang[2], heading, reset );

  // Determine throttle adjustment
  double tilt_adj = ( 1 - ( cos(att[0]) * cos(att[1]) ) ) * tilt;
  double thresh = ( 0.5 * ( dial + 1.0 ) * ( tmax - tmin ) ) + tmin - trange;
  if ( in[CH_T] <= -0.6 )  cmd[t] = ( 2.50 * ( in[CH_T] + 1.0 ) * ( thresh + 1.0 ) ) - 1.0 + tilt_adj;
  else                     cmd[t] = ( 1.25 * ( in[CH_T] + 0.6 ) * trange ) + thresh + tilt_adj; 

  // Assign motor outputs
  if ( in[CH_T] > -0.9 ) {
    out[QUAD_FL] = cmd[t] + cmd[x] + cmd[y] - cmd[z];
    out[QUAD_BL] = cmd[t] + cmd[x] - cmd[y] + cmd[z];
    out[QUAD_BR] = cmd[t] - cmd[x] - cmd[y] - cmd[z];
    out[QUAD_FR] = cmd[t] - cmd[x] + cmd[y] + cmd[z];
  } else {
    out[QUAD_FL] = -1.0;
    out[QUAD_BL] = -1.0;
    out[QUAD_BR] = -1.0;
    out[QUAD_FR] = -1.0;
  }

  // Push control data
  pthread_mutex_lock(&stab.mutex);
  for ( i=0; i<4; i++ )  stab.cmd[i] = cmd[i];
  stab.heading = heading;
  pthread_mutex_unlock(&stab.mutex);

  // Push system outputs
  io_setnorm( QUAD_FL, out[QUAD_FL] );
  io_setnorm( QUAD_BL, out[QUAD_BL] );
  io_setnorm( QUAD_BR, out[QUAD_BR] );
  io_setnorm( QUAD_FR, out[QUAD_FR] );

  return;
}


/**
 *  stab_pid
 *  Apply PID contorl loop
 */
/*
double stab_pid ( pid_struct *pid, double xp, double xd, double ref, bool reset )  {

  // Local variables
  double dt, cmd;
  double perr, ierr, derr;
  double pgain, igain, dgain;
  bool wrap;

  // Pull structure data
  pthread_mutex_lock(&pid->mutex);
  dt    = pid->dt;
  wrap  = pid->wrap;
  ierr  = pid->ierr;
  pgain = pid->pgain;
  igain = pid->igain;
  dgain = pid->dgain;
  pthread_mutex_unlock(&pid->mutex);

  // Calculate error values
  perr = -xp + ref;
  if (wrap)  {
    while ( perr >   M_PI )  perr -= 2.0 * M_PI;
    while ( perr <= -M_PI )  perr += 2.0 * M_PI;
  }
  derr = -xd;
  if (reset)  ierr = 0.0; 
  else        ierr += perr * dt;

  // Calculate output command
  cmd = perr * pgain + 
        ierr * igain + 
        derr * dgain;

  // Push to data structure
  pthread_mutex_lock(&pid->mutex);
  pid->perr = perr;
  pid->ierr = ierr;
  pid->derr = derr;
  pthread_mutex_unlock(&pid->mutex);

  return cmd;
}
*/

/**
 *  stab_adapt
 *  Apply adaptive contorl loop
 */
/*
double stab_adapt ( adapt_struct *adapt, double p, double d, double r, bool areset )  {

  // Local variables
  double Gp, Gd, G;
  double kp, kd, k;
  double p_prev,  d_prev, r_prev;
  double kp_prev, kd_prev;

  // Lock struct while pulling data
  pthread_mutex_lock(&adapt->mutex);

  // Get adaptive law gains
  Gp = adapt->Gp;
  Gd = adapt->Gd;
  //Gr = adapt->Gr;
  G  = adapt->G;

  // Get current state feedback gains
  kp = adapt->kp;
  kd = adapt->kd;
  //kr = adapt->kr;
  k  = adapt->k;

  // Get previous states
  p_prev = adapt->p;
  d_prev = adapt->d;
  r_prev = adapt->r;

  // Get previous state feedback gains
  kp_prev = adapt->kp_prev;
  kd_prev = adapt->kd_prev;
  //kr_prev = adapt->kr_prev;

  // Assign previous state gains (before they are updated)
  adapt->kp_prev = kp;
  adapt->kd_prev = kd;
  //adapt->kr_prev = kr;

  // Unlock data structure
  pthread_mutex_unlock(&adapt->mutex);

  // Cancel adaptation
  if ( areset )  {
    Gp = 0.0;
    Gd = 0.0;
    //Gr = 0.0;
    G  = 0.0;
  }

  // Scale down adaptive gains
  else {
    Gp /= 1000.0;
    Gd /= 1000.0;
    G  /= 1000.0;
  }

  // Control input 
  double u = - kp * p - kd * d + kp * r;

  // Auxilliary signals
  double xi =  - ( kp - kp_prev ) * p_prev  - ( kd - kd_prev ) * d_prev  + ( kp - kp_prev ) * r_prev;
  double eps = ( p - r_prev ) + ( k * xi );

  // Normalizing signal
  double m = 1 + p_prev * p_prev + d_prev * d_prev + r_prev * r_prev + xi * xi;
  double n = eps / m;

  // Adaptive laws
  kp += Gp * p_prev * n;
  kd += Gd * d_prev * n;
  //kr -= Gr * r_prev * n;
  k  -= G  * xi     * n;

  // Lock struct while pushing data
  pthread_mutex_lock(&adapt->mutex);

  // Update states
  adapt->p = p;
  adapt->d = d;
  adapt->r = r;
  adapt->u = u;

  // Update current gains
  adapt->kp = kp;
  adapt->kd = kd;
  //adapt->kr = kr;
  adapt->k  = k;

  // Unlock data structure
  pthread_mutex_unlock(&adapt->mutex);

  return u;
}
*/

/**
 *  stab_disarm
 *  Disarm the motors and reset control surfaces
 */
void stab_disarm ( void )  {
  ushort ch;
  for ( ch=0; ch<OUT_CH; ch++ )  io_setnorm( ch, stab.off[ch] );
  return;
}



