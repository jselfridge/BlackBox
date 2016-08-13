

#include "stab.h"
#include <math.h>
#include <stdio.h>
#include "imu.h"
#include "io.h"
#include "sys.h"
#include "timer.h"


static void    stab_quad    ( void );
static double  stab_sf      ( sf_struct *sf, double r, double zp, double zd );
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

  // Initalize data struct values
  sfX.r = 0.0;  sfX.xp = 0.0;  sfX.xd = 0.0;
  sfY.r = 0.0;  sfY.xp = 0.0;  sfY.xd = 0.0;
  sfZ.r = 0.0;  sfZ.xp = 0.0;  sfZ.xd = 0.0;

  // Wrap values of pi
  sfX.wrap = true;
  sfY.wrap = true;
  sfZ.wrap = true;

  // Assign desired characteristics
  sfX.ts = 1.60;  sfX.mp = 0.001;  sfX.b = 105.0;  stab_refmdl( &sfX );
  sfY.ts = 1.60;  sfY.mp = 0.001;  sfY.b = 105.0;  stab_refmdl( &sfY );
  sfZ.ts = 1.60;  sfZ.mp = 0.001;  sfZ.b = 105.0;  stab_refmdl( &sfZ );
  if (DEBUG)  {
    printf("          Ts    Mp    zeta  nfreq    sigma  dfreq          ap      ad        kp      kd  \n" );
    printf("  X:    %4.2f  %4.1f    %4.2f  %5.2f    %5.2f  %5.2f    %8.3f  %6.3f    %6.4f  %6.4f  \n", 
      sfX.ts, sfX.mp, sfX.zeta, sfX.nfreq, sfX.sigma, sfX.dfreq, sfX.ap, sfX.ad, sfX.kp, sfX.kd );
    printf("  Y:    %4.2f  %4.1f    %4.2f  %5.2f    %5.2f  %5.2f    %8.3f  %6.3f    %6.4f  %6.4f  \n", 
      sfY.ts, sfY.mp, sfY.zeta, sfY.nfreq, sfY.sigma, sfY.dfreq, sfY.ap, sfY.ad, sfY.kp, sfY.kd );
    printf("  Z:    %4.2f  %4.1f    %4.2f  %5.2f    %5.2f  %5.2f    %8.3f  %6.3f    %6.4f  %6.4f  \n", 
      sfZ.ts, sfZ.mp, sfZ.zeta, sfZ.nfreq, sfZ.sigma, sfZ.dfreq, sfZ.ap, sfZ.ad, sfZ.kp, sfZ.kd );
    fflush(stdout);
  }

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
 *  stab_refmdl
 *  Take desired system char and determine reference model
 */
void stab_refmdl ( sf_struct *sf )  {

  // Local variables
  double ts, mp, ln, b;
  double sigma, zeta;
  double nfreq, dfreq;
  double ap, ad, kp, kd;

  // Get desired system characteristics
  pthread_mutex_lock(&sf->mutex);
  ts = sf->ts;
  mp = sf->mp / 100.0;
  b  = sf->b;
  pthread_mutex_unlock(&sf->mutex);

  // Calculate parameters
  sigma = 4.6 / ts;
  ln = log(mp) * log(mp);
  zeta = sqrt( ln / ( M_PI * M_PI + ln ) );
  nfreq = sigma / zeta;
  dfreq = nfreq * sqrt( 1 - zeta * zeta );
  ap = nfreq * nfreq;
  ad = 2.0 * zeta * nfreq;
  kp = ap / b;
  kd = ad / b;

  // Assign reference model parameters
  pthread_mutex_lock(&sf->mutex);
  sf->sigma = sigma;
  sf->zeta  = zeta;
  sf->nfreq = nfreq;
  sf->dfreq = dfreq;
  sf->ap    = ap;
  sf->ad    = ad;
  sf->kp    = kp;
  sf->kd    = kd;
  pthread_mutex_unlock(&sf->mutex);

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

  // Apply state feedback function
  cmd[x] = stab_sf( &sfX, ref[x], att[x],  ang[x] );
  cmd[y] = stab_sf( &sfY, ref[y], att[y],  ang[y] );
  cmd[z] = stab_sf( &sfZ, ref[z], heading, ang[z] );

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

  // Push stabilization data
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
 *  stab_sf
 *  Apply state feedback control loop
 */
double stab_sf ( sf_struct *sf, double r, double zp, double zd )  {

  // Local variables
  double dt, ap, ad, kp, kd, xp, xd, xa, u;

  // Pull data from structure
  pthread_mutex_lock(&sf->mutex);
  dt = sf->dt;
  ap = sf->ap;
  ad = sf->ad;
  kp = sf->kp;
  kd = sf->kd;
  xp = sf->xp;
  xd = sf->xd;
  pthread_mutex_unlock(&sf->mutex);

  // Determine referenace states
  xa  = r * ap - xp * ap - xd * ad;
  xd += dt * xa;
  xp += dt * xd + 0.5 * dt * dt * xa;

  // Determine control input
  u = - kp * zp - kd * zd + kp * r;

  // Push data to structure
  pthread_mutex_lock(&sf->mutex);
  sf->r  = r;
  sf->xp = xp;
  sf->xd = xd;
  sf->u  = u;
  pthread_mutex_unlock(&sf->mutex);

  return u;
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



