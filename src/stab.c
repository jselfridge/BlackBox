

#include "stab.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "imu.h"
#include "io.h"
#include "sys.h"
#include "timer.h"


static void    stab_quad    ( void );
static double  stab_sf      ( sf_struct *sf, double r, double zp, double zd, bool areset );
static void    stab_id      ( id_struct *id, double u, double y );
static void    stab_disarm  ( void );


/**
 *  stab_init
 *  Initializes the stabilization control loop.
 */
void stab_init ( void )  {
  if (DEBUG)  printf("Initializing stabilization \n");

  // Local variables
  int i;
  FILE *f;
  char buff [32];  memset( buff, 0, sizeof(buff) );
  char path [32];  memset( path, 0, sizeof(path) );

  // Enable mutex locks
  pthread_mutex_init( &stab.mutex, NULL );
  pthread_mutex_init( &sfx.mutex,  NULL );
  pthread_mutex_init( &sfy.mutex,  NULL );
  pthread_mutex_init( &sfz.mutex,  NULL );
  //pthread_mutex_init( &idx.mutex,  NULL );
  //pthread_mutex_init( &idy.mutex,  NULL );
  //pthread_mutex_init( &idz.mutex,  NULL );

  // Set timing value
  double dt = 1.0 / HZ_STAB;;
  stab.dt = dt;
  //sfx.dt  = dt;
  //sfy.dt  = dt;
  //sfz.dt  = dt;

  // Asign disarming array values
  sprintf( path, "./param/%s/off", systype );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (stab_init): File for '%s off' not found. \n", systype );
  for ( i=0; i<10; i++ )  {
    fgets( buff, 32, f );
    stab.off[i] = atof(buff);
  }
  fclose(f);

  // Display disarm values
  if (DEBUG)  {
    printf("  Disarm values:    ");
    for ( i=0; i<10; i++ ) printf("%4.1f  ", stab.off[i] );
    printf("\n");
  }

  // Reference ranges
  sprintf( path, "./param/%s/range", systype );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (stab_init): File for '%s range' not found. \n", systype );
  for ( i=0; i<4; i++ )  {
    fgets( buff, 32, f );
    stab.range[i] = atof(buff);
  }
  fclose(f);

  // Display reference range values
  if (DEBUG)  {
    printf("  Range values:     ");
    for ( i=0; i<4; i++ ) printf("%4.1f  ", stab.range[i] );
    printf("\n");
  }

  // Throttle values
  sprintf( path, "./param/%s/thrl", systype );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (stab_init): File for '%s thrl' not found. \n", systype );
  for ( i=0; i<3; i++ )  {
    fgets( buff, 32, f );
    stab.thrl[i] = atof(buff);
  }
  fclose(f);

  // Display throttle values
  if (DEBUG)  {
    printf("  Throttle values:  ");
    for ( i=0; i<3; i++ ) printf("%4.1f  ", stab.thrl[i] );
    printf("\n");
  }

  // Wrap values of pi
  sfx.wrap = true;
  sfy.wrap = true;
  sfz.wrap = true;

  // Initalize state feedback data struct values
  sfx.r = 0.0;  sfx.zp = 0.0;  sfx.zd = 0.0;
  sfy.r = 0.0;  sfy.zp = 0.0;  sfy.zd = 0.0;
  sfz.r = 0.0;  sfz.zp = 0.0;  sfz.zd = 0.0;

  // Assign fixed PD gains
  sfx.kp = 0.140;  sfx.kd = 0.050;
  sfy.kp = 0.140;  sfy.kd = 0.050;
  sfz.kp = 0.080;  sfz.kd = 0.000;

  /*/ Assign desired characteristics
  sfx.ts = 1.70;  sfx.mp =  5.0;  sfx.j = 220.0;  stab_refmdl( &sfx );
  sfy.ts = 1.70;  sfy.mp =  5.0;  sfy.j = 220.0;  stab_refmdl( &sfy );
  sfz.ts = 1.70;  sfz.mp =  5.0;  sfz.j = 220.0;  stab_refmdl( &sfz );
  if (DEBUG)  {
    printf("  Desired system response \n");
    printf("          Ts    Mp    zeta  nfreq    sigma  dfreq          ap      ad        kp      kd  \n" );
    printf("  X:    %4.2f  %4.1f    %4.2f  %5.2f    %5.2f  %5.2f    %8.3f  %6.3f    %6.4f  %6.4f  \n", 
      sfX.ts, sfX.mp, sfX.zeta, sfX.nfreq, sfX.sigma, sfX.dfreq, sfX.ap, sfX.ad, sfX.kp, sfX.kd );
    printf("  Y:    %4.2f  %4.1f    %4.2f  %5.2f    %5.2f  %5.2f    %8.3f  %6.3f    %6.4f  %6.4f  \n", 
      sfY.ts, sfY.mp, sfY.zeta, sfY.nfreq, sfY.sigma, sfY.dfreq, sfY.ap, sfY.ad, sfY.kp, sfY.kd );
    printf("  Z:    %4.2f  %4.1f    %4.2f  %5.2f    %5.2f  %5.2f    %8.3f  %6.3f    %6.4f  %6.4f  \n", 
      sfZ.ts, sfZ.mp, sfZ.zeta, sfZ.nfreq, sfZ.sigma, sfZ.dfreq, sfZ.ap, sfZ.ad, sfZ.kp, sfZ.kd );
    fflush(stdout);
  }
  */
  /*/ Assign adaptive gains
  sfx.Gp = 0.0;  sfx.Gd = 0.0;  sfx.Gu = 0.0;
  sfy.Gp = 0.0;  sfy.Gd = 0.0;  sfy.Gu = 0.0;
  sfz.Gp = 0.0;  sfz.Gd = 0.0;  sfz.Gu = 0.0;
  if (DEBUG)  {
    printf("  Adaptive gain settings \n");
    printf("       Gp   Gd   Gu  \n");
    printf("  X:  %3.1f  %3.1f  %3.1f  \n", sfx.Gp, sfx.Gd, sfx.Gu );
    printf("  Y:  %3.1f  %3.1f  %3.1f  \n", sfy.Gp, sfy.Gd, sfy.Gu );
    printf("  Z:  %3.1f  %3.1f  %3.1f  \n", sfz.Gp, sfz.Gd, sfz.Gu );
    fflush(stdout);
  }
  */

  // Initialize sysid param values
  //idx.z1 = 0.001;  idx.z2 = 0.0;  idx.p1 = -2.0;  idx.p2 = 1.0;
  //idy.z1 = 0.001;  idy.z2 = 0.0;  idy.p1 = -2.0;  idy.p2 = 1.0;
  //idz.z1 = 0.001;  idz.z2 = 0.0;  idz.p1 = -2.0;  idz.p2 = 1.0;

  // Initialize sysid signal values
  //idx.u1 = 0.0;  idx.u2 = 0.0;  idx.y1 = 0.0;  idx.y2 = 0.0;
  //idy.u1 = 0.0;  idy.u2 = 0.0;  idy.y1 = 0.0;  idy.y2 = 0.0;
  //idz.u1 = 0.0;  idz.u2 = 0.0;  idz.y1 = 0.0;  idz.y2 = 0.0;

  return;
}


/**
 *  stab_exit
 *  Exits the stabilization controller code.
 */
void stab_exit ( void )  {
  if (DEBUG)  printf("Close stabilization \n");
  pthread_mutex_destroy(&stab.mutex);
  pthread_mutex_destroy(&sfx.mutex);
  pthread_mutex_destroy(&sfy.mutex);
  pthread_mutex_destroy(&sfz.mutex);
  //pthread_mutex_destroy(&idx.mutex);
  //pthread_mutex_destroy(&idy.mutex);
  //pthread_mutex_destroy(&idz.mutex);
  return;
}


/**
 *  stab_refmdl
 *  Take desired system char and determine reference model
 */
/*
void stab_refmdl ( sf_struct *sf )  {

  // Local variables
  double ts, mp, ln, j;
  double sigma, zeta;
  double nfreq, dfreq;
  double ap, ad;
  double kp, kd, ku;

  // Get desired system characteristics
  pthread_mutex_lock(&sf->mutex);
  ts = sf->ts;
  mp = sf->mp / 100.0;
  j  = sf->j;
  pthread_mutex_unlock(&sf->mutex);

  // Calculate parameters
  sigma = 4.6 / ts;
  ln = log(mp) * log(mp);
  zeta = sqrt( ln / ( M_PI * M_PI + ln ) );
  nfreq = sigma / zeta;
  dfreq = nfreq * sqrt( 1 - zeta * zeta );
  ap = nfreq * nfreq;
  ad = 2.0 * zeta * nfreq;
  kp = ap / j;
  kd = ad / j;
  ku = 1.0;  //--- WIP ---//

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
  sf->ku    = ku;
  pthread_mutex_unlock(&sf->mutex);

  return;
}
*/

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
  bool reset;

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
  reset = ( in[CH_T] < -0.2 );
  cmd[x] = stab_sf( &sfx, ref[x], att[x], ang[x], reset );
  cmd[y] = stab_sf( &sfy, ref[y], att[y], ang[y], reset );
  cmd[z] = stab_sf( &sfz, ref[z], ang[z],      0, reset );

  // Perform system identification
  //if (!reset)  {
    //stab_id( &idx, cmd[x], att[x] );
    //stab_id( &idy, cmd[y], att[y] );
    //stab_id( &idz, cmd[z], att[z] );
  //}

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
double stab_sf ( sf_struct *sf, double r, double zp, double zd, bool areset )  {

  // Local variables
  bool   wrap;
  double kp, kd;
  double u;
  double diff;
  //double dt;
  //double ap, ad, j;
  //double ku;
  //double xp, xd, xa;
  //double Gp, Gd, Gu;
  //double p_tilde, d_tilde;
  //double kp_dot, kd_dot, ku_dot;

  // Pull data from structure
  pthread_mutex_lock(&sf->mutex);
  wrap = sf->wrap;
  kp   = sf->kp;
  kd   = sf->kd;
  //dt = sf->dt;
  //ap = sf->ap;
  //ad = sf->ad;
  //j  = sf->j;
  //ku = sf->ku;
  //xp = sf->xp;
  //xd = sf->xd;
  //Gp = sf->Gp;
  //Gd = sf->Gd;
  //Gu = sf->Gu;
  pthread_mutex_unlock(&sf->mutex);

  // Determine control input
  diff = r - zp;
  if (wrap)  {
    if ( diff >   PI )  diff -= 2.0 * PI;
    if ( diff <= -PI )  diff += 2.0 * PI;
  }
  u = diff * kp - zd * kd;

  /*/ Determine ref model states
  diff = r - xp;
  if (wrap)  {
    if ( diff >   PI )  diff -= 2.0 * PI;
    if ( diff <= -PI )  diff += 2.0 * PI;
  }
  xa  = diff * ap - xd * ad;
  xd += dt * xa;
  xp += dt * xd + 0.5 * dt * dt * xa;
  */

  // Reset adaptive gains if needed
  //if (areset)  {  Gp = 0.0;  Gd = 0.0;  Gu = 0.0;  }

  /*/ Adaptive update laws
  p_tilde = xp - zp;
  d_tilde = xd - zd;
  if (wrap)  {
    if ( p_tilde >   PI )  p_tilde -= 2.0 * PI;
    if ( p_tilde <= -PI )  p_tilde += 2.0 * PI;
  }
  kp_dot = - Gp * j * ( p_tilde + 2 * d_tilde ) * zp;
  kd_dot = - Gd * j * ( p_tilde + 2 * d_tilde ) * zd;
  ku_dot = - Gu * j * ( p_tilde + 2 * d_tilde ) * u;
  */

  /*/ Projection operator  //--- WIP ---//
  double kp_dot_max = 0.0001;
  double kd_dot_max = 0.0001;
  double ku_dot_max = 0.0;
  if ( kp_dot > kp_dot_max )  kp_dot = kp_dot_max;  if ( kp_dot < -kp_dot_max )  kp_dot = -kp_dot_max;
  if ( kd_dot > kd_dot_max )  kd_dot = kd_dot_max;  if ( kd_dot < -kd_dot_max )  kd_dot = -kd_dot_max;
  if ( ku_dot > ku_dot_max )  ku_dot = ku_dot_max;  if ( ku_dot < -ku_dot_max )  ku_dot = -ku_dot_max;
  */

  /*/ Update gain values
  kp += dt * kp_dot;
  kd += dt * kd_dot;
  ku += dt * ku_dot;
  */

  /*/ Projection operator
  double kp_max = 0.120;
  double kd_max = 0.080;
  double ku_max = 1.0;
  if ( kp > kp_max )  kp = kp_max;
  if ( kd > kd_max )  kd = kd_max;
  if ( ku > ku_max )  ku = ku_max;
  */

  // Push data to structure
  pthread_mutex_lock(&sf->mutex);
  sf->r  = r;
  sf->zp = zp;
  sf->zd = zd;
  sf->u  = u;
  //sf->kp = kp;
  //sf->kd = kd;
  //sf->ku = ku;
  pthread_mutex_unlock(&sf->mutex);

  return u;
}


/**
 *  stab_id
 *  Perform system identification update.
 */
void stab_id ( id_struct *id, double u, double y )  {

  // Local variables
  double z1, z2, p1, p2;
  double u1, u2, y1, y2;
  double eps, m, n;

  // Pull data from structure
  pthread_mutex_lock(&id->mutex);
  z1 = id->z1;
  z2 = id->z2;
  p1 = id->p1;
  p2 = id->p2;
  u1 = id->u1;
  u2 = id->u2;
  y1 = id->y1;
  y2 = id->y2;
  pthread_mutex_unlock(&id->mutex);

  // Calculate auxilliary signals
  eps = z2 * u2 + z1 * u1 - p2 * y2 - p1 * y1 - y;
  m = 1 + u2 * u2 + u1 * u1 + y2 * y2 + y1 * y1;
  n = eps/m;

  // Update parameter estimates
  z1 -= n * u1;
  z2 -= n * u2;
  p1 -= n * y1;
  p2 -= n * y2;

  // Shift data back a time step
  u2 = u1;  u1 = u;
  y2 = y1;  y1 = y;

  // Push data to structure
  pthread_mutex_lock(&id->mutex);
  id->z1 = z1;
  id->z2 = z2;
  id->p1 = p1;
  id->p2 = p2;
  id->u1 = u1;
  id->u2 = u2;
  id->y1 = y1;
  id->y2 = y2;
  pthread_mutex_unlock(&id->mutex);

  return;
}


/**
 *  stab_disarm
 *  Disarm the motors and reset control surfaces
 */
void stab_disarm ( void )  {
  ushort ch;
  for ( ch=0; ch<OUT_CH; ch++ )  io_setnorm( ch, stab.off[ch] );
  return;
}



