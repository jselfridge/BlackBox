

#include "stab.h"
#include <stdio.h>
#include "ahrs.h"
#include "imu.h"
#include "io.h"
#include "sys.h"
#include "timer.h"


static void    stab_quad    ( void );
static double  stab_pid     ( pid_struct *pid, double xp, double xd, double ref, bool reset );
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
  pidX.dt = dt;
  pidY.dt = dt;
  pidZ.dt = dt;

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
  stab.range[CH_R] = 0.0;
  stab.range[CH_P] = 0.0;
  stab.range[CH_Y] = 0.0;
  stab.range[CH_T] = 0.0;

  // Throttle values (TODO: Move into radio or transmitter function)
  stab.thrl[0] =  0.0;  // Tmin
  stab.thrl[1] =  0.0;  // Tmax
  stab.thrl[2] =  0.0;  // Ttilt

  // Wrap values of pi
  pidX.wrap = true;
  pidY.wrap = true;
  pidZ.wrap = true;

  // P gain values
  pidX.pgain = 0.00;
  pidY.pgain = 0.00;
  pidZ.pgain = 0.00;

  // I gain values
  pidX.igain = 0.00;
  pidY.igain = 0.00;
  pidZ.igain = 0.00;

  // D gain values
  pidX.dgain = 0.00;
  pidY.dgain = 0.00;
  pidZ.dgain = 0.00;

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
  //bool reset;
  ushort i;
  ushort x=0, y=1, z=2, t=3;
  double state[6], in[4], ref[4], cmd[4], out[4];
  double trange, tmin, tmax, tilt, heading, dial;

  // Zero out states
  for ( i=0; i<6; i++ )  state[i] = 0.0;

  // IMUA states
  if (IMUA_ENABLED)  {

    // Comp filter values (no yaw value)
    pthread_mutex_lock(&imuA.mutex);
    state[0] += imuA.roll;
    state[1] += imuA.pitch;
    pthread_mutex_unlock(&imuA.mutex);

    // Loop through states
    for ( i=0; i<3; i++ )  {

      // LPF gyro values
      pthread_mutex_lock(&gyrA.mutex);
      state[i+3] += gyrA.filter[i];
      pthread_mutex_unlock(&gyrA.mutex);

      // AHRS values
      pthread_mutex_lock(&ahrsA.mutex);
      state[i]   += ahrsA.eul[i];
      state[i+3] += ahrsA.deul[i];
      pthread_mutex_unlock(&ahrsA.mutex);

    }
  }

  // IMUB states
  if (IMUB_ENABLED)  {

    // Comp filter values (no yaw value)
    pthread_mutex_lock(&imuB.mutex);
    state[0] += imuB.roll;
    state[1] += imuB.pitch;
    pthread_mutex_unlock(&imuB.mutex);

    // Loop through states
    for ( i=0; i<3; i++ )  {

      // LPF gyro values
      pthread_mutex_lock(&gyrB.mutex);
      state[i+3] += gyrB.filter[i];
      pthread_mutex_unlock(&gyrB.mutex);

      // AHRS values
      pthread_mutex_lock(&ahrsB.mutex);
      state[i]   += ahrsB.eul[i];
      state[i+3] += ahrsB.deul[i];
      pthread_mutex_unlock(&ahrsB.mutex);

    }
  }

  // Average all the data sources
  if ( IMUA_ENABLED && IMUB_ENABLED )  {  for ( i=0; i<6; i++ )  state[i] /= 4.0;  }
  else                                 {  for ( i=0; i<6; i++ )  state[i] /= 2.0;  }

  // Correction b/c comp filter has no yaw value  
  state[2] *= 2.0; 

  // Obtain inputs
  pthread_mutex_lock(&input.mutex);
  for ( i=0; i<4; i++ )  in[i] = input.norm[i];
  dial = input.norm[CH_D];
  pthread_mutex_unlock(&input.mutex);

  // Obtain ctrl parameters
  pthread_mutex_lock(&stab.mutex);
  tmin    = stab.thrl[0];
  tmax    = stab.thrl[1];
  tilt    = stab.thrl[2];
  trange  = stab.range[3];
  heading = stab.heading;
  pthread_mutex_unlock(&stab.mutex);

  // Calculate reference signals
  for ( i=0; i<4; i++ )  ref[i] = in[i] * stab.range[i];

  // Determine desired heading
  if ( in[CH_T] > -0.9 && fabs(in[CH_Y]) > 0.1 )  heading += ref[CH_Y] * stab.dt;
  while ( heading >   M_PI )  heading -= 2.0 * M_PI;
  while ( heading <= -M_PI )  heading += 2.0 * M_PI;

  /*
  // Determine roll (X) adjustment
  perr[x] = -eul[x] + ref[CH_R];
  derr[x] = -deul[x];
  reset = ( in[CH_R] < -IRESET || in[CH_R] > IRESET );
  if (reset)  ierr[x] = 0.0;
  else        ierr[x] += perr[x] * stab.dt;
  cmd[x] = perr[x] * stab.pgain[x] +
           ierr[x] * stab.igain[x] +
           derr[x] * stab.dgain[x];

  // Determine pitch (Y) adjustment
  perr[y] = -eul[y] + ref[CH_P];
  derr[y] = -deul[y];
  reset = ( in[CH_P] < -IRESET || in[CH_P] > IRESET );
  if (reset)  ierr[y] = 0.0; 
  else        ierr[y] += perr[y] * stab.dt;
  cmd[y] = perr[y] * stab.pgain[y] + 
           ierr[y] * stab.igain[y] + 
           derr[y] * stab.dgain[y];

  // Determine yaw (Z) adjustment
  perr[z] = -eul[z] + heading;
  while ( perr[z] >   M_PI )  perr[z] -= 2.0 * M_PI;
  while ( perr[z] <= -M_PI )  perr[z] += 2.0 * M_PI;
  derr[z] = -deul[z];
  reset = ( in[CH_Y] < -IRESET || in[CH_Y] > IRESET );
  if (reset)  ierr[z] = 0.0; 
  else        ierr[z] += perr[z] * stab.dt;
  cmd[z] = perr[z] * stab.pgain[z] + 
           ierr[z] * stab.igain[z] + 
           derr[z] * stab.dgain[z];
  */

  // Apply PID on attitude
  cmd[x] = stab_pid( &pidX, state[0], state[3], ref[CH_R], ( in[CH_R] < -IRESET || in[CH_R] > IRESET ) );
  cmd[y] = stab_pid( &pidY, state[1], state[4], ref[CH_P], ( in[CH_P] < -IRESET || in[CH_P] > IRESET ) );
  cmd[z] = stab_pid( &pidZ, state[2], state[5], heading,   ( in[CH_Y] < -IRESET || in[CH_Y] > IRESET ) );

  // Determine throttle adjustment
  double tilt_adj = ( 1 - ( cos(state[0]) * cos(state[1]) ) ) * tilt;
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
  for ( i=0; i<6; i++ )  stab.state[i] = state[i];
  for ( i=0; i<4; i++ )  stab.cmd[i]   = cmd[i];
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


/**
 *  stab_disarm
 *  Disarm the motors and reset control surfaces
 */
void stab_disarm ( void )  {
  ushort ch;
  for ( ch=0; ch<OUT_CH; ch++ )  io_setnorm( ch, stab.off[ch] );
  return;
}



