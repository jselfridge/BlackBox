

#include "ctrl.h"


/**
 *  ctrl_init
 *  Initializes the control structure.
 */
void ctrl_init ( void )  {
  if (DEBUG)  printf("Initializing controller \n");

  // Array index
  ushort x=0, y=1, z=2;

  // Set timing values (make 'const' during initialization)
  ctrl.dt = 1.0 / HZ_CTRL;

  // Set 'quad' parameters
  if ( !strcmp( SYSTEM, "quad" ) )  {

    // Asign disarming array values
    ctrl.off[0] = QUAD_OFF0;
    ctrl.off[1] = QUAD_OFF1;
    ctrl.off[2] = QUAD_OFF2;
    ctrl.off[3] = QUAD_OFF3;
    ctrl.off[4] = QUAD_OFF4;
    ctrl.off[5] = QUAD_OFF5;
    ctrl.off[6] = QUAD_OFF6;
    ctrl.off[7] = QUAD_OFF7;
    ctrl.off[8] = QUAD_OFF8;
    ctrl.off[9] = QUAD_OFF9;

    // Reference ranges
    ctrl.scale[CH_R] = QUAD_X_RANGE;
    ctrl.scale[CH_P] = QUAD_Y_RANGE;
    ctrl.scale[CH_Y] = QUAD_Z_RANGE;
    ctrl.scale[CH_T] = QUAD_T_RANGE;

    // Gain values
    ctrl.pgain[x] = QUAD_PX;  ctrl.pgain[y] = QUAD_PY;  ctrl.pgain[z] = QUAD_PZ;
    ctrl.igain[x] = QUAD_IX;  ctrl.igain[y] = QUAD_IY;  ctrl.igain[z] = QUAD_IZ;
    ctrl.dgain[x] = QUAD_DX;  ctrl.dgain[y] = QUAD_DY;  ctrl.dgain[z] = QUAD_DZ;

    // Throttle values
    ctrl.thrl[0] = QUAD_TMIN;
    ctrl.thrl[1] = QUAD_TMAX;
    ctrl.thrl[2] = QUAD_TILT;

  }

  /*
  // Set 'plane' parameters
  if ( !strcmp( SYSTEM, "plane" ) )  {

    // Asign disarming array values (make 'const' in header file? adjustable via GCS?) 
    ctrl.off[0] = PLANE_OFF0;
    ctrl.off[1] = PLANE_OFF1;
    ctrl.off[2] = PLANE_OFF2;
    ctrl.off[3] = PLANE_OFF3;
    ctrl.off[4] = PLANE_OFF4;
    ctrl.off[5] = PLANE_OFF5;
    ctrl.off[6] = PLANE_OFF6;
    ctrl.off[7] = PLANE_OFF7;
    ctrl.off[8] = PLANE_OFF8;
    ctrl.off[9] = PLANE_OFF9;

    // Reference ranges
    //ctrl.scale[CH_R] = PLANE_R_RANGE;
    //ctrl.scale[CH_Y] = PLANE_Y_RANGE;
    //ctrl.scale[CH_P] = PLANE_P_RANGE;
    //ctrl.scale[CH_T] = PLANE_T_RANGE;

    // Gain values
    //ctrl.pgain[x] = PLANE_PX;  ctrl.pgain[y] = PLANE_PY;  ctrl.pgain[z] = PLANE_PZ;
    //ctrl.dgain[x] = PLANE_DX;  ctrl.dgain[y] = PLANE_DY;  ctrl.dgain[z] = PLANE_DZ;
    //ctrl.igain[x] = 0.0;       ctrl.igain[y] = 0.0;       ctrl.igain[z] = 0.0;

  }
  */

  // Display system
  if (DEBUG)  printf( "  System: %s \n", SYSTEM );

  return;
}


/**
 *  ctrl_exit
 *  Exits the controller code.
 */
void ctrl_exit ( void )  {
  if (DEBUG)  printf("Close controller \n");
  // Add exit code as needed...
  return;
}


/**
 *  ctrl_update
 *  Executes the top level logic to update each control loop.
 */
void ctrl_update ( void )  {
  if (armed)  {
    if ( !strcmp( SYSTEM, "quad"  ) )  ctrl_quad();
    //if ( !strcmp( SYSTEM, "plane" ) )  ctrl_plane();
  }
  else  ctrl_disarm();
  return;
}


/**
 *  ctrl_quad
 *  Apply control to quadrotor system.
 */
void ctrl_quad ( void )  {

  // Local variables
  bool reset;
  ushort ch;
  ushort x=0, y=1, z=2, t=3;
  double eul[3], ang[3], in[4], ref[4], cmd[4], out[4];
  double trange, tmin, tmax, tilt, heading, dial;
  static double perr[3] = { 0.0, 0.0, 0.0 };
  static double ierr[3] = { 0.0, 0.0, 0.0 };
  static double derr[3] = { 0.0, 0.0, 0.0 };

  // Obtain states
  pthread_mutex_lock(&mutex_eul);
  for ( ch=0; ch<3; ch++ )  {  eul[ch] = ahrs.eul[ch];  ang[ch] = ahrs.deul[ch];  }
  pthread_mutex_unlock(&mutex_eul);

  // Obtain inputs
  pthread_mutex_lock(&mutex_input);
  for ( ch=0; ch<4; ch++ )  in[ch] = input.norm[ch];
  dial = input.norm[CH_D];
  pthread_mutex_unlock(&mutex_input);

  // Obtain vehicle heading
  pthread_mutex_lock(&mutex_ctrl);
  trange  = ctrl.scale[3];
  tmin    = ctrl.thrl[0];
  tmax    = ctrl.thrl[1];
  tilt    = ctrl.thrl[2];
  heading = ctrl.heading;
  pthread_mutex_unlock(&mutex_ctrl);

  // Calculate reference signals
  for ( ch=0; ch<4; ch++ )  ref[ch] = in[ch] * ctrl.scale[ch];

  // Determine desired heading
  if ( in[CH_T] > -0.9 && fabs(in[CH_Y]) > 0.15 )  heading += ref[CH_Y] * ctrl.dt;
  while ( heading >   PI )  heading -= 2.0*PI;
  while ( heading <= -PI )  heading += 2.0*PI;

  // Determine roll (X) adjustment
  perr[x] = -eul[x] + ref[CH_R];
  derr[x] = -ang[x];
  reset = ( in[CH_R] < -IRESET || in[CH_R] > IRESET );
  if (reset)  ierr[x] = 0.0;
  else        ierr[x] += perr[x] * ctrl.dt;
  cmd[x] = perr[x] * ctrl.pgain[x] +
           ierr[x] * ctrl.igain[x] +
           derr[x] * ctrl.dgain[x];

  // Determine pitch (Y) adjustment
  perr[y] = -eul[y] + ref[CH_P];
  derr[y] = -ang[y];
  reset = ( in[CH_P] < -IRESET || in[CH_P] > IRESET );
  if (reset)  ierr[y] = 0.0; 
  else        ierr[y] += perr[y] * ctrl.dt;
  cmd[y] = perr[y] * ctrl.pgain[y] + 
           ierr[y] * ctrl.igain[y] + 
           derr[y] * ctrl.dgain[y];

  // Determine yaw (Z) adjustment
  perr[z] = -eul[z] + heading;
  while ( perr[z] >   PI )  heading -= 2.0*PI;
  while ( perr[z] <= -PI )  heading += 2.0*PI;
  derr[z] = -ang[z];
  reset = ( in[CH_Y] < -IRESET || in[CH_Y] > IRESET );
  if (reset)  ierr[z] = 0.0; 
  else        ierr[z] += perr[z] * ctrl.dt;
  cmd[z] = perr[z] * ctrl.pgain[z] + 
           ierr[z] * ctrl.igain[z] + 
           derr[z] * ctrl.dgain[z];

  // Determine throttle adjustment
  double tilt_adj = ( 1 - ( cos(eul[x]) * cos(eul[y]) ) ) * tilt;
  double thresh = ( 0.5 * ( dial + 1.0 ) * ( tmax - tmin ) ) + tmin - trange;
  if ( in[CH_T] <= -0.6 )  cmd[t] = ( 2.50 * ( in[CH_T] + 1.0 ) * ( thresh + 1.0 ) ) - 1.0 + tilt_adj;
  else                     cmd[t] = ( 1.25 * ( in[CH_T] + 0.6 ) * trange ) + thresh + tilt_adj; 

  // Assign motor outputs
  if ( in[CH_T] > -0.9 ) {
  out[QUAD_FR] = cmd[t] - cmd[x] + cmd[y] + cmd[z];
  out[QUAD_BR] = cmd[t] - cmd[x] - cmd[y] - cmd[z];
  out[QUAD_BL] = cmd[t] + cmd[x] - cmd[y] + cmd[z];
  out[QUAD_FL] = cmd[t] + cmd[x] + cmd[y] - cmd[z];
  } else {  for ( ch=0; ch<4; ch++ )  out[ch] = -1.0;  }

  // Push control data
  pthread_mutex_lock(&mutex_ctrl);
  for ( ch=0; ch<3; ch++ )  {
    ctrl.perr[ch] = perr[ch];
    ctrl.ierr[ch] = ierr[ch];
    ctrl.derr[ch] = derr[ch];
  }
  for ( ch=0; ch<4; ch++ )  ctrl.cmd[ch] = cmd[ch];
  ctrl.heading = heading;
  pthread_mutex_unlock(&mutex_ctrl);

  // Push system outputs
  for ( ch=0; ch<4; ch++ )  sio_setnorm( ch, out[ch] );

  return;
}


/**
 *  ctrl_plane
 *  Apply control to plane configuration.
 */
/*void ctrl_plane ( void )  {

  // Local variables
  double elev, rudd, thrl, dial, prop, thresh;

  // Obtain inputs
  pthread_mutex_lock(&mutex_input);
  elev = input.norm[CH_P];
  rudd = input.norm[CH_Y];
  thrl = input.norm[CH_T];
  dial = input.norm[CH_D];
  pthread_mutex_unlock(&mutex_input);

  // Determine throttle adjustment
  thresh = ( 0.5 * ( dial + 1.0 ) * ( PLANE_TMAX - PLANE_TMIN ) ) + PLANE_TMIN - PLANE_T_RANGE;
  if ( thrl <= -0.6 )  prop = ( 2.50 * ( thrl + 1.0 ) * ( thresh + 1.0 ) ) - 1.0;
  else                 prop = ( 1.25 * ( thrl + 0.6 ) * PLANE_T_RANGE ) + thresh;

  // Assign signal outputs
  if ( thrl > -0.9 )  {
    sio_setnorm( PLANE_ELEV,  elev );
    sio_setnorm( PLANE_RUDD, -rudd );
    sio_setnorm( PLANE_THRL,  prop );
  }
  else  ctrl_disarm();

  return;
}
*/

  /*
  // Local variables
  ushort ch;
  ushort x=0, y=1, z=2, t=3;
  double eul[3], ang[3];
  double in[4], ref[4], cmd[4], out[4];
  double bank, climb, heading, dial;
  static double perr[3] = { 0.0, 0.0, 0.0 };
  static double derr[3] = { 0.0, 0.0, 0.0 };

  // Obtain states
  pthread_mutex_lock(&mutex_eul);
  for ( ch=0; ch<3; ch++ )  {  eul[ch] = ahr.eul[ch];  ang[ch] = ahr.deul[ch];  }
  pthread_mutex_unlock(&mutex_eul);

  // Obtain inputs
  pthread_mutex_lock(&mutex_input);
  for ( ch=0; ch<4; ch++ )  in[ch] = input.norm[ch];
  dial = input.norm[CH_D];
  pthread_mutex_unlock(&mutex_input);

  // Obtain desired vehicle state
  pthread_mutex_lock(&mutex_ctrl);
  bank    = ctrl.bank;
  climb   = ctrl.climb;
  heading = ctrl.heading;
  pthread_mutex_unlock(&mutex_ctrl);

  // Calculate reference signals
  for ( ch=0; ch<4; ch++ )  ref[ch] = in[ch] * ctrl.scale[ch];

  // Determine desired bank
  if ( in[CH_T] > -0.9 && fabs(in[CH_R]) > 0.15 )  bank += ref[CH_R] * ctrl.dt;
  while ( bank >   PI )  bank -= 2.0*PI;
  while ( bank <= -PI )  bank += 2.0*PI;

  // Determine desired climb
  if ( in[CH_T] > -0.9 && fabs(in[CH_P]) > 0.15 )  climb += ref[CH_P] * ctrl.dt;
  while ( climb >   PI )  climb -= 2.0*PI;
  while ( climb <= -PI )  climb += 2.0*PI;

  // Determine desired heading
  if ( in[CH_T] > -0.9 && fabs(in[CH_Y]) > 0.15 )  heading += ref[CH_Y] * ctrl.dt;
  while ( heading >   PI )  heading -= 2.0*PI;
  while ( heading <= -PI )  heading += 2.0*PI;

  // Determine roll (X) adjustment
  perr[x] = -eul[x] + bank;
  while ( perr[x] >   PI )  perr[x] -= 2.0*PI;
  while ( perr[x] <= -PI )  perr[x] += 2.0*PI;
  derr[x] = -ang[x];
  cmd[x] = ( perr[x] * ctrl.pgain[x] ) + ( derr[x] * ctrl.dgain[x] );

  // Determine pitch (Y) adjustment
  perr[y] = -eul[y] + climb;
  while ( perr[y] >   PI )  perr[y] -= 2.0*PI;
  while ( perr[y] <= -PI )  perr[y] += 2.0*PI;
  derr[y] = -ang[y];
  cmd[y] = ( perr[y] * ctrl.pgain[y] ) + ( derr[y] * ctrl.dgain[y] );

  // Determine yaw (Z) adjustment
  perr[z] = -eul[z] + heading;
  while ( perr[z] >   PI )  perr[z] -= 2.0*PI;
  while ( perr[z] <= -PI )  perr[z] += 2.0*PI;
  derr[z] = -ang[z];
  cmd[z] = ( perr[z] * ctrl.pgain[z] ) + ( derr[z] * ctrl.dgain[z] );

  // Determine throttle adjustment
  double thresh = ( 0.5 * ( dial + 1.0 ) * ( PLANE_TMAX - PLANE_TMIN ) ) + PLANE_TMIN - PLANE_T_RANGE;
  if ( in[CH_T] <= -0.6 )  cmd[t] = ( 2.50 * ( in[CH_T] + 1.0 ) * ( thresh + 1.0 ) ) - 1.0;
  else                     cmd[t] = ( 1.25 * ( in[CH_T] + 0.6 ) * PLANE_T_RANGE ) + thresh; 

  // Assign signal outputs
  if ( in[CH_T] > -0.9 )  {  
    out[0] = cmd[x];
    out[1] = cmd[y];
    out[3] = cmd[z];
    out[4] = cmd[t];
  }
  else  {
    out[0] = 0.0;
    out[1] = 0.0;
    out[3] = 0.0;
    out[4] = -1.0;
  }

  // Push control data
  pthread_mutex_lock(&mutex_ctrl);
  for ( ch=0; ch<3; ch++ )  {
    ctrl.perr[ch] = perr[ch];
    ctrl.derr[ch] = derr[ch];
    ctrl.ierr[ch] = 0.0;
  }
  for ( ch=0; ch<4; ch++ )  ctrl.cmd[ch] = cmd[ch];
  ctrl.bank    = bank;
  ctrl.climb   = climb;
  ctrl.heading = heading;
  pthread_mutex_unlock(&mutex_ctrl);

  // Push system outputs
  for ( ch=0; ch<4; ch++ )  sio_setnorm( ch, out[ch] );

  return;

  */


/**
 *  ctrl_disarm
 *  Set all system outputs to their disarmed state.
 */
void ctrl_disarm ( void )  {
  ushort ch;
  for ( ch=0; ch<OUT_CH; ch++ )  sio_setnorm( ch, ctrl.off[ch] );
  return;
}



