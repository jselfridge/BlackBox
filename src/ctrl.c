
//============================================================
//  ctrl.c
//  Justin M Selfridge
//============================================================
#include "ctrl.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctl_init
//  Initializes the control structure.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctl_init ( void )  {
  if (DEBUG)  printf("Initializing controller \n");

  // Array index
  ushort x=0, y=1, z=2;

  // Set timing values (make 'const' during initialization)
  ctrl.dt = 1.0 / HZ_CTRL;

  // Set reference ranges (make 'const' during initialization)
  ctrl.scale[CH_R] = R_RANGE;
  ctrl.scale[CH_P] = P_RANGE;
  ctrl.scale[CH_Y] = Y_RANGE;
  ctrl.scale[CH_T] = T_RANGE;

  // Set gain values (make 'const' during initialization)
  ctrl.xgain[p] = GAIN_XP;  ctrl.ygain[p] = GAIN_YP;  ctrl.zgain[p] = GAIN_ZP;
  ctrl.xgain[i] = GAIN_XI;  ctrl.ygain[i] = GAIN_YI;  ctrl.zgain[i] = GAIN_ZI;
  ctrl.xgain[d] = GAIN_XD;  ctrl.ygain[d] = GAIN_YD;  ctrl.zgain[d] = GAIN_ZD;

  // Display system
  if (DEBUG)  printf( "  System: %s \n", SYSTEM );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctl_exit
//  Exits the controller code.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctl_exit ( void )  {
  if (DEBUG)  printf("Close controller \n");
  // Add exit code as needed...
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctl_exec
//  Executes the top level logic for each control loop.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctl_exec ( void )  {
  if (armed)  {
    if ( !strcmp( SYSTEM, "quad"  ) )  ctl_quad();
    if ( !strcmp( SYSTEM, "plane" ) )  ctl_plane();
  }
  else        sio_disarm();
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctl_quad
//  Apply control to quadrotor system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctl_quad ( void )  {

  // Local variables
  bool reset;
  ushort i;
  ushort p=0, i=1, d=2;
  ushort x=0, y=1, z=2, t=3;
  double xstate[3], ystate[3], zstate[3];
  //double eul[3], ang[3], in[4], ref[4], cmd[4], out[4], heading, dial;
  //static double perr[3] = { 0.0, 0.0, 0.0 };
  //static double ierr[3] = { 0.0, 0.0, 0.0 };
  //static double derr[3] = { 0.0, 0.0, 0.0 };

  // Obtain states
  pthread_mutex_lock(&mutex_eul);
  xstate[p] = ahr.eul[x];  xstate[d] = ahr.deul[x];
  ystate[p] = ahr.eul[y];  ystate[d] = ahr.deul[y];
  zstate[p] = ahr.eul[z];  zstate[d] = ahr.deul[z];
  pthread_mutex_unlock(&mutex_eul);

  // Obtain inputs
  pthread_mutex_lock(&mutex_input);
  for ( i=0; i<4; i++ )  in[i] = input.norm[i];
  dial = input.norm[CH_D];
  pthread_mutex_unlock(&mutex_input);

  // Obtain vehicle heading
  pthread_mutex_lock(&mutex_ctrl);
  heading = ctrl.heading;
  pthread_mutex_unlock(&mutex_ctrl);

  // Calculate reference signals
  for ( i=0; i<4; i++ )  ref[i] = in[i] * ctrl.scale[i];

  // Determine desired heading
  if ( in[CH_T] > -0.9 && fabs(in[CH_Y]) > 0.15 )  heading += ref[CH_Y] * ctrl.dt;
  while ( heading >   PI )  heading -= 2.0*PI;
  while ( heading <= -PI )  heading += 2.0*PI;


  // Roll (x) command input
  reset = ( in[CH_R] < -IRESET || in[CH_R] > IRESET );
  cmd[x] = ctl_pid ( double Y[3], ctrl.xgain, double R, reset, ctrl.dt );

  // Pitch (y) command input
  reset = ( in[CH_P] < -IRESET || in[CH_P] > IRESET );
  UP = ctl_pid ( double Y[3], ctrl.Pgain, double R, reset, ctrl.dt );

  // Yaw (z) command input
  reset = ( in[CH_Y] < -IRESET || in[CH_Y] > IRESET );
  UY = ctl_pid ( double Y[3], ctrl.Ygain, double R, reset, ctrl.dt );


  // Determine roll (X) adjustment
  //~~~~~~~~~~~~~~~~
  // double ctrl_pid_pos( double eul, double ang, double in, double ref, double ierr );
  // double perr, derr;
  // bool reset;
  /*
  perr[x] = -eul[x] + ref[CH_R];
  derr[x] = -ang[x];
  reset = ( in[CH_R] < -IRESET || in[CH_R] > IRESET );
  if (reset)  ierr[x] = 0.0;
  else        ierr[x] += perr[x] * ctrl.dt;
  cmd[x] = perr[x] * ctrl.pgain[x] +
           ierr[x] * ctrl.igain[x] +
           derr[x] * ctrl.dgain[x];
  // return cmd;
  //~~~~~~~~~~~~~~*/

  /*
  // Determine pitch (Y) adjustment
  perr[y] = -eul[y] + ref[CH_P];
  derr[y] = -ang[y];
  reset = ( in[CH_P] < -IRESET || in[CH_P] > IRESET );
  if (reset)  ierr[y] = 0.0; 
  else        ierr[y] += perr[y] * ctrl.dt;
  cmd[y] = perr[y] * ctrl.pgain[y] + 
           ierr[y] * ctrl.igain[y] + 
           derr[y] * ctrl.dgain[y];
  */


  /*
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
  */


  // Determine throttle adjustment
  double tilt = ( 1 - ( cos(eul[x]) * cos(eul[y]) ) ) * TILT;
  double thresh = ( 0.5 * ( dial + 1.0 ) * ( TMAX - TMIN ) ) + TMIN - T_RANGE;
  if ( in[CH_T] <= -0.6 )  cmd[t] = ( 2.50 * ( in[CH_T] + 1.0 ) * ( thresh + 1.0 ) ) - 1.0 + tilt;
  else                     cmd[t] = ( 1.25 * ( in[CH_T] + 0.6 ) * T_RANGE ) + thresh + tilt; 
  cmd[t] = 0.0;   //--  DEBUGGING  --//

  // Assign motor outputs
  if ( in[CH_T] > -0.9 ) {
  out[MOT_FR] = cmd[t] - cmd[x] + cmd[y] + cmd[z];
  out[MOT_BL] = cmd[t] + cmd[x] - cmd[y] + cmd[z];
  out[MOT_FL] = cmd[t] + cmd[x] + cmd[y] - cmd[z];
  out[MOT_BR] = cmd[t] - cmd[x] - cmd[y] - cmd[z];
  } else {  for ( i=0; i<4; i++ )  out[i] = -1.0;  }


  // Push control data
  pthread_mutex_lock(&mutex_ctrl);
  for ( i=0; i<3; i++ )  {
    ctrl.perr[i] = perr[i];
    ctrl.ierr[i] = ierr[i];
    ctrl.derr[i] = derr[i];
  }
  for ( i=0; i<4; i++ )  ctrl.cmd[i] = cmd[i];
  ctrl.heading = heading;
  pthread_mutex_unlock(&mutex_ctrl);

  // Push system outputs
  for ( i=0; i<4; i++ )  sio_setnorm( i, out[i] );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctl_plane
//  Apply control to plane configuration.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctl_plane ( void )  {

  // Local variables
  ushort ch;
  double in[4], out[4];

  // Move to #define section
  double center[4] = { 0.00, 0.00, 0.00, 0.00 };
  double scale[4]  = { 1.00, 1.00, 1.00, 1.00 };

  // Obtain inputs
  pthread_mutex_lock(&mutex_input);
  for ( ch=0; ch<4; ch++ )  in[ch] = input.norm[ch];
  pthread_mutex_unlock(&mutex_input);

  // Scale and center outputs
  for ( ch=0; ch<4; ch++ )  out[ch] = in[ch] * scale[ch] + center[ch];

  // Push system outputs
  for ( ch=0; ch<4; ch++ )  sio_setnorm( ch, out[ch] );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctl_debug
//  Debugging control law with direct input to output mapping.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctl_debug ( void )  {
  ushort ch;
  for ( ch=0; ch<10; ch++ )  sio_setpwm( ch, input.pwm[ch] );
  return;
}



