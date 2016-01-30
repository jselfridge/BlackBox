
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

  // Local variables
  //ushort i, j;

  // Set timing values (make 'const' during initialization)
  ctrl.dt = 1.0 / HZ_CTRL;

  // Set reference ranges (make 'const' during initialization)
  ctrl.scale[CH_R] = R_RANGE;
  ctrl.scale[CH_P] = P_RANGE;
  ctrl.scale[CH_Y] = Y_RANGE;
  ctrl.scale[CH_T] = T_RANGE;

  // Set gain values (make 'const' during initialization)
  // ORIG:   P  150  I 0  D 35
  // NORM:   P 0.30  I 0  D 0.06
  ctrl.pgain[X] = 0.00;  ctrl.pgain[Y] = 0.00;  ctrl.pgain[Z] = 0.00;
  ctrl.igain[X] = 0.00;  ctrl.igain[Y] = 0.00;  ctrl.igain[Z] = 0.00;
  ctrl.dgain[X] = 0.00;  ctrl.dgain[Y] = 0.00;  ctrl.dgain[Z] = 0.00;

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
  if (armed)  ctl_pid();
  else        sio_disarm();
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctl_pid
//  Apply PID control to vehcile attitude.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctl_pid ( void )  {

  // Local variables
  ushort i, T=4;
  bool reset;
  double eul[3], ang[3], in[4], ref[4], cmd[4], out[4], heading;
  static double perr[3] = { 0.0, 0.0, 0.0 };
  static double ierr[3] = { 0.0, 0.0, 0.0 };
  static double derr[3] = { 0.0, 0.0, 0.0 };

  // Obtain states
  pthread_mutex_lock(&mutex_eul);
  for ( i=0; i<3; i++ )  {  eul[i] = ahr.eul[i];  ang[i] = ahr.deul[i];  }
  pthread_mutex_unlock(&mutex_eul);

  // Obtain inputs
  pthread_mutex_lock(&mutex_input);
  for ( i=0; i<4; i++ )  in[i] = input.norm[i];
  pthread_mutex_unlock(&mutex_input);

  // Obtain vehicle heading
  pthread_mutex_lock(&mutex_ctrl);
  heading = ctrl.heading;
  pthread_mutex_unlock(&mutex_ctrl);

  // Calculate reference signals
  for ( i=0; i<4; i++ )  ref[i] = in[i] * ctrl.scale[i];

  // Determine roll (X) adjustment
  perr[X] = -eul[X] + ref[CH_R];
  derr[X] = -ang[X];
  reset = ( in[CH_R] < -IRESET || in[CH_R] > IRESET );
  if (reset)  ierr[X] = 0.0;
  else        ierr[X] += perr[X] * ctrl.dt;
  cmd[X] = perr[X] * ctrl.pgain[X] +
           ierr[X] * ctrl.igain[X] +
           derr[X] * ctrl.dgain[X];

  // Determine pitch (Y) adjustment
  perr[Y] = -eul[Y] + ref[CH_P];
  derr[Y] = -ang[Y];
  reset = ( in[CH_P] < -IRESET || in[CH_P] > IRESET );
  if (reset)  ierr[Y] = 0.0; 
  else        ierr[Y] += perr[Y] * ctrl.dt;
  cmd[Y] = perr[Y] * ctrl.pgain[Y] + 
           ierr[Y] * ctrl.igain[Y] + 
           derr[Y] * ctrl.dgain[Y];

  // Determine yaw (Z) adjustment
  if ( in[CH_T] > -0.9 && fabs(in[CH_Y]) > 0.15 )  heading += ref[CH_Y] * ctrl.dt;
  while ( heading >   PI )  heading -= 2.0*PI;
  while ( heading <= -PI )  heading += 2.0*PI;
  perr[Z] = -eul[Z] + heading;
  while ( perr[Z] >   PI )  heading -= 2.0*PI;
  while ( perr[Z] <= -PI )  heading += 2.0*PI;
  derr[Z] = -ang[Z];
  reset = ( in[CH_Y] < -IRESET || in[CH_Y] > IRESET );
  if (reset)  ierr[Z] = 0.0; 
  else        ierr[Z] += perr[Z] * ctrl.dt;
  cmd[Z] = perr[Z] * ctrl.pgain[Z] + 
           ierr[Z] * ctrl.igain[Z] + 
           derr[Z] * ctrl.dgain[Z];

  /* // Determine throttle adjustment
  double tilt, threshold, range;
  tilt = 1 - ( cos(eul[X]) * cos(eul[Y]) );
  threshold = T_MIN + (0.5) * ( norm[CH_D] + 1.0 ) * ( T_MAX - T_MIN ) - T_RANGE;
  if ( norm[CH_T] <=0 )  range = threshold - 1000;
  else                   range = 2.0 * T_RANGE;
  cmd[T] = threshold + norm[CH_T] * range + T_TILT * tilt;
  */


  cmd[T] = 0.0;  // Debugging Hover

  // Assign motor outputs
  if ( in[CH_T] > -0.9 ) {
  out[MOT_FR] = cmd[T] - cmd[X] + cmd[Y] + cmd[Z];
  out[MOT_BL] = cmd[T] + cmd[X] - cmd[Y] + cmd[Z];
  out[MOT_FL] = cmd[T] + cmd[X] + cmd[Y] - cmd[Z];
  out[MOT_BR] = cmd[T] - cmd[X] - cmd[Y] - cmd[Z];
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
//  ctl_debug
//  Debugging control law with direct input to output mapping.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctl_debug ( void )  {
  ushort ch;
  for ( ch=0; ch<10; ch++ )  sio_setpwm( ch, input.pwm[ch] );
  return;
}



