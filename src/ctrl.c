
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

  // Set gain values (make 'const' during initialization) P 150  I 0  D 35
  ctrl.pgain[X] =   0.0;  ctrl.pgain[Y] =   0.0;  ctrl.pgain[Z] =   0.0;
  ctrl.igain[X] =   0.0;  ctrl.igain[Y] =   0.0;  ctrl.igain[Z] =   0.0;
  ctrl.dgain[X] =   0.0;  ctrl.dgain[Y] =   0.0;  ctrl.dgain[Z] =   0.0;

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
  ushort i;
  bool reset;
  double eul[3], ang[3], norm[4], ref[4], cmd[4];
  static double perr[3] = { 0.0, 0.0, 0.0 };
  static double ierr[3] = { 0.0, 0.0, 0.0 };
  static double derr[3] = { 0.0, 0.0, 0.0 };

  // Obtain states
  pthread_mutex_lock(&mutex_eul);
  for ( i=0; i<3; i++ )  {  eul[i] = ahr.eul[i];  ang[i] = ahr.deul[i];  }
  pthread_mutex_unlock(&mutex_eul);

  // Obtain inputs and references
  pthread_mutex_lock(&mutex_input);
  for ( i=0; i<4; i++ )  norm[i] = input.norm[i];
  pthread_mutex_unlock(&mutex_input);
  for ( i=0; i<4; i++ )  ref[i] = norm[i] * ctrl.scale[i];

  // Determine roll (X) adjustment
  perr[X] = -eul[X] + ref[CH_R];
  derr[X] = -ang[X];
  reset = ( norm[CH_R] < -IRESET || norm[CH_R] > IRESET );
  if (reset)  ierr[X] = 0.0;
  else        ierr[X] += perr[X] * ctrl.dt;
  cmd[X] = perr[X] * ctrl.pgain[X] +
           ierr[X] * ctrl.igain[X] +
           derr[X] * ctrl.dgain[X];

  // Determine pitch (Y) adjustment
  perr[Y] = -eul[Y] + ref[CH_P];
  derr[Y] = -ang[Y];
  reset = ( norm[CH_P] < -IRESET || norm[CH_P] > IRESET );
  if (reset)  ierr[Y] = 0.0; 
  else        ierr[Y] += perr[Y] * ctrl.dt;
  cmd[Y] = perr[Y] * ctrl.pgain[Y] + 
           ierr[Y] * ctrl.igain[Y] + 
           derr[Y] * ctrl.dgain[Y];
  /*
  // Determine yaw (Z) adjustment
  double Y_KIreset;
  if ( ctrl.norm[CH_T] > -0.9 && fabs(ctrl.norm[CH_Y]) > 0.15 )  ctrl.heading += ctrl.ref[CH_Y] * ctrl.dt;
  while ( ctrl.heading >   PI )  ctrl.heading -= 2.0*PI;
  while ( ctrl.heading <= -PI )  ctrl.heading += 2.0*PI;
  ctrl.err[Z][P] = -Eul[Z] + ctrl.heading;
  while ( ctrl.err[Z][P] >   PI )  ctrl.heading -= 2.0*PI;
  while ( ctrl.err[Z][P] <= -PI )  ctrl.heading += 2.0*PI;
  ctrl.err[Z][D] = -dEul[Z];
  Y_KIreset = ctrl.ref[CH_Y] / ctrl.range[CH_Y];
  if ( Y_KIreset < -I_RESET || Y_KIreset > I_RESET )
    ctrl.err[Z][I] = 0; 
  else
    ctrl.err[Z][I] += ctrl.err[Z][P] * ctrl.dt;
  ctrl.input[Z] = ctrl.err[Z][P] * ctrl.gain[Z][P] + 
                  ctrl.err[Z][I] * ctrl.gain[Z][I] + 
                  ctrl.err[Z][D] * ctrl.gain[Z][D];

  // Determine throttle adjustment
  double tilt, threshold, range;
  tilt = 1 - ( cos(Eul[X]) * cos(Eul[Y]) );
  threshold = T_MIN + (0.5) * ( ctrl.norm[CH_D] + 1.0 ) * ( T_MAX - T_MIN ) - T_RANGE;
  if ( ctrl.norm[CH_T] <=0 )  range = threshold - 1000;
  else                        range = 2.0 * T_RANGE;
  ctrl.input[T] = threshold + ctrl.norm[CH_T] * range + T_TILT * tilt;
  ctrl.input[T] = 1500;  // Debugging

  // Set motor outputs
  ushort i;
  if ( ctrl.norm[CH_T] > -0.9 ) {
  sys.output[MOT_FR] = ctrl.input[T] - ctrl.input[X] + ctrl.input[Y] + ctrl.input[Z];
  sys.output[MOT_BL] = ctrl.input[T] + ctrl.input[X] - ctrl.input[Y] + ctrl.input[Z];
  sys.output[MOT_FL] = ctrl.input[T] + ctrl.input[X] + ctrl.input[Y] - ctrl.input[Z];
  sys.output[MOT_BR] = ctrl.input[T] - ctrl.input[X] - ctrl.input[Y] - ctrl.input[Z];
  } else {  for ( i=0; i<4; i++ )  sys.output[i] = 1000;  }
  */

  // Push to data structure
  pthread_mutex_lock(&mutex_ctrl);
  for ( i=0; i<3; i++ )  {
    ctrl.perr[i] = perr[i];
    ctrl.ierr[i] = ierr[i];
    ctrl.derr[i] = derr[i];
  }
  for ( i=0; i<4; i++ )  ctrl.cmd[i] = cmd[i];
  pthread_mutex_unlock(&mutex_ctrl);

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



