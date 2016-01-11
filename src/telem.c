
//============================================================
//  telem.c
//  Justin M Selfridge
//============================================================
#include "telem.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  telem_init
//  Initializes the telemetry system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void telem_init ( void )  {

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  telem_exit
//  Exits the telemetry system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void telem_exit ( void )  {

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  telem_update
//  Updates the telemetry system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void telem_update ( void )  {

  return;
}

/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctrl_disarm
//  Disarms all the motors
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctrl_disarm ( void )  {
  ushort i;
  for ( i=0; i<10; i++ ) {
    sys.output[i] = 1000;
    pru_send_pulse( i, sys.output[i] );
  }
  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctrl_law
//  Top level function to run the control law
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctrl_law ( void )  {
  ctrl_ref();
  ctrl_flags();
  ctrl_switch();
  ctrl_limit();
  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctrl_ref
//  Takes PWM inputs and converts to suitable reference signals.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctrl_ref ( void )  {
  ushort i;
  for ( i=0; i<10; i++ ) {
    ctrl.norm[i] = 2.0 * ( sys.input[i] - IN_MID ) / (float)( IN_MAX - IN_MIN );
    if ( ctrl.norm[i] >  1.0 )  ctrl.norm[i] =  1.0;
    if ( ctrl.norm[i] < -1.0 )  ctrl.norm[i] = -1.0;
    if (i<4)  ctrl.ref[i] = ctrl.norm[i] * ctrl.range[i];
  }
  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctrl_flags
//  Checks radio stick positions for commands. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctrl_flags ( void )  {

  // Update full stick counter
  ushort i;
  for ( i=0; i<4; i++ ) {
    if ( ctrl.norm[i] >  0.95 ) {  ctrl.fullStick[i][HIGH]++;  }  else {  ctrl.fullStick[i][HIGH] = 0;  }
    if ( ctrl.norm[i] < -0.95 ) {  ctrl.fullStick[i][LOW]++;   }  else {  ctrl.fullStick[i][LOW]  = 0;  }
  }

  // Only execute stick commands with zero throttle
  if ( ctrl.fullStick[CH_T][DOWN] >= ctrl.stickHold )  {
    
    // Data log: roll stick only, no yaw command
    if ( !ctrl.fullStick[CH_Y][LEFT] && !ctrl.fullStick[CH_Y][RIGHT] )  {
      if ( ctrl.fullStick[CH_R][LEFT]  >= ctrl.stickHold ) {  datalog.enabled = true;  led_on(LED_LOG);   }
      if ( ctrl.fullStick[CH_R][RIGHT] >= ctrl.stickHold ) {  datalog.enabled = false; led_off(LED_LOG);  }
    }

    // Motor arming: yaw stick only, no roll command
    if ( !ctrl.fullStick[CH_R][LEFT] && !ctrl.fullStick[CH_R][RIGHT] )  {
      if ( ctrl.fullStick[CH_Y][RIGHT] >= ctrl.stickHold ) {
	ctrl.motorsArmed = true;
	led_on(LED_MOT);
	ctrl.heading = imu1.Eul[Z];
	if(DEBUG) printf( "Armed at %7.4f deg\n", ctrl.heading );
      }
      if ( ctrl.fullStick[CH_Y][LEFT]  >= ctrl.stickHold ) {
	ctrl.motorsArmed = false;
	led_off(LED_MOT);
      }
    }

    // Exit program: roll and yaw together to exit program
    if (  ctrl.fullStick[CH_Y][LEFT] >= ctrl.stickHold  &&  ctrl.fullStick[CH_R][RIGHT] >= ctrl.stickHold  ) {
      sys.running = false;
      led_off(LED_IMU);
      led_off(LED_PRU);
      led_off(LED_LOG);
      led_off(LED_MOT);
    }

  }

  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctrl_switch
//  Monitor radio switch to implement different control laws.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctrl_switch ( void )  {

  // Check arming flag
  if (ctrl.motorsArmed) {

  // Run switch command
  ctrl_pid();
  //ctrl_siso_sf();
  //ctrl_mimo_sf();
  //ctrl_sysid();
  //ctrl_mrac();

  }

  else ctrl_disarm();

  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctrl_pid
//  Apply PID control to vehcile attitude.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctrl_pid ( void )  {

  // Determine roll (X) adjustment
  double R_KIreset;
  ctrl.err[X][P] = -imu1.Eul[X] + ctrl.ref[CH_R];
  ctrl.err[X][D] = -imu1.dEul[X];
  R_KIreset = ctrl.ref[CH_R] / ctrl.range[CH_R];
  if ( R_KIreset < -I_RESET || R_KIreset > I_RESET )  
    ctrl.err[X][I] = 0; 
  else
    ctrl.err[X][I] += ctrl.err[X][P] * ctrl.dt;
  ctrl.input[X] = ctrl.err[X][P] * ctrl.gain[X][P] + 
                  ctrl.err[X][I] * ctrl.gain[X][I] + 
                  ctrl.err[X][D] * ctrl.gain[X][D];

  // Determine pitch (Y) adjustment
  double P_KIreset;
  ctrl.err[Y][P] = -imu1.Eul[Y] + ctrl.ref[CH_P];
  ctrl.err[Y][D] = -imu1.dEul[Y];
  P_KIreset = ctrl.ref[CH_P] / ctrl.range[CH_P];
  if ( P_KIreset < -I_RESET || P_KIreset > I_RESET )
    ctrl.err[Y][I] = 0; 
  else
    ctrl.err[Y][I] += ctrl.err[Y][P] * ctrl.dt;
  ctrl.input[Y] = ctrl.err[Y][P] * ctrl.gain[Y][P] + 
                  ctrl.err[Y][I] * ctrl.gain[Y][I] + 
                  ctrl.err[Y][D] * ctrl.gain[Y][D];

  // Determine yaw (Z) adjustment
  double Y_KIreset;
  if ( ctrl.norm[CH_T] > -0.9 && fabs(ctrl.norm[CH_Y]) > 0.15 )  ctrl.heading += ctrl.ref[CH_Y] * ctrl.dt;
  while ( ctrl.heading >   PI )  ctrl.heading -= 2.0*PI;
  while ( ctrl.heading <= -PI )  ctrl.heading += 2.0*PI;
  ctrl.err[Z][P] = -imu1.Eul[Z] + ctrl.heading;
  while ( ctrl.err[Z][P] >   PI )  ctrl.heading -= 2.0*PI;
  while ( ctrl.err[Z][P] <= -PI )  ctrl.heading += 2.0*PI;
  ctrl.err[Z][D] = -imu1.dEul[Z];
  Y_KIreset = ctrl.ref[CH_Y] / ctrl.range[CH_Y];
  if ( Y_KIreset < -I_RESET || Y_KIreset > I_RESET )
    ctrl.err[Z][I] = 0; 
  else
    ctrl.err[Z][I] += ctrl.err[Z][P] * ctrl.dt;
  ctrl.input[Z] = ctrl.err[Z][P] * ctrl.gain[Z][P] + 
                  ctrl.err[Z][I] * ctrl.gain[Z][I] + 
                  ctrl.err[Z][D] * ctrl.gain[Z][D];

  // Determine throttle adjustment
  double tilt, range;
  tilt = 1 - ( cos(imu1.Eul[X]) * cos(imu1.Eul[Y]) );
  if ( ctrl.norm[CH_D] <=0 )
    range = T_MIN - 1000;
  else
    range = T_MAX - T_MIN;
  ctrl.input[T] = T_MIN + range * ctrl.norm[CH_D] + ctrl.ref[CH_T] + tilt * T_TILT;

  // Set motor outputs
  ushort i;
  if ( ctrl.norm[CH_T] > -0.9 ) {
  sys.output[MOT_FR] = ctrl.input[T] - ctrl.input[X] + ctrl.input[Y] + ctrl.input[Z];
  sys.output[MOT_BL] = ctrl.input[T] + ctrl.input[X] - ctrl.input[Y] + ctrl.input[Z];
  sys.output[MOT_FL] = ctrl.input[T] + ctrl.input[X] + ctrl.input[Y] - ctrl.input[Z];
  sys.output[MOT_BR] = ctrl.input[T] - ctrl.input[X] - ctrl.input[Y] - ctrl.input[Z];
  } else {  for ( i=0; i<4; i++ )  sys.output[i] = 1000;  }

  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctrl_limit
//  Limits the motor outputs to within their min and max ranges.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctrl_limit ( void )  {
  ushort i;
  for ( i=0; i<4; i++ ) {
    if ( sys.output[i] > OUT_MAX )  sys.output[i] = OUT_MAX;
    if ( sys.output[i] < OUT_MIN )  sys.output[i] = OUT_MIN;
  }
  return;
}
*/

