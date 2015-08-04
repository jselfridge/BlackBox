
//============================================================
//  ctrl.c
//  Justin M Selfridge
//============================================================
#include "uav.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctrl_init
//  Initializes the control structure.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctrl_init ( void )  {

  // Increment counter
  ushort i;

  // Ensure motors are off
  ctrl_disarm();

  // Set reference ranges
  range[CH_R] = R_RANGE;
  range[CH_P] = P_RANGE;
  range[CH_Y] = Y_RANGE;
  range[CH_T] = T_RANGE;

  // Set full stick counters
  for ( i=0; i<4; i++ ) {
    fullStick[i][HIGH] = 0;
    fullStick[i][LOW]  = 0;
  }
  stickHold = STICK_HOLD * FREQ;

  // Set command flags
  motorsArmed = false;

  // Set integral accumulators
  R_KIerr = 0;
  P_KIerr = 0;
  Y_KIerr = 0;
  heading = 0;  // Assign value when arming motors

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctrl_disarm
//  Disarms all the motors
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctrl_disarm ( void )  {
  ushort i;
  for ( i=0; i<8; i++ ) {
    uav.servo[i] = 1000;
    pru_send_pulse( i, uav.servo[i] );
  }
  return;
}


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


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctrl_ref
//  Takes PWM inputs and converts to suitable reference signals.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctrl_ref ( void )  {
  ushort i;
  for ( i=0; i<8; i++ ) {
    norm[i] = 2.0 * ( uav.radio[i] - IN_MID ) / (float)( IN_MAX - IN_MIN );
    if ( norm[i] >  1.0 )  norm[i] =  1.0;
    if ( norm[i] < -1.0 )  norm[i] = -1.0;
    if (i<4)  ref[i] = norm[i] * range[i];
  }
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctrl_flags
//  Checks radio stick positions for commands. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctrl_flags ( void )  {

  // Update full stick counter
  ushort i;
  for ( i=0; i<4; i++ ) {
    if ( norm[i] >  0.95 ) {  fullStick[i][HIGH]++;  }  else {  fullStick[i][HIGH] = 0;  }
    if ( norm[i] < -0.95 ) {  fullStick[i][LOW]++;   }  else {  fullStick[i][LOW]  = 0;  }
  }

  // Only execute stick commands with zero throttle
  if ( fullStick[CH_T][DOWN] >= stickHold )  {
    
    // Data log: roll stick only, no yaw command
    if ( !fullStick[CH_Y][LEFT] && !fullStick[CH_Y][RIGHT] )  {
      if ( fullStick[CH_R][LEFT]  >= stickHold ) {  uav.logdata = true;  led_on(LED_LOG);   }
      if ( fullStick[CH_R][RIGHT] >= stickHold ) {  uav.logdata = false; led_off(LED_LOG);  }
    }

    // Motor arming: yaw stick only, no roll command
    if ( !fullStick[CH_R][LEFT] && !fullStick[CH_R][RIGHT] )  {
      if ( fullStick[CH_Y][RIGHT] >= stickHold ) {  motorsArmed = true;  led_on(LED_MOT);   heading = mpu1.Eul[Z];  }
      if ( fullStick[CH_Y][LEFT]  >= stickHold ) {  motorsArmed = false; led_off(LED_MOT);                         }
    }

    // Exit program: roll and yaw together to exit program
    if (  fullStick[CH_Y][LEFT] >= stickHold  &&  fullStick[CH_R][RIGHT] >= stickHold  ) {
      uav.running = false;
      led_off(LED_MPU);
      led_off(LED_PRU);
      led_off(LED_LOG);
      led_off(LED_MOT);
    }

  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctrl_switch
//  Monitor radio switch to implement different control laws.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctrl_switch ( void )  {

  // Check arming flag
  if (motorsArmed) {

  // Run switch command
  ctrl_pid();
  //ctrl_siso_sf();
  //ctrl_mimo_sf();
  //ctrl_sysid();
  //ctrl_mrac();
  //ctrl_L1();

  }

  else ctrl_disarm();

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctrl_pid
//  Apply PID control to vehcile attitude.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctrl_pid ( void )  {

  // Determine roll adjustment
  double R_KPerr, R_KDerr, R_KIreset, R_adj;
  R_KPerr = ref[CH_R] - mpu1.Eul[X];
  R_KDerr = -mpu1.dEul[X];
  R_KIreset = ref[CH_R] / range[CH_R];
  if ( R_KIreset < -I_RESET || R_KIreset > I_RESET )  R_KIerr = 0; 
  else  R_KIerr += R_KPerr * DT;
  R_adj  = R_KPerr * R_KP + R_KIerr * R_KI + R_KDerr * R_KD;

  // Determine pitch adjustment
  double P_KPerr, P_KDerr, P_KIreset, P_adj;
  P_KPerr = ref[CH_P] - mpu1.Eul[Y];
  P_KDerr = -mpu1.dEul[Y];
  P_KIreset = ref[CH_P] / range[CH_P];
  if ( P_KIreset < -I_RESET || P_KIreset > I_RESET )  P_KIerr = 0; 
  else  P_KIerr += P_KPerr * DT;
  P_adj  = P_KPerr * P_KP + P_KIerr * P_KI + P_KDerr * P_KD;

  // Determine yaw adjustment
  double Y_KPerr, Y_KDerr, Y_KIreset, Y_adj;
  if ( norm[CH_T] > -0.9 && fabs(norm[CH_Y]) > 0.15 )  heading += ref[CH_Y] * DT;
  if ( heading >   PI )  heading -= PI2;
  if ( heading <= -PI )  heading += PI2;
  Y_KPerr = heading - mpu1.Eul[Z];
  if ( Y_KPerr >   PI )  heading -= PI2;
  if ( Y_KPerr <= -PI )  heading += PI2;
  Y_KDerr = -mpu1.dEul[Z];
  Y_KIreset = ref[CH_Y] / range[CH_Y];
  if ( Y_KIreset < -I_RESET || Y_KIreset > I_RESET )  Y_KIerr = 0; 
  else  Y_KIerr += Y_KPerr * DT;
  Y_adj = Y_KPerr * Y_KP + Y_KIerr * Y_KI + Y_KDerr * Y_KD;

  // Determine throttle adjustment
  //double T_adj = ref[CH_T] + norm[CH_D] * T_TUNE;
  double tilt = 1 - ( cos(mpu1.Eul[X]) * cos(mpu1.Eul[Y]) );
  double T_adj = ref[CH_T] + norm[CH_D] * T_TUNE + tilt * T_TILT;

  // Set motor outputs
  ushort i;
  if ( norm[CH_T] > -0.9 ) {
  uav.servo[MOT_FR] = T_HOVER + T_adj - R_adj + P_adj + Y_adj;
  uav.servo[MOT_BL] = T_HOVER + T_adj + R_adj - P_adj + Y_adj;
  uav.servo[MOT_FL] = T_HOVER + T_adj + R_adj + P_adj - Y_adj;
  uav.servo[MOT_BR] = T_HOVER + T_adj - R_adj - P_adj - Y_adj;
  } else {  for ( i=0; i<4; i++ )  uav.servo[i] = 1000;  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ctrl_limit
//  Limits the motor outputs to within their min and max ranges.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ctrl_limit ( void )  {
  ushort i;
  for ( i=0; i<4; i++ ) {
    if ( uav.servo[i] > 2000 )  uav.servo[i] = 2000;
    if ( uav.servo[i] < 1000 )  uav.servo[i] = 1000;
  }
  return;
}



