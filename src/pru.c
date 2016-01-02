
//============================================================
//  pru.c
//  Justin M Selfridge
//============================================================
#include "pru.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  pru_init
//  Initializes the PRU subsystems.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void pru_init ( void )  {
  if(DEBUG)  printf("Initializing PRU \n");

  // Initialise driver
  prussdrv_init();
  
  // Open interrupt
  sys.ret = prussdrv_open(PRU_EVTOUT_0);
  sys_err( sys.ret, "Error (pru_init): prussdrv_open open failed" );

  //Initialise interrupt
  tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
  prussdrv_pruintc_init(&pruss_intc_initdata);
  
  // Point to PRU shared memory
  static void* sharedMem;
  prussdrv_map_prumem( PRUSS0_SHARED_DATARAM, &sharedMem );
  memoryPtr = (unsigned int*) sharedMem;
  memset( memoryPtr, 0, 4*24 );

  // Loops per PWM period [21800 => 400Hz]
  memoryPtr[ OUT_OFFSET -1 ] = 44000;

  // Load assembly code
  prussdrv_exec_program ( 0, "bin/input.bin" );
  prussdrv_exec_program ( 1, "bin/output.bin" );

  // Set LED indicator
  led_on(LED_PRU);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  pru_exit
//  Exits the PRU subsystems.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void pru_exit ( void )  {
  if(DEBUG)  printf("Closing PRU \n");
  prussdrv_pru_disable(0);
  prussdrv_pru_disable(1);
  prussdrv_exit(); 
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  pru_read_pulse
//  Reads pulse value of PWM signal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
float pru_read_pulse ( int ch )  {
  sys_err( ch<0 || ch>=IN_CH, "Error (pru_read_pulse): Radio channel must be between 0-7."    );
  sys_err( memoryPtr == NULL, "Error (pru_read_pulse): PRU radio controller not initialized." );
  return memoryPtr[ IN_OFFSET +ch ] * (30.0/200);
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  pru_read_norm
//  Reads normalized value of PWM signal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
float pru_read_norm ( int ch )  {
  sys_err( ch<0 || ch>=IN_CH, "Error (pru_read_norm): Radio channel must be between 0-7.");
  float pwm = pru_read_pulse (ch);
  float norm = ( pwm - IN_MIN ) / (float)( IN_MAX - IN_MIN );
  if (norm>1.0)  norm = 1.0;
  if (norm<0.0)  norm = 0.0;
  return norm;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  pru_send_pulse
//  Sends pulse value of PWM signal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void pru_send_pulse ( int ch, int pwm )  {
  sys_err( ch<0 || ch>=OUT_CH, "Error (pru_send_pulse): Servo channel must be between 0-9."     );
  sys_err( memoryPtr == NULL, "Error (pru_send_pulse): PRU servo controller not initialized." );
  memoryPtr[ OUT_OFFSET +ch ] = (pwm*200)/23;  // ORIG: /19
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  pru_send_norm
//  Sends normalized value of PWM signal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void pru_send_norm ( int ch, float norm )  {
  sys_err( ch<0 || ch>=OUT_CH,  "Error (pru_send_norm): Servo channel must be between 0-7."        );
  sys_err( norm<0.0 || norm>1.0, "Error (pru_send_norm): Normalized input must be between 0.0-1.0." );
  float pulse = OUT_MIN + ( norm * ( OUT_MAX - OUT_MIN ) );
  pru_send_pulse ( ch, pulse );
  return;
}



