
//============================================================
//  sio.c
//  Justin M Selfridge
//============================================================
#include "sio.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sio_init
//  Initializes the system input/output PRU subcomponents.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sio_init ( void )  {
  if(DEBUG)  printf("Initializing system inputs/outputs \n");

  ushort iarray[10]  = { 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900 };
  ushort oarray[10] = { 1900, 1800, 1700, 1600, 1500, 1400, 1300, 1200, 1100, 1000 };

  ushort i;
  for ( i=0; i<10; i++ ) {
    input.pwm[i]  = iarray[i];
    output.pwm[i] = oarray[i];
  }

  // Initialise driver
  prussdrv_init();

  // Open interrupt
  if( prussdrv_open(PRU_EVTOUT_0) )
    printf( "Error (sio_init): prussdrv_open open failed. \n" );

  //Initialise interrupt
  tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
  prussdrv_pruintc_init(&pruss_intc_initdata);

  // Point to PRU shared memory
  static void* sharedMem;
  prussdrv_map_prumem( PRUSS0_SHARED_DATARAM, &sharedMem );
  memoryPtr = (uint*) sharedMem;
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
//  sio_exit
//  Exits the system input/output PRU subcomponents.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sio_exit ( void )  {
  if(DEBUG)  printf("Close system input/output \n");
  prussdrv_pru_disable(0);
  prussdrv_pru_disable(1);
  prussdrv_exit(); 
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  pru_read_pulse
//  Reads pulse value of PWM signal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*float pru_read_pulse ( int ch )  {
  sys_err( ch<0 || ch>=IN_CH, "Error (pru_read_pulse): Radio channel must be between 0-7."    );
  sys_err( memoryPtr == NULL, "Error (pru_read_pulse): PRU radio controller not initialized." );
  return memoryPtr[ IN_OFFSET +ch ] * (30.0/200);
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  pru_read_norm
//  Reads normalized value of PWM signal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*float pru_read_norm ( int ch )  {
  sys_err( ch<0 || ch>=IN_CH, "Error (pru_read_norm): Radio channel must be between 0-7.");
  float pwm = pru_read_pulse (ch);
  float norm = ( pwm - IN_MIN ) / (float)( IN_MAX - IN_MIN );
  if (norm>1.0)  norm = 1.0;
  if (norm<0.0)  norm = 0.0;
  return norm;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  pru_send_pulse
//  Sends pulse value of PWM signal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void pru_send_pulse ( int ch, int pwm )  {
  sys_err( ch<0 || ch>=OUT_CH, "Error (pru_send_pulse): Servo channel must be between 0-9."     );
  sys_err( memoryPtr == NULL, "Error (pru_send_pulse): PRU servo controller not initialized." );
  memoryPtr[ OUT_OFFSET +ch ] = (pwm*200)/23;  // ORIG: /19
  return;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  pru_send_norm
//  Sends normalized value of PWM signal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*void pru_send_norm ( int ch, float norm )  {
  sys_err( ch<0 || ch>=OUT_CH,  "Error (pru_send_norm): Servo channel must be between 0-7."        );
  sys_err( norm<0.0 || norm>1.0, "Error (pru_send_norm): Normalized input must be between 0.0-1.0." );
  float pulse = OUT_MIN + ( norm * ( OUT_MAX - OUT_MIN ) );
  pru_send_pulse ( ch, pulse );
  return;
}
*/


