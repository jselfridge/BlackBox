
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

  // Set LED indicator
  led_blink( LED_SIO, 200, 200 );
  usleep(2000000);  // debugging

  // Load PRU driver
  if(DEBUG)  printf("  Load driver \n");
  if( prussdrv_init() ) {
    printf( "Error (sio_init): prussdrv_init failed. \n" ); }

  // Open interrupt
  if(DEBUG)  printf("  Open interrupt \n");
  if( prussdrv_open(PRU_EVTOUT_0) )
    printf( "Error (sio_init): prussdrv_open open failed. \n" );

  // Setup interrupt
  if(DEBUG)  printf("  Setup interrupt \n");
  tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
  prussdrv_pruintc_init(&pruss_intc_initdata);

  // Point to PRU shared memory
  if(DEBUG)  printf("  Memory pointer \n");
  static void* sharedMem;
  prussdrv_map_prumem( PRUSS0_SHARED_DATARAM, &sharedMem );
  memoryPtr = (uint*) sharedMem;
  memset( memoryPtr, 0, 4*24 );

  // Loops per PWM period  [ 21800 loops => 400Hz ]
  memoryPtr[ OUT_OFFSET -1 ] = 21800;

  // Load assembly code
  if(DEBUG)  printf("  Load PRU binaries \n");
  prussdrv_exec_program ( 0, "bin/input.bin" );
  prussdrv_exec_program ( 1, "bin/output.bin" );

  // Asign values to disarm array (move to flag?)
  off[0] = SIO_OFF0;
  off[1] = SIO_OFF1;
  off[2] = SIO_OFF2;
  off[3] = SIO_OFF3;
  off[4] = SIO_OFF4;
  off[5] = SIO_OFF5;
  off[6] = SIO_OFF6;
  off[7] = SIO_OFF7;
  off[8] = SIO_OFF8;
  off[9] = SIO_OFF9;

  // Set LED indicator
  led_on(LED_SIO);

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
  led_off(LED_SIO);
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sio_update
//  Obtains input values and assigns output values.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sio_update ( void )  {

  // Local variables
  ushort ch;
  double norm;

  // Loop through input channels
  for ( ch=0; ch<IN_CH; ch++ ) {

    // Get raw register value
    input.reg[ch] = memoryPtr[ IN_OFFSET + ch ];

    // Convert to PWM units
    input.pwm[ch] = input.reg[ch] * IN_REG2PWM;

    // Determine normalized input value
    norm = ( 2.0 * (double) ( input.reg[ch] - IN_MIN ) / (double) ( IN_MAX - IN_MIN ) ) - 1.0;
    if ( norm >  1.0 )  norm =  1.0;
    if ( norm < -1.0 )  norm = -1.0;
    input.norm[ch] = norm;

  }

  // Assign output register values
  for ( ch=0; ch<OUT_CH; ch++ )  memoryPtr[ OUT_OFFSET + ch ] = output.reg[ch];

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sio_setreg
//  Assign register value output, and sync data structure.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sio_setreg ( ushort ch, ushort reg )  {

  // Check function inputs
  if( ch<0 || ch>=OUT_CH )
    printf( "Error (sio_setreg): Output channel must be between 0-9. \n" );

  // Assign register value
  output.reg[ch] = reg;

  // Calculate PWM value
  output.pwm[ch] = reg * OUT_REG2PWM;

  // Determine normalized input value
  double norm = ( 2.0 * (double) ( output.reg[ch] - OUT_MIN ) / (double) ( OUT_MAX - OUT_MIN ) ) - 1.0;
  if ( norm >  1.0 )  norm =  1.0;
  if ( norm < -1.0 )  norm = -1.0;
  output.norm[ch] = norm;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sio_setpwm
//  Assign PWM value output, and sync data structure.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sio_setpwm ( ushort ch, ushort pwm )  {

  // Check function inputs
  if( ch<0 || ch>=OUT_CH )
    printf( "Error (sio_setpwm): Output channel must be between 0-9. \n" );

  // Assign PWM value
  output.pwm[ch] = pwm;

  // Calculate register value
  output.reg[ch] = pwm * OUT_PWM2REG;

  // Determine normalized input value
  double norm = ( 2.0 * (double) ( output.reg[ch] - OUT_MIN ) / (double) ( OUT_MAX - OUT_MIN ) ) - 1.0;
  if ( norm >  1.0 )  norm =  1.0;
  if ( norm < -1.0 )  norm = -1.0;
  output.norm[ch] = norm;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sio_setnorm
//  Assign normalized value output, and sync data structure.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sio_setnorm ( ushort ch, double norm )  {

  // Check function inputs
  if( ch<0 || ch>=OUT_CH )
    printf( "Error (sio_setnorm): Output channel must be between 0-9. \n" );

  // Assign normalized value
  if ( norm >  1.0 )  norm =  1.0;
  if ( norm < -1.0 )  norm = -1.0;
  output.norm[ch] = norm;

  // Determine register input value
  ushort reg = (ushort) ( (1/2.0) * (double) ( OUT_MAX - OUT_MIN ) * ( norm + 1.0 ) ) + OUT_MIN;
  output.reg[ch] = reg;

  // Calculate pwm value
  output.pwm[ch] = reg * OUT_REG2PWM;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sio_disarm
//  Set all system outputs to their disarmed state.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sio_disarm ( void )  {
  ushort ch;
  for ( ch=0; ch<OUT_CH; ch++ )  sio_setpwm( ch, off[ch] );
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sio_debug
//  BLAH...
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sio_debug() {
  pthread_mutex_lock(&mutex_sio);
  sio_update();
  ushort ch;
  float norm;
  for ( ch=0; ch<10; ch++ ) {
    norm = input.norm[ch];
    sio_setnorm( ch, norm );
  }
  pthread_mutex_unlock(&mutex_sio);
  return;
}



