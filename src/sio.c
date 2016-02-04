
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

  // Channel index
  ushort ch;

  // Update input channels
  pthread_mutex_lock(&mutex_input);
  for ( ch=0; ch<IN_CH; ch++ ) {
    input.reg[ch]  = memoryPtr[ IN_OFFSET + ch ];
    input.pwm[ch]  = input.reg[ch] * IN_REG2PWM;
    input.norm[ch] = sio_norm( input.reg[ch], 'i' );
  }
  pthread_mutex_unlock(&mutex_input);

  // Assign output signal register values
  pthread_mutex_lock(&mutex_output);
  for ( ch=0; ch<OUT_CH; ch++ )  memoryPtr[ OUT_OFFSET + ch ] = output.reg[ch];
  pthread_mutex_unlock(&mutex_output);

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

  // Assign output values
  pthread_mutex_lock(&mutex_output);
  output.reg[ch]  = reg;
  output.pwm[ch]  = reg * OUT_REG2PWM;
  output.norm[ch] = sio_norm( reg, 'o' );
  pthread_mutex_unlock(&mutex_output);

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

  // Assign output values
  pthread_mutex_lock(&mutex_output);
  output.pwm[ch]  = pwm;
  output.reg[ch]  = pwm * OUT_PWM2REG;
  output.norm[ch] = sio_norm( output.reg[ch], 'o' );
  pthread_mutex_unlock(&mutex_output);

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

  // Perform local calculations
  if ( norm >  1.0 )  norm =  1.0;
  if ( norm < -1.0 )  norm = -1.0;
  ushort reg = (ushort) ( (1/2.0) * (double) ( OUT_MAX - OUT_MIN ) * ( norm + 1.0 ) ) + OUT_MIN;

  // Assign output values
  pthread_mutex_lock(&mutex_output);
  output.norm[ch] = norm;
  output.reg[ch]  = reg;
  output.pwm[ch]  = reg * OUT_REG2PWM;
  pthread_mutex_unlock(&mutex_output);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sio_norm
//  Converts a register value into a normalized range.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double sio_norm ( ushort reg, char dir )  {

  // Local variables
  double num, den, norm;

  // Identify correct range
  switch(dir) {

    case 'i' :
      num = (double) (    reg - IN_MIN );
      den = (double) ( IN_MAX - IN_MIN );
      break;

    case 'o' :
      num = (double) (     reg - OUT_MIN );
      den = (double) ( OUT_MAX - OUT_MIN );
      break;

    default :
      printf( "Error (sio_norm): Specify the value as an input(i) or an output(o). \n" );
      num = 1.0;
      den = 2.0;
      break;

  }

  // Calculate normalized value
  norm = ( 2.0 * num / den ) - 1.0;
  if ( norm >  1.0 )  norm =  1.0;
  if ( norm < -1.0 )  norm = -1.0;

  return norm;
}



