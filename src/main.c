
//============================================================
//  main.c
//  Justin M Selfridge
//============================================================
#include "main.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  main
//  Primary code that runs the UAV avionics.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main ( void )  {

  // Begin BlackBox program
  if(DEBUG)  printf("\n--- Begin BlackBox program ---\n");
  led_off(LED_IMU);  led_off(LED_PRU);  led_off(LED_LOG);  led_off(LED_MOT);

  // Initialize subsystems
  sys_init();
  //usleep(250000);
  //imu_init();
  //sio_init();
  //log_init();  //~~~ DEBUGGING ~~~//   

  // Initialize threads
  if(DEBUG)  printf("Initializing threads \n");
  pthread_attr_t attr;  thr_attr(&attr);

  tmr_struct tmr_gyr;
  tmr_struct tmr_debug;

  tmr_gyr.name   = "gyr";
  tmr_debug.name = "debug";

  tmr_gyr.period   = 1000000 / HZ_GYR;
  tmr_debug.period = 1000000 / HZ_DEBUG;

  tmr_gyr.priority   = PRIO_GYR;
  tmr_debug.priority = PRIO_DEBUG;



  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct sched_param param;
  //thr_init( &thr_gyr,   &attr, fcn_gyr   );
  //thr_init( &thr_debug, &attr, fcn_debug );
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Declare thread priority
  param.sched_priority = tmr_gyr.priority;

  // Assign thread priority and attributes
  if( pthread_attr_setschedparam( &attr, &param ) )
    printf( "Error (thr_init): Failed to set '%s' priority. \n", tmr_gyr.name );

  // Create gyro sensor structure
  sensor_struct gyr_sensor;

  // Create gyro argument struct
  gyr_arg_struct gyr_arg;
  gyr_arg.gyr_sensor = &gyr_sensor;
  gyr_arg.gyr_tmr    = &tmr_gyr;

  // Create thread
  if( pthread_create ( &tmr_gyr.id, &attr, &fcn_gyr, &gyr_arg ) )
    printf( "Error (thr_init): Failed to create '%s' thread. \n", tmr_gyr.name );

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Declare thread priority
  param.sched_priority = tmr_debug.priority;

  // Assign thread priority and attributes
  if( pthread_attr_setschedparam( &attr, &param ) )
    printf( "Error (thr_init): Failed to set '%s' priority. \n", tmr_debug.name );

  // Create debug argument struct
  debug_arg_struct debug_arg;
  debug_arg.debug_tmr  = &tmr_debug;
  debug_arg.gyr_sensor = &gyr_sensor;

  // Create thread
  if( pthread_create ( &tmr_debug.id, &attr, &fcn_debug, &debug_arg ) )
    printf( "Error (thr_init): Failed to create '%s' thread. \n", tmr_debug.name );

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




  // Keep the program running
  while(running)  usleep(100000);
  usleep(200000);


  // Exit program
  if(DEBUG)  printf("\n--- Exit BlackBox program ---\n");
  led_off(LED_IMU);  led_off(LED_PRU);  led_off(LED_LOG);  led_off(LED_MOT);

  // Exiting threads
  if(DEBUG)  printf("Closing threads  \n");
  thr_exit(&tmr_gyr);
  thr_exit(&tmr_debug);

  // Close subsystems
  //usleep(200000);
  sys_exit();
  //log_exit();  //~~~ DEBUGGING ~~~//
  //imu_exit();
  //sio_exit();

  return 0;
}









  //int uart1_fd;  //, uart2_fd;
  /*
  // Configure UART1
  printf("Configure UART1 \n");
  struct termios uart1;
  memset( &uart1, 0, sizeof(uart1) );
  uart1.c_iflag     = 0;
  uart1.c_oflag     = 0;
  uart1.c_cflag     = CS8 | CREAD | CLOCAL;
  uart1.c_lflag     = 0;
  uart1.c_cc[VTIME] = 0;
  uart1.c_cc[VMIN]  = 1;
  */
  /*
  // Configure UART2
  printf("Configure UART2 \n");
  struct termios uart2;
  memset( &uart2, 0, sizeof(uart2) );
  uart2.c_iflag     = 0;
  uart2.c_oflag     = 0;
  uart2.c_cflag     = CS8 | CREAD | CLOCAL;
  uart2.c_lflag     = 0;
  uart2.c_cc[VTIME] = 0;
  uart2.c_cc[VMIN]  = 1;
  */

  // Open file descriptors
  //uart1_fd = open ( "/dev/ttyO1", O_RDWR | O_NOCTTY );
  //sys_err( uart1_fd <0, "Error (BLAH): Couldn't open UART1." );
  //uart2_fd = open ( "/dev/ttyO2", O_RDWR | O_NOCTTY );
  //sys_err( uart2_fd <0, "Error (BLAH): Couldn't open UART2." );

  // Set baud rates
  //sys.ret = cfsetispeed( &uart1, B115200 );
  //sys_err( sys.ret <0, "Error (BLAH): Couldn't set UART1 buad rate." );
  //sys.ret = cfsetispeed( &uart2, B115200 );
  //sys_err( sys.ret <0, "Error (BLAH): Couldn't set UART2 buad rate." );

  // Assign attributes
  //sys.ret = tcsetattr( uart1_fd, TCSAFLUSH, &uart1 );
  //sys_err( sys.ret <0, "Error (BLAH): Failed to assign UART1 parameters." );
  //sys.ret = tcsetattr( uart2_fd, TCSAFLUSH, &uart2 );
  //sys_err( sys.ret <0, "Error (BLAH): Failed to assign UART2 parameters." );

  // Send serial data
  //char buf_tx[64] = "Hello there!";
  //int i = write( uart1_fd, &buf_tx, 10 );
  //printf("i: %d  buf_tx: %s \n", i, buf_tx );

  // Obtain serial data
  //char buf_rx[64];
  //memset( &buf_rx, 0, sizeof(buf_rx) );
  //int j = read( uart1_fd, &buf_rx, sizeof(buf_rx) );
  //tcflush( uart1_fd, TCIOFLUSH );
  //printf("j: %d  buf_rx: %s \n", j, buf_rx );

  // Close file descriptors
  //sys.ret = close(uart1_fd);
  //sys_err( sys.ret<0, "Error (BLAH): Couldn't close UART1." );
  //sys.ret = close(uart2_fd);
  //sys_err( sys.ret<0, "Error (BLAH): Couldn't close UART2." );



