
//============================================================
//  sys.h
//  Justin M Selfridge
//============================================================
#ifndef _SYS_H_
#define _SYS_H_
#include <main.h>


// Define statements
#define SYS_LOOP        10000000u
#define SYS_FREQ        (double)( NSEC_PER_SEC / SYS_LOOP )
#define SYS_DT          (double)( 1/SYS_FREQ )


// System structure
typedef struct {
  short input[10];
  short output[10];
  bool  running;
} sys_struct;
sys_struct  sys;


// Global variables
struct sigaction sys_signal;
//uint* memoryPtr;


// Function declarations
void sys_err    ( bool cond, char* msg );
void sys_init   (  );
void sys_loop   (  );
void sys_debug  (  );
void sys_exit   (  );
void sys_memory ( void );


#endif



