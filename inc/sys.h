

#ifndef SYS_H
#define SYS_H


#include <signal.h>
#include <stdbool.h>


#define DEBUG       true
#define SYS_STACK  ( 100 * 1024 * 1024 )


struct sigaction sys_signal;


void sys_init   ( void );
void sys_exit   ( void );
void sys_quit   ( );
void sys_update ( void );


#endif



