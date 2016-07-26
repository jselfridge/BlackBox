

#ifndef ADAPT_H
#define ADAPT_H


typedef struct adapt_struct {
  double kx1;
  double kx2;
  double kr;
  //pthread_mutex_t mutex;
} adapt_struct;
adapt_struct adapt_test;

void    adapt_init    ( void );
void    adapt_exit    ( void );
void    adapt_update  ( void );


#endif



