

#ifndef ADAPT_H
#define ADAPT_H


typedef struct adapt_struct {
  double x1;
  double x2;
  double r;
  double u;
  double kx1;
  double kx2;
  double kr;
  double kp;
  double kx1p;
  double kx2p;
  double krp;
  double Gx1;
  double Gx2;
  double Gr;
  double Gp;
  //pthread_mutex_t mutex;
} adapt_struct;
adapt_struct adapt_test;

void    adapt_init    ( void );
void    adapt_exit    ( void );
void    adapt_update  ( void );


#endif



