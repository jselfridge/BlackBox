

#ifndef STAB_H
#define STAB_H


#include <stdbool.h>
#include <sys/types.h>


#define IRESET        0.25

#define QUAD_FL       0
#define QUAD_BL       1
#define QUAD_BR       4
#define QUAD_FR       5


typedef struct stab_struct {
  double  dt;
  double  off   [10];
  double  range [4];
  double  thrl  [3];
  double  cmd   [4];
  double  bank;
  double  climb;
  double  heading;
  pthread_mutex_t mutex;
} stab_struct;
stab_struct stab;


typedef struct sf_struct {
  double  dt;
  bool    wrap;
  double  ts;
  double  mp;
  double  sigma;
  double  zeta;
  double  nfreq;
  double  dfreq;
  double  ap;
  double  ad;
  double  b;
  double  kp;
  double  kd;
  double  r;
  double  xp;
  double  xd;
  double  u;
  double  Gp;
  double  Gd;
  double  Gu;
  pthread_mutex_t mutex;
} sf_struct;
sf_struct sfX;
sf_struct sfY;
sf_struct sfZ;


void    stab_init    ( void );
void    stab_exit    ( void );
void    stab_update  ( void );
void    stab_refmdl  ( sf_struct *sf );

#endif



