

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
  double  bank;
  double  climb;
  double  heading;
  pthread_mutex_t mutex;
} stab_struct;
stab_struct stab;


typedef struct sf_struct {
  bool    wrap;
  double  r;
  double  zp;
  double  zd;
  double  kp;
  double  kd;
  double  u;
  //double  dt;
  //double  ts;
  //double  mp;
  //double  sigma;
  //double  zeta;
  //double  nfreq;
  //double  dfreq;
  //double  ap;
  //double  ad;
  //double  j;
  //double  ku;
  //double  r;
  //double  xp;
  //double  xd;
  //double  Gp;
  //double  Gd;
  //double  Gu;
  pthread_mutex_t mutex;
} sf_struct;
sf_struct sfx;
//sf_struct sfy;
//sf_struct sfz;


typedef struct id_struct {
  double z1;
  double z2;
  double p1;
  double p2;
  double u1;
  double u2;
  double y1;
  double y2;
  pthread_mutex_t mutex;
} id_struct;
id_struct idx;
//id_struct idy;
//id_struct idz;


void    stab_init    ( void );
void    stab_exit    ( void );
void    stab_update  ( void );
//void    stab_refmdl  ( sf_struct *sf );

#endif



