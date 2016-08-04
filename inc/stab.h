

#ifndef STAB_H
#define STAB_H


#include <stdbool.h>
#include <sys/types.h>


#define STAB_PID      true

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


typedef struct pid_struct {
  double  dt;
  bool    wrap;
  double  pgain;
  double  igain;
  double  dgain;
  double  perr;
  double  ierr;
  double  derr;
  pthread_mutex_t mutex;
} pid_struct;
pid_struct pidX;
pid_struct pidY;
pid_struct pidZ;


typedef struct adapt_struct {
  double cmd;
  double xp;
  double xd;
  double ref;
  double Gxp;
  double Gxd;
  double Gref;
  double G;
  double kxp;
  double kxd;
  double kref;
  double k;
  double kxp_prev;
  double kxd_prev;
  double kref_prev;
  pthread_mutex_t mutex;
} adapt_struct;
adapt_struct adaptX;
adapt_struct adaptY;
adapt_struct adaptZ;


void    stab_init    ( void );
void    stab_exit    ( void );
void    stab_update  ( void );


#endif



