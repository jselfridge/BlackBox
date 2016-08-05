

#ifndef LOG_H
#define LOG_H


#include <stdbool.h>
#include <stdio.h>
#include <sys/types.h>


#define LOG_MAX_DUR    (20*60)
#define LOG_MAX_PARAM  256


enum log_index {
  LOG_PARAM = 0,
  LOG_IO,
  LOG_IMUA,
  LOG_IMUB,
  LOG_AHRSA,
  LOG_AHRSB,
  LOG_STAB
} log_index;


typedef struct log_param_struct {
  ulong  count;
  ulong  limit;
  float  *time;
  float  *values;
} log_param_struct;
log_param_struct log_param;


typedef struct log_io_struct {
  ulong  count;
  ulong  limit;
  float  *time;
  float  *data;
} log_io_struct;
log_io_struct log_input;
log_io_struct log_output;


typedef struct log_imu_struct {
  ulong count;
  ulong limit;
  float *time;
  ulong *dur;
  short *raw;
  float *scaled;
  float *filter;
} log_imu_struct;
log_imu_struct log_gyrA;
log_imu_struct log_accA;
log_imu_struct log_magA;
log_imu_struct log_gyrB;
log_imu_struct log_accB;
log_imu_struct log_magB;


typedef struct log_comp_struct {
  ulong count;
  ulong limit;
  float *time;
  ulong *dur;
  float *roll;
  float *pitch;
} log_comp_struct;
log_comp_struct log_compA;
log_comp_struct log_compB;


typedef struct log_ahrs_struct {
  ulong count;
  ulong limit;
  float *time;
  ulong *dur;
  float *quat;
  float *dquat;
  float *eul;
  float *deul;
  float *bias;
  float *fx;
  float *fz;
} log_ahrs_struct;
log_ahrs_struct log_ahrsA;
log_ahrs_struct log_ahrsB;


typedef struct log_stab_struct {
  ulong  count;
  ulong  limit;
  float  *time;
  ulong  *dur;
  float  *att;
  float  *ang;
  float  *cmd;
} log_stab_struct;
log_stab_struct log_stab;


typedef struct log_pid_struct {
  double *perr;
  double *ierr;
  double *derr;
} log_pid_struct;
log_pid_struct log_pidX;
log_pid_struct log_pidY;
log_pid_struct log_pidZ;


typedef struct log_adapt_struct {
  double *u;
  double *p;
  double *d;
  double *r;
  double *kp;
  double *kd;
  double *kr;
  double *k;
} log_adapt_struct;
log_adapt_struct log_adaptX;
log_adapt_struct log_adaptY;
log_adapt_struct log_adaptZ;


typedef struct datalog_struct {
  bool   enabled;
  bool   setup;
  bool   saving;
  float  offset;
  char   *dir;
  char   *path;
  FILE   *param;
  FILE   *in;
  FILE   *out;
  FILE   *gyrA;
  FILE   *accA;
  FILE   *magA;
  FILE   *compA;
  FILE   *ahrsA;
  FILE   *gyrB;
  FILE   *accB;
  FILE   *magB;
  FILE   *compB;
  FILE   *ahrsB;
  FILE   *stab;
} datalog_struct;
datalog_struct datalog;


void  log_init   ( void );
void  log_exit   ( void );
void  log_start  ( void );
void  log_record ( enum log_index index );
void  log_finish ( void );


#endif



