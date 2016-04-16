

#ifndef LOG_H
#define LOG_H


#include <stdbool.h>
#include <stdio.h>
#include <sys/types.h>


#define MAX_LOG_DUR  (20*60)


enum log_index {
  LOG_IO   = 0,
  LOG_IMUA = 1,
  LOG_IMUB = 2,
  LOG_AHRS = 3,
  LOG_GPS  = 4,
  LOG_CTRL = 5
} log_index;


typedef struct log_io_struct {
  ulong  count;
  ulong  limit;
  float  *time;
  ushort *reg;
  ushort *pwm;
  float  *norm;
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


typedef struct log_ahrs_struct {
  ulong count;
  ulong limit;
  float *time;
  ulong *dur;
  float *gyr;
  float *acc;
  float *mag;
  float *quat;
  float *dquat;
  float *eul;
  float *deul;
  float *bias;
  float *fx;
  float *fz;
} log_ahrs_struct;
log_ahrs_struct log_ahrs;


typedef struct log_gps_struct {
  ulong count;
  ulong limit;
  float *time;
  ulong *dur;
  char  *msg;
} log_gps_struct;
log_gps_struct log_gps;

/*
typedef struct log_ctrl_struct {
  ulong  count;
  ulong  limit;
  float  *time;
  ulong  *dur;
  float  *perr;
  float  *ierr;
  float  *derr;
  float  *cmd;
} log_ctrl_struct;
log_ctrl_struct log_ctrl;
*/

typedef struct datalog_struct {
  bool   enabled;
  bool   setup;
  bool   saving;
  float  offset;
  char   *dir;
  char   *path;

  FILE   *fnote;
  FILE   *fin;
  FILE   *fout;
  FILE   *fgyrA;
  FILE   *faccA;
  FILE   *fmagA;
  FILE   *fgyrB;
  FILE   *faccB;
  FILE   *fmagB;
  FILE   *fahrs;
  FILE   *fgps;
  FILE   *fctrl;

} datalog_struct;
datalog_struct datalog;


void  log_init   ( void );
void  log_exit   ( void );
void  log_start  ( void );
void  log_finish ( void );
void  log_record ( enum log_index index );


#endif



