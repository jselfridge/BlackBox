

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
  LOG_IMU,
  LOG_STAB,
  LOG_INS, 
  LOG_NAV
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
} log_imu_struct;
log_imu_struct log_imu;


typedef struct log_imudata_struct {
  short *raw;
  float *scaled;
  float *filter;
} log_imudata_struct;
log_imudata_struct log_gyrA;
log_imudata_struct log_accA;
log_imudata_struct log_magA;
log_imudata_struct log_gyrB;
log_imudata_struct log_accB;
log_imudata_struct log_magB;


typedef struct log_comp_struct {
  float *roll;
  float *pitch;
} log_comp_struct;
log_comp_struct log_compA;
log_comp_struct log_compB;


typedef struct log_ahrs_struct {
  float *quat;
  float *dquat;
  float *eul;
  float *deul;
} log_ahrs_struct;
log_ahrs_struct log_ahrsA;
log_ahrs_struct log_ahrsB;


typedef struct log_rot_struct {
  float *att;
  float *ang;
} log_rot_struct;
log_rot_struct log_rot;


typedef struct log_stab_struct {
  ulong  count;
  ulong  limit;
  float  *time;
  ulong  *dur;
} log_stab_struct;
log_stab_struct log_stab;


typedef struct log_sf_struct {
  float *r;
  float *xp;
  float *xd;
  float *zp;
  float *zd;
  float *u;
} log_sf_struct;
log_sf_struct log_sfx;
log_sf_struct log_sfy;
log_sf_struct log_sfz;


typedef struct log_id_struct {
  float *z1;
  float *z2;
  float *p1;
  float *p2;
} log_id_struct;
//log_id_struct log_idx;
//log_id_struct log_idy;
//log_id_struct log_idz;

/*
typedef struct log_ekf_struct {
  ulong  count;
  ulong  limit;
  float  *time;
  ulong  *dur;
  float  *x;
  float  *z;
  float  *f;
  float  *h;
  float  *F;
  float  *P;
  float  *T;
  float  *S;
} log_ekf_struct;
log_ekf_struct log_ekf;
*/
/*
typedef struct log_ins_struct {
  ulong  count;
  ulong  limit;
  float  *time;
  ulong  *dur;
  float  *K;
} log_ins_struct;
log_ins_struct log_ins;
*/


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
  FILE   *imu;
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
  FILE   *rot;
  FILE   *stab;
  FILE   *sfx;
  FILE   *sfy;
  FILE   *sfz;
} datalog_struct;
datalog_struct datalog;


void  log_init   ( void );
void  log_exit   ( void );
void  log_start  ( void );
void  log_record ( enum log_index index );
void  log_finish ( void );


#endif



