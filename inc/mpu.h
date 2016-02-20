
//============================================================
//  mpu.h
//  Justin M Selfridge
//============================================================
#ifndef _MPU_H_
#define _MPU_H_
#include <main.h>


/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_mpu.h
 *      @brief      An I2C-based driver for Invensense gyroscopes.
 *      @details    This driver currently works for the following devices:
 *                  MPU6050
 *                  MPU6500
 *                  MPU9150 (or MPU6050 w/ AK8975 on the auxiliary bus)
 *                  MPU9250 (or MPU6500 w/ AK8963 on the auxiliary bus)
 */


#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)


struct int_param_s {
#if defined EMPL_TARGET_MSP430 || defined MOTION_DRIVER_TARGET_MSP430
    void (*cb)(void);
    unsigned short pin;
    unsigned char lp_exit;
    unsigned char active_low;
#elif defined EMPL_TARGET_UC3L0
    unsigned long pin;
    void (*cb)(volatile void*);
    void *arg;
#else
    unsigned int pin;
#endif
};


#define MPU_INT_STATUS_DATA_READY       (0x0001)
#define MPU_INT_STATUS_DMP              (0x0002)
#define MPU_INT_STATUS_PLL_READY        (0x0004)
#define MPU_INT_STATUS_I2C_MST          (0x0008)
#define MPU_INT_STATUS_FIFO_OVERFLOW    (0x0010)
#define MPU_INT_STATUS_ZMOT             (0x0020)
#define MPU_INT_STATUS_MOT              (0x0040)
#define MPU_INT_STATUS_FREE_FALL        (0x0080)
#define MPU_INT_STATUS_DMP_0            (0x0100)
#define MPU_INT_STATUS_DMP_1            (0x0200)
#define MPU_INT_STATUS_DMP_2            (0x0400)
#define MPU_INT_STATUS_DMP_3            (0x0800)
#define MPU_INT_STATUS_DMP_4            (0x1000)
#define MPU_INT_STATUS_DMP_5            (0x2000)


// Set up APIs
int mpu_init(int fd, struct int_param_s *int_param);
int mpu_init_slave(void);
int mpu_set_bypass(int fd, unsigned char bypass_on);

// Configuration APIs
int mpu_lp_accel_mode(int fd, unsigned short rate);
int mpu_lp_motion_interrupt(int fd, unsigned short thresh, unsigned char time,
    unsigned short lpa_freq);
int mpu_set_int_level(unsigned char active_low);
int mpu_set_int_latched(int fd, unsigned char enable);

int mpu_set_dmp_state(int fd, unsigned char enable);
int mpu_get_dmp_state(unsigned char *enabled);

int mpu_get_lpf(unsigned short *lpf);
int mpu_set_lpf(int fd, unsigned short lpf);

int mpu_get_gyro_fsr(unsigned short *fsr);
int mpu_set_gyro_fsr(int fd, unsigned short fsr);

int mpu_get_accel_fsr(unsigned char *fsr);
int mpu_set_accel_fsr(int fd, unsigned char fsr);

int mpu_get_compass_fsr(unsigned short *fsr);

int mpu_get_gyro_sens(float *sens);
int mpu_get_accel_sens(unsigned short *sens);

int mpu_get_sample_rate(unsigned short *rate);
int mpu_set_sample_rate(int fd, unsigned short rate);
int mpu_get_compass_sample_rate(unsigned short *rate);
int mpu_set_compass_sample_rate(int fd, unsigned short rate);

int mpu_get_fifo_config(unsigned char *sensors);
int mpu_configure_fifo(int fd, unsigned char sensors);

int mpu_get_power_state(unsigned char *power_on);
int mpu_set_sensors(int fd, unsigned char sensors);

int mpu_read_6500_accel_bias(int fd, long *accel_bias);
int mpu_set_gyro_bias_reg(int fd, long * gyro_bias);
int mpu_set_accel_bias_6500_reg(int fd, const long *accel_bias);
int mpu_read_6050_accel_bias(int fd, long *accel_bias);
int mpu_set_accel_bias_6050_reg(int fd, const long *accel_bias);

// Data getter/setter APIs
int mpu_get_gyro_reg(int fd, short *data, unsigned long *timestamp);
int mpu_get_accel_reg(int fd, short *data, unsigned long *timestamp);
int mpu_get_compass_reg(int fd, short *data, unsigned long *timestamp);
int mpu_get_temperature(int fd, long *data, unsigned long *timestamp);

int mpu_get_int_status(int fd, short *status);
int mpu_read_fifo(int fd, short *gyro, short *accel, unsigned long *timestamp,
    unsigned char *sensors, unsigned char *more);
int mpu_read_fifo_stream(int fd, unsigned short length, unsigned char *data,
    unsigned char *more);
int mpu_reset_fifo(int fd);

int mpu_write_mem(int fd, unsigned short mem_addr, unsigned short length,
    unsigned char *data);
int mpu_read_mem(int fd, unsigned short mem_addr, unsigned short length,
    unsigned char *data);
int mpu_load_firmware(int fd, unsigned short length, const unsigned char *firmware,
    unsigned short start_addr, unsigned short sample_rate);

int mpu_reg_dump(int fd);
int mpu_read_reg(int fd, unsigned char reg, unsigned char *data);
int mpu_run_self_test(int fd, long *gyro, long *accel);
int mpu_run_6500_self_test(long *gyro, long *accel, unsigned char debug);
int mpu_register_tap_cb(void (*func)(unsigned char, unsigned char));

#endif



