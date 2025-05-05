#ifndef __ICM_20948_H
#define __ICM_20948_H

/**
 * TODO:
 * Add functions to read data
 * Self-test functions
 * Data ready functions
 * DMP functions? or want to do onboard filtering
 * ICM20948 struct
 * Config functions
 * Find out how to access the magnetometer
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32h5xx_hal.h"

#define I2C_WRITE                       0x00
#define I2C_READ                        0x01

/* Address defines */
#define ICM_20948_ADDR                  0b1101000 // pin tied low
// #define ICM_20948_ADDR                  0b1101001 // pin tied high

/* Register defines */
/* User bank 1*/
#define ICM_20948_WHOAMI                0x00

#define ICM_20948_USER_CTRL             0x03

#define ICM_20948_LP_CONFIG             0x05
#define ICM_20948_PWR_MGMT_1            0x06
#define ICM_20948_PWR_MGMT_2            0x07

#define ICM_20948_INT_PIN_CFG           0x0F
#define ICM_20948_INT_ENABLE            0x10
#define ICM_20948_INT_ENABLE_1          0x11
#define ICM_20948_INT_ENABLE_2          0x12
#define ICM_20948_INT_ENABLE_3          0x13
#define ICM_20948_I2C_MST_STATUS        0x17
#define ICM_20948_INT_STATUS            0x19
#define ICM_20948_INT_STATUS_1          0x1A
#define ICM_20948_INT_STATUS_2          0x1B
#define ICM_20948_INT_STATUS_3          0x1C

#define ICM_20948_DELAY_TIMEH           0x28
#define ICM_20948_DELAY_TIMEL           0x29

#define ICM_20948_ACCEL_XOUT_H          0x2D
#define ICM_20948_ACCEL_XOUT_L          0x2E
#define ICM_20948_ACCEL_YOUT_H          0x2F
#define ICM_20948_ACCEL_YOUT_L          0x30
#define ICM_20948_ACCEL_ZOUT_H          0x31
#define ICM_20948_ACCEL_ZOUT_L          0x32

#define ICM_20948_GYRO_XOUT_H           0x33
#define ICM_20948_GYRO_XOUT_L           0x34
#define ICM_20948_GYRO_YOUT_H           0x35
#define ICM_20948_GYRO_YOUT_L           0x36
#define ICM_20948_GYRO_ZOUT_H           0x37
#define ICM_20948_GYRO_ZOUT_L           0x38

#define ICM_20948_TEMP_OUT_H            0x39
#define ICM_20948_TEMP_OUT_L            0x3A

#define ICM_20948_FIFO_RST              0x68
#define ICM_20948_FIFO_MODE             0x69
#define ICM_20948_FIFO_COUNTH           0x70
#define ICM_20948_FIFO_COUNTL           0x71
#define ICM_20948_FIFO_R_W              0x72
#define ICM_20948_DATA_RDY_STATUS       0x74
#define ICM_20948_FIFO_CFG              0x76

#define ICM_20948_REG_BANK_SEL          0x7F

/* User bank 2 */
#define ICM_20948_SELF_TEST_X_GYRO      0x02
#define ICM_20948_SELF_TEST_Y_GYRO      0x03
#define ICM_20948_SELF_TEST_Z_GYRO      0x04
#define ICM_20948_SELF_TEST_X_ACCEL     0x0E
#define ICM_20948_SELF_TEST_Y_ACCEL     0x0F
#define ICM_20948_SELF_TEST_Z_ACCEL     0x10

#define ICM_20948_XA_OFFS_H             0x14
#define ICM_20948_XA_OFFS_L             0x15
#define ICM_20948_YA_OFFS_H             0x17
#define ICM_20948_YA_OFFS_L             0x18
#define ICM_20948_ZA_OFFS_H             0x1A
#define ICM_20948_ZA_OFFS_L             0x1B

/* User bank 3 */
#define ICM_20948_GYRO_SMPLRT_DIV       0x00
#define ICM_20948_GYRO_CONFIG_1         0x01
#define ICM_20948_GYRO_CONFIG_2         0x02

#define ICM_20948_XG_OFFS_USRH          0x03
#define ICM_20948_XG_OFFS_USRL          0x04
#define ICM_20948_YG_OFFS_USRH          0x05
#define ICM_20948_YG_OFFS_USRL          0x06
#define ICM_20948_ZG_OFFS_USRH          0x07
#define ICM_20948_ZG_OFFS_USRL          0x08

#define ICM_20948_ACCEL_SMPLRT_DIV_1    0x10
#define ICM_20948_ACCEL_SMPLRT_DIV_2    0x11
#define ICM_20948_ACCEL_INTEL_CTRL      0x12
#define ICM_20948_ACCEL_WOM_THR         0x13
#define ICM_20948_ACCEL_CONFIG          0x14
#define ICM_20948_ACCEL_CONFIG_2        0x15

#define ICM_20948_FSYNC_CONFIG          0x52

#define ICM_20948_TEMP_CONFIG           0x53

#define ICM_20948_MOD_CTRL_USR          0x84

/* Magnetometer registers */
#define ICM_20948_MAG_WIA               0x01
#define ICM_20948_MAG_ST1               0x10

#define ICM_20948_MAG_HXL               0x11
#define ICM_20948_MAG_HXH               0x12
#define ICM_20948_MAG_HYL               0x13
#define ICM_20948_MAG_HYH               0x14
#define ICM_20948_MAG_HZL               0x15
#define ICM_20948_MAG_HZH               0x16

#define ICM_20948_MAG_ST2               0x18
#define ICM_20948_MAG_CNTL2             0x31
#define ICM_20948_MAG_CNTL3             0x32
#define ICM_20948_MAG_TS1               0x33
#define ICM_20948_MAG_TS2               0x34

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} Vector3;

typedef struct {
    int32_t a;
    int32_t b;
    int32_t c;
    int32_t d;
} Quaternion; // in the form a + bi + cj + dk

#endif